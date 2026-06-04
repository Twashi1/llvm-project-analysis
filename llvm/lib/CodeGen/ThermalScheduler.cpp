#include "llvm/CodeGen/ThermalScheduler.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineScheduler.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include "llvm/CodeGen/ScheduleDAGInstrs.h"
#include "llvm/CodeGen/ScheduleDAGMutation.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetSchedule.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "thermal"

namespace llvm {
static bool isFP(const MachineInstr &MI) {
  const MachineFunction &MF = *MI.getMF();
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();

  for (const MachineOperand &MO : MI.operands()) {
    if (!MO.isReg() || !Register::isVirtualRegister(MO.getReg()))
      continue;

    const TargetRegisterClass *RC = MF.getRegInfo().getRegClass(MO.getReg());

    if (RC && TRI->isTypeLegalForClass(*RC, MVT::f32)) {
      LLVM_DEBUG(dbgs() << "Detected floating point instruction\n");

      return true;
    }
  }

  return false;
}

void ThermalScheduler::initPolicy(MachineBasicBlock::iterator Begin,
                                  MachineBasicBlock::iterator End,
                                  unsigned NumRegionInstrs) {
  GenericScheduler::initPolicy(Begin, End, NumRegionInstrs);

  // TODO: should remove later, just makes some logic easier
  RegionPolicy.OnlyTopDown = true;
}

bool ThermalScheduler::tryCandidate(SchedCandidate &Cand,
                                    SchedCandidate &TryCand,
                                    SchedBoundary *Zone) const {
  // Initialize the candidate if needed.
  if (!Cand.isValid()) {
    TryCand.Reason = FirstValid;
    return true;
  }

  // Bias PhysReg Defs and copies to their uses and defined respectively.
  if (tryGreater(biasPhysReg(TryCand.SU, TryCand.AtTop),
                 biasPhysReg(Cand.SU, Cand.AtTop), TryCand, Cand, PhysReg))
    return TryCand.Reason != NoCand;

  // Avoid exceeding the target's limit.
  if (DAG->isTrackingPressure() &&
      tryPressure(TryCand.RPDelta.Excess, Cand.RPDelta.Excess, TryCand, Cand,
                  RegExcess, TRI, DAG->MF))
    return TryCand.Reason != NoCand;

  // Avoid increasing the max critical pressure in the scheduled region.
  if (DAG->isTrackingPressure() &&
      tryPressure(TryCand.RPDelta.CriticalMax, Cand.RPDelta.CriticalMax,
                  TryCand, Cand, RegCritical, TRI, DAG->MF))
    return TryCand.Reason != NoCand;

  // Custom heuristic here
  const MachineInstr *CandInstr = Cand.SU->getInstr();
  const MachineInstr *TryCandInstr = TryCand.SU->getInstr();

  // I think "Weak" is stronger than "NodeOrder"
  if (tryGreater(isFP(*TryCandInstr), isFP(*CandInstr), TryCand, Cand,
                 NodeOrder)) {
    return TryCand.Reason != NoCand;
  }

  // We only compare a subset of features when comparing nodes between
  // Top and Bottom boundary. Some properties are simply incomparable, in many
  // other instances we should only override the other boundary if something
  // is a clear good pick on one boundary. Skip heuristics that are more
  // "tie-breaking" in nature.
  bool SameBoundary = Zone != nullptr;
  if (SameBoundary) {
    // For loops that are acyclic path limited, aggressively schedule for
    // latency. Within an single cycle, whenever CurrMOps > 0, allow normal
    // heuristics to take precedence.
    if (Rem.IsAcyclicLatencyLimited && !Zone->getCurrMOps() &&
        tryLatency(TryCand, Cand, *Zone))
      return TryCand.Reason != NoCand;

    // Prioritize instructions that read unbuffered resources by stall cycles.
    if (tryLess(Zone->getLatencyStallCycles(TryCand.SU),
                Zone->getLatencyStallCycles(Cand.SU), TryCand, Cand, Stall))
      return TryCand.Reason != NoCand;
  }

  // Keep clustered nodes together to encourage downstream peephole
  // optimizations which may reduce resource requirements.
  //
  // This is a best effort to set things up for a post-RA pass. Optimizations
  // like generating loads of multiple registers should ideally be done within
  // the scheduler pass by combining the loads during DAG postprocessing.
  const ClusterInfo *CandCluster = Cand.AtTop ? TopCluster : BotCluster;
  const ClusterInfo *TryCandCluster = TryCand.AtTop ? TopCluster : BotCluster;
  if (tryGreater(TryCandCluster && TryCandCluster->contains(TryCand.SU),
                 CandCluster && CandCluster->contains(Cand.SU), TryCand, Cand,
                 Cluster))
    return TryCand.Reason != NoCand;

  if (SameBoundary) {
    // Weak edges are for clustering and other constraints.
    if (tryLess(getWeakLeft(TryCand.SU, TryCand.AtTop),
                getWeakLeft(Cand.SU, Cand.AtTop), TryCand, Cand, Weak))
      return TryCand.Reason != NoCand;
  }

  // Avoid increasing the max pressure of the entire region.
  if (DAG->isTrackingPressure() &&
      tryPressure(TryCand.RPDelta.CurrentMax, Cand.RPDelta.CurrentMax, TryCand,
                  Cand, RegMax, TRI, DAG->MF))
    return TryCand.Reason != NoCand;

  if (SameBoundary) {
    // Avoid critical resource consumption and balance the schedule.
    TryCand.initResourceDelta(DAG, SchedModel);
    if (tryLess(TryCand.ResDelta.CritResources, Cand.ResDelta.CritResources,
                TryCand, Cand, ResourceReduce))
      return TryCand.Reason != NoCand;
    if (tryGreater(TryCand.ResDelta.DemandedResources,
                   Cand.ResDelta.DemandedResources, TryCand, Cand,
                   ResourceDemand))
      return TryCand.Reason != NoCand;

    // Avoid serializing long latency dependence chains.
    // For acyclic path limited loops, latency was already checked above.
    if (!RegionPolicy.DisableLatencyHeuristic && TryCand.Policy.ReduceLatency &&
        !Rem.IsAcyclicLatencyLimited && tryLatency(TryCand, Cand, *Zone))
      return TryCand.Reason != NoCand;

    // Fall through to original instruction order.
    if ((Zone->isTop() && TryCand.SU->NodeNum < Cand.SU->NodeNum) ||
        (!Zone->isTop() && TryCand.SU->NodeNum > Cand.SU->NodeNum)) {
      LLVM_DEBUG(dbgs() << "Defaulting to source order\n");
      TryCand.Reason = NodeOrder;
      return true;
    }
  }

  return false;
}

static ScheduleDAGInstrs *
createThermalMachineScheduler(MachineSchedContext *C) {
  return new ScheduleDAGMILive(C, std::make_unique<ThermalScheduler>(C));
}

static MachineSchedRegistry
    ThermalSchedRegistry("thermal", "Depth-biased GenericScheduler",
                         createThermalMachineScheduler);

// I think this module kept getting optimised as dead-code, so we reference
// these in MachineScheduler.cpp to ensure that doesn't happen?
extern volatile int ThermalSchedulerAnchorSource;
volatile int ThermalSchedulerAnchorSource = 0;
} // namespace llvm

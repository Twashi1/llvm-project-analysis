#include "llvm/CodeGen/ThermalScheduler.h"
#include "llvm/CodeGen/MachineScheduler.h"
#include "llvm/CodeGen/ScheduleDAG.h"
#include "llvm/CodeGen/ScheduleDAGInstrs.h"
#include "llvm/CodeGen/ScheduleDAGMutation.h"
#include "llvm/CodeGen/TargetSchedule.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "thermal"

namespace llvm {
bool ThermalScheduler::tryCandidate(SchedCandidate &Cand,
                                    SchedCandidate &TryCand,
                                    SchedBoundary *Zone) const {

  // // Let GenericScheduler do its normal comparison first.
  // if (GenericScheduler::tryCandidate(Cand, TryCand, Zone))
  //   return true;
  //
  // // If GenericScheduler preferred the current candidate,
  // // add a depth-based tie breaker.
  //
  // if (!Cand.SU || !TryCand.SU)
  //   return false;
  //
  // unsigned CandDepth = Cand.SU->getDepth();
  // unsigned TryDepth = TryCand.SU->getDepth();
  //
  // if (TryDepth > CandDepth)
  //   return true;
  //
  // return false;
  return GenericScheduler::tryCandidate(Cand, TryCand, Zone);
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

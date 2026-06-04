#ifndef LLVM_CODEGEN_MYSCHEDULER_H
#define LLVM_CODEGEN_MYSCHEDULER_H

#include "llvm/CodeGen/MachineScheduler.h"

namespace llvm {

class ThermalScheduler : public GenericScheduler {
public:
  explicit ThermalScheduler(const MachineSchedContext *C)
      : GenericScheduler(C) {}

protected:
  bool tryCandidate(SchedCandidate &Cand, SchedCandidate &TryCand,
                    SchedBoundary *Zone) const override;

  void initPolicy(MachineBasicBlock::iterator Begin,
                  MachineBasicBlock::iterator End,
                  unsigned NumRegionInstrs) override;
};

} // namespace llvm

#endif

#ifndef LLVM_ANALYSIS_EXTLOOPSPLITTINGPASS_H
#define LLVM_ANALYSIS_EXTLOOPSPLITTINGPASS_H

#include "llvm/Analysis/LoopPass.h"

namespace llvm {

class ExtLoopSplittingPass : public LoopPass {
public:
  static char ID;
  ExtLoopSplittingPass() : LoopPass(ID) {}

  bool runOnLoop(Loop *L, LPPassManager &LPM) override;
  void getAnalysisUsage(AnalysisUsage &AU) const override;
  StringRef getPassName() const override { return "Ext Loop Splitting Pass"; }
};

// Factory function
LoopPass *createExtLoopSplittingPass();

} // namespace llvm

#endif // LLVM_ANALYSIS_EXTLOOPSPLITTINGPASS_H

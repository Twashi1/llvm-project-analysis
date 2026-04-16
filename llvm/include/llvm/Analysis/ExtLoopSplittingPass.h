#ifndef LLVM_ANALYSIS_EXTLOOPSPLITTINGPASS_H
#define LLVM_ANALYSIS_EXTLOOPSPLITTINGPASS_H

#include "llvm/Analysis/LoopPass.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"

#include <optional>

namespace llvm {

std::optional<uint64_t> extGetTripCount(Loop *L, ScalarEvolution &SE);
uint64_t extEstimateLoopCycles(Loop *L);
bool extTileLoop(Loop *L, LoopInfo &LI, uint64_t tileConstant);

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

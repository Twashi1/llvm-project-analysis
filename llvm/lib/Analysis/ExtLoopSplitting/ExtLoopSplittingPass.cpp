#include "llvm/Analysis/ExtLoopSplittingPass.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/InitializePasses.h"
#include "llvm/Pass.h"
#include "llvm/Support/raw_ostream.h"

#include "llvm/PassRegistry.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "ext-loop-splitting"

namespace llvm {

char ExtLoopSplittingPass::ID = 0;

bool ExtLoopSplittingPass::runOnLoop(Loop *L, LPPassManager &LPM) {
  // TODO: does loop have getName()
  LLVM_DEBUG(dbgs() << "Running ExtLoopSplittingPass on " << L->getName()
                    << "\n");
  // true since we probably modify?
  return true;
}

void ExtLoopSplittingPass::getAnalysisUsage(AnalysisUsage &AU) const {
  LoopPass::getAnalysisUsage(AU);
}
} // namespace llvm

INITIALIZE_PASS(ExtLoopSplittingPass, "ext-loop-splitting",
                "Ext Loop Splitting Pass", false, false)

namespace llvm {
LoopPass *createExtLoopSplittingPass() { return new ExtLoopSplittingPass(); }

#undef DEBUG_TYPE

} // namespace llvm

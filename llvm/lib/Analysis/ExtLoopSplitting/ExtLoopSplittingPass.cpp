#include "llvm/Analysis/ExtLoopSplittingPass.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/PassManager.h"
#include "llvm/InitializePasses.h"
#include "llvm/Pass.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Transforms/Utils/Cloning.h"

#include "llvm/PassRegistry.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "ext-loop-splitting"

namespace llvm {

std::optional<uint64_t> extGetTripCount(Loop *L, ScalarEvolution &SE) {
  const SCEV *BTC = SE.getBackedgeTakenCount(L);
  if (isa<SCEVCouldNotCompute>(BTC))
    return std::nullopt;

  if (auto *C = dyn_cast<SCEVConstant>(BTC)) {
    return C->getAPInt().getZExtValue() + 1;
  }

  return std::nullopt;
}

uint64_t extEstimateLoopCycles(Loop *L) {
  uint64_t cost = 0;
  for (auto *BB : L->blocks()) {
    // Assuming 1 cycle per instruction
    for (auto &I : *BB) {
      cost += 1;
    }
  }
  return cost;
}

bool extTileLoop(Loop *L, LoopInfo *LI, uint64_t tileConstant) {
  BasicBlock *Preheader = L->getLoopPreheader();
  BasicBlock *Header = L->getHeader();
  BasicBlock *Latch = L->getLoopLatch();

  if (!Preheader || !Header || !Latch)
    return false;

  PHINode *IndVar = L->getCanonicalInductionVariable();
  if (!IndVar)
    return false;

  LLVMContext &Ctx = Header->getContext();
  IRBuilder<> PreB(Preheader->getTerminator());

  Type *Ty = IndVar->getType();
  Value *TileSize = ConstantInt::get(Ty, tileConstant);

  // Outer loop blocks
  Function *F = Header->getParent();

  BasicBlock *OuterHeader = BasicBlock::Create(Ctx, "outer.header", F, Header);

  BasicBlock *OuterLatch = BasicBlock::Create(Ctx, "outer.latch", F, Header);

  // Redirect preheader to outer loop
  Preheader->getTerminator()->eraseFromParent();
  IRBuilder<>(Preheader).CreateBr(OuterHeader);

  // Outer IV
  IRBuilder<> OB(OuterHeader);
  PHINode *OuterIV = OB.CreatePHI(Ty, 2);

  OuterIV->addIncoming(ConstantInt::get(Ty, 0), Preheader);

  // Inner loop starts from OuterIV
  IndVar->setIncomingValueForBlock(Preheader, OuterIV);

  // Inner increment
  IRBuilder<> LB(Latch->getTerminator());
  Value *InnerNext = LB.CreateAdd(IndVar, ConstantInt::get(Ty, 1));
  IndVar->setIncomingValueForBlock(Latch, InnerNext);

  // Outer increment
  IRBuilder<> OLB(OuterLatch);
  Value *OuterNext = OLB.CreateAdd(OuterIV, TileSize);
  OuterIV->addIncoming(OuterNext, OuterLatch);

  OLB.CreateBr(OuterHeader);

  // Fix latch jump
  Latch->getTerminator()->eraseFromParent();
  IRBuilder<>(Latch).CreateBr(OuterLatch);

  return true;
}

char ExtLoopSplittingPass::ID = 0;

bool ExtLoopSplittingPass::runOnLoop(Loop *L, LPPassManager &LPM) {
  LLVM_DEBUG(dbgs() << "[LoopSplittingPass] Running on loop: ");

  BasicBlock *Header = L->getHeader();

  if (Header && Header->hasName()) {
    LLVM_DEBUG(dbgs() << Header->getName());
  } else {
    LLVM_DEBUG(dbgs() << "<unnamed>");
  }

  LLVM_DEBUG(dbgs() << "\n");

  auto &LI = getAnalysis<LoopInfoWrapperPass>().getLoopInfo();
  auto &SE = getAnalysis<ScalarEvolutionWrapperPass>().getSE();

  uint64_t cost = extEstimateLoopCycles(L);
  auto trip = extGetTripCount(L, SE);

  // If missing cost/trip count just exit
  if (cost == 0 || !trip) {
    return false;
  }

  uint64_t totalCost = cost * (*trip);

  // use 2 million cycles as our threshold
  if (totalCost < 2'000'000)
    return false;

  uint64_t closestExponent = 0;
  uint64_t closestDistance =
      std::abs(static_cast<int64_t>(totalCost) - 1'000'000);

  for (uint64_t i = 1; i <= 10; i++) {
    uint64_t tileConstant = 1 << i;
    uint64_t innerTile = totalCost / tileConstant;
    uint64_t distance = std::abs(1'000'000 - static_cast<int64_t>(innerTile));

    if (distance < closestDistance) {
      closestDistance = distance;
      closestExponent = i;
    }
  }

  if (closestExponent == 0) {
    return false;
  }

  return extTileLoop(L, &LI, 1 << closestExponent);
}

void ExtLoopSplittingPass::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<LoopInfoWrapperPass>();
  AU.addRequired<ScalarEvolutionWrapperPass>();
  AU.setPreservesCFG();
  LoopPass::getAnalysisUsage(AU);
}
} // namespace llvm

INITIALIZE_PASS(ExtLoopSplittingPass, "ext-loop-splitting",
                "Ext Loop Splitting Pass", false, false)

namespace llvm {
LoopPass *createExtLoopSplittingPass() { return new ExtLoopSplittingPass(); }

#undef DEBUG_TYPE

} // namespace llvm

#include "llvm/CodeGen/RegisterAccessPostRAPass.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/InitializePasses.h"
#include "llvm/Pass.h"
#include "llvm/Support/raw_ostream.h"

#include "llvm/PassRegistry.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "reg-access-postra"

namespace llvm {

char RegisterAccessPostRAPass::ID = 0;

bool RegisterAccessPostRAPass::runOnMachineFunction(MachineFunction &MF) {
  LLVM_DEBUG(dbgs() << "Running RegisterAccessPostRAPass on " << MF.getName()
                    << "\n");

  /*
  auto &MBFI = getAnalysis<MachineBlockFrequencyInfoWrapperPass>().getMBFI();
  */

  for (auto &MBB : MF) {
    // uint64_t Freq = MBFI.getBlockFreq(&MBB).getFrequency();

    unsigned NumLoads = 0;
    unsigned NumStores = 0;
    unsigned NumReads = 0;
    unsigned NumWrites = 0;
    // TODO: num spills/reloads from frame index operands

    for (auto &MI : MBB) {
      // skip pseudo or debug instructions
      if (MI.isDebugInstr() || MI.isPseudo())
        continue;

      // number of stores/loads, so modelling cache hopefully
      if (MI.mayLoad())
        ++NumLoads;
      if (MI.mayStore())
        ++NumStores;

      if (MI.getOpcode() == TargetOpcode::STACKMAP ||
          MI.getOpcode() == TargetOpcode::PATCHPOINT)
        continue;

      // iterate operands to count spills/reloads
      for (unsigned i = 0; i < MI.getNumOperands(); ++i) {
        const MachineOperand &MO = MI.getOperand(i);

        // if not a register
        if (!MO.isReg())
          continue;

        if (MO.isUse())
          ++NumReads;
        if (MO.isDef())
          ++NumWrites;
      }
    }
  }
  // no modification
  return false;
}

} // namespace llvm

INITIALIZE_PASS_BEGIN(RegisterAccessPostRAPass, "reg-access-postra",
                      "Register Access PostRA Pass", false, false)
// TODO: does this do anything
INITIALIZE_PASS_DEPENDENCY(MachineBlockFrequencyInfoWrapperPass)
INITIALIZE_PASS_END(RegisterAccessPostRAPass, "reg-access-postra",
                    "Register Access PostRA Pass", false, false)

namespace llvm {
FunctionPass *createRegisterAccessPostRAPass() {
  return new RegisterAccessPostRAPass();
}

#undef DEBUG_TYPE

} // namespace llvm

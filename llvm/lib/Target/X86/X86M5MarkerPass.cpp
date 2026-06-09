#include "X86M5MarkerPass.h"

#include "X86InstrInfo.h"
#include "X86Subtarget.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetPassConfig.h"

#include "llvm/IR/DebugLoc.h"

#define DEBUG_TYPE "x86-m5-marker"

namespace llvm {
char X86M5MarkerPass::ID = 0;

X86M5MarkerPass::X86M5MarkerPass() : MachineFunctionPass(ID) {}

bool X86M5MarkerPass::runOnMachineFunction(MachineFunction &MF) {
  const TargetInstrInfo *TII = MF.getSubtarget<X86Subtarget>().getInstrInfo();

  MachineRegisterInfo &MRI = MF.getRegInfo();

  for (MachineBasicBlock &MBB : MF) {
    LLVM_DEBUG(dbgs() << "  [X86M5Marker] MBB #" << MBB.getNumber() << "\n");
    auto InsertPt = MBB.getFirstNonPHI();

    if (InsertPt == MBB.end()) {
      continue;
    }

    DebugLoc DL;

    Register IdVReg = MRI.createVirtualRegister(&X86::GR32RegClass);
    MachineInstr *LoadMI =
        BuildMI(MBB, InsertPt, DL, TII->get(X86::MOV32ri), IdVReg).addImm(1);
    auto LoadMIIterator = LoadMI->getIterator();
    BuildMI(MBB, std::next(LoadMIIterator), DL, TII->get(X86::M5_MARKER))
        .addReg(IdVReg);
  }

  return true;
}

StringRef X86M5MarkerPass::getPassName() const { return "X86M5MarkerPass"; }

// static RegisterPass<X86M5MarkerPass>
//     X("x86-m5-marker",
//       "Insert m5_work_begin_addr after first non-PHI instruction", false,
//       false);

MachineFunctionPass *createX86M5MarkerPass() { return new X86M5MarkerPass(); }
} // namespace llvm

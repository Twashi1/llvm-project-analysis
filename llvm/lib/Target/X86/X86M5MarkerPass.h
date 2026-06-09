#pragma once

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

class X86M5MarkerPass : public MachineFunctionPass {
public:
  static char ID;
  X86M5MarkerPass();

  bool runOnMachineFunction(MachineFunction &MF) override;
  StringRef getPassName() const override;
};

MachineFunctionPass *createX86M5MarkerPass();

} // namespace llvm

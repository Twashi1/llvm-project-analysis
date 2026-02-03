#ifndef LLVM_CODEGEN_REGISTERACCESSPOSTRAPASS_H
#define LLVM_CODEGEN_REGISTERACCESSPOSTRAPASS_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

class RegisterAccessPostRAPass : public MachineFunctionPass {
public:
  static char ID;
  RegisterAccessPostRAPass() : MachineFunctionPass(ID) {}

  bool runOnMachineFunction(MachineFunction &MF) override;
  StringRef getPassName() const override {
    return "Register Access PostRA Pass";
  }
};

// Factory function
FunctionPass *createRegisterAccessPostRAPass();

} // namespace llvm

#endif // LLVM_CODEGEN_REGISTERACCESSPOSTRAPASS_H

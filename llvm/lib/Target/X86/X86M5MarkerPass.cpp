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

#include <fstream>
#include <string>
#include <unordered_map>

#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"

#define DEBUG_TYPE "x86-m5-marker"

namespace llvm {
char X86M5MarkerPass::ID = 0;

X86M5MarkerPass::X86M5MarkerPass() : MachineFunctionPass(ID) {}

bool X86M5MarkerPass::runOnMachineFunction(MachineFunction &MF) {
  const TargetInstrInfo *TII = MF.getSubtarget<X86Subtarget>().getInstrInfo();

  MachineRegisterInfo &MRI = MF.getRegInfo();

  // TODO: should just open this file once, not per-machine function, but
  // probably insignificant cost anyway
  std::ifstream InputDVS("DVSInsertionData.csv");
  if (!InputDVS.is_open()) {
    errs() << "Failed to open DVS insertion data, skipping inserting DVS "
              "calling points\n";
    return false;
  }

  // TODO: can optimise to use string views/ranges instead
  std::unordered_map<int, uint32_t> BlockIDToDVS;
  std::string Line;
  // Note block numbers are unique to the machine function, not global
  while (std::getline(InputDVS, Line)) {
    // TODO: explicitly deal with the first row, which should be column titles
    StringRef S(Line);
    SmallVector<StringRef, 4> Fields;
    S.split(Fields, ',');

    if (Fields.size() != 3) {
      errs() << "Improper CSV format, expected 3 fields for DVS insertion "
                "data, skipping row\n";
      continue;
    }

    StringRef MFName = Fields[0];
    StringRef BlockIDStr = Fields[1];
    StringRef BlockVFLevelStr = Fields[2];

    int BlockID;
    int BlockVFLevel;

    if (BlockIDStr.getAsInteger(10, BlockID)) {
      errs()
          << "Failed to convert BlockID (Field index 1) to integer, field was:"
          << BlockIDStr << "\n";

      continue;
    }

    if (BlockVFLevelStr.getAsInteger(10, BlockVFLevel)) {
      errs() << "Failed to convert BlockVFLevel (Field index 2) to integer\n";

      continue;
    }

    if (MFName != MF.getName()) {
      continue;
    }

    BlockIDToDVS.insert({BlockID, static_cast<uint32_t>(BlockVFLevel)});
  }

  LLVM_DEBUG(dbgs() << "[X86M5Marker] MachineFunction: " << MF.getName()
                    << "\n");

  // Debug printing
  for (auto It = BlockIDToDVS.begin(); It != BlockIDToDVS.end(); ++It) {
    int BlockID = It->first;
    uint32_t BlockVF = It->second;

    LLVM_DEBUG(dbgs() << "[X86M5Marker] BlockID: " << BlockID
                      << ", BlockVF: " << BlockVF << "\n");
  }

  for (MachineBasicBlock &MBB : MF) {
    LLVM_DEBUG(dbgs() << "  [X86M5Marker] MBB #" << MBB.getNumber() << "\n");
    auto InsertPt = MBB.getFirstNonPHI();

    if (InsertPt == MBB.end()) {
      continue;
    }

    // Find the correct voltage level based on block ID
    int LocalBlockID = MBB.getNumber();

    // Skip if we don't have the voltage level
    if (BlockIDToDVS.find(LocalBlockID) == BlockIDToDVS.end()) {
      continue;
    }

    int BlockVFLevel = BlockIDToDVS[LocalBlockID];
    // 7 voltage levels, performance level should be
    // 0-6, with 0 indicating highest voltage, and 6 lowest voltage
    // note in config 0 is lowest voltage; so we have to do a reverse
    int PerformanceLevel = 7 - BlockVFLevel - 1;

    DebugLoc DL;

    LLVM_DEBUG(dbgs() << "[X86M5Marker] Inserting performance level "
                      << PerformanceLevel << " into block name "
                      << MBB.getName() << "\n");

    Register IdVReg = MRI.createVirtualRegister(&X86::GR32RegClass);
    MachineInstr *LoadMI =
        BuildMI(MBB, InsertPt, DL, TII->get(X86::MOV32ri), IdVReg)
            .addImm(PerformanceLevel);
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

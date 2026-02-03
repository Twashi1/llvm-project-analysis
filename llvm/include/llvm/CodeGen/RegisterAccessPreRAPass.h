#ifndef LLVM_CODEGEN_REGISTERACCESSPRERAPASS_H
#define LLVM_CODEGEN_REGISTERACCESSPRERAPASS_H

#include "llvm/CodeGen/MachineFunctionPass.h"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace llvm {

struct ExtBBStats {
  double Cycles;
  double Freq;
  double GlobalFreq;
  double Loads;
  double Stores;
  double Spills;
  double Reloads;
  double Reads;
  double Writes;
  double InstrCount;
  double IntInstrCount;
  double FloatInstrCount;
  double BranchInstrCount;
  double LoadStoreInstrCount;
  double FunctionCalls;
  double ContextSwitches;
  double MulAccess;
  double FPAccess;
  double IntALUAccess;
  double IntRegfileReads;
  double FloatRegfileReads;
  double IntRegfileWrites;
  double FloatRegfileWrites;
  std::string Name;
  std::string FunctionName;
  std::string ModuleName;
};

struct ExtBlockEdgeData {
  double Probability;
  unsigned BlockIDStart;
  std::string FunctionStart;
  unsigned BlockIDEnd;
  std::string FunctionEnd;
  bool IsFunctionEdge; // Signifies if this edge is between two different
                       // functions
};

bool extIsProbablyFloatingInstruction(const MachineInstr &MI,
                                      const TargetInstrInfo *TII);
bool extIsProbablyIntegerInstruction(const MachineInstr &MI,
                                     const TargetInstrInfo *TII);
bool extIsProbablyIntReg(StringRef R);
bool extIsProbablyFloatReg(StringRef R);
bool extIsProbablyIALU(StringRef N);
bool extIsProbablyFPU(StringRef N);
bool extIsProbablyMUL(StringRef N);
bool extIsProbablyCall(StringRef N);
bool extIsProbablyReturn(StringRef N);
std::vector<ExtBBStats> extProfileToBBStats(StringRef fileName);

// use functions in conjunction, they produce more headers/values than BB itself
std::stringstream extOutputBBStats(const ExtBBStats &values);
std::string extBBHeaders();

struct ExtFunctionMetadata {
  // FunctionIDs of successors
  std::vector<unsigned> Successors;
  // TODO: terrible structure, just make it match up with successors
  std::vector<std::pair<unsigned, unsigned>> CallerBlockToFunctionID;
  unsigned EntryBasicBlock;
  std::string FunctionName;
};

struct ExtPathCollector {
  std::unordered_map<std::string, unsigned> FunctionIDs;
  std::unordered_map<uint64_t, unsigned> BlockIDs;
  // NOTE: map instead of unordered map because no hash function for pair by
  // default and I don't want to implement one
  std::map<std::pair<unsigned, unsigned>, ExtBlockEdgeData> BlockEdgeData;
  std::vector<ExtBBStats> BlockStats;
  std::vector<ExtFunctionMetadata> FunctionMetadata;
  std::vector<std::vector<unsigned>> GlobalAdjacencyList;

  unsigned BlockIDCount = 0;
  unsigned FunctionIDCount = 0;

  // critical path stuff
  // mapping from block IDs to component
  std::vector<int> CompIDs;
  // for each component, the list of blocks in that component
  std::vector<std::vector<unsigned>> BlocksInComp;
  // topologically sorted components
  std::vector<int> TopoSortedComp;
  // the weight of each component (cycles * global freq)
  std::vector<double> CompWeight;
  // adjacency list of SCCs
  std::vector<std::vector<int>> DAGAdjacency;
  // TODO: rename in order
  // critical path of components
  std::vector<int> CriticalPathComps;

  // list of block ids of disjoint subgraphs
  // TODO: consider AoS approach instead for per-path data
  std::vector<std::vector<unsigned>> DisjointSubgraphBlocks;
  std::vector<std::vector<unsigned>> PotentialStartBlocks;
  std::vector<std::vector<unsigned>> PotentialExitBlocks;

  void addMachineFunctionEdge(const std::string &Caller,
                              unsigned LocalCallerBlock,
                              const std::string &Callee);
  void addMachineBlockEdgeLocal(const std::string &FunctionName,
                                unsigned LocalParent, unsigned LocalSuccessor,
                                double Probability);
  unsigned registerFunction(const std::string &FunctionName);
  unsigned registerBasicBlock(const std::string &FunctionName,
                              unsigned LocalBlockID);
  uint64_t getUniqueBlockIdentifier(const std::string &FunctionName,
                                    unsigned LocalBlockID);
  ExtBBStats &getBBStats(const std::string &FunctionName,
                         unsigned LocalBlockID);
  // TODO: note its a much more likely access pattern that you would change the
  // FunctionMetadata vector while preserving a reference to function metadata
  // thus we must have this getter/setter stuff
  ExtFunctionMetadata getFunctionMetadata(const std::string &FunctionName);
  void setFunctionMetadata(const ExtFunctionMetadata &FunctionMetadata,
                           const std::string &FunctionName);
  void buildCriticalPath();
  void outputCriticalPath();
};

class RegisterAccessPreRAPass : public MachineFunctionPass {
public:
  static char ID;
  static unsigned Processed;
  static unsigned Total;
  static ExtPathCollector PC;
  // TODO: think this is for old BBCounts, but never used, so can remove?
  static std::mutex MapLock;
  RegisterAccessPreRAPass() : MachineFunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const override;
  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override {
    return "Register Access Pre-RA Pass";
  }
};

FunctionPass *createRegisterAccessPreRAPass();

} // namespace llvm

#endif // LLVM_CODEGEN_REGISTERACCESSPRERAPASS_H

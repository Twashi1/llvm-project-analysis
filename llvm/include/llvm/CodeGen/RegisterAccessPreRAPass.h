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

  int LocalBlockNumber;
};

enum class ExtHotSpotUnitName : uint32_t {
  __FIRST = 0,
  L2_LEFT = __FIRST,
  L2,
  L2_RIGHT,
  ICACHE,
  DCACHE,
  BPRED_0,
  BPRED_1,
  BPRED_2,
  DTB_0,
  DTB_1,
  DTB_2,
  FPADD_0,
  FPADD_1,
  FPREG_0,
  FPREG_1,
  FPREG_2,
  FPREG_3,
  FPMUL_0,
  FPMUL_1,
  FPMAP_0,
  FPMAP_1,
  INTMAP,
  INTQ,
  INTREG_0,
  INTREG_1,
  INTEXEC,
  FPQ,
  LDSTQ,
  ITB_0,
  ITB_1,
  __LAST,
};

// TODO: add in L2 cache
// TODO: add in bounds __FIRST and __LAST for iteration
enum class ExtMcPATUnitName : uint32_t {
  PROCESSOR = 0,
  CORE,
  INSTRUCTION_FETCH_UNIT,
  INSTRUCTION_CACHE,
  BRANCH_TARGET_BUFFER,
  BRANCH_PREDICTOR,
  INSTRUCTION_BUFFER,
  INSTRUCTION_DECODER,
  RENAMING_UNIT,
  INT_FRONT_END_RAT,
  FP_FRONT_END_RAT,
  FREE_LIST,
  INT_RETIRE_RAT,
  FP_RETIRE_RAT,
  FP_FREE_LIST,
  LOAD_STORE_UNIT,
  DATA_CACHE,
  LOADQ,
  STOREQ,
  MEMORY_MANAGEMENT_UNIT,
  ITLB,
  DTLB,
  EXECUTION_UNIT,
  REGISTER_FILES,
  INTEGER_RF,
  FLOATING_POINT_RF,
  INSTRUCTION_SCHEDULER,
  INSTRUCTION_WINDOW,
  FP_INSTRUCTION_WINDOW,
  ROB,
  INTEGER_ALU,
  FLOATING_POINT_UNIT,
  RESULTS_BROADCAST_BUS,
  UNDIFFERENTIATED_CORE // Must be max
};

struct ExtMcPATUnit {
  float AreaMetresSquared;
  float PeakDynamic;
  float RuntimeDynamic;
  float SubthresholdLeakage;
  float GateLeakage;
};

struct ExtMcPATOutput {
  int NodeSize;
  int BlockID; // TODO: rename; not always BlockID, sometimes a SCC or even
               // combination of SCCs (subgraph)
  float Voltage;
  float ClockRateHz;

  // TODO: just use a vector
  std::unordered_map<ExtMcPATUnitName, ExtMcPATUnit> UnitMap;
};

struct ExtHotSpotFlpUnit {
  // all units in metres
  float Left;
  float Bottom;
  float Width;
  float Height;
  float Area;
};

struct ExtHotSpotFloorplan {
  std::vector<ExtHotSpotFlpUnit> Units;
};

struct ExtMcPatInput {
  float TempKelvin;
  int NodeSize;
  float Voltage;
  float ClockRateHz;
  int BlockID;

  int CycleCount;
  int BusyCycles;
  int IdleCycles;
  int InstrCount;
  int IntInstrCount;
  int FloatInstrCount;
  int BranchInstrCount;
  int BranchMispredictions;
  int Loads;
  int Stores;

  int ROBReads;
  int ROBWrites;
  int RenameReads;
  int RenameWrites;
  int FpRenameReads;
  int FpRenameWrites;
  int InstWindowReads;
  int InstWindowWrites;
  int InstWindowWakeupAccesses;
  int FpInstWindowReads;
  int FpInstWindowWrites;
  int FpInstWindowWakeupAccesses;
  int IntRegfileReads;
  int IntRegfileWrites;
  int FloatRegfileReads;
  int FloatRegfileWrites;
  int FunctionCalls;
  int ContextSwitches;
  int IAluAccess;
  int FpuAccess;
  int MulAccess;
  int CdbALUAccess;
  int CdbFpAccess;
  int CdbMulAccess;
  int ItlbAccess;
  int ItlbReads; // TODO: [rename] icache reads
  int DtlbAccess;
  int DtlbReads; // TODO: [rename] dcache reads/writes
  int DtlbWrites;
  int BtbReads;
  int BtbWrites;
};

// TODO: read/write from cfg file
struct ExtConfigData {
  bool UseResiduals = false;
  int HeatsinkOffset = 0;
  // Assumes both are given in increasing order
  std::vector<float> FrequenciesGHz = {3.0f, 3.1f, 3.2f, 3.3f,
                                       3.4f, 3.5f, 3.6f};
  std::vector<float> Voltages = {0.6f, 0.65f, 0.7f, 0.75f, 0.8f, 0.85f, 0.9f};
  float BaselineVoltage = 0.8f;
  float BaselineFrequencyGHz = 3.0f;
  float VoltageAllowedError = 0.02f;
};

// TODO: for each component that shows up in the floorplan (and some of the
// extras), what is their temperature
// TODO: instead of a ton of variables, convert to a map of enums to values,
// easier to write code for
// All units kelvin
struct ExtHotSpotTempUnit {
  float Unit;
  float Hsp;
  float Iface;
  float Hsink;
};

struct ExtHotSpotTempInit {
  std::vector<ExtHotSpotTempUnit> Units;
  std::array<float, 12> Inode; // Temps of Inode
};

struct ExtHotSpotTempTrace {
  std::vector<std::vector<float>> Temps;
};

// TODO: mapping from McPATOutputData to ExtHotSpotPowerInput
// TODO: writing this to a file, running hotspot, and parsing the output
// TODO: in the step above, also need to read, modify, and write the config file
struct ExtHotSpotPowerInput {
  // Total timespan; will be divied by number of samples
  float Timespan;
  int NumSamples;

  std::vector<float> UnitPower;
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

struct ExtFinalGraphAnalysisContext {
  std::vector<ExtMcPatInput> McPatInputs;
  std::vector<ExtMcPATOutput> McPatOutputs;
  std::vector<ExtBBStats> Blocks;
  std::map<std::pair<unsigned, unsigned>, ExtBlockEdgeData> BlockEdgeData;
  std::vector<std::vector<unsigned>>
      SubgraphToBlocks; // Map from subgraph ID to blocks
  std::vector<std::vector<unsigned>>
      GlobalAdjacencyList; // Should be the adjacency list of subgraphs
  std::vector<std::vector<unsigned>> DisjointSubgraphBlocks;
  // TODO: do we need subgraph roots/internal end blocks? - internal end blocks
  // required for identification of DVS calling points
};

struct ExtVFPair {
  float FrequencyHz;
  float Voltage;
};

float teiGetVoltage(float TempKelvin, float FrequencyHz);
float teiVoltageToDiscreteLevel(float Voltage, ExtConfigData const &Config);
std::vector<ExtVFPair> teiGetSFVVCandidates(float TempKelvin,
                                            ExtConfigData const &Config);
std::vector<ExtVFPair> teiGetVFVVCandidates(float TempKelvin,
                                            ExtConfigData const &Config);

std::vector<std::string> splitString(std::string const &Str,
                                     std::string const &Delimiters);
bool stringStartsWith(std::string const &Str, std::string const &Prefix);
char const *hotSpotUnitNameToString(ExtHotSpotUnitName Name);
ExtHotSpotUnitName hotSpotStringToUnitName(std::string Name);

// TODO: prefix with McPAT to distinguish against hotspot name conversion
char const *unitNameToString(ExtMcPATUnitName const Name);
void createMcPATInputFile(char const *FileName, ExtMcPatInput const &Input);
ExtMcPATOutput readMcPATOutput(char const *FileName);
std::string programNameToMcPATFile(std::string ProgramName,
                                   ExtMcPatInput const &Input);
ExtMcPATOutput runMcPAT(std::string ProgramName, ExtMcPatInput const &Input);
float getPowerMcPAT(ExtMcPATOutput const &Output, ExtMcPATUnitName Name);
float getAreaMcPAT(ExtMcPATOutput const &Output, ExtMcPATUnitName Name);
// Expects stats have already been multiplied out by Frequency
ExtMcPatInput blockStatsToMcPAT(int Id, float Voltage, float ClockRateHz,
                                int NodeSize,
                                std::vector<ExtBBStats> const &BlockStats);
ExtHotSpotFloorplan readHotSpotFloorplan(char const *FileName);
ExtHotSpotTempTrace readHotSpotTempTrace(char const *FileName);
// TODO: take config for heatsink offset?
void writeHotSpotTempInit(float InitialTemperature, char const *FileName);
void writeHotSpotTempInit(ExtHotSpotTempTrace PreviousTrace,
                          char const *FileName);
ExtHotSpotTempTrace
aggregateTracesAverage(std::vector<ExtHotSpotTempTrace> const &Traces);
// TODO: function that aggregates traces and outputs a single one based on
// something
ExtHotSpotTempInit readHotSpotTempInit(char const *FileName);
void writeHotSpotPowerTrace(ExtHotSpotPowerInput Power, char const *FileName);

ExtHotSpotPowerInput
mapMcPATPowerToHotspotPower(ExtMcPATOutput const &McPatPower,
                            ExtHotSpotFloorplan const &HotSpotFlp,
                            ExtConfigData const &Config);

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
  std::vector<std::vector<unsigned>> SubgraphInternalEndBlocks;
  std::vector<int> SubgraphRoots;

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
  RegisterAccessPreRAPass() : MachineFunctionPass(ID) {}

  void getAnalysisUsage(AnalysisUsage &AU) const override;
  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override {
    return "Register Access Pre-RA Pass";
  }
};

// TODO: we don't register this properly with the pass manager, and so we can't
// actually properly choose when the pass does/doesn't run
FunctionPass *createRegisterAccessPreRAPass();

} // namespace llvm

#endif // LLVM_CODEGEN_REGISTERACCESSPRERAPASS_H

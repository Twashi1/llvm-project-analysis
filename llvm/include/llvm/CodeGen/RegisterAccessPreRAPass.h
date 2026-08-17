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

// TODO: if we find the time in future, we use these for easier to understand
// semantics
typedef unsigned ExtSubgraphID;
typedef unsigned ExtComponentID;
typedef unsigned ExtBlockID;

struct ExtVFPair {
  float FrequencyHz;
  float Voltage;
};

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
  L2_CACHE,
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
  int HeatsinkOffset = 0;
  // Assumes both are given in increasing order
  std::vector<float> FrequenciesGHz = {};
  std::vector<float> Voltages = {};

  // We expect a one-to-one correspondence between these vectors
  // We test baselines consisting only of
  // BaselineFrequenciesGHz[i], BaselineVoltages[i], i == i, i: 1..n
  std::vector<float> BaselineFrequenciesGHz = {};
  std::vector<float> BaselineVoltages = {};

  float VoltageAllowedError = 0.02f;
  float InitialTemperatureCelsius = 77.0f;
  int NodeSize = 14;

  float ColdVoltageAdjustment = 0.0f;
  bool AdjustVoltageWhenCold = false;
  float ColdTemperatureKelvin = 0.0f;
  float MaximumTemperatureKelvin = 355.0f;

  bool UseCachedPowerOutputs = false;
  bool VaryFrequency = false;

  uint32_t NumSamplesHotSpot = 10;

  int TransitionLatencyNs = 50;
};

// Just the properties we need to overwrite
struct ExtHotSpotConfig {
  float TimePerSample = 0.0f;
  float ClockFreqHz = 0.0f;
};

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
  // All units kelvin
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

struct ExtOutputStats {
  float Energy;
  float Time;
  float Power;
  float EnergyDelayProduct;
  float IPS;
  float Instructions;
  float FloatInstructions;
  float IntInstructions;
  float Cycles;
  float Frequency;
  float Voltage;
  float TimeWeightedTemp;
  float PeakTemp;
  float DVSTransitions;
  float TransitionCost;

  ExtBlockID BlockID;
};

struct ExtFinalAnalysisContext {
  // We take as input
  // - the list of topologically sorted subgraphs
  // - for each candidate VF level
  //  - we take the stats of each subgraph and create McPATInput
  //  - we record the output for each VF level
  //  - we compute optimal EDP
  //  - we find every child subgraph from a global adjacency list on the
  //  subgraphs
  //  - we find the heat data (assumed recorded), take an aggregate
  //  - we run hotspot, and record output heat data
  //  - we loop if output heat data too hot (or if temperature violation found)
  // - we finalise with the start, peak, and final temperature, vf configuration
  // of each subgraph
  //  - we output the required information to identify every start block, and
  //  every internal end block
  //  - we can then either insert the required vf changes into those blocks, or
  //  insert additional header/ender blocks, find blocks that post/pre-dominate
  //  etc.

  // TODO: one thing we can consider doing: instead of summing stats per
  // component/block, then running through our pipeline
  // - we instead simulate the CFG within a subgraph (as best as we can), and
  //    get readings per-block of power, temperature, etc.
  // - allows us to maybe get a view on the peak temperature any individual
  //    block reaches, and to see if temperature ever dips below the point where
  //    the current VF level can be sustained

  // From this; required information
  // - adjacency list on subgraphs
  // - (approximately) topologically sorted subgraphs
  // - mapping from subgraph -> sum of block stats
  // - mapping from subgraph -> list of blocks
  // - mapping from subgraph -> list of entry blocks
  // - mapping from subgraph -> list of internal exit blocks
  // - mapping from VF level + subgraph id -> McPAT input
  // - mapping from VF level + subgraph id -> McPAT output
  // - mapping from VF level + subgraph id -> HotSpot output
  std::vector<std::vector<ExtSubgraphID>> SubgraphAdjacencyList;
  std::vector<ExtSubgraphID> TopoSortedSubgraphs;
  std::vector<ExtBBStats> SubgraphStats; // Summed stats per subgraph
  std::vector<ExtBBStats> BlockStats;    // Stats per block
  std::vector<std::vector<ExtBlockID>>
      SubgraphBlocks; // Blocks in each subgraph
  std::vector<std::vector<ExtBlockID>> SubgraphEntryBlocks;
  std::vector<std::vector<ExtBlockID>> SubgraphInternalExitBlocks;
  std::vector<std::vector<ExtMcPATOutput>>
      SubgraphMcPATOutput; // Indexable by ExtSubgraphID, gives list of outputs
                           // for the various tested VF levels
  std::vector<std::vector<ExtHotSpotTempTrace>>
      SubgraphHotSpotOutput; // Indexable by ExtSubgraphID, gives list of
                             // outputs for various tested VF levels
  std::vector<std::optional<ExtHotSpotTempTrace>>
      SubgraphHotSpotFinalTemp; // The final temperature of a subgraph given
                                // whatever VF level was selected
  std::vector<std::optional<ExtVFPair>> SubgraphBestVFPair;
  std::vector<std::optional<ExtOutputStats>> SubgraphOutputStats;
};

struct ExtBlockDVSInformation {
  ExtOutputStats OutputStats;
  ExtBBStats BlockStats;

  int PerformanceLevel;
  float Voltage;
  float Frequency;
};

std::string pathDVSInsertionData(int BaselineIndex);
std::string pathEfficiencyStats(int BaselineIndex);
std::string pathSubgraphStats(int BaselineIndex);

ExtConfigData readConfigData(char const *FileName);
void performFullAnalysis(ExtFinalAnalysisContext &Context,
                         ExtConfigData const &Config, int BaselineIndex,
                         bool ForceBaselineRun);
void writeAllOutputStats(ExtFinalAnalysisContext const &Context,
                         std::string FileName);
void evaluatePerformanceAndOutput(ExtFinalAnalysisContext const &ETCRun,
                                  ExtFinalAnalysisContext const &BaselineRun,
                                  ExtConfigData const &ConfigData,
                                  std::string FileName);
int getVoltageIndex(ExtConfigData const &Config, float Voltage);
std::string cleanModuleName(char const *ModuleName);
std::vector<ExtBlockDVSInformation>
getBlockDVSInformation(ExtFinalAnalysisContext const &Context);
void writeDVSInformation(std::vector<ExtBlockDVSInformation> const &Information,
                         ExtConfigData const &Config, std::string FileName);
ExtOutputStats calculateOutputStats(ExtMcPATOutput const &McPAT,
                                    ExtHotSpotTempTrace const &TempTrace,
                                    ExtHotSpotFloorplan const &Floorplan,
                                    ExtConfigData const &Config,
                                    ExtBBStats const &Stats);
// Takes time-weighted averages of Frequency, Voltage, Power, IPS
ExtOutputStats
combineOutputStats(std::vector<ExtOutputStats> const &OutputStats);

float teiGetVoltage(float TempKelvin, float FrequencyHz);
float teiVoltageToDiscreteLevel(float Voltage, ExtConfigData const &Config);
std::vector<ExtVFPair> teiGetCandidates(float TempKelvin,
                                        ExtConfigData const &Config,
                                        int BaselineIndex);

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
ExtMcPATOutput runMcPAT(std::string ProgramName, ExtMcPatInput const &Input,
                        ExtConfigData const &Config);
float getPowerMcPAT(ExtMcPATOutput const &Output, ExtMcPATUnitName Name);
float getAreaMcPAT(ExtMcPATOutput const &Output, ExtMcPATUnitName Name);
// Expects stats have already been multiplied out by Frequency
ExtMcPatInput blockStatsToMcPAT(int Id, float Voltage, float ClockRateHz,
                                int NodeSize,
                                std::vector<ExtBBStats> const &BlockStats);
ExtHotSpotFloorplan readHotSpotFloorplan(char const *FileName);
ExtHotSpotTempTrace readHotSpotTempTrace(char const *FileName);
ExtHotSpotTempTrace initDefaultHotSpotTrace(float AssumedTemperatureKelvin);
ExtHotSpotTempTrace runHotSpot(std::string ProgramName,
                               ExtMcPATOutput const &Power,
                               ExtBBStats const &Stats,
                               ExtHotSpotTempTrace const &InitialTrace,
                               ExtConfigData const &Config);
// TODO: take config for heatsink offset?
void writeHotSpotTempInit(ExtHotSpotTempTrace PreviousTrace,
                          char const *FileName);
// TODO: nice if we had ExtHotSpotTempTrace, but then ExtHotSpotTempReading,
// where a reading is a single trace line, for reference, this would return
// ExtHotSpotTempReading
ExtHotSpotTempTrace
aggregateTracesAverage(std::vector<ExtHotSpotTempTrace> const &Traces);

ExtHotSpotTempTrace
aggregateTracesHottest(std::vector<ExtHotSpotTempTrace> const &Traces);

// TODO: function that aggregates traces and outputs a single one based on
// something
float areaWeightedCoreTemp(ExtHotSpotTempTrace const &TempTrace,
                           ExtHotSpotFloorplan const &Flp);
float peakTemp(ExtHotSpotTempTrace const &TempTrace);
ExtHotSpotTempInit readHotSpotTempInit(char const *FileName);
void writeHotSpotPowerTrace(ExtHotSpotPowerInput const &Power,
                            ExtConfigData const &Config, char const *FileName);

ExtHotSpotPowerInput
mapMcPATPowerToHotspotPower(ExtMcPATOutput const &McPatPower,
                            ExtHotSpotFloorplan const &HotSpotFlp,
                            ExtConfigData const &Config);

void editHotSpotConfig(ExtHotSpotConfig const &Config, char const *FileName);

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
  // TODO: unused?
  // critical path of components
  std::vector<int> CriticalPathComps;

  // list of block ids of disjoint subgraphs
  // TODO: consider AoS approach instead for per-path data
  std::vector<std::vector<unsigned>> DisjointSubgraphBlocks;
  std::vector<std::vector<unsigned>> PotentialStartBlocks;
  std::vector<std::vector<unsigned>> PotentialExitBlocks;
  std::vector<std::vector<int>>
      SCCsInSubgraph; // List of components in each subgraph
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

ExtFinalAnalysisContext createAnalysisContext(ExtPathCollector const &PC);

} // namespace llvm

#endif // LLVM_CODEGEN_REGISTERACCESSPRERAPASS_H

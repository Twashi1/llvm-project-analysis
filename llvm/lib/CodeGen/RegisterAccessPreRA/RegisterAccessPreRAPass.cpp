#include "llvm/CodeGen/RegisterAccessPreRAPass.h"
#include "llvm/CodeGen/MachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineBranchProbabilityInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetSchedule.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Module.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/ErrorOr.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/raw_ostream.h"

#include "llvm/PassRegistry.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/raw_ostream.h"
// #include "llvm/XRay/xray_interface.h"

#include <algorithm>
#include <deque>
#include <iterator>
#include <stack>
#include <unordered_map>
#include <unordered_set>
// TODO: use numeric limits instead
#include <cfloat>

#define DEBUG_TYPE "reg-access-prera"

using namespace llvm;

namespace llvm {
char RegisterAccessPreRAPass::ID = 0;
unsigned RegisterAccessPreRAPass::Processed = 0;
unsigned RegisterAccessPreRAPass::Total = 0;
ExtPathCollector RegisterAccessPreRAPass::PC = {};
std::mutex RegisterAccessPreRAPass::MapLock;

std::stringstream extOutputBBStats(const ExtBBStats &values,
                                   unsigned UniqueBlockID) {
  std::stringstream ss;

  ss << values.ModuleName << "," << values.FunctionName << "," << values.Name
     << "," << UniqueBlockID << "," << values.Cycles << "," << values.Freq
     << "," << values.GlobalFreq << "," << values.Loads << "," << values.Stores
     << "," << values.Spills << "," << values.Reloads << "," << values.Reads
     << "," << values.Writes << "," << values.InstrCount << ","
     << values.IntInstrCount << "," << values.FloatInstrCount << ","
     << values.BranchInstrCount << "," << values.LoadStoreInstrCount << ","
     << values.FunctionCalls << "," << values.ContextSwitches << ","
     << values.MulAccess << "," << values.FPAccess << "," << values.IntALUAccess
     << "," << values.IntRegfileReads << "," << values.IntRegfileWrites << ","
     << values.FloatRegfileReads << "," << values.FloatRegfileWrites;

  return ss;
}

std::string extBBHeaders() {
  const char *headers =
      "module_name,function_name,block_name,block_id,cycle_count,freq,global_"
      "freq,loads,"
      "stores,spills,"
      "reloads,reads,writes,instr_count,int_instr_count,float_instr_count,"
      "branch_instr_count,load_store_instr_count,function_calls,context_"
      "switches,mul_access,fp_access,ialu_access,int_regfile_reads,int_"
      "regfile_writes,float_regfile_reads,float_regfile_writes";

  return std::string(headers);
}

void ExtPathCollector::buildCriticalPath() {
  std::error_code EC;
  raw_fd_ostream OutFile("reg_stats.csv", EC, sys::fs::OF_Append);

  if (EC) {
    errs() << "Error opening file: " << EC.message() << "\n";
    return;
  }

  LLVM_DEBUG(dbgs() << "Finalising global adjacency list\n");

  // Build global adjacency list
  // - for each basic block, we need its personal list of successors
  // - for each machine function, we have its basic block, and all machine
  //    functions it links to
  // Need to add the machine functions into this global adjacency list
  for (const ExtFunctionMetadata &Metadata : FunctionMetadata) {
    for (unsigned i = 0; i < Metadata.Successors.size(); i++) {
      unsigned SuccessorFunctionID = Metadata.Successors[i];

      // Connection from our entry block, to the successor function's entry
      // block
      auto SuccessorData = FunctionMetadata[SuccessorFunctionID];
      unsigned CallerBlock = Metadata.CallerBlockToFunctionID[i].first;

      // indicates that this is some external function that we didn't run our MF
      // pass on
      if (SuccessorData.EntryBasicBlock == UINT32_MAX) {
        continue;
      }

      // TODO: need to verify all connections are unique!!!
      GlobalAdjacencyList[CallerBlock].push_back(
          FunctionMetadata[SuccessorFunctionID].EntryBasicBlock);

      // Construct edge data
      ExtBlockEdgeData FunctionEdgeData;
      // TODO: is this necessarily true? probably depends on some comparison
      // result
      FunctionEdgeData.Probability = 1.0;
      FunctionEdgeData.BlockIDStart = CallerBlock;
      FunctionEdgeData.FunctionStart = Metadata.FunctionName;
      FunctionEdgeData.BlockIDEnd = SuccessorData.EntryBasicBlock;
      FunctionEdgeData.FunctionStart = SuccessorData.FunctionName;
      FunctionEdgeData.IsFunctionEdge = true;

      BlockEdgeData[std::pair<unsigned, unsigned>(
          CallerBlock, SuccessorData.EntryBasicBlock)] = FunctionEdgeData;
    }
  }

  LLVM_DEBUG(dbgs() << "Finalised adjacency list\n");

  // test global adjacency list that all numbers make sense
  unsigned MaxIDSeen = 0;

  for (unsigned BlockID = 0; BlockID < GlobalAdjacencyList.size(); BlockID++) {
    MaxIDSeen = std::max(MaxIDSeen, BlockID);

    if (BlockID >= BlockIDCount) {
      LLVM_DEBUG(dbgs() << "Block ID " << BlockID
                        << " is larger than expected maximum " << BlockIDCount
                        << "\n");
    }

    std::vector<unsigned> &Successors = GlobalAdjacencyList[BlockID];

    // Ensure list of successors is unique
    std::unordered_set<int> SeenSuccessors;

    // Preserve original order (not necessary afaik), while removing duplicates
    auto it = Successors.begin();
    while (it != Successors.end()) {
      if (!SeenSuccessors.insert(*it).second) {
        it = Successors.erase(it);
      } else {
        ++it;
      }
    }

    for (unsigned ChildID : Successors) {
      MaxIDSeen = std::max(MaxIDSeen, ChildID);

      if (ChildID >= BlockIDCount) {
        LLVM_DEBUG(dbgs() << "Child ID " << ChildID
                          << " is larger than expected maximum " << BlockIDCount
                          << "\n");
      }
    }
  }

  LLVM_DEBUG(dbgs() << "Expected total " << BlockIDCount << " and maximum ID "
                    << MaxIDSeen << "\n");

  // condense strongly connected components into one large node
  // using Tarjan's articulation points algorithm
  // all to build a DAG
  unsigned N = BlockIDCount;
  std::vector<int> Index(N, -1);
  std::vector<int> LowLink(N, -1);
  std::vector<int> OnStack(N, 0);
  std::stack<unsigned> S;

  // component IDs
  CompIDs = std::vector<int>(N, -1);
  unsigned IndexTarjan = 0;
  int CompCount = 0;

  // TODO: refactor to standalone function
  // can't use auto because of recursion
  std::function<void(unsigned)> StronglyConnect = [&](unsigned v) {
    Index[v] = IndexTarjan;
    LowLink[v] = IndexTarjan;
    IndexTarjan++;

    S.push(v);
    OnStack[v] = 1;

    for (unsigned w : GlobalAdjacencyList[v]) {
      if (Index[w] == -1) {
        StronglyConnect(w);
        LowLink[v] = std::min(LowLink[v], LowLink[w]);
      } else if (OnStack[w]) {
        LowLink[v] = std::min(LowLink[v], Index[w]);
      }
    }

    if (LowLink[v] == Index[v]) {
      // start new component
      // this condition shouldn't really matter
      //  but just for sanity, I don't want a while true
      while (!S.empty()) {
        unsigned w = S.top();
        S.pop();

        OnStack[w] = 0;
        CompIDs[w] = CompCount;

        if (w == v) {
          break;
        }
      }

      CompCount++;
    }
  };

  LLVM_DEBUG(dbgs() << "About to create SCCs\n");

  // create SCCs
  for (unsigned v = 0; v < N; v++) {
    if (Index[v] == -1) {
      StronglyConnect(v);
    }
  }

  LLVM_DEBUG(dbgs() << "Created SCCs, now computing costs\n");

  // build DAG from SCCs
  std::vector<double> CompCost(CompCount, 0.0);
  CompWeight = std::vector<double>(CompCount, 0.0);
  std::vector<double> CompMinFrequency(CompCount, FLT_MAX);

  for (unsigned v = 0; v < N; v++) {
    int c = CompIDs[v];
    // TODO: cost is not used anywhere
    CompCost[c] += BlockStats[v].Cycles;
    // Note both cases we're using GlobalFreq, we want this to be irrespective
    // of the function's call frequency
    CompWeight[c] += BlockStats[v].Cycles * BlockStats[v].GlobalFreq;
    CompMinFrequency[c] =
        std::min(CompMinFrequency[c], BlockStats[v].GlobalFreq);
  }

  LLVM_DEBUG(dbgs() << "Computed costs, now finding DAG adjacency\n");

  DAGAdjacency = std::vector<std::vector<int>>(CompCount);
  // duplicate detection, two vertices packed
  std::unordered_set<uint64_t> DAGEdges;

  for (unsigned u = 0; u < N; u++) {
    for (unsigned v : GlobalAdjacencyList[u]) {
      int cu = CompIDs[u];
      int cv = CompIDs[v];

      if (cu != cv) {
        // maybe order them, but uv not same as vu?
        uint64_t key =
            (static_cast<uint64_t>(cu) << 32) | static_cast<uint32_t>(cv);

        // no contains in c++17 :(
        if (!DAGEdges.count(key)) {
          DAGAdjacency[cu].push_back(cv);
          DAGEdges.insert(key);
        }
      }
    }
  }

  LLVM_DEBUG(dbgs() << "Found DAG adjacency, now Topological sort\n");

  // topological sort (Kahn's algorithm)
  // InDegree can be thought as number of unfulfilled dependencies
  std::vector<int> InDegree(CompCount, 0);

  for (int u = 0; u < CompCount; u++) {
    for (int v : DAGAdjacency[u]) {
      InDegree[v]++;
    }
  }

  TopoSortedComp.reserve(CompCount);
  std::deque<int> q;

  // get all starting nodes, ones with no dependencies
  for (int i = 0; i < CompCount; i++) {
    if (InDegree[i] == 0) {
      q.push_back(i);
    }
  }

  while (!q.empty()) {
    int u = q.front();
    q.pop_front();

    TopoSortedComp.push_back(u);

    for (int v : DAGAdjacency[u]) {
      // if all dependencies fulfilled, we can schedule it
      if (--InDegree[v] == 0) {
        q.push_back(v);
      }
    }
  }

  LLVM_DEBUG(
      dbgs() << "Topological sort finished, now finding critical path\n");

  // TODO: -DOUBLE_MAX? i mean good enough anyway
  const double NEG_INF = -FLT_MAX;
  std::vector<int> InSubgraph = std::vector<int>(CompCount, 0);
  std::vector<int> IsStartNode = std::vector<int>(CompCount, 0);

  // Sort by frequency
  // we want start nodes to have low frequency; applying DVS less
  std::vector<int> ComponentsByFrequency = TopoSortedComp;
  std::sort(ComponentsByFrequency.begin(), ComponentsByFrequency.end(),
            [&](const auto &a, const auto &b) {
              return CompMinFrequency[a] < CompMinFrequency[b];
            });

  LLVM_DEBUG(dbgs() << "Components by frequency: [");

  for (int i = 0; i < ComponentsByFrequency.size(); i++) {
    if (i > 0) {
      LLVM_DEBUG(dbgs() << ",");
    }

    LLVM_DEBUG(dbgs() << ComponentsByFrequency[i]);
  }

  LLVM_DEBUG(dbgs() << "]\n");

  const double SUBGRAPH_THRESHOLD = 1e6;

  // Find accumulated weight
  // - Get reverse topological sort (so leaves -> root)
  std::vector<int> ReverseTopo;
  ReverseTopo.reserve(CompCount);

  for (int j = TopoSortedComp.size() - 1; j >= 0; j--) {
    int c = TopoSortedComp[j];

    ReverseTopo.push_back(TopoSortedComp[j]);
  }

  // TODO: we could re-calculate accumulated weight based on what's in the
  // subgraph
  //  but considering that sometimes we go out of the subgraph, maybe its better
  //  to calculate this only here
  // Works because reverse topo
  std::vector<double> AccumWeight = std::vector<double>(CompCount, 0.0);
  for (int i = 0; i < ReverseTopo.size(); i++) {
    int SCC = ReverseTopo[i];
    // Note should be safe, since reverse topo
    AccumWeight[SCC] += CompWeight[SCC];

    for (int Successor : DAGAdjacency[SCC]) {
      AccumWeight[SCC] += AccumWeight[Successor];
    }
  }

  // With accumulated weight and components by frequency
  // - start at lowest frequency not in subgraph
  // - get the successor with lowest accumulated weight above the threshold
  //    - add successor, repeat until subtree is below threshold
  // - else get the successor with maximum accumulated weight
  //    - add entire sub-tree
  std::vector<std::vector<int>> AllSubgraphs;
  std::vector<std::vector<int>> SCCsInSubgraph;
  std::vector<int> SubgraphRoots;
  std::vector<std::vector<int>> SubgraphLeaves;

  // While all nodes are not in a sub-graph
  while (std::any_of(InSubgraph.begin(), InSubgraph.end(),
                     [](int i) { return i == 0; })) {
    // Select the first non-consumed node in ComponentsByFrequency
    int StartSCC = 0;

    for (int i = 0; i < ComponentsByFrequency.size(); i++) {
      if (InSubgraph[i]) {
        continue;
      }

      StartSCC = i;
      break;
    }

    // Now, perform DFS from this component, stopping when we reach threshold
    double CurrentWeight = 0.0;
    double TargetWeight = SUBGRAPH_THRESHOLD;

    std::vector<int> SCCStack = {StartSCC};
    std::vector<int> Predecessors = std::vector<int>(CompCount, -1);
    std::vector<int> SCCs;
    std::vector<int> Leaves;

    SubgraphRoots.push_back(StartSCC);

    while (!SCCStack.empty()) {
      // Get successors
      int Current = SCCStack.back();
      SCCStack.pop_back();
      InSubgraph[Current] = 1;
      SCCs.push_back(Current);

      CurrentWeight += CompWeight[Current];

      if (CurrentWeight >= SUBGRAPH_THRESHOLD) {
        Leaves.push_back(Current);

        break;
      }

      // Of the successors, we go down the maximum path which isn't in the
      // subgraph
      int BestSuccessor = -1;
      double BestSuccessorWeight = 0.0;

      for (int Successor : DAGAdjacency[Current]) {
        if (InSubgraph[Successor])
          continue;

        double Weight = AccumWeight[Successor];

        if (Weight < BestSuccessorWeight)
          continue;

        BestSuccessor = Successor;
        BestSuccessorWeight = Weight;
      }

      if (BestSuccessor == -1) {
        Leaves.push_back(Current);

        continue;
      }

      // We add this successor
      SCCStack.push_back(BestSuccessor);
      Predecessors[BestSuccessor] = Current;
    }

    AllSubgraphs.push_back(Predecessors);
    SCCsInSubgraph.push_back(SCCs);
    SubgraphLeaves.push_back(Leaves);
  }

  LLVM_DEBUG(dbgs() << "Split DAG into subgraphs\n");

  // We could express subgraph as graph of basic blocks, but we don't need this
  // - instead, just get a list of all basic blocks in the subgraph, while
  // notating the start/end block(s)
  // - if we want to implement this, we just need all the blocks where we do an
  // API call for scaling
  // - if some subgraph leads into another subgraph, an API call might be needed
  // - but detecting this doesn't require the subgraph itself, just a list of
  // basic blocks and the global graph
  //    - if one basic block in subgraph A has an edge to a basic block in
  //    subgraph B, just the list is sufficient to detect (as well as the graph)

  // Get the list of basic blocks in each subgraph
  std::vector<std::vector<unsigned>> SubgraphMBBList;
  std::vector<std::vector<unsigned>> SCCToMBBList =
      std::vector<std::vector<unsigned>>(CompCount);

  // Create reverse mapping
  for (int i = 0; i < N; i++) {
    int SCC = CompIDs[i];

    SCCToMBBList[SCC].push_back(i);
  }

  for (int i = 0; i < SCCsInSubgraph.size(); i++) {
    const std::vector<int> &SCCs = SCCsInSubgraph[i];

    // TODO: enforce uniqueness
    std::vector<unsigned> SubgraphBlocksSet;
    std::vector<unsigned> StartBlocks;
    std::vector<unsigned> EndBlocks;
    // Index by SCC
    std::vector<uint8_t> IsSCCInSubgraph = std::vector<uint8_t>(CompCount, 0);
    int StartSCC = SubgraphRoots[i];

    for (int SCC : SCCs) {
      IsSCCInSubgraph[SCC] = 1;
    }

    // TODO: really expensive and tedious, will not scale well for large
    //  graphs, can be made many times asymptotically faster

    // Iterate every basic block
    //  if that basic block leads into our SCC
    //  find the basic block within our SCC that has been lead into
    //  mark that basic block as an entry block
    //
    //  if that basic block is in our SCC
    //  if that basic block leads to an SCC outside the subgraph
    //  then mark the basic block it has lead to, as the exit block
    for (unsigned BlockID = 0; BlockID < N; BlockID++) {
      int BlockSCC = CompIDs[BlockID];

      for (unsigned Successor : GlobalAdjacencyList[BlockID]) {
        int SCCOfSuccessor = CompIDs[Successor];

        if (IsSCCInSubgraph[BlockSCC] && !IsSCCInSubgraph[SCCOfSuccessor]) {
          // We are in the subgraph, but our child is not
          // thus we're an exit block
          // We can insert an instruction at the end of the exit block
          // but inserting instructions at the end of a block is non-trivial
          // TODO: consider inserting instruction at end of block whenever
          // possible So instead, we have to consider every child and add an
          // exit at that child
          EndBlocks.push_back(Successor);
        }

        if (!IsSCCInSubgraph[BlockSCC] && SCCOfSuccessor == StartSCC) {
          // There is some outside block, that links into our starter component
          // thus Successor block is a start point
          StartBlocks.push_back(Successor);
        }
      }
    }

    for (int SCC : SCCs) {
      std::vector<unsigned> MBBs = SCCToMBBList[SCC];

      SubgraphBlocksSet.insert(SubgraphBlocksSet.end(), MBBs.begin(),
                               MBBs.end());
    }

    SubgraphMBBList.push_back(SubgraphBlocksSet);
    PotentialStartBlocks.push_back(StartBlocks);
    PotentialExitBlocks.push_back(EndBlocks);
  }

  DisjointSubgraphBlocks = SubgraphMBBList;
}

std::vector<ExtBBStats> extProfileToBBStats(StringRef fileName) {
  std::vector<ExtBBStats> results;

  ErrorOr<std::unique_ptr<MemoryBuffer>> BufferOrErr =
      MemoryBuffer::getFile(fileName);

  if (!BufferOrErr) {
    errs()
        << "Failed to open file with profiling data. Not created yet? Error: "
        << BufferOrErr.getError().message() << "\n";
    return results;
  }

  MemoryBuffer &Buffer = **BufferOrErr;
  StringRef Content = Buffer.getBuffer();

  std::vector<std::vector<std::string>> CSVMatrix;

  while (!Content.empty()) {
    StringRef Line;
    std::tie(Line, Content) = Content.split("\n");
    Line = Line.rtrim("\r\n");

    std::vector<std::string> Fields;

    while (!Line.empty()) {
      StringRef Field;
      std::tie(Field, Line) = Line.split(",");
      Fields.push_back(Field.str());
    }

    CSVMatrix.push_back(std::move(Fields));
  }

  // With CSV matrix, need to parse
  // expected columsn
  // file, function_name, block_number, count
  std::vector<std::string> const &ColumnNames = CSVMatrix[0];

  LLVM_DEBUG(dbgs() << "Got columns of profdata.csv as: ");

  for (uint64_t i = 0; i < ColumnNames.size(); i++) {
    if (i > 0) {
      LLVM_DEBUG(dbgs() << ", ");
    }

    LLVM_DEBUG(dbgs() << ColumnNames[i]);
  }

  for (int i = 1; i < CSVMatrix.size(); i++) {
    std::vector<std::string> Row = CSVMatrix[i];

    // TODO: the file name is not the same as the module name
    // module name is something akin to objects/gemm.ll, file name is gemm.c
    std::string FileName = Row[0];
    std::string FunctionName = Row[1];
    // TODO: this block name is actualy a block number, and we don't have a
    // great mapping
    std::string BlockName = Row[2];
    std::string CycleCount = Row[3];

    int CycleCountInt = std::stoi(CycleCount);

    ExtBBStats ProfStats;
    // TODO: bad mapping! not correct!
    ProfStats.ModuleName = FileName;
    ProfStats.FunctionName = FunctionName;
    // TODO: bad mapping! not correct!
    ProfStats.Name = BlockName;
    ProfStats.Cycles = CycleCountInt;

    results.push_back(ProfStats);
  }

  return results;
}

void ExtPathCollector::outputCriticalPath() {
  // TODO: function is poorly named, there are multiple critical paths
  // TODO: not even critical paths anymore, DVS calling points instead
  std::error_code EC;
  raw_fd_ostream OutFile("CritPath.csv", EC, sys::fs::OF_Append);

  if (EC) {
    errs() << "Error opening file: " << EC.message() << "\n";
    return;
  }

  std::error_code EC2;

  // TODO: output the component of each node in the CFG too
  raw_fd_ostream OutCFGFile("CFG.csv", EC2, sys::fs::OF_Append);

  if (EC2) {
    errs() << "Error opening file: " << EC2.message() << "\n";
    return;
  }

  std::error_code EC_DAG;
  std::error_code EC_TopoComp;
  std::error_code EC_BlockAdditional;
  std::error_code EC_MBBStats;

  raw_fd_ostream OutDAGFile("DAG.csv", EC_DAG, sys::fs::OF_Append);

  if (EC_DAG) {
    errs() << "Error opening file: " << EC_DAG.message() << "\n";
    return;
  }

  raw_fd_ostream OutTopoComp("TopoComp.csv", EC_TopoComp, sys::fs::OF_Append);

  if (EC_TopoComp) {
    errs() << "Error opening file: " << EC_TopoComp.message() << "\n";
    return;
  }

  raw_fd_ostream OutBlockAdditional("PerBlockAdditional.csv",
                                    EC_BlockAdditional, sys::fs::OF_Append);

  if (EC_BlockAdditional) {
    errs() << "Error opening file: " << EC_BlockAdditional.message() << "\n";
    return;
  }

  raw_fd_ostream OutMBB("MBB_stats.csv", EC_MBBStats, sys::fs::OF_Append);

  if (EC_MBBStats) {
    errs() << "Error opening file: " << EC_MBBStats.message() << "\n";
    return;
  }

  // TODO: required CFG data
  //    1. we want to output the full DAG
  //    2. the list of basic block IDs for every component in DAG
  //    3. the full adjacency list between blocks, not just DAGs
  //      - should contain branch probability info (obtained by EdgeData)
  //    4. PathBlocks.csv should contain the same block IDs

  // For some given path
  //   we want to be able to re-construct the full tree of this path
  //   associate each node with the mcpat output files
  //   associate each edge with branch probability info

  // 1. add block IDs to PathBlocks.csv
  // 2. create CFG.csv data format as follows
  // start_function_name,start_block_name,start_block_id,exit_function_name,exit_block_name,exit_block_id,branch_prob,start_path_index,end_path_index,is_start_entry
  OutCFGFile
      << "module_name,start_function_name,start_block_name,start_block_id,exit_"
         "function_"
         "name,exit_block_name,exit_block_id,branch_prob,start_path_index,end_"
         "path_index,is_start_entry\n";

  OutDAGFile << "module_name,start_comp,end_comp\n";
  OutTopoComp << "module_name,comp_id,comp_priority\n";
  OutBlockAdditional << "module_name,block_id,comp_id,execution_cycles\n";
  OutMBB << extBBHeaders().c_str() << "\n";

  std::error_code EC3;

  raw_fd_ostream BlockOutFile("PathBlocks.csv", EC3, sys::fs::OF_Append);

  if (EC3) {
    errs() << "Error opening file: " << EC3.message() << "\n";
    return;
  }

  // TODO: this is unused
  // NOTE: both freq and global_freq lose meaning when summed across blocks
  OutFile
      << "module_name,function_name,start_block,end_func,end_block,cycle_count,"
         "writes,reads,loads,stores,instr_count,freq,global_freq,int_instr_"
         "count,float_instr_count,branch_instr_count,loadstore_instr,function_"
         "calls,context_switches,mul_access,fp_access,ialu_access,int_regfile_"
         "reads,float_regfile_reads,int_regfile_writes,float_regfile_writes\n";

  BlockOutFile
      << "module_name,path_index,function_name,block_name,is_entry,is_exit,"
         "cycle_count,writes,"
         "reads,loads,stores,instr_count,global_freq,freq,int_instr_count,"
         "float_"
         "instr_count,"
         "branch_instr_count,loadstore_instr,function_calls,context_switches,"
         "mul_"
         "access,"
         "fp_access,ialu_access,int_regfile_reads,float_regfile_reads,int_"
         "regfile_writes,float_regfile_writes,block_id\n";

  // TODO: need to output the full MBB list somewhere
  // TODO: need to ensure we apply DVS to the correct block,
  // start_func/start_block pair should
  //  be this block, but not sure if its guaranteed currently
  //  since we just take first block in the first index SCC, not entry blocks of
  //  the SCC
  // TODO: also it's first indexed SCC, we likely need more information than the
  // list of blocks
  //
  // TODO: for each block in each subgraph, we want to associate the path that
  // block belongs to, with the block, we can additionally flag if a block
  // belongs to multiple paths

  std::vector<int> PathIndexOfBlock = std::vector<int>(BlockIDCount, -1);
  std::vector<int> MapIsEntryBlock = std::vector<int>(BlockIDCount, 0);

  for (int i = 0; i < DisjointSubgraphBlocks.size(); i++) {
    const std::vector<unsigned> &MBBSubgraph = DisjointSubgraphBlocks[i];
    const std::vector<unsigned> &StartBlocks = PotentialStartBlocks[i];
    // NOTE: we have the exit blocks for a particular subgraph
    //  but where do we print them?
    const std::vector<unsigned> &EndBlocks = PotentialExitBlocks[i];

    std::set<unsigned> StartSet =
        std::set<unsigned>(StartBlocks.begin(), StartBlocks.end());
    std::set<unsigned> EndSet =
        std::set<unsigned>(EndBlocks.begin(), EndBlocks.end());

    // Note these really aren't in any particular order
    // only first block really counts
    // TODO: is there a guarantee the first block of the first SCC is actually
    // the one we should be attaching the DVS to?
    // TODO: these used to be here, I think signifying first function or first
    // SCC... but lost what they originally meant
    bool IsFirst = (i == 0);
    bool IsLast = (i == DisjointSubgraphBlocks.size() - 1);

    double Cycles = 0.0;
    double Writes = 0.0;
    double Reads = 0.0;
    double Loads = 0.0;
    double Stores = 0.0;
    double Instrs = 0.0;
    double Freq = 0.0;
    double GlobalFreq = 0.0;
    double TotalTime = 0.0;
    double IntInstrs = 0.0;
    double FloatInstrs = 0.0;
    double BranchInstrs = 0.0;
    double LoadStoreInstrs = 0.0;
    double FunctionCalls = 0.0;
    double ContextSwitches = 0.0;
    double MulAccess = 0.0;
    double FPAccess = 0.0;
    double IntALUAccess = 0.0;
    double IntRegfileReads = 0.0;
    double FloatRegfileReads = 0.0;
    double IntRegfileWrites = 0.0;
    double FloatRegfileWrites = 0.0;
    unsigned StartBlock = UINT32_MAX;
    unsigned EndBlock = UINT32_MAX;
    std::string StartBlockName = "";
    std::string EndBlockName = "";
    std::string StartBlockFunc = "";
    std::string EndBlockFunc = "";
    std::string ModuleName = "";

    // TODO: instead of iterating all blocks in the subgraph
    //  iterate all start/exit blocks and just print those
    //  some of those blocks won't technically be in the subgraph, since they'll
    //  be the exit blocks
    // TODO: some exit blocks of subgraph A might be start/exit blocks of
    // subgraph B
    //  we will need duplicate entries...
    for (int j = 0; j < MBBSubgraph.size(); j++) {
      bool IsEntryBlock = false;
      bool IsExitBlock = false;

      bool IsFirstBlock = (j == 0);
      bool IsLastBlock = (j == MBBSubgraph.size() - 1);

      unsigned Block = MBBSubgraph[j];

      ExtBBStats BlockStat = BlockStats[Block];

      PathIndexOfBlock[Block] = i;

      LLVM_DEBUG(dbgs() << "Block " << BlockStat.Name << " from function "
                        << BlockStat.FunctionName
                        << ", is being parsed in path index: " << i << "\n");

      // TODO: just make these into one-liners
      if (StartSet.find(Block) != StartSet.end()) {
        IsEntryBlock = true;
      }

      if (EndSet.find(Block) != EndSet.end()) {
        IsExitBlock = true;
      }

      // Could skip printing for these blocks, but we still need to compute
      // stats
      if (!IsEntryBlock && !IsExitBlock) {
      }

      MapIsEntryBlock[Block] = static_cast<int>(IsEntryBlock);

      // Frequency-adjusted stats
      double CyclesFreq = BlockStat.Cycles * BlockStat.Freq;
      double WritesFreq = BlockStat.Writes * BlockStat.Freq;
      double ReadsFreq = BlockStat.Reads * BlockStat.Freq;
      double LoadsFreq = BlockStat.Loads * BlockStat.Freq;
      double StoresFreq = BlockStat.Stores * BlockStat.Freq;
      double InstrsFreq = BlockStat.InstrCount * BlockStat.Freq;
      double IntInstrsFreq = BlockStat.IntInstrCount * BlockStat.Freq;
      double FloatInstrsFreq = BlockStat.FloatInstrCount * BlockStat.Freq;
      double BranchInstrsFreq = BlockStat.BranchInstrCount * BlockStat.Freq;
      double LoadStoreInstrsFreq =
          BlockStat.LoadStoreInstrCount * BlockStat.Freq;
      double FunctionCallsFreq = BlockStat.FunctionCalls * BlockStat.Freq;
      double ContextSwitchesFreq = BlockStat.ContextSwitches * BlockStat.Freq;
      double MulAccessFreq = BlockStat.MulAccess * BlockStat.Freq;
      double FPAccessFreq = BlockStat.FPAccess * BlockStat.Freq;
      double IntALUAccessFreq = BlockStat.IntALUAccess * BlockStat.Freq;
      double IntRegfileReadsFreq = BlockStat.IntRegfileReads * BlockStat.Freq;
      double FloatRegfileReadsFreq =
          BlockStat.FloatRegfileReads * BlockStat.Freq;
      double IntRegfileWritesFreq = BlockStat.IntRegfileWrites * BlockStat.Freq;
      double FloatRegfileWritesFreq =
          BlockStat.FloatRegfileWrites * BlockStat.Freq;

      BlockOutFile << BlockStat.ModuleName << "," << i << ","
                   << BlockStat.FunctionName << "," << BlockStat.Name << ","
                   << IsEntryBlock << "," << IsExitBlock << "," << CyclesFreq
                   << "," << WritesFreq << "," << ReadsFreq << "," << LoadsFreq
                   << "," << StoresFreq << "," << InstrsFreq << ","
                   << BlockStat.GlobalFreq << "," << BlockStat.Freq << ","
                   << IntInstrsFreq << "," << FloatInstrsFreq << ","
                   << BranchInstrsFreq << "," << LoadStoreInstrsFreq << ","
                   << FunctionCallsFreq << "," << ContextSwitchesFreq << ","
                   << MulAccessFreq << "," << FPAccessFreq << ","
                   << IntALUAccessFreq << "," << IntRegfileReadsFreq << ","
                   << FloatRegfileReadsFreq << "," << IntRegfileWritesFreq
                   << "," << FloatRegfileWritesFreq << "," << Block << "\n";

      Cycles += CyclesFreq;
      Writes += WritesFreq;
      Freq += BlockStat.Freq;
      GlobalFreq += BlockStat.GlobalFreq;
      Reads += ReadsFreq;
      // LLVM_DEBUG(dbgs() << "Loads before: " << Loads << ", adding: " <<
      // BlockStat.Loads << ", times " << BlockStat.Freq << ", to get: " <<
      // BlockStat.Loads * BlockStat.Freq << "\n");
      Loads += LoadsFreq;
      Stores += StoresFreq;
      Instrs += InstrsFreq;
      IntInstrs += IntInstrsFreq;
      FloatInstrs += FloatInstrsFreq;
      BranchInstrs += BranchInstrsFreq;
      LoadStoreInstrs += LoadStoreInstrsFreq;
      FunctionCalls += FunctionCallsFreq;
      ContextSwitches += ContextSwitchesFreq;
      MulAccess += MulAccessFreq;
      FPAccess += FPAccessFreq;
      IntALUAccess += IntALUAccessFreq;
      IntRegfileReads += IntRegfileReadsFreq;
      FloatRegfileReads += FloatRegfileReadsFreq;
      IntRegfileWrites += IntRegfileWritesFreq;
      FloatRegfileWrites += FloatRegfileWritesFreq;
      TotalTime += CyclesFreq;

      if (IsFirstBlock) {
        StartBlock = Block;
        StartBlockName = BlockStat.Name;
        StartBlockFunc = BlockStat.FunctionName;
      }

      if (IsLastBlock) {
        EndBlock = Block;
        EndBlockName = BlockStat.Name;
        EndBlockFunc = BlockStat.FunctionName;
      }

      if (ModuleName == "") {
        ModuleName = BlockStat.ModuleName;
      }
    }

    // OPT: can just not print below a million cycles for cleaner output
    if (Cycles < 1e6) {
      // continue;
    }

    OutFile << ModuleName << "," << StartBlockFunc << "," << StartBlockName
            << "," << EndBlockFunc << "," << EndBlockName << "," << Cycles
            << "," << Writes << "," << Reads << "," << Loads << "," << Stores
            << "," << Instrs << "," << Freq << "," << GlobalFreq << ","
            << IntInstrs << "," << FloatInstrs << "," << BranchInstrs << ","
            << LoadStoreInstrs << "," << FunctionCalls << "," << ContextSwitches
            << "," << MulAccess << "," << FPAccess << "," << IntALUAccess << ","
            << IntRegfileReads << "," << FloatRegfileReads << ","
            << IntRegfileWrites << "," << FloatRegfileWrites << "\n";
  }

  // Write full CFG data to CFG.csv
  // start_function_name,start_block_name,start_block_id,exit_function_name,exit_block_name,exit_block_id,branch_prob,start_path_index,end_path_index,is_start_entry
  for (unsigned u = 0; u < GlobalAdjacencyList.size(); u++) {
    std::vector<unsigned> const &Neighbours = GlobalAdjacencyList[u];
    unsigned StartBlock = u;
    ExtBBStats StartStats = BlockStats[StartBlock];

    for (unsigned v = 0; v < Neighbours.size(); v++) {
      unsigned EndBlock = Neighbours[v];
      ExtBBStats EndStats = BlockStats[EndBlock];

      // TODO: get edge data
      std::pair<unsigned, unsigned> EdgePair =
          std::pair<unsigned, unsigned>(StartBlock, EndBlock);

      double EdgeProbability = 0.0;

      // Edge has associated data, so assign probability
      if (BlockEdgeData.count(EdgePair)) {
        ExtBlockEdgeData Edge = BlockEdgeData[EdgePair];
        EdgeProbability = Edge.Probability;
      }

      // print to CFG data in format
      OutCFGFile << StartStats.ModuleName << "," << StartStats.FunctionName
                 << "," << StartStats.Name << "," << StartBlock << ","
                 << EndStats.FunctionName << "," << EndStats.Name << ","
                 << EndBlock << "," << EdgeProbability << ","
                 << PathIndexOfBlock[StartBlock] << ","
                 << PathIndexOfBlock[EndBlock] << ","
                 << MapIsEntryBlock[StartBlock] << "\n";
    }
  }

  // OutDAGFile << "module_name,start_comp,end_comp\n";
  // OutTopoComp << "module_name,comp_id,comp_priority\n";
  // OutBlockAdditional << "module_name,block_id,comp_id\n";

  // TODO: module name is going to be the same across all components, just grab
  // an arbitraty one and precompute it here
  std::string ModuleName = "";

  // Write out the DAG
  for (unsigned u = 0; u < DAGAdjacency.size(); u++) {
    unsigned StartComp = u;

    std::vector<int> const &Neighbours = DAGAdjacency[u];

    for (unsigned BlockID = 0; BlockID < CompIDs.size(); BlockID++) {
      if (CompIDs[BlockID] == StartComp) {
        ModuleName = BlockStats[BlockID].ModuleName;

        break;
      }
    }

    for (unsigned v = 0; v < Neighbours.size(); v++) {
      int EndComp = Neighbours[v];

      OutDAGFile << ModuleName << "," << StartComp << "," << EndComp << "\n";
    }
  }

  // Write out the topologically sorted components
  for (unsigned i = 0; i < TopoSortedComp.size(); i++) {
    unsigned Comp = TopoSortedComp[i];

    for (unsigned BlockID = 0; BlockID < CompIDs.size(); BlockID++) {
      if (CompIDs[BlockID] == Comp) {
        ModuleName = BlockStats[BlockID].ModuleName;

        break;
      }
    }

    OutTopoComp << ModuleName << "," << Comp << "," << i << "\n";
  }

  for (unsigned BlockID = 0; BlockID < CompIDs.size(); BlockID++) {
    unsigned CompID = CompIDs[BlockID];
    ExtBBStats BlockStat = BlockStats[BlockID];

    // TODO: unsure if cycle count already adjusted for frequency, probably not?
    OutBlockAdditional << ModuleName << "," << BlockID << "," << CompID << ","
                       << BlockStat.Cycles * BlockStat.Freq << "\n";

    // TODO: fix all this trash
    ExtBBStats OutputStatsBB;
    OutputStatsBB.Cycles = BlockStat.Cycles * BlockStat.Freq;
    OutputStatsBB.Freq = BlockStat.Freq;
    OutputStatsBB.GlobalFreq = BlockStat.GlobalFreq;
    OutputStatsBB.Loads = BlockStat.Loads * BlockStat.Freq;
    OutputStatsBB.Stores = BlockStat.Stores * BlockStat.Freq;
    OutputStatsBB.Spills = BlockStat.Spills * BlockStat.Freq;
    OutputStatsBB.Reloads = BlockStat.Reloads * BlockStat.Freq;
    OutputStatsBB.Reads = BlockStat.Reads * BlockStat.Freq;
    OutputStatsBB.Writes = BlockStat.Writes * BlockStat.Freq;
    OutputStatsBB.InstrCount = BlockStat.InstrCount * BlockStat.Freq;
    OutputStatsBB.IntInstrCount = BlockStat.IntInstrCount * BlockStat.Freq;
    OutputStatsBB.FloatInstrCount = BlockStat.FloatInstrCount * BlockStat.Freq;
    OutputStatsBB.BranchInstrCount =
        BlockStat.BranchInstrCount * BlockStat.Freq;
    OutputStatsBB.LoadStoreInstrCount =
        BlockStat.LoadStoreInstrCount * BlockStat.Freq;
    OutputStatsBB.FunctionCalls = BlockStat.FunctionCalls * BlockStat.Freq;
    OutputStatsBB.ContextSwitches = BlockStat.ContextSwitches * BlockStat.Freq;
    OutputStatsBB.MulAccess = BlockStat.MulAccess * BlockStat.Freq;
    OutputStatsBB.FPAccess = BlockStat.FPAccess * BlockStat.Freq;
    OutputStatsBB.IntALUAccess = BlockStat.IntALUAccess * BlockStat.Freq;
    OutputStatsBB.IntRegfileReads = BlockStat.IntRegfileReads * BlockStat.Freq;
    OutputStatsBB.FloatRegfileReads =
        BlockStat.FloatRegfileReads * BlockStat.Freq;
    OutputStatsBB.IntRegfileWrites =
        BlockStat.IntRegfileWrites * BlockStat.Freq;
    OutputStatsBB.FloatRegfileWrites =
        BlockStat.FloatRegfileWrites * BlockStat.Freq;
    OutputStatsBB.Name = BlockStat.Name;
    OutputStatsBB.FunctionName = BlockStat.FunctionName;
    OutputStatsBB.ModuleName = BlockStat.ModuleName;

    // TODO: This blockID is the per-basic block one, not the global one?, have
    // to fix?
    OutMBB << extOutputBBStats(OutputStatsBB, BlockID).str().c_str() << "\n";

    LLVM_DEBUG(dbgs() << "Machine basic block ID " << BlockID << ", name "
                      << BlockStat.Name << " had cycles " << BlockStat.Cycles
                      << ", frequency " << BlockStat.Freq
                      << ", loads: " << OutputStatsBB.Loads
                      << ", stores: " << OutputStatsBB.Stores << "\n");
  }

  OutFile.close();
  BlockOutFile.close();
  OutCFGFile.close();
  OutBlockAdditional.close();
  OutTopoComp.close();
  OutDAGFile.close();
  OutMBB.close();
}

void ExtPathCollector::addMachineBlockEdgeLocal(const std::string &FunctionName,
                                                unsigned LocalParent,
                                                unsigned LocalSuccessor,
                                                double Probability) {
  unsigned u = registerBasicBlock(FunctionName, LocalParent);
  unsigned v = registerBasicBlock(FunctionName, LocalSuccessor);

  GlobalAdjacencyList[u].push_back(v);

  ExtBlockEdgeData EdgeData;
  EdgeData.Probability = Probability;
  EdgeData.BlockIDStart = u;
  EdgeData.BlockIDEnd = v;
  EdgeData.FunctionStart = FunctionName;
  EdgeData.FunctionEnd = FunctionName;
  EdgeData.IsFunctionEdge = false;

  BlockEdgeData[std::pair<unsigned, unsigned>(u, v)] = EdgeData;
}

void ExtPathCollector::addMachineFunctionEdge(const std::string &Caller,
                                              unsigned LocalCallerBlock,
                                              const std::string &Callee) {
  registerFunction(Caller);
  registerFunction(Callee);
  unsigned GlobalCallerBlock = registerBasicBlock(Caller, LocalCallerBlock);

  unsigned CallerID = FunctionIDs[Caller];
  unsigned CalleeID = FunctionIDs[Callee];

  FunctionMetadata[CallerID].Successors.push_back(CalleeID);
  FunctionMetadata[CallerID].CallerBlockToFunctionID.push_back(
      std::pair<unsigned, unsigned>(GlobalCallerBlock, CalleeID));
}

unsigned ExtPathCollector::registerFunction(const std::string &FunctionName) {
  if (!FunctionIDs.count(FunctionName)) {
    FunctionIDs[FunctionName] = FunctionIDCount++;
    ExtFunctionMetadata Metadata;
    Metadata.FunctionName = FunctionName;
    Metadata.EntryBasicBlock = UINT32_MAX;
    FunctionMetadata.push_back(Metadata);
  }

  return FunctionIDs[FunctionName];
}

unsigned ExtPathCollector::registerBasicBlock(const std::string &FunctionName,
                                              unsigned LocalBlockID) {
  registerFunction(FunctionName);

  uint64_t BlockUniqueIdentifier =
      getUniqueBlockIdentifier(FunctionName, LocalBlockID);

  if (!BlockIDs.count(BlockUniqueIdentifier)) {
    BlockIDs[BlockUniqueIdentifier] = BlockIDCount++;
    // TODO: push_back can probably auto-call initializer
    ExtBBStats Stats;
    Stats.FunctionName = FunctionName;
    BlockStats.push_back(Stats);
    GlobalAdjacencyList.push_back(std::vector<unsigned>({}));
  }

  return BlockIDs[BlockUniqueIdentifier];
}

void RegisterAccessPreRAPass::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<MachineBranchProbabilityInfoWrapperPass>();
  AU.addRequired<MachineBlockFrequencyInfoWrapperPass>();
  // TODO: might need to add required of children?
  // AU.setPreservesAll();
  MachineFunctionPass::getAnalysisUsage(AU);
}

ExtFunctionMetadata
ExtPathCollector::getFunctionMetadata(const std::string &FunctionName) {
  registerFunction(FunctionName);
  unsigned FunctionID = FunctionIDs[FunctionName];

  return FunctionMetadata[FunctionID];
}
void ExtPathCollector::setFunctionMetadata(
    const ExtFunctionMetadata &FunctionMetadata,
    const std::string &FunctionName) {
  registerFunction(FunctionName);
  unsigned FunctionID = FunctionIDs[FunctionName];

  this->FunctionMetadata[FunctionID] = FunctionMetadata;
}
uint64_t
ExtPathCollector::getUniqueBlockIdentifier(const std::string &FunctionName,
                                           unsigned LocalBlockID) {
  registerFunction(FunctionName);

  unsigned FunctionID = FunctionIDs[FunctionName];
  // misnomer, but can't think of a good name
  uint64_t BlockUniqueIdentifier = (static_cast<uint64_t>(FunctionID) << 32) |
                                   static_cast<uint32_t>(LocalBlockID);

  return BlockUniqueIdentifier;
}

ExtBBStats &ExtPathCollector::getBBStats(const std::string &FunctionName,
                                         unsigned LocalBlockID) {
  registerBasicBlock(FunctionName, LocalBlockID);

  uint64_t BlockUniqueIdentifier =
      getUniqueBlockIdentifier(FunctionName, LocalBlockID);
  unsigned BlockID = BlockIDs[BlockUniqueIdentifier];

  return BlockStats[BlockID];
}

bool extIsProbablyFloatingInstruction(const MachineInstr &MI,
                                      const TargetInstrInfo *TII) {
  // this approach sucks, but I cant access x86 directly (build dependency
  // issues?), so this is best I can do
  const MCInstrDesc &Desc = MI.getDesc();
  StringRef Name = TII->getName(MI.getOpcode());

  // note regular flags don't give us much information, so need to use
  // target-specific
  // TODO: maybe we can check the flag "MayRaiseFPException" (regular flag, not
  // target-specific)
  static const char *FPPrefixes[] = {
      "FADD",   "FSUB",    "FMUL",   "FDIV",    "FSQRT",  "FREM",    "FCHS",
      "FABS",   "ADDSS",   "SUBSS",  "MULSS",   "DIVSS",  "SQRTSS",  "MINSS",
      "MAXSS",  "ADDPS",   "SUBPS",  "MULPS",   "DIVPS",  "SQRTPS",  "MINPS",
      "MAXPS",  "VADDSS",  "VSUBSS", "VMULSS",  "VDIVSS", "VSQRTSS", "VADDPS",
      "VSUBPS", "VMULPS",  "VDIVPS", "VSQRTPS", "VADDPD", "VSUBPD",  "VMULPD",
      "VDIVPD", "VSQRTPD",
  };

  for (const char *Prefix : FPPrefixes)
    if (Name.starts_with(StringRef(Prefix))) {
      return true;
    }

  return false;
}

bool extIsProbablyIntegerInstruction(const MachineInstr &MI,
                                     const TargetInstrInfo *TII) {
  const MCInstrDesc &Desc = MI.getDesc();
  StringRef Name = TII->getName(MI.getOpcode());

  static const char *IntPrefixes[] = {
      "ADD", "SUB", "MUL", "IMUL", "DIV",   "IDIV",  "INC", "DEC",
      "NEG", "AND", "OR",  "XOR",  "NOT",   "SHL",   "SAL", "SHR",
      "SAR", "ROL", "ROR", "MOV",  "MOVSX", "MOVZX", "CMP", "TEST",
      "BSF", "BSR", "BT",  "BTS",  "BTR",   "BTC"};

  for (const char *Prefix : IntPrefixes)
    if (Name.starts_with(StringRef(Prefix))) {
      return true;
    }

  return false;
}

bool extIsProbablyIntReg(StringRef R) {
  static const char *IRegExact[] = {
      "RAX", "RBX", "RCX", "RDX", "RSI", "RDI", "RBP",    "RSP",   "EAX",
      "EBX", "ECX", "EDX", "ESI", "EDI", "EBP", "ESP",    "AX",    "BX",
      "CX",  "DX",  "SI",  "DI",  "SP",  "BP",  "EFLAGS", "RFLAGS"};

  // checking RX registers
  if (R.starts_with("R") && R.size() >= 2 && isdigit(R[1])) {
    return true;
  }

  for (const char *Prefix : IRegExact) {
    if (R.compare(StringRef(Prefix))) {
      return true;
    }
  }

  // 8-bit partial registers
  if (R.size() == 2 && (R[1] == 'L' || R[1] == 'H')) {
    return true;
  }

  return false;
}

bool extIsProbablyFloatReg(StringRef R) {
  static const char *FPRegExact[] = {"XMM", "YMM", "ZMM", "ST"};

  for (const char *Prefix : FPRegExact) {
    if (R.compare(StringRef(Prefix))) {
      return true;
    }
  }

  return false;
}

bool extIsProbablyIALU(StringRef N) {
  static const char *IALUPrefixes[] = {"ADD", "SUB", "INC", "DEC",
                                       "AND", "XOR", "OR",  "SAR",
                                       "SHR", "SHL", "CMP", "TEST"};

  return false;
}

bool extIsProbablyFPU(StringRef N) {
  static const char *FPUPrefixes[] = {
      "ADDSS", "ADDSD", "SUBSS", "SUBSD", "MULSS", "MULSD", "DIVSS",
      "DIVSD", "SQRT",  "FADD",  "FSUB",  "FMUL",  "FDIV"};

  for (const char *Prefix : FPUPrefixes) {
    if (N.starts_with_insensitive(Prefix)) {
      return true;
    }
  }

  return false;
}

bool extIsProbablyMUL(StringRef N) {
  return N.contains_insensitive("MUL") || N.contains_insensitive("DIV");
}

bool extIsProbablyCall(StringRef N) { return N.contains_insensitive("CALL"); }

bool extIsProbablyReturn(StringRef N) {
  return N.starts_with_insensitive("RET");
}

bool RegisterAccessPreRAPass::runOnMachineFunction(MachineFunction &MF) {
  // count total number of functions so we know when we're on the last one
  if (!Total) {
    for (const Function &F : *MF.getFunction().getParent()) {
      if (!F.isDeclaration()) {
        LLVM_DEBUG(dbgs() << "Found machine function name: " << F.getName()
                          << "\n");

        Total++;
      }
    }
  }

  const Module *M = MF.getFunction().getParent();
  StringRef ModuleName = M->getName();

  LLVM_DEBUG(dbgs() << "Running on module: " << ModuleName << "\n");

  LLVM_DEBUG(dbgs() << "Found " << Total << " machine functions\n");

  LLVM_DEBUG(dbgs() << "Running RegisterAccessPreRAPass on " << MF.getName()
                    << "\n");

  const std::string MFName = MF.getName().str();
  bool FunctionHasProfileData = MF.getFunction().hasProfileData();

  LLVM_DEBUG(dbgs() << "Function " << MFName << ", has profile data: "
                    << FunctionHasProfileData << "\n");

  PC.registerFunction(MFName);

  const TargetSubtargetInfo &TSI = MF.getSubtarget();
  const TargetInstrInfo *TII = TSI.getInstrInfo();
  const TargetRegisterInfo *TRI = TSI.getRegisterInfo();
  TargetSchedModel SchedModel;
  SchedModel.init(&TSI);

  auto *MBPIWrapper =
      getAnalysisIfAvailable<MachineBranchProbabilityInfoWrapperPass>();
  MachineBlockFrequencyInfoWrapperPass *MBFIWrapper =
      getAnalysisIfAvailable<MachineBlockFrequencyInfoWrapperPass>();
  MachineBlockFrequencyInfo *MBFI = nullptr;
  MachineBranchProbabilityInfo *MBPI = nullptr;

  if (MBFIWrapper == nullptr) {
    LLVM_DEBUG(dbgs() << "MBFI wrapper is nullptr\n");

    return false;
  } else {
    MBFI = &MBFIWrapper->getMBFI();

    LLVM_DEBUG(dbgs() << "MBFI wrapper is not nullptr\n");
  }

  if (MBPIWrapper == nullptr) {
    LLVM_DEBUG(dbgs() << "MBPI wrapper is nullptr\n");
  } else {
    MBPI = &MBPIWrapper->getMBPI();

    LLVM_DEBUG(dbgs() << "MBPI wrapper is not nullptr\n");
  }

  // assign local ID to each block
  // TODO: is Blocks ever used?
  std::vector<MachineBasicBlock *> Blocks;
  std::unordered_map<MachineBasicBlock *, unsigned> BlockIDs;
  Blocks.reserve(MF.size());

  // TODO: note manually disabled profData for now, we will rely on LLVM
  // correctly using the profdata we passed
  // std::vector<ExtBBStats> profData = extProfileToBBStats("outprof.csv");
  std::vector<ExtBBStats> profData = {};

  unsigned BlockID = 0;
  for (auto &MBB : MF) {
    LLVM_DEBUG(dbgs() << "Collecting info for MBB: " << MBB.getName() << "\n");

    // this is the entry block, record entry block ID for this machine function
    if (BlockID == 0) {
      ExtFunctionMetadata FunctionMetadata = PC.getFunctionMetadata(MFName);
      FunctionMetadata.EntryBasicBlock = PC.registerBasicBlock(MFName, BlockID);
      PC.setFunctionMetadata(FunctionMetadata, MFName);
    }

    Blocks.push_back(&MBB);
    BlockIDs.insert({&MBB, BlockID});

    ExtBBStats &BlockStat = PC.getBBStats(MFName, BlockIDs[&MBB]);
    BlockStat.Cycles = 0.0;
    BlockStat.Freq = 1.0;
    BlockStat.GlobalFreq = 1.0;
    BlockStat.InstrCount = 0.0;
    BlockStat.Loads = 0.0;
    BlockStat.Stores = 0.0;
    BlockStat.Reads = 0.0;
    BlockStat.Writes = 0.0;
    BlockStat.Reloads = 0.0;
    BlockStat.Spills = 0.0;
    BlockStat.IntInstrCount = 0.0;
    BlockStat.FloatInstrCount = 0.0;
    BlockStat.BranchInstrCount = 0.0;
    BlockStat.LoadStoreInstrCount = 0.0;
    BlockStat.FunctionCalls = 0.0;
    BlockStat.ContextSwitches = 0.0;
    BlockStat.MulAccess = 0.0;
    BlockStat.FPAccess = 0.0;
    BlockStat.IntALUAccess = 0.0;
    BlockStat.IntRegfileReads = 0.0;
    BlockStat.FloatRegfileReads = 0.0;
    BlockStat.IntRegfileWrites = 0.0;
    BlockStat.FloatRegfileWrites = 0.0;

    BlockStat.Name = MBB.getName().str();
    BlockStat.FunctionName = MFName;
    BlockStat.ModuleName = ModuleName;

    unsigned UniqueBlockID = PC.getUniqueBlockIdentifier(MFName, BlockID);

    const BasicBlock *BB = MBB.getBasicBlock();
    if (BB != nullptr) {
      LLVM_DEBUG(dbgs() << "Machine Basic Block " << BlockStat.Name
                        << " still had associated BB data, name: "
                        << BB->getName() << "\n");
    }

    for (auto &MI : MBB) {
      if (MI.isDebugInstr() || MI.isPseudo()) {
        continue;
      }

      const MCInstrDesc &Desc = MI.getDesc();
      StringRef Op = TII->getName(MI.getOpcode());

      if (extIsProbablyIALU(Op)) {
        BlockStat.IntALUAccess += 1.0;
      }

      if (extIsProbablyFPU(Op)) {
        BlockStat.FPAccess += 1.0;
      }

      if (extIsProbablyMUL(Op)) {
        BlockStat.MulAccess += 1.0;
      }

      if (extIsProbablyCall(Op)) {
        BlockStat.FunctionCalls += 1.0;
        BlockStat.ContextSwitches += 1.0;
      }

      if (extIsProbablyReturn(Op)) {
        BlockStat.ContextSwitches += 1.0;
      }

      if (Desc.isBranch()) {
        BlockStat.BranchInstrCount += 1.0;
      }

      // float/int
      if (extIsProbablyFloatingInstruction(MI, TII)) {
        BlockStat.FloatInstrCount += 1.0;
      } else if (extIsProbablyIntegerInstruction(MI, TII)) {
        BlockStat.IntInstrCount += 1.0;
      }

      // instruction latency
      unsigned Latency = 1;
      BlockStat.InstrCount += 1.0;

      if (SchedModel.hasInstrSchedModel()) {
        Latency = SchedModel.computeInstrLatency(&MI);
      }

      BlockStat.Cycles += static_cast<double>(Latency);

      // TODO: num spills/reloads from frame index operands
      // number of stores/loads, so modelling cache hopefully
      // if (MI.mayLoad()) {
      // BlockStat.Loads += 1.0;
      // BlockStat.LoadStoreInstrCount += 1.0;
      // }
      //
      // if (MI.mayStore()) {
      // BlockStat.Stores += 1.0;
      // BlockStat.LoadStoreInstrCount += 1.0;
      // }

      for (const MachineMemOperand *MMO : MI.memoperands()) {
        BlockStat.Loads += MMO->isLoad();
        BlockStat.Stores += MMO->isStore();

        if (MMO->isLoad()) {
          const MCInstrDesc &Desc = MI.getDesc();
          StringRef Name = TII->getName(MI.getOpcode());

          // LLVM_DEBUG(dbgs() << "Detected load in MI: " << Name << " for BB "
          // << BlockStat.Name << "\n");
        }

        if (MMO->isStore()) {
          const MCInstrDesc &Desc = MI.getDesc();
          StringRef Name = TII->getName(MI.getOpcode());

          // LLVM_DEBUG(dbgs() << "Detected store in MI: " << Name << " for BB "
          // << BlockStat.Name << "\n");
        }
      }

      for (unsigned i = 0; i < MI.getNumOperands(); i++) {
        const MachineOperand &MO = MI.getOperand(i);

        if (MI.isCall()) {
          const MachineOperand *Callee =
              MI.getOperand(0).isGlobal() ? &MI.getOperand(0) : nullptr;

          if (Callee) {
            const Function *F = dyn_cast<Function>(Callee->getGlobal());

            if (F) {
              std::string CalleeName = F->getName().str();
              std::string OurName = MF.getFunction().getName().str();

              PC.addMachineFunctionEdge(OurName, BlockID, CalleeName);
              // LLVM_DEBUG(dbgs() << "Adding function edge between: " <<
              // OurName << ", and " << CalleeName << "\n");
            }
          }
        }

        // TODO: this might cover the above opcode conditions
        if (!MO.isReg()) {
          continue;
        }

        if (MO.isUse()) {
          BlockStat.Reads += 1.0;
        }

        if (MO.isDef()) {
          BlockStat.Writes += 1.0;
        }

        StringRef R = TRI->getRegAsmName(MO.getReg());

        if (extIsProbablyIntReg(R)) {
          if (MO.isUse()) {
            BlockStat.IntRegfileReads += 1.0;
          } else if (MO.isDef()) {
            BlockStat.IntRegfileWrites += 1.0;
          }
        } else if (extIsProbablyFloatReg(R)) {
          if (MO.isUse()) {
            BlockStat.FloatRegfileReads += 1.0;
          } else if (MO.isDef()) {
            BlockStat.FloatRegfileWrites += 1.0;
          }
        }
      }
    }

    // if info available, get execution frequency
    if (MBFI != nullptr) {
      BlockStat.Freq = MBFI->getBlockFreqRelativeToEntryBlock(&MBB);
      BlockStat.GlobalFreq =
          static_cast<double>(MBFI->getBlockFreq(&MBB).getFrequency());
    }

    BlockStat.Freq = std::max(BlockStat.Freq, 1.0);
    BlockStat.GlobalFreq = std::max(BlockStat.GlobalFreq, 1.0);

    // TODO: don't need this code anymore, we put in the profile count into the
    // code itself
    bool FoundProfileData = false;

    for (int i = 0; i < profData.size(); i++) {
      ExtBBStats ProfileBlockStat = profData[i];
      std::string FileName = BlockStat.ModuleName.substr(
          BlockStat.ModuleName.find_last_of('/') + 1);

      LLVM_DEBUG(dbgs() << "Getting block ID from " << ProfileBlockStat.Name
                        << "\n");
      int ProfileBlockID = std::stoi(ProfileBlockStat.Name);

      bool BlockIndexMatch = ProfileBlockID == BlockID;
      bool FunctionMatch =
          ProfileBlockStat.FunctionName == BlockStat.FunctionName;
      // TODO: module match... but difficult!
      // use some regex pattern

      if (BlockIndexMatch && FunctionMatch) {
        LLVM_DEBUG(dbgs() << "Using profile data for block with name "
                          << BlockStat.Name << ", function "
                          << BlockStat.FunctionName << ", file " << FileName
                          << "\n");
        BlockStat.Cycles = ProfileBlockStat.Cycles;
        FoundProfileData = true;
        break;
      } else {
        LLVM_DEBUG(dbgs() << "Failed to match: Name - " << BlockStat.Name
                          << " :: " << ProfileBlockStat.Name << ", "
                          << BlockStat.FunctionName << " :: "
                          << ProfileBlockStat.FunctionName << ", " << FileName
                          << " :: " << ProfileBlockStat.FunctionName << "\n");
      }
    }

    // NOTE: disabled since not used
    if (!FoundProfileData && false) {
      LLVM_DEBUG(dbgs() << "Couldn't find profile data for block "
                        << BlockStat.Name << ", " << BlockStat.FunctionName
                        << ", " << BlockStat.ModuleName << "\n");
    }

    BlockID++;
  }

  // make adjacency list
  // TODO: what if index fails (theoretically shouldn't, successors are only
  // within the MF)
  for (auto *MBB : Blocks) {
    unsigned u = BlockIDs[MBB];

    for (auto *Successor : MBB->successors()) {
      unsigned v = BlockIDs[Successor];

      BranchProbability Probability = MBPI->getEdgeProbability(MBB, Successor);
      double ProbabilityAsDecimal =
          static_cast<double>(Probability.getNumerator()) /
          static_cast<double>(Probability.getDenominator());

      LLVM_DEBUG(dbgs() << "Adding machine edge: " << u << "->" << v
                        << ", p: " << ProbabilityAsDecimal << "\n");

      PC.addMachineBlockEdgeLocal(MFName, u, v, ProbabilityAsDecimal);
    }
  }

  ++Processed;
  if (Processed == Total) {
    // TODO: perform critical path computation
    LLVM_DEBUG(dbgs() << "Perform critical path computation now...\n");

    PC.buildCriticalPath();
    PC.outputCriticalPath();
  }

  return false;
}
} // namespace llvm

INITIALIZE_PASS(RegisterAccessPreRAPass, "reg-access-prera",
                "Register Access Pre-RA Pass", false, false)
// INITIALIZE_PASS_END(RegisterAccessPreRAPass, "reg-access-prera",
//                    "Register Access Pre-RA Pass", false, false)

namespace llvm {
FunctionPass *createRegisterAccessPreRAPass() {
  return new RegisterAccessPreRAPass();
}

#undef DEBUG_TYPE

} // namespace llvm

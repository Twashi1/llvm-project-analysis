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
#include <filesystem>
#include <ios>
#include <iterator>
#include <stack>
#include <unordered_map>
#include <unordered_set>
// TODO: use numeric limits instead
#include <cfloat>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <optional>
#include <regex>

#define DEBUG_TYPE "reg-access-prera"

using namespace llvm;

namespace llvm {
char RegisterAccessPreRAPass::ID = 0;
unsigned RegisterAccessPreRAPass::Processed = 0;
unsigned RegisterAccessPreRAPass::Total = 0;
ExtPathCollector RegisterAccessPreRAPass::PC = {};

std::vector<ExtVFPair> teiGetSFVVCandidates(float TempKelvin,
                                            ExtConfigData const &Config) {
  // Target frequency is baseline
  float MinimumVoltage = teiVoltageToDiscreteLevel(
      teiGetVoltage(TempKelvin, Config.BaselineFrequencyGHz * 1.0e9f), Config);

  ExtVFPair Pair;
  Pair.Voltage = MinimumVoltage;
  Pair.FrequencyHz = Config.BaselineFrequencyGHz * 1.0e9f;

  // TODO: we can add the other criteria, like if temperature is too low we try
  // to adjust voltage up

  return std::vector<ExtVFPair>({Pair});
}

std::vector<ExtVFPair> teiGetVFVVCandidates(float TempKelvin,
                                            ExtConfigData const &Config) {
  std::vector<ExtVFPair> UnfilteredResults;

  for (uint32_t i = 0; i < Config.FrequenciesGHz.size(); i++) {
    float FrequencyHz = Config.FrequenciesGHz[i] * 1.0e9f;
    float Voltage = teiVoltageToDiscreteLevel(
        teiGetVoltage(TempKelvin, FrequencyHz), Config);

    ExtVFPair Pair;
    Pair.FrequencyHz = FrequencyHz;
    Pair.Voltage = Voltage;

    UnfilteredResults.push_back(Pair);
  }

  // Look through for TEI sweet spots briefly to eliminate
  // Frequencies being sorted already makes this simpler
  float PreviousVoltage = FLT_MAX;
  std::vector<ExtVFPair> Results;

  for (uint32_t i = UnfilteredResults.size() - 1; i >= 0; i--) {
    ExtVFPair Candidate = UnfilteredResults[i];

    // Candidate is of lower frequency (we iterate frequencies in descending
    // order) Candidate has higher or equal voltage, for lower frequency Hence
    // Candidate is at least worse than the previous pair; i.e. previous pair
    // was a TEI sweet spot Skip candidate
    if (Candidate.Voltage >= PreviousVoltage) {
      continue;
    }

    PreviousVoltage = Candidate.Voltage;
    Results.push_back(Candidate);
  }

  // Not necessary, but it means our frequencies are in ascending order again
  std::reverse(Results.begin(), Results.end());

  return Results;
}

float teiVoltageToDiscreteLevel(float Voltage, ExtConfigData const &Config) {
  Voltage += Config.VoltageAllowedError;

  for (uint32_t i = 0; i < Config.Voltages.size(); i++) {
    float DiscreteVoltage = Config.Voltages[i];

    if (Voltage < DiscreteVoltage)
      continue;

    return DiscreteVoltage;
  }

  LLVM_DEBUG(
      dbgs() << "Warning: couldn't select high enough discrete voltage level "
                "for given frequency, returning highest possible voltage\n");

  return Config.Voltages.back();
}

float teiGetVoltage(float TempKelvin, float FrequencyHz) {
  float TempCelsius = TempKelvin - 273.15f;
  float FrequencyGHz = FrequencyHz * 1.0e-9f;

  float D0 = -4.27f;
  float D1 = 0.0042f;
  float D2 = 0.0052f;
  float D3 = 10.06f;
  float D4 = -2.66f;

  float B1 = D1 * TempCelsius + D3;
  float B2 = D2 * TempCelsius + D4 - FrequencyGHz;

  float MinimumVoltage =
      (-B1 + std::sqrt(B1 * B1 - 4.0f * D0 * B2)) / (2.0f * D0);

  return MinimumVoltage;
}

std::vector<std::string> splitString(std::string const &Str,
                                     std::string const &Delimiters) {
  std::vector<std::string> Result;
  std::string Current;

  for (char c : Str) {
    if (Delimiters.find(c) != std::string::npos) {
      if (!Current.empty()) {
        Result.push_back(std::move(Current));
        Current.clear();
      }
    } else {
      Current += c;
    }
  }

  if (!Current.empty()) {
    Result.push_back(std::move(Current));
  }

  return Result;
}

bool stringStartsWith(std::string const &Str, std::string const &Prefix) {
  return Str.size() >= Prefix.size() &&
         Str.compare(0, Prefix.size(), Prefix) == 0;
}

static float getAreaHotSpotFlp(ExtHotSpotFloorplan Flp,
                               ExtHotSpotUnitName Name) {
  return Flp
      .Units[static_cast<std::underlying_type_t<ExtHotSpotUnitName>>(Name)]
      .Area;
}

static float &getPowerHotSpot(ExtHotSpotPowerInput &Power,
                              ExtHotSpotUnitName Name) {
  return Power
      .UnitPower[static_cast<std::underlying_type_t<ExtHotSpotUnitName>>(Name)];
}

char const *hotSpotUnitNameToString(ExtHotSpotUnitName Name) {
  switch (Name) {
  case ExtHotSpotUnitName::L2_LEFT:
    return "L2_left";
  case ExtHotSpotUnitName::L2:
    return "L2";
  case ExtHotSpotUnitName::L2_RIGHT:
    return "L2_right";
  case ExtHotSpotUnitName::ICACHE:
    return "Icache";
  case ExtHotSpotUnitName::DCACHE:
    return "Dcache";
  case ExtHotSpotUnitName::BPRED_0:
    return "Bpred_0";
  case ExtHotSpotUnitName::BPRED_1:
    return "Bpred_1";
  case ExtHotSpotUnitName::BPRED_2:
    return "Bpred_2";
  case ExtHotSpotUnitName::DTB_0:
    return "DTB_0";
  case ExtHotSpotUnitName::DTB_1:
    return "DTB_1";
  case ExtHotSpotUnitName::DTB_2:
    return "DTB_2";
  case ExtHotSpotUnitName::FPADD_0:
    return "FPAdd_0";
  case ExtHotSpotUnitName::FPADD_1:
    return "FPAdd_1";
  case ExtHotSpotUnitName::FPMUL_0:
    return "FPMul_0";
  case ExtHotSpotUnitName::FPMUL_1:
    return "FPMul_1";
  case ExtHotSpotUnitName::FPREG_0:
    return "FPReg_0";
  case ExtHotSpotUnitName::FPREG_1:
    return "FPReg_1";
  case ExtHotSpotUnitName::FPREG_2:
    return "FPReg_2";
  case ExtHotSpotUnitName::FPREG_3:
    return "FPReg_3";
  case ExtHotSpotUnitName::FPMAP_0:
    return "FPMap_0";
  case ExtHotSpotUnitName::FPMAP_1:
    return "FPMap_1";
  case ExtHotSpotUnitName::INTMAP:
    return "IntMap";
  case ExtHotSpotUnitName::INTQ:
    return "IntQ";
  case ExtHotSpotUnitName::INTREG_0:
    return "IntReg_0";
  case ExtHotSpotUnitName::INTREG_1:
    return "IntReg_1";
  case ExtHotSpotUnitName::INTEXEC:
    return "IntExec";
  case ExtHotSpotUnitName::FPQ:
    return "FPQ";
  case ExtHotSpotUnitName::LDSTQ:
    return "LdStQ";
  case ExtHotSpotUnitName::ITB_0:
    return "ITB_0";
  case ExtHotSpotUnitName::ITB_1:
    return "ITB_1";
  default:
    return "Unknown";
  };
}

ExtHotSpotUnitName hotSpotStringToUnitName(std::string Name) {
  using T = std::underlying_type_t<ExtHotSpotUnitName>;

  for (T i = static_cast<T>(ExtHotSpotUnitName::__FIRST);
       i < static_cast<T>(ExtHotSpotUnitName::__LAST); i++) {
    if (Name == hotSpotUnitNameToString(static_cast<ExtHotSpotUnitName>(i))) {
      return static_cast<ExtHotSpotUnitName>(i);
    }
  }

  LLVM_DEBUG(errs() << "Failed to find unit name for unit " << Name << "\n");

  return ExtHotSpotUnitName::__LAST;
}

ExtHotSpotTempTrace readHotSpotTempTrace(char const *FileName) {
  std::ifstream File(FileName);

  ExtHotSpotTempTrace Trace;

  if (!File) {
    LLVM_DEBUG(errs() << "Failed to open file: " << FileName << "\n");
    return Trace;
  }

  std::string Line;
  int LineNumber = 0;
  std::vector<ExtHotSpotUnitName> UnitOrder;
  while (std::getline(File, Line)) {
    std::vector<std::string> Parts = splitString(Line, "\n\t ");

    if (Parts.size() != 30) {
      LLVM_DEBUG(errs() << "Potentially malformed line in output temperature "
                           "trace at line number: "
                        << LineNumber + 1 << "\n");
      continue;
    }

    if (LineNumber++ == 0) {
      // TODO: reserve space or use index
      for (std::string Part : Parts) {
        ExtHotSpotUnitName UnitName = hotSpotStringToUnitName(Part);

        UnitOrder.push_back(UnitName);
      }

      continue;
    }

    for (uint32_t i = 0; i < Parts.size(); i++) {
      ExtHotSpotUnitName UnitName = UnitOrder[i];
      float Temp = std::stof(Parts[i]);

      Trace.Temps.at(static_cast<uint32_t>(UnitName)).push_back(Temp);
    }
  }

  return Trace;
}

void writeHotSpotTempInit(float InitialTemperature, char const *FileName) {
  std::ofstream File(FileName);

  std::stringstream TempStream;
  TempStream << std::fixed << std::setprecision(3) << InitialTemperature;
  std::string TempString = TempStream.str();

  for (uint32_t i = 0; i < static_cast<uint32_t>(ExtHotSpotUnitName::__LAST);
       i++) {
    ExtHotSpotUnitName UnitEnum = static_cast<ExtHotSpotUnitName>(i);

    std::string Line = std::string(hotSpotUnitNameToString(UnitEnum)) + '\t' +
                       TempString + '\n';
    File << Line;
    File << "hsp_" << Line;
    File << "hsink_" << Line;
    File << "iface_" << Line;
  }

  for (uint32_t i = 0; i < 12; i++) {
    File << "inode_" << i << "\t" << TempString << "\n";
  }
}

ExtHotSpotTempTrace
aggregateTracesAverage(std::vector<ExtHotSpotTempTrace> const &Traces) {
  if (Traces.size() == 0) {
    LLVM_DEBUG(errs() << "Cannot aggregate 0 traces\n");
  }

  uint32_t NumUnits = Traces[0].Temps.size();

  ExtHotSpotTempTrace Output;
  Output.Temps = std::vector<std::vector<float>>(NumUnits);

  std::vector<float> TotalTempPerUnit = std::vector<float>(NumUnits);

  for (uint32_t UnitIndex = 0; UnitIndex < Output.Temps.size(); UnitIndex++) {
    float TotalTemp = 0.0f;

    for (uint32_t i = 0; i < Traces.size(); i++) {
      std::vector<float> const &PerSampleTemps = Traces[i].Temps[UnitIndex];
      float AverageReading = 0.0f;

      for (float Reading : PerSampleTemps) {
        AverageReading += Reading;
      }

      AverageReading /= static_cast<float>(PerSampleTemps.size());

      TotalTemp += AverageReading;
    }

    Output.Temps.at(UnitIndex) =
        std::vector<float>({TotalTemp / static_cast<float>(Traces.size())});
  }

  return Output;
}

void writeHotSpotTempInit(ExtHotSpotTempTrace PreviousTrace,
                          char const *FileName) {
  std::ofstream File(FileName);

  File << std::fixed << std::setprecision(3);

  for (uint32_t i = 0; i < static_cast<uint32_t>(ExtHotSpotUnitName::__LAST);
       i++) {
    ExtHotSpotUnitName UnitEnum = static_cast<ExtHotSpotUnitName>(i);

    std::vector<float> TempReadings = PreviousTrace.Temps[i];
    float AverageTemp = 0.0f;

    for (float Reading : TempReadings) {
      AverageTemp += Reading;
    }

    AverageTemp /= static_cast<float>(TempReadings.size());

    std::stringstream TempStream;
    TempStream << std::fixed << std::setprecision(3) << AverageTemp;

    std::string Line = std::string(hotSpotUnitNameToString(UnitEnum)) + '\t' +
                       TempStream.str() + '\n';
    File << Line;
    // TODO: we assume all readings are same as unit temperature, but reality is
    // likely slightly different?
    // TODO: heatsink offset
    File << "hsp_" << Line;
    File << "hsink_" << Line;
    File << "iface_" << Line;
  }

  for (uint32_t i = 0; i < 12; i++) {
    // TODO: use an area-weighted average for Inode temperatures
    File << "inode_" << i << "\t" << 350.0f << "\n";
  }
}

ExtHotSpotTempInit readHotSpotTempInit(char const *FileName) {
  std::ifstream File(FileName);

  ExtHotSpotTempInit Trace;

  if (!File) {
    LLVM_DEBUG(errs() << "Failed to open file: " << FileName << "\n");
    return Trace;
  }

  std::string Line;
  while (std::getline(File, Line)) {
    std::vector<std::string> Parts = splitString(Line, "\n\t ");

    if (Parts.size() != 2) {
      // Possibly malformed, or just a blank line
    } else {
      std::string UnitName = Parts[0];
      float Temp = std::stof(Parts[1]);

      // UnitName could be prefixed by
      // nothing -> Unit
      // hsp_ -> Heatspreader
      // iface_ -> Interface
      // hsink_ -> Heatsink

      // And alternatively...
      // inode_x where x [0, 11]
      ExtHotSpotUnitName UnitEnum = ExtHotSpotUnitName::__LAST;

      // TODO: use proper std::underlying_type_t
      if (stringStartsWith(UnitName, "hsp_")) {
        UnitEnum = hotSpotStringToUnitName(UnitName.substr(strlen("hsp_")));
        Trace.Units.at(static_cast<uint32_t>(UnitEnum)).Hsp = Temp;
      } else if (stringStartsWith(UnitName, "iface_")) {
        UnitEnum = hotSpotStringToUnitName(UnitName.substr(strlen("iface_")));
        Trace.Units.at(static_cast<uint32_t>(UnitEnum)).Iface = Temp;
      } else if (stringStartsWith(UnitName, "hsink_")) {
        UnitEnum = hotSpotStringToUnitName(UnitName.substr(strlen("hsink_")));
        Trace.Units.at(static_cast<uint32_t>(UnitEnum)).Hsink = Temp;
      } else if (stringStartsWith(UnitName, "inode_")) {
        int InodeIndex = std::stoi(UnitName.substr(strlen("inode_")));

        Trace.Inode[InodeIndex] = Temp;

        continue;
      } else {
        UnitEnum = hotSpotStringToUnitName(UnitName);
        Trace.Units.at(static_cast<uint32_t>(UnitEnum)).Unit = Temp;
      }

      if (UnitEnum == ExtHotSpotUnitName::__LAST) {
        LLVM_DEBUG(errs() << "Invalid file format for line: " << Line << "\n");

        continue;
      }
    }
  }

  return Trace;
}

ExtHotSpotFloorplan readHotSpotFloorplan(char const *FileName) {
  std::ifstream File(FileName);

  ExtHotSpotFloorplan Floorplan;

  if (!File) {
    LLVM_DEBUG(errs() << "Failed to open file: " << FileName << "\n");
    return Floorplan;
  }

  std::string Line;
  while (std::getline(File, Line)) {
    std::vector<std::string> Parts = splitString(Line, "\n\t ");

    if (Parts.size() != 5) {
      // Possibly malformed; should warn (could also just be blank lines)
    } else {
      std::string UnitName = Parts[0];
      float Left = std::stof(Parts[1]);
      float Bottom = std::stof(Parts[2]);
      float Width = std::stof(Parts[3]);
      float Height = std::stof(Parts[4]);

      ExtHotSpotUnitName UnitEnum = hotSpotStringToUnitName(UnitName);
      ExtHotSpotFlpUnit &Unit = Floorplan.Units.at(
          static_cast<std::underlying_type_t<ExtHotSpotUnitName>>(UnitEnum));
      Unit.Left = Left;
      Unit.Bottom = Bottom;
      Unit.Width = Width;
      Unit.Height = Height;
      Unit.Area = Width * Height;
    }
  }

  return Floorplan;
}

ExtHotSpotPowerInput
mapMcPATPowerToHotspotPower(ExtMcPATOutput const &McPatPower,
                            ExtHotSpotFloorplan const &HotSpotFlp,
                            ExtConfigData const &Config) {
  ExtHotSpotPowerInput HotSpotPower;

  // Floorplan is already scaled by McPAT area (by a python script, but its just
  // setup so acceptable?)

  // TODO: L2 cache temperature
  // TODO: unused residuals
  float IfuResidual =
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::INSTRUCTION_FETCH_UNIT) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::INSTRUCTION_CACHE) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::BRANCH_TARGET_BUFFER) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::BRANCH_PREDICTOR) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::INSTRUCTION_DECODER) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::INSTRUCTION_BUFFER);

  float RenameResidual =
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::RENAMING_UNIT) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::INT_FRONT_END_RAT) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::FP_FRONT_END_RAT) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::INT_RETIRE_RAT) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::FP_RETIRE_RAT) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::FP_FREE_LIST) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::FREE_LIST);

  float LoadStoreResidual =
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::LOAD_STORE_UNIT) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::DATA_CACHE) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::LOADQ) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::STOREQ);

  float MemoryManagementResidual =
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::MEMORY_MANAGEMENT_UNIT) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::ITLB) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::DTLB);

  float ExecutionResidual =
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::EXECUTION_UNIT) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::REGISTER_FILES) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::INSTRUCTION_SCHEDULER) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::INTEGER_ALU) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::FLOATING_POINT_UNIT) -
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::RESULTS_BROADCAST_BUS);

  float FloatPointMap =
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::FP_FREE_LIST) +
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::FP_RETIRE_RAT) +
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::FP_FRONT_END_RAT);

  float IntMap = getPowerMcPAT(McPatPower, ExtMcPATUnitName::FREE_LIST) +
                 getPowerMcPAT(McPatPower, ExtMcPATUnitName::INT_RETIRE_RAT) +
                 getPowerMcPAT(McPatPower, ExtMcPATUnitName::INT_FRONT_END_RAT);

  float FpUnit =
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::FLOATING_POINT_UNIT);
  float FpRf = getPowerMcPAT(McPatPower, ExtMcPATUnitName::FLOATING_POINT_RF);
  float Dtb = getPowerMcPAT(McPatPower, ExtMcPATUnitName::DTLB);
  float Itb = getPowerMcPAT(McPatPower, ExtMcPATUnitName::ITLB);
  float Bpred = getPowerMcPAT(McPatPower, ExtMcPATUnitName::BRANCH_PREDICTOR);
  float IntQ = getPowerMcPAT(McPatPower, ExtMcPATUnitName::INSTRUCTION_WINDOW);
  float IntRf = getPowerMcPAT(McPatPower, ExtMcPATUnitName::INTEGER_RF);
  float IntAlu = getPowerMcPAT(McPatPower, ExtMcPATUnitName::INTEGER_ALU);
  float FloatQ =
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::FP_INSTRUCTION_WINDOW);
  float LoadStoreQ = getPowerMcPAT(McPatPower, ExtMcPATUnitName::LOADQ) +
                     getPowerMcPAT(McPatPower, ExtMcPATUnitName::STOREQ);

  float TotalExecutionArea =
      getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPADD_0) +
      getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPADD_1) +
      getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPREG_0) +
      getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPREG_1) +
      getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPREG_2) +
      getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPREG_3) +
      getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPMUL_0) +
      getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPMUL_1) +
      getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPQ) +
      getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::INTQ) +
      getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::INTEXEC) +
      getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::INTREG_0) +
      getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::INTREG_1);

  // TODO: need to get L2 stats
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::ICACHE) =
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::INSTRUCTION_CACHE);
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::DCACHE) =
      getPowerMcPAT(McPatPower, ExtMcPATUnitName::DATA_CACHE);

  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::BPRED_0) = Bpred / 3.0;
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::BPRED_1) = Bpred / 3.0;
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::BPRED_2) = Bpred / 3.0;

  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::DTB_0) = Dtb / 3.0;
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::DTB_1) = Dtb / 3.0;
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::DTB_2) = Dtb / 3.0;

  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::FPADD_0) =
      FpUnit / 4.0 +
      (getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPADD_0) /
       TotalExecutionArea) *
          ExecutionResidual;
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::FPADD_1) =
      FpUnit / 4.0 +
      (getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPADD_1) /
       TotalExecutionArea) *
          ExecutionResidual;
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::FPMUL_0) =
      FpUnit / 4.0 +
      (getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPMUL_0) /
       TotalExecutionArea) *
          ExecutionResidual;
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::FPMUL_1) =
      FpUnit / 4.0 +
      (getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPMUL_1) /
       TotalExecutionArea) *
          ExecutionResidual;

  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::FPREG_0) =
      FpRf / 4.0 + (getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPREG_0) /
                    TotalExecutionArea) *
                       ExecutionResidual;
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::FPREG_1) =
      FpRf / 4.0 + (getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPREG_1) /
                    TotalExecutionArea) *
                       ExecutionResidual;
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::FPREG_2) =
      FpRf / 4.0 + (getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPREG_2) /
                    TotalExecutionArea) *
                       ExecutionResidual;
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::FPREG_3) =
      FpRf / 4.0 + (getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPREG_3) /
                    TotalExecutionArea) *
                       ExecutionResidual;

  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::FPMAP_0) =
      FloatPointMap / 2.0 +
      (getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPMAP_0) /
       TotalExecutionArea) *
          ExecutionResidual;
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::FPMAP_1) =
      FloatPointMap / 2.0 +
      (getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPMAP_1) /
       TotalExecutionArea) *
          ExecutionResidual;

  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::INTREG_0) =
      IntRf / 2.0 +
      (getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::INTREG_0) /
       TotalExecutionArea) *
          ExecutionResidual;
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::INTREG_1) =
      IntRf / 2.0 +
      (getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::INTREG_1) /
       TotalExecutionArea) *
          ExecutionResidual;

  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::INTMAP) = IntMap;

  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::ITB_0) = Itb / 2.0;
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::ITB_1) = Itb / 2.0;

  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::LDSTQ) = LoadStoreQ;
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::FPQ) =
      FloatQ + (getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::FPQ) /
                TotalExecutionArea) *
                   ExecutionResidual;
  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::INTQ) =
      IntQ + (getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::INTQ) /
              TotalExecutionArea) *
                 ExecutionResidual;

  getPowerHotSpot(HotSpotPower, ExtHotSpotUnitName::INTEXEC) =
      IntAlu + (getAreaHotSpotFlp(HotSpotFlp, ExtHotSpotUnitName::INTEXEC) /
                TotalExecutionArea) *
                   ExecutionResidual;

  // execution_units = [
  //     "FPAdd_0",
  //     "FPAdd_1",
  //     "FPReg_0",
  //     "FPReg_1",
  //     "FPReg_2",
  //     "FPReg_3",
  //     "FPMul_0",
  //     "FPMul_1",
  //     "FPQ",
  //     "IntQ",
  //     "IntExec",
  //     "IntReg_0",
  //     "IntReg_1",
  // ]
  //
  // l2_power = get_static_dynamic_power(unit_stats, ["L2"])
  //
  //     "L2_left": l2_power / 3.0,
  //     "L2": l2_power / 3.0,
  //     "L2_right": l2_power / 3.0,

  //     "FPAdd_0": fpu_power / 4.0
  //     + area_fraction("FPAdd_0", execution_total_area) *
  //     execution_residual, "FPAdd_1": fpu_power / 4.0
  //     + area_fraction("FPAdd_1", execution_total_area) *
  //     execution_residual, "FPReg_0": fp_rf_power / 4.0
  //     + area_fraction("FPReg_0", execution_total_area) *
  //     execution_residual, "FPReg_1": fp_rf_power / 4.0
  //     + area_fraction("FPReg_1", execution_total_area) *
  //     execution_residual, "FPReg_2": fp_rf_power / 4.0
  //     + area_fraction("FPReg_2", execution_total_area) *
  //     execution_residual, "FPReg_3": fp_rf_power / 4.0
  //     + area_fraction("FPReg_3", execution_total_area) *
  //     execution_residual, "FPMul_0": fpu_power / 4.0
  //     + area_fraction("FPMul_0", execution_total_area) *
  //     execution_residual, "FPMul_1": fpu_power / 4.0
  //     + area_fraction("FPMul_1", execution_total_area) *
  //     execution_residual, "FPMap_0": fp_map_power / 2.0, "FPMap_1":
  //     fp_map_power / 2.0, "IntMap": int_map_power, "IntQ": intq_power
  //     + area_fraction("IntQ", execution_total_area) * execution_residual,
  //     "IntReg_0": int_rf_power / 2.0
  //     + area_fraction("IntReg_0", execution_total_area) *
  //     execution_residual, "IntReg_1": int_rf_power / 2.0
  //     + area_fraction("IntReg_1", execution_total_area) *
  //     execution_residual, "IntExec": int_alu_power
  //     + area_fraction("IntExec", execution_total_area) *
  //     execution_residual, "FPQ": fpq_power
  //     + area_fraction("FPQ", execution_total_area) * execution_residual,
  //     "LdStQ": lsq_power,
  //     "ITB_0": itb_power / 2.0,
  //     "ITB_1": itb_power / 2.0,
  // }
  //
  // return hotspot_mapping

  return HotSpotPower;
}

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
    // TODO: this is actually not correct, we should use Freq instead in reality
    // although it doesn't matter too much
    // Note both cases we're using GlobalFreq, we want this to be irrespective
    // of the function's call frequency
    CompWeight[c] += BlockStats[v].Cycles * BlockStats[v].GlobalFreq;
    // TODO: instead, only look at potential entry blocks within the component
    // additionally, take the maximum of the entry blocks as its both the
    // worst-case and average-case scenario
    CompMinFrequency[c] =
        std::max(CompMinFrequency[c], BlockStats[v].GlobalFreq);
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
    std::vector<unsigned> InternalEndBlocks;
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
          InternalEndBlocks.push_back(BlockID);
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
    SubgraphInternalEndBlocks.push_back(InternalEndBlocks);
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
  std::error_code EC_PathRoots;
  raw_fd_ostream OutPathRoots("PathRoots.csv", EC_PathRoots,
                              sys::fs::OF_Append);

  // TODO: function for checking error code?
  if (EC_PathRoots) {
    errs() << "Error opening file: " << EC_PathRoots.message() << "\n";
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
  std::error_code EC_PathCFG;

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

  raw_fd_ostream OutPathCFG("PathCFG.csv", EC_PathCFG, sys::fs::OF_Append);

  if (EC_PathCFG) {
    errs() << "Error opening file: " << EC_PathCFG.message() << "\n";
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

  OutPathCFG
      << "module_name,start_function_name,start_block_name,start_block_id,exit_"
         "function_"
         "name,exit_block_name,exit_block_id,branch_prob,start_path_index,end_"
         "path_index,is_start_entry\n";

  OutPathRoots
      << "module_name,path_index,block_id,local_block_id,function_name\n";

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
  }

  // TODO: write path CFG, a CFG related strictly to our program paths
  // Cache for the total execution cycles of a given subgraph
  std::vector<float> SubgraphExitExecutionFrequency = {};
  for (unsigned i = 0; i < PotentialExitBlocks.size(); i++) {
    std::vector<unsigned> const &SubgraphExitBlocks = PotentialExitBlocks[i];

    float TotalExecutionFrequency = 0.0;

    for (unsigned u = 0; u < SubgraphExitBlocks.size(); u++) {
      unsigned BlockID = SubgraphExitBlocks[u];
      ExtBBStats Stats = BlockStats[BlockID];

      TotalExecutionFrequency += Stats.Freq;
    }

    SubgraphExitExecutionFrequency.push_back(TotalExecutionFrequency);
  }

  // 1. calculate total execution frequency of all exit blocks for a given entry
  // block
  // 2. assume singular entry block for a given subgraph
  // 3. create a connection from the entry block to the exit block; if the exit
  // block is within the subgraph
  //    take the root of that subgraph to be the exit block
  // 4. consider execution frequency of the exit block to be the weight, take it
  // over our total weight to find probability
  // Need mapping BlockID -> Subgraph (PathIndexOfBlock)
  // // TODO: code is not running? Not outputtig?
  for (unsigned u = 0; u < GlobalAdjacencyList.size(); u++) {
    std::vector<unsigned> const &Neighbours = GlobalAdjacencyList[u];
    unsigned StartBlock = u;
    ExtBBStats StartStats = BlockStats[StartBlock];

    int StartSubgraphID = PathIndexOfBlock[StartBlock];

    for (unsigned v = 0; v < Neighbours.size(); v++) {
      unsigned EndBlock = Neighbours[v];
      ExtBBStats EndStats = BlockStats[EndBlock];

      int EndSubgraphID = PathIndexOfBlock[EndBlock];

      if (StartSubgraphID == EndSubgraphID)
        continue;

      // We take the probability of thsi connection to be our execution
      // frequency relative to the execution frequency of all exit blocks summed
      // Poor accuracy likely, but a simple heuristic to use
      float SubgraphExecutionFrequency =
          SubgraphExitExecutionFrequency[StartSubgraphID];
      float EdgeProbability = 1.0;

      if (SubgraphExecutionFrequency > 0.0) {
        EdgeProbability = EndStats.Freq / SubgraphExecutionFrequency;
      }

      // print to CFG data in format
      OutPathCFG << StartStats.ModuleName << "," << StartStats.FunctionName
                 << "," << StartStats.Name << "," << StartSubgraphID << ","
                 << EndStats.FunctionName << "," << EndStats.Name << ","
                 << EndSubgraphID << "," << EdgeProbability << ","
                 << StartSubgraphID << "," << EndSubgraphID << ","
                 << MapIsEntryBlock[StartBlock] << "\n";
    }
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

    if (MapIsEntryBlock[BlockID]) {
      OutPathRoots << ModuleName << "," << CompID << "," << BlockID << ","
                   << BlockStat.LocalBlockNumber << ","
                   << BlockStat.FunctionName << "\n";
    }

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

  BlockOutFile.close();
  OutCFGFile.close();
  OutBlockAdditional.close();
  OutTopoComp.close();
  OutDAGFile.close();
  OutPathCFG.close();
  OutPathRoots.close();
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
    ExtBBStats Stats;
    Stats.FunctionName = FunctionName;
    Stats.LocalBlockNumber = LocalBlockID;
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

char const *unitNameToString(ExtMcPATUnitName const Name) {
  switch (Name) {
  case ExtMcPATUnitName::PROCESSOR:
    return "Processor";
  case ExtMcPATUnitName::CORE:
    return "Core";
  case ExtMcPATUnitName::INSTRUCTION_FETCH_UNIT:
    return "Instruction Fetch Unit";
  case ExtMcPATUnitName::INSTRUCTION_CACHE:
    return "Instruction Cache";
  case ExtMcPATUnitName::BRANCH_TARGET_BUFFER:
    return "Branch Target Buffer";
  case ExtMcPATUnitName::BRANCH_PREDICTOR:
    return "Branch Predictor";
  case ExtMcPATUnitName::INSTRUCTION_BUFFER:
    return "Instruction Buffer";
  case ExtMcPATUnitName::INSTRUCTION_DECODER:
    return "Instruction Decoder";
  case ExtMcPATUnitName::RENAMING_UNIT:
    return "Renaming Unit";
  case ExtMcPATUnitName::INT_FRONT_END_RAT:
    return "Int Front End RAT";
  case ExtMcPATUnitName::FP_FRONT_END_RAT:
    return "FP Front End RAT";
  case ExtMcPATUnitName::FREE_LIST:
    return "Free List";
  case ExtMcPATUnitName::INT_RETIRE_RAT:
    return "Int Retire RAT";
  case ExtMcPATUnitName::FP_RETIRE_RAT:
    return "FP Retire RAT";
  case ExtMcPATUnitName::FP_FREE_LIST:
    return "FP Free List";
  case ExtMcPATUnitName::LOAD_STORE_UNIT:
    return "Load Store Unit";
  case ExtMcPATUnitName::DATA_CACHE:
    return "Data Cache";
  case ExtMcPATUnitName::LOADQ:
    return "LoadQ";
  case ExtMcPATUnitName::STOREQ:
    return "StoreQ";
  case ExtMcPATUnitName::MEMORY_MANAGEMENT_UNIT:
    return "Memory Management Unit";
  case ExtMcPATUnitName::ITLB:
    return "Itlb";
  case ExtMcPATUnitName::DTLB:
    return "Dtlb";
  case ExtMcPATUnitName::EXECUTION_UNIT:
    return "Execution Unit";
  case ExtMcPATUnitName::REGISTER_FILES:
    return "Register Files";
  case ExtMcPATUnitName::INTEGER_RF:
    return "Integer RF";
  case ExtMcPATUnitName::FLOATING_POINT_RF:
    return "Floating Point RF";
  case ExtMcPATUnitName::INSTRUCTION_SCHEDULER:
    return "Instruction Scheduler";
  case ExtMcPATUnitName::INSTRUCTION_WINDOW:
    return "Instruction Window";
  case ExtMcPATUnitName::FP_INSTRUCTION_WINDOW:
    return "FP Instruction Window";
  case ExtMcPATUnitName::ROB:
    return "ROB";
  case ExtMcPATUnitName::INTEGER_ALU:
    return "Integer ALU";
  case ExtMcPATUnitName::FLOATING_POINT_UNIT:
    return "Floating Point Unit";
  case ExtMcPATUnitName::RESULTS_BROADCAST_BUS:
    return "Results Broadcast Bus";
  case ExtMcPATUnitName::UNDIFFERENTIATED_CORE:
    return "Undifferentiated Core";
  default:
    return "Unknown";
  };
}

static std::optional<ExtMcPATUnit> getUnitStats(std::string const &Name,
                                                std::string const &Text) {
  // Assumes Name is sanitised/doesn't contain special characters
  // C++ default regex implementation is very slow, but we can't use CTREs
  // Regex pattern is something along the lines of
  // (whitespace) unit name (possible extra text): some value to capture (units)
  std::regex Pattern("\\s*" + Name + ".*\\n((?:.+\\s*=\\s*.+\\n)+)",
                     std::regex_constants::ECMAScript |
                         std::regex_constants::multiline);
  std::smatch Match;

  if (!std::regex_search(Text, Match, Pattern)) {
    return std::nullopt;
  }

  std::string Block = Match[1].str();

  ExtMcPATUnit Unit;

  std::vector<std::string> const Keys = {"Area", "Peak Dynamic",
                                         "Subthreshold Leakage", "Gate Leakage",
                                         "Runtime Dynamic"};

  auto MatchKey = [&](std::string const Key) -> std::optional<float> {
    std::regex FieldPattern("\\s+" + Key + "\\s*=\\s*(\\S+)\\s.+\\n");
    std::smatch FieldMatch;
    if (std::regex_search(Block, FieldMatch, FieldPattern)) {
      return std::make_optional(std::stof(FieldMatch[1].str()));
    }

    return std::nullopt;
  };

  if (auto V = MatchKey("Area"))
    Unit.AreaMetresSquared = (*V) * 1.0e-6;

  if (auto V = MatchKey("Peak Dynamic"))
    Unit.PeakDynamic = (*V);

  if (auto V = MatchKey("Subthreshold Leakage"))
    Unit.SubthresholdLeakage = (*V);

  if (auto V = MatchKey("Gate Leakage"))
    Unit.GateLeakage = (*V);

  if (auto V = MatchKey("Runtime Dynamic"))
    Unit.RuntimeDynamic = (*V);

  return Unit;
};

ExtMcPATOutput readMcPATOutput(char const *FileName) {
  std::ifstream File(FileName);

  ExtMcPATOutput Output = {0};
  // We don't set node size, clock rate, or vdd here; while they are findable in
  // the McPAT output file, its too annoying to parse

  if (!File) {
    LLVM_DEBUG(errs() << "Failed to open file " << FileName << "\n");
    return Output;
  }

  // most vexing parse
  std::string Text = std::string(std::istreambuf_iterator<char>(File),
                                 std::istreambuf_iterator<char>());

  auto GetStatsOrPanic = [&](ExtMcPATUnitName Key) {
    std::string Name = unitNameToString(Key);

    if (auto V = getUnitStats(Name, Text)) {
      Output.UnitMap.insert({Key, *V});
    } else {
      LLVM_DEBUG(errs() << "Failed to grab unit stats for unit name: " << Name
                        << " for file name: " << FileName << "\n");
    }
  };

  // TODO: add bounds to the enum itself so we're less likely to break this
  for (std::underlying_type_t<ExtMcPATUnitName> i = 0;
       i < static_cast<std::underlying_type_t<ExtMcPATUnitName>>(
               ExtMcPATUnitName::UNDIFFERENTIATED_CORE);
       i++) {
    GetStatsOrPanic(static_cast<ExtMcPATUnitName>(i));
  }

  return Output;
}

float getPowerMcPAT(ExtMcPATOutput const &Output, ExtMcPATUnitName Name) {
  if (Output.UnitMap.find(Name) != Output.UnitMap.end()) {
    ExtMcPATUnit const &Unit = Output.UnitMap.at(Name);

    return Unit.RuntimeDynamic + Unit.SubthresholdLeakage + Unit.GateLeakage;
  }

  return 0.0f;
}

float getAreaMcPAT(ExtMcPATOutput const &Output, ExtMcPATUnitName Name) {
  if (Output.UnitMap.find(Name) != Output.UnitMap.end()) {
    ExtMcPATUnit const &Unit = Output.UnitMap.at(Name);

    return Unit.AreaMetresSquared;
  }

  return 0.0f;
}

std::string programNameToMcPATFile(std::string ProgramName,
                                   ExtMcPatInput const &Input) {
  // We don't encode temperature here because we don't vary it
  // (including temperature would make it very rare that we have values
  // available already)
  // TODO: directory path needs to be here too
  return ProgramName + "__n" + std::to_string(Input.NodeSize) + "_v" +
         std::to_string(static_cast<int>(Input.Voltage * 1000.0f)) + "_f" +
         std::to_string(static_cast<int>(Input.ClockRateHz * 1.0e-6)) + "_id" +
         std::to_string(Input.BlockID);
}

static ExtMcPATOutput populateExtraMcPATOutputFields(ExtMcPatInput const &Input,
                                                     ExtMcPATOutput Output) {
  Output.BlockID = Input.BlockID;
  Output.Voltage = Input.Voltage;
  Output.ClockRateHz = Input.ClockRateHz;
  Output.NodeSize = Input.NodeSize;

  return Output;
}

ExtMcPatInput blockStatsToMcPAT(int Id, float Voltage, float ClockRateHz,
                                int NodeSize,
                                std::vector<ExtBBStats> const &BlockStats) {

  ExtMcPatInput Input;
  Input.BlockID = Id;
  Input.NodeSize = NodeSize;
  Input.Voltage = Voltage;
  Input.ClockRateHz = ClockRateHz;
  Input.TempKelvin = 350; // TODO: from a config file as before

  for (ExtBBStats const &Stats : BlockStats) {
    Input.CycleCount += Stats.Cycles;
    Input.InstrCount += Stats.InstrCount;
    Input.IntInstrCount += Stats.IntInstrCount;
    Input.FloatInstrCount += Stats.FloatInstrCount;
    Input.BranchInstrCount += Stats.BranchInstrCount;
    Input.Loads += Stats.Loads;
    Input.Stores += Stats.Stores;
    Input.FunctionCalls += Stats.FunctionCalls;
    Input.ContextSwitches += Stats.ContextSwitches;
    Input.MulAccess += Stats.MulAccess;
    Input.FpuAccess += Stats.FPAccess;
    Input.IAluAccess += Stats.IntALUAccess;
    Input.IntRegfileReads += Stats.IntRegfileReads;
    Input.FloatRegfileReads += Stats.FloatRegfileReads;
    Input.IntRegfileWrites += Stats.IntRegfileWrites;
    Input.FloatRegfileWrites += Stats.FloatRegfileWrites;
  }

  // Fill in extra stats with assumptions
  Input.BusyCycles = Input.CycleCount;
  Input.IdleCycles = 0;
  Input.BranchMispredictions = Input.BranchInstrCount / 20;
  Input.ROBReads = Input.InstrCount;
  Input.ROBWrites = Input.InstrCount;
  Input.RenameReads = Input.InstrCount * 2;
  Input.RenameWrites = Input.InstrCount;
  Input.FpRenameReads = Input.InstrCount * 2;
  Input.FpRenameWrites = Input.InstrCount;
  Input.InstWindowReads = Input.InstrCount;
  Input.InstWindowWrites = Input.InstrCount;
  Input.InstWindowWakeupAccesses = Input.InstrCount * 2;
  Input.FpInstWindowReads = Input.FloatInstrCount;
  Input.FpInstWindowWrites = Input.FloatInstrCount;
  Input.FpInstWindowWakeupAccesses = Input.FloatInstrCount * 2;
  Input.CdbALUAccess = Input.IAluAccess;
  Input.CdbFpAccess = Input.FpuAccess;
  Input.CdbMulAccess = Input.MulAccess;
  Input.BtbWrites = 0;
  Input.BtbReads = Input.InstrCount;

  return Input;
}

ExtMcPATOutput runMcPAT(std::string ProgramName, ExtMcPatInput const &Input) {
  std::string FileName = programNameToMcPATFile(ProgramName, Input);
  std::string OutFile = "./mcpat_out/" + ProgramName + "/" + FileName + ".xml";
  std::string InFile =
      "./mcpat_inputs/" + ProgramName + "/" + FileName + ".txt";

  if (std::filesystem::is_regular_file(OutFile)) {
    // Just read the file and return
    return populateExtraMcPATOutputFields(Input,
                                          readMcPATOutput(OutFile.c_str()));
  }

  // TODO: ideally command should just be running McPAT binary
  // TODO: even better; build McPAT as dll or statically link so we can just
  // call directly
  std::string Command =
      "./run_mcpat_specific.sh " + ProgramName + " " + FileName;

  int Ret = std::system(Command.c_str());

  if (Ret != 0) {
    LLVM_DEBUG(dbgs() << "Running McPAT failed on: " << ProgramName
                      << ", file: " << FileName);
  }

  return populateExtraMcPATOutputFields(Input,
                                        readMcPATOutput(OutFile.c_str()));
}

void createMcPATInputFile(char const *FileName, ExtMcPatInput const &Input) {
  std::ofstream File(FileName);

  if (!File) {
    LLVM_DEBUG(errs() << "Failed to open file " << FileName << "\n");
    return;
  }

  File << "<?xml version='1.0' encoding='utf-8'?>\n";

  auto WriteParam = [&File](char const *Name, std::string const &Value) {
    File << "<param name='" << Name << "' value='" << Value << "' />";
  };

  auto WriteStat = [&File](char const *Name, std::string const &Value) {
    File << "<stat name='" << Name << "' value='" << Value << "' />";
  };

  auto WriteComponentStart = [&File](char const *Id, char const *Name) {
    File << "<component id='" << Id << "' name='" << Name << "'>";
  };

  auto WriteComponentEnd = [&File](void) { File << "</component>"; };

  WriteComponentStart("root", "root");

  { // Start root
    WriteComponentStart("system", "system");

    { // Start system
      WriteParam("number_of_cores", std::to_string(1));
      WriteParam("number_of_L1Directories", std::to_string(0));
      WriteParam("number_of_L2Directories", std::to_string(1));
      WriteParam("number_of_L2s", std::to_string(1));
      WriteParam("Private_L2", std::to_string(0));
      WriteParam("number_of_L3s", std::to_string(0));
      WriteParam("number_of_NoCs", std::to_string(1));
      WriteParam("homogeneous_cores", std::to_string(1));
      WriteParam("homogeneous_L2s", std::to_string(1));
      WriteParam("homogeneous_L1Directories", std::to_string(1));
      WriteParam("homogeneous_L2Directories", std::to_string(1));
      WriteParam("homogeneous_L3s", std::to_string(1));
      WriteParam("homogeneous_ccs", std::to_string(1));
      WriteParam("homogeneous_NoCs", std::to_string(1));
      WriteParam("core_tech_node", std::to_string(Input.NodeSize));
      WriteParam("target_core_clockrate",
                 std::to_string(Input.ClockRateHz * 1.0e-6)); // Hz -> MHz
      WriteParam("temperature", std::to_string(Input.TempKelvin));
      WriteParam("number_cache_levels", std::to_string(2));
      WriteParam("interconnect_projection_type", std::to_string(0));
      WriteParam("device_type", std::to_string(0));
      WriteParam("longer_channel_device", std::to_string(0));
      WriteParam("power_gating", std::to_string(0));
      WriteParam("machine_bits", std::to_string(64));
      WriteParam("virtual_address_width", std::to_string(64));
      WriteParam("physical_address_width", std::to_string(52));
      WriteParam("virtual_memory_page_size", std::to_string(4096));

      WriteStat("total_cycles", std::to_string(Input.CycleCount));
      WriteStat("idle_cycles", std::to_string(Input.IdleCycles));
      WriteStat("busy_cycles", std::to_string(Input.BusyCycles));

      WriteComponentStart("system.core0", "core0");

      { // Start system.core0
        WriteParam("clock_rate",
                   std::to_string(Input.ClockRateHz * 1.0e-6)); // Hz -> MHz

        WriteParam("opt_local", std::to_string(1));
        WriteParam("instruction_length", std::to_string(32));
        WriteParam("opcode_width", std::to_string(7));
        WriteParam("x86", std::to_string(0));
        WriteParam("micro_opcode_width", std::to_string(8));
        WriteParam("machine_type", std::to_string(0));
        WriteParam("number_hardware_threads", std::to_string(1));
        WriteParam("fetch_width", std::to_string(4));
        WriteParam("number_instruction_fetch_ports", std::to_string(1));
        WriteParam("decode_width", std::to_string(4));
        WriteParam("issue_width", std::to_string(4));
        WriteParam("peak_issue_width", std::to_string(6));
        WriteParam("commit_width", std::to_string(4));
        WriteParam("fp_issue_width", std::to_string(2));
        WriteParam("prediction_width", std::to_string(1));
        WriteParam("pipelines_per_core", "1,1");
        WriteParam("pipeline_depth", "7,7");
        WriteParam("ALU_per_core", std::to_string(4));
        WriteParam("MUL_per_core", std::to_string(0));
        WriteParam("FPU_per_core", std::to_string(1));
        WriteParam("instruction_buffer_size", std::to_string(32));
        WriteParam("decoded_stream_buffer_size", std::to_string(16));
        WriteParam("instruction_window_scheme", std::to_string(0));
        WriteParam("instruction_window_size", std::to_string(20));
        WriteParam("fp_instruction_window_size", std::to_string(15));
        WriteParam("ROB_size", std::to_string(80));
        WriteParam("archi_Regs_IRF_size", std::to_string(32));
        WriteParam("archi_Regs_FRF_size", std::to_string(32));
        WriteParam("phy_Regs_IRF_size", std::to_string(80));
        WriteParam("phy_Regs_FRF_size", std::to_string(72));
        WriteParam("rename_scheme", std::to_string(1));
        WriteParam("register_windows_size", std::to_string(0));
        WriteParam("LSU_order", "inorder");
        WriteParam("store_buffer_size", std::to_string(32));
        WriteParam("load_buffer_size", std::to_string(32));
        WriteParam("memory_ports", std::to_string(2));
        WriteParam("RAS_size", std::to_string(32));

        WriteStat("total_instructions", std::to_string(Input.InstrCount));
        WriteStat("int_instructions", std::to_string(Input.IntInstrCount));
        WriteStat("fp_instructions", std::to_string(Input.FloatInstrCount));
        WriteStat("branch_instructions",
                  std::to_string(Input.BranchInstrCount));
        WriteStat("branch_mispredictions",
                  std::to_string(Input.BranchMispredictions));
        WriteStat("load_instructions", std::to_string(Input.Loads));
        WriteStat("store_instructions", std::to_string(Input.Stores));
        WriteStat("committed_instructions", std::to_string(Input.InstrCount));
        WriteStat("committed_int_instructions",
                  std::to_string(Input.IntInstrCount));
        WriteStat("committed_fp_instructions",
                  std::to_string(Input.FloatInstrCount));
        WriteStat("pipeline_duty_cycle", std::to_string(1));

        WriteStat("total_cycles", std::to_string(Input.CycleCount));
        WriteStat("idle_cycles", std::to_string(Input.IdleCycles));
        WriteStat("busy_cycles", std::to_string(Input.BusyCycles));

        WriteStat("ROB_reads", std::to_string(Input.ROBReads));
        WriteStat("ROB_writes", std::to_string(Input.ROBWrites));

        WriteStat("rename_reads", std::to_string(Input.RenameReads));
        WriteStat("rename_writes", std::to_string(Input.RenameWrites));
        WriteStat("fp_rename_reads", std::to_string(Input.FpRenameReads));
        WriteStat("fp_rename_writes", std::to_string(Input.FpRenameWrites));

        WriteStat("inst_window_reads", std::to_string(Input.InstWindowReads));
        WriteStat("inst_window_writes", std::to_string(Input.InstWindowWrites));
        WriteStat("inst_window_wakeup_accesses",
                  std::to_string(Input.InstWindowWakeupAccesses));
        WriteStat("fp_inst_window_reads",
                  std::to_string(Input.FpInstWindowReads));
        WriteStat("fp_inst_window_writes",
                  std::to_string(Input.FpInstWindowWrites));
        WriteStat("fp_inst_window_wakeup_accesses",
                  std::to_string(Input.FpInstWindowWakeupAccesses));

        WriteStat("int_regfile_reads", std::to_string(Input.IntRegfileReads));
        WriteStat("float_regfile_reads",
                  std::to_string(Input.FloatRegfileReads));
        WriteStat("int_regfile_writes", std::to_string(Input.IntRegfileWrites));
        WriteStat("float_regfile_writes",
                  std::to_string(Input.FloatRegfileWrites));

        WriteStat("function_calls", std::to_string(Input.FunctionCalls));
        WriteStat("context_switches", std::to_string(Input.ContextSwitches));

        WriteStat("ialu_accesses", std::to_string(Input.IAluAccess));
        WriteStat("fpu_accesses", std::to_string(Input.FpuAccess));
        WriteStat("mul_accesses", std::to_string(Input.MulAccess));
        WriteStat("cdb_alu_accesses", std::to_string(Input.CdbALUAccess));
        WriteStat("cdb_mul_accesses", std::to_string(Input.MulAccess));
        WriteStat("cdb_fpu_accesses", std::to_string(Input.FpuAccess));

        WriteStat("IFU_duty_cycle", std::to_string(0.5));
        WriteStat("LSU_duty_cycle", std::to_string(0.5));
        WriteStat("MemManU_I_duty_cycle", std::to_string(0.5));
        WriteStat("MemManU_D_duty_cycle", std::to_string(0.5));
        WriteStat("ALU_duty_cycle", std::to_string(1));
        WriteStat("MUL_duty_cycle", std::to_string(0.3));
        WriteStat("FPU_duty_cycle", std::to_string(1));
        WriteStat("ALU_cdb_duty_cycle", std::to_string(1));
        WriteStat("MUL_cdb_duty_cycle", std::to_string(0.3));
        WriteStat("FPU_cdb_duty_cycle", std::to_string(1));
        WriteParam("number_of_BPT", std::to_string(2));

        WriteComponentStart("system.core0.predictor", "PBT");

        {
          WriteParam("local_predictor_size", "10, 3");
          WriteParam("local_predictor_entries", std::to_string(1024));
          WriteParam("global_predictor_entries", std::to_string(4096));
          WriteParam("global_predictor_bits", std::to_string(2));
          WriteParam("chooser_predictor_entries", std::to_string(4096));
          WriteParam("chooser_predictor_bits", std::to_string(2));
        }

        WriteComponentEnd();

        WriteComponentStart("system.core0.itlb", "itlb");

        {
          WriteParam("number_entries", std::to_string(128));
          WriteStat("total_accesses", std::to_string(Input.ItlbAccess));
          WriteStat("total_misses", std::to_string(4));
          WriteStat("conflicts", std::to_string(0));
        }

        WriteComponentEnd();

        WriteComponentStart("system.core0.icache", "icache");

        {
          WriteParam("icache_config", "65536,16,2,1,1,2,16,0");
          WriteParam("buffer_sizes", "16,16,16,0");

          WriteStat("read_accesses", std::to_string(Input.ItlbReads));
          WriteStat("read_misses", std::to_string(0));
          WriteStat("conflicts", std::to_string(0));
        }

        WriteComponentEnd();

        WriteComponentStart("system.core0.dtlb", "dtlb");

        {
          WriteParam("number_entries", std::to_string(128));
          WriteStat("total_accesses", std::to_string(Input.DtlbAccess));
          WriteStat("total_misses", std::to_string(0));
          WriteStat("conflicts", std::to_string(0));
        }

        WriteComponentEnd();

        WriteComponentStart("system.core0.dcache", "dcache");

        {
          WriteParam("dcache_config", "65536, 16, 2, 1, 1, 3, 16, 0");
          WriteParam("buffer_sizes", "16, 16, 16, 16");

          WriteStat("read_accesses", std::to_string(Input.DtlbReads));
          WriteStat("write_accesses", std::to_string(Input.DtlbWrites));
          WriteStat("read_misses", std::to_string(0));
          WriteStat("write_misses", std::to_string(0));
          WriteStat("conflicts", std::to_string(0));
        }

        WriteComponentEnd();

        WriteParam("number_of_BTB", std::to_string(2));

        WriteComponentStart("system.core0.BTB", "BTB");

        {
          WriteParam("BTB_config", "6144, 4, 2, 1, 1, 3");

          WriteStat("read_accesses", std::to_string(Input.BtbReads));
          WriteStat("write_accesses", std::to_string(Input.BtbWrites));
        }

        WriteComponentEnd();

        WriteParam("vdd", std::to_string(Input.Voltage));

      } // End system.core0

      WriteComponentEnd();

      WriteComponentStart("system.L1Directory0", "L1Directory0");

      {
        WriteParam("Directory_type", std::to_string(0));
        WriteParam("Dir_config", "4096, 2, 0, 1, 100, 100, 8");
        WriteParam("buffer_sizes", "8, 8, 8, 8");
        WriteParam("clockrate", std::to_string(Input.ClockRateHz * 1.0e-6));
        WriteParam("ports", "1, 1, 1");
        WriteParam("device_type", std::to_string(0));

        WriteStat("read_accesses", std::to_string(Input.InstrCount * 2));
        WriteStat("write_accesses", std::to_string(0));
        WriteStat("read_misses", std::to_string(0));
        WriteStat("write_misses", std::to_string(0));
        WriteStat("conflicts", std::to_string(0));
      }

      WriteComponentEnd();

      WriteComponentStart("system.L2Directory0", "L2Directory0");

      {
        WriteParam("Directory_type", std::to_string(0));

        WriteParam("Dir_config", "512, 4, 0, 1, 1, 1");
        WriteParam("buffer_sizes", "16, 16, 16, 16");

        WriteParam("clockrate", std::to_string(Input.ClockRateHz * 1.0e-6));
        WriteParam("ports", "1, 1, 1");

        WriteParam("device_type", std::to_string(0));

        WriteStat("read_accesses", std::to_string(0));
        WriteStat("write_accesses", std::to_string(0));
        WriteStat("read_misses", std::to_string(0));
        WriteStat("write_misses", std::to_string(0));
        WriteStat("conflicts", std::to_string(100));
      }

      WriteComponentEnd();

      WriteComponentStart("system.L20", "L20");

      {
        WriteParam("L2_config", "1835008,16, 8, 16, 32, 32, 12, 1");

        WriteParam("buffer_sizes", "16, 16, 16, 16");

        WriteParam("clockrate", std::to_string(Input.ClockRateHz * 1.0e-6));
        WriteParam("ports", "1, 1, 1");

        WriteParam("device_type", std::to_string(0));
        WriteStat("read_accesses", std::to_string(0));
        WriteStat("write_accesses", std::to_string(0));
        WriteStat("read_misses", std::to_string(0));
        WriteStat("write_misses", std::to_string(0));
        WriteStat("conflicts", std::to_string(0));
        WriteStat("duty_cycle", std::to_string(1.0));
      }

      WriteComponentEnd();

      WriteComponentStart("system.L30", "L30");

      {
        WriteParam("L3_config", "16777216,64,16, 16, 16, 100,1");

        WriteParam("clockrate", std::to_string(850));
        WriteParam("ports", "1, 1, 1");

        WriteParam("device_type", std::to_string(0));
        WriteParam("buffer_sizes", "16, 16, 16, 16");

        WriteStat("read_accesses", std::to_string(0));
        WriteStat("write_accesses", std::to_string(0));
        WriteStat("read_misses", std::to_string(0));
        WriteStat("write_misses", std::to_string(0));
        WriteStat("conflicts", std::to_string(0));
        WriteStat("duty_cycle", std::to_string(1.0));
      }

      WriteComponentEnd();

      WriteComponentStart("system.NoC0", "noc0");

      {
        WriteParam("clockrate", std::to_string(Input.ClockRateHz * 1.0e-6));
        WriteParam("type", std::to_string(1));

        WriteParam("horizontal_nodes", std::to_string(1));
        WriteParam("vertical_nodes", std::to_string(1));
        WriteParam("has_global_link", std::to_string(1));

        WriteParam("link_throughput", std::to_string(1));
        WriteParam("link_latency", std::to_string(1));

        WriteParam("input_ports", std::to_string(8));
        WriteParam("output_ports", std::to_string(7));

        WriteParam("virtual_channel_per_port", std::to_string(2));
        WriteParam("input_buffer_entries_per_vc", std::to_string(128));
        WriteParam("flit_bits", std::to_string(40));
        WriteParam("chip_coverage", std::to_string(1));

        WriteParam("link_routing_over_percentage", std::to_string(1.0));

        WriteStat("total_accesses", std::to_string(Input.InstrCount / 4));

        WriteStat("duty_cycle", std::to_string(1));
      }
      WriteComponentEnd();

      WriteComponentStart("system.mc", "mc");

      {
        WriteParam("type", std::to_string(0));
        WriteParam("mc_clock", std::to_string(800));
        WriteParam("peak_transfer_rate", std::to_string(1600));
        WriteParam("block_size", std::to_string(16));
        WriteParam("number_mcs", std::to_string(2));

        WriteParam("memory_channels_per_mc", std::to_string(2));
        WriteParam("number_ranks", std::to_string(2));
        WriteParam("withPHY", std::to_string(0));

        WriteParam("req_window_size_per_channel", std::to_string(32));
        WriteParam("IO_buffer_size_per_channel", std::to_string(32));
        WriteParam("databus_width", std::to_string(32));
        WriteParam("addressbus_width", std::to_string(32));

        WriteStat("memory_accesses", std::to_string(Input.InstrCount / 10));
        WriteStat("memory_reads", std::to_string(Input.InstrCount / 20));
        WriteStat("memory_writes", std::to_string(Input.InstrCount / 20));
      }

      WriteComponentEnd();

      WriteComponentStart("system.niu", "niu");

      {
        WriteParam("type", std::to_string(0));
        WriteParam("clockrate", std::to_string(350));
        WriteParam("number_units", std::to_string(0));
        WriteStat("duty_cycle", std::to_string(1.0));
        WriteStat("total_load_perc", std::to_string(0.7));
      }

      WriteComponentEnd();

      WriteComponentStart("system.pcie", "pcie");

      {
        WriteParam("type", std::to_string(0));
        WriteParam("withPHY", std::to_string(1));
        WriteParam("clockrate", std::to_string(350));
        WriteParam("number_units", std::to_string(0));
        WriteParam("num_channels", std::to_string(8));
        WriteStat("duty_cycle", std::to_string(1.0));
        WriteStat("total_load_perc", std::to_string(0.7));
      }

      WriteComponentEnd();

      WriteComponentStart("system.flashc", "flashc");

      {
        WriteParam("number_flashcs", std::to_string(0));
        WriteParam("type", std::to_string(1));
        WriteParam("withPHY", std::to_string(1));
        WriteParam("peak_transfer_rate", std::to_string(200));
        WriteStat("duty_cycle", std::to_string(1.0));
        WriteStat("total_load_perc", std::to_string(0.7));
      }

      WriteComponentEnd();

    } // End system

    WriteComponentEnd();
  } // End root

  WriteComponentEnd();
}

bool extIsProbablyFloatingInstruction(const MachineInstr &MI,
                                      const TargetInstrInfo *TII) {
  // this approach sucks, but I cant access x86 directly (build dependency
  // issues?), so this is best I can do
  const MCInstrDesc &Desc = MI.getDesc();
  StringRef Name = TII->getName(MI.getOpcode());

  // note regular flags don't give us much information, so need to use
  // target-specific
  // TODO: maybe we can check the flag "MayRaiseFPException" (regular flag,
  // not target-specific)
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
      "ADD",    "ADC",   "SUB",   "SBB",    "MUL",  "IMUL",  "MULX", "DIV",
      "IDIV",   "INC",   "DEC",   "NEG",    "AND",  "OR",    "XOR",  "NOT",
      "SHL",    "SAL",   "SHR",   "SAR",    "ROL",  "ROR",   "RCL",  "RCR",
      "MOV",    "MOVSX", "MOVZX", "MOVSXD", "XCHG", "CMP",   "TEST", "LEA",
      "SET",    "CMOV",  "BSF",   "BSR",    "BT",   "BTS",   "BTR",  "BTC",
      "POPCNT", "LZCNT", "TZCNT", "BSWAP",  "ANDN", "BEXTR", "BLSI", "BLSMSK",
      "BLSR",   "PEXT",  "PDEP",  "SHLX",   "SHRX", "SARX"};

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
  static const char *IALUPrefixes[] = {
      "ADD",  "ADC",    "SUB",   "SBB",   "MUL",   "IMUL", "MULX", "DIV",
      "IDIV", "INC",    "DEC",   "NEG",   "AND",   "OR",   "XOR",  "NOT",
      "SHL",  "SAL",    "SHR",   "SAR",   "ROL",   "ROR",  "RCL",  "RCR",
      "CMP",  "TEST",   "LEA",   "BSF",   "BSR",   "BT",   "BTS",  "BTR",
      "BTC",  "POPCNT", "LZCNT", "TZCNT", "BSWAP", "CMOV"};

  for (const char *Prefix : IALUPrefixes) {
    if (N.starts_with_insensitive(StringRef(Prefix))) {
      return true;
    }
  }

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

    // this is the entry block, record entry block ID for this machine
    // function
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

      // TODO: change to classify int/float instructions based on registers?
      // or use a combination of the approaches currently extIsProbablyFPU
      // classifies off different set of prefixes/instruction opcodes compared
      // to extIsPorbablyFloatingInstruction
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

          // LLVM_DEBUG(dbgs() << "Detected load in MI: " << Name << " for BB
          // "
          // << BlockStat.Name << "\n");
        }

        if (MMO->isStore()) {
          const MCInstrDesc &Desc = MI.getDesc();
          StringRef Name = TII->getName(MI.getOpcode());

          // LLVM_DEBUG(dbgs() << "Detected store in MI: " << Name << " for BB
          // "
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

    // TODO: don't need this code anymore, we put in the profile count into
    // the code itself
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

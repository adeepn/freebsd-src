//===-- llvm-tli-checker.cpp - Compare TargetLibraryInfo to SDK libraries -===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/ADT/SmallString.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/Triple.h"
#include "llvm/Analysis/TargetLibraryInfo.h"
#include "llvm/Config/llvm-config.h"
#include "llvm/Demangle/Demangle.h"
#include "llvm/Object/Archive.h"
#include "llvm/Object/ELFObjectFile.h"
#include "llvm/Option/ArgList.h"
#include "llvm/Option/Option.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/InitLLVM.h"
#include "llvm/Support/Path.h"
#include "llvm/Support/WithColor.h"

using namespace llvm;
using namespace llvm::object;

// Command-line option boilerplate.
namespace {
enum ID {
  OPT_INVALID = 0, // This is not an option ID.
#define OPTION(PREFIX, NAME, ID, KIND, GROUP, ALIAS, ALIASARGS, FLAGS, PARAM,  \
               HELPTEXT, METAVAR, VALUES)                                      \
  OPT_##ID,
#include "Opts.inc"
#undef OPTION
};

#define PREFIX(NAME, VALUE) const char *const NAME[] = VALUE;
#include "Opts.inc"
#undef PREFIX

const opt::OptTable::Info InfoTable[] = {
#define OPTION(PREFIX, NAME, ID, KIND, GROUP, ALIAS, ALIASARGS, FLAGS, PARAM,  \
               HELPTEXT, METAVAR, VALUES)                                      \
  {                                                                            \
      PREFIX,      NAME,      HELPTEXT,                                        \
      METAVAR,     OPT_##ID,  opt::Option::KIND##Class,                        \
      PARAM,       FLAGS,     OPT_##GROUP,                                     \
      OPT_##ALIAS, ALIASARGS, VALUES},
#include "Opts.inc"
#undef OPTION
};

class TLICheckerOptTable : public opt::OptTable {
public:
  TLICheckerOptTable() : OptTable(InfoTable) {}
};
} // namespace

// We have three levels of reporting.
enum class ReportKind {
  Error,       // For argument parsing errors.
  Summary,     // Report counts but not details.
  Discrepancy, // Report where TLI and the library differ.
  Full         // Report for every known-to-TLI function.
};

// Most of the ObjectFile interfaces return an Expected<T>, so make it easy
// to ignore those.
template <typename T> T unwrapIgnoreError(Expected<T> E) {
  if (E)
    return std::move(*E);
  // Sink the error and return a nothing value.
  consumeError(E.takeError());
  return T();
}

static void fail(const Twine &Message) {
  WithColor::error() << Message << '\n';
  exit(EXIT_FAILURE);
}

// Some problem occurred with an archive member; complain and continue.
static void reportArchiveChildIssue(const object::Archive::Child &C, int Index,
                                    StringRef ArchiveFilename) {
  // First get the member name.
  std::string ChildName;
  Expected<StringRef> NameOrErr = C.getName();
  if (NameOrErr)
    ChildName = std::string(NameOrErr.get());
  else {
    // Ignore the name-fetch error, just report the index.
    consumeError(NameOrErr.takeError());
    ChildName = "<file index: " + std::to_string(Index) + ">";
  }

  WithColor::warning() << ArchiveFilename << "(" << ChildName
                       << "): member is not usable\n";
}

// Return Name, and if Name is mangled, append "aka" and the demangled name.
static std::string PrintableName(StringRef Name) {
  std::string OutputName = "'";
  OutputName += Name;
  OutputName += "'";
  if (Name.startswith("_Z") || Name.startswith("??")) {
    OutputName += " aka ";
    OutputName += demangle(Name.str());
  }
  return OutputName;
}

// Store all the names that TargetLibraryInfo knows about; the bool indicates
// whether TLI has it marked as "available" for the target of interest.
// This is a vector to preserve the sorted order for better reporting.
struct TLINameList : std::vector<std::pair<StringRef, bool>> {
  // Record all the TLI info in the vector.
  void initialize(StringRef TargetTriple);
  // Print out what we found.
  void dump();
};
TLINameList TLINames;

void TLINameList::initialize(StringRef TargetTriple) {
  Triple T(TargetTriple);
  TargetLibraryInfoImpl TLII(T);
  TargetLibraryInfo TLI(TLII);

  reserve(LibFunc::NumLibFuncs);
  size_t NumAvailable = 0;
  for (unsigned FI = 0; FI != LibFunc::NumLibFuncs; ++FI) {
    LibFunc LF = (LibFunc)FI;
    bool Available = TLI.has(LF);
    // getName returns names only for available funcs.
    TLII.setAvailable(LF);
    emplace_back(TLI.getName(LF), Available);
    if (Available)
      ++NumAvailable;
  }
  outs() << "TLI knows " << LibFunc::NumLibFuncs << " symbols, " << NumAvailable
         << " available for '" << TargetTriple << "'\n";
}

void TLINameList::dump() {
  // Assume this gets called after initialize(), so we have the above line of
  // output as a header.  So, for example, no need to repeat the triple.
  for (auto &TLIName : TLINames) {
    outs() << (TLIName.second ? "    " : "not ")
           << "available: " << PrintableName(TLIName.first) << '\n';
  }
}

// Store all the exported symbol names we found in the input libraries.
// We use a map to get hashed lookup speed; the bool is meaningless.
class SDKNameMap : public StringMap<bool> {
  void populateFromObject(ObjectFile *O);
  void populateFromArchive(Archive *A);

public:
  void populateFromFile(StringRef LibDir, StringRef LibName);
};
SDKNameMap SDKNames;

// Given an ObjectFile, extract the global function symbols.
void SDKNameMap::populateFromObject(ObjectFile *O) {
  // FIXME: Support COFF.
  if (!O->isELF()) {
    WithColor::warning() << "Only ELF-format files are supported\n";
    return;
  }
  auto *ELF = cast<const ELFObjectFileBase>(O);

  for (auto I = ELF->getDynamicSymbolIterators().begin();
       I != ELF->getDynamicSymbolIterators().end(); ++I) {
    // We want only global function symbols.
    SymbolRef::Type Type = unwrapIgnoreError(I->getType());
    uint32_t Flags = unwrapIgnoreError(I->getFlags());
    StringRef Name = unwrapIgnoreError(I->getName());
    if (Type == SymbolRef::ST_Function && (Flags & SymbolRef::SF_Global))
      insert({Name, true});
  }
}

// Unpack an archive and populate from the component object files.
// This roughly imitates dumpArchive() from llvm-objdump.cpp.
void SDKNameMap::populateFromArchive(Archive *A) {
  Error Err = Error::success();
  int Index = -1;
  for (auto &C : A->children(Err)) {
    ++Index;
    Expected<std::unique_ptr<object::Binary>> ChildOrErr = C.getAsBinary();
    if (!ChildOrErr) {
      if (auto E = isNotObjectErrorInvalidFileType(ChildOrErr.takeError())) {
        // Issue a generic warning.
        consumeError(std::move(E));
        reportArchiveChildIssue(C, Index, A->getFileName());
      }
      continue;
    }
    if (ObjectFile *O = dyn_cast<ObjectFile>(&*ChildOrErr.get()))
      populateFromObject(O);
    // Ignore non-object archive members.
  }
  if (Err)
    WithColor::defaultErrorHandler(std::move(Err));
}

// Unpack a library file and extract the global function names.
void SDKNameMap::populateFromFile(StringRef LibDir, StringRef LibName) {
  // Pick an arbitrary but reasonable default size.
  SmallString<255> Filepath(LibDir);
  sys::path::append(Filepath, LibName);
  if (!sys::fs::exists(Filepath)) {
    WithColor::warning() << "Could not find '" << StringRef(Filepath) << "'\n";
    return;
  }
  outs() << "\nLooking for symbols in '" << StringRef(Filepath) << "'\n";
  auto ExpectedBinary = createBinary(Filepath);
  if (!ExpectedBinary) {
    // FIXME: Report this better.
    WithColor::defaultWarningHandler(ExpectedBinary.takeError());
    return;
  }
  OwningBinary<Binary> OBinary = std::move(*ExpectedBinary);
  Binary &Binary = *OBinary.getBinary();
  size_t Precount = size();
  if (Archive *A = dyn_cast<Archive>(&Binary))
    populateFromArchive(A);
  else if (ObjectFile *O = dyn_cast<ObjectFile>(&Binary))
    populateFromObject(O);
  else {
    WithColor::warning() << "Not an Archive or ObjectFile: '"
                         << StringRef(Filepath) << "'\n";
    return;
  }
  if (Precount == size())
    WithColor::warning() << "No symbols found in '" << StringRef(Filepath)
                         << "'\n";
  else
    outs() << "Found " << size() - Precount << " global function symbols in '"
           << StringRef(Filepath) << "'\n";
}

int main(int argc, char *argv[]) {
  InitLLVM X(argc, argv);
  BumpPtrAllocator A;
  StringSaver Saver(A);
  TLICheckerOptTable Tbl;
  opt::InputArgList Args = Tbl.parseArgs(argc, argv, OPT_UNKNOWN, Saver,
                                         [&](StringRef Msg) { fail(Msg); });

  if (Args.hasArg(OPT_help)) {
    std::string Usage(argv[0]);
    Usage += " [options] library-file [library-file...]";
    Tbl.printHelp(outs(), Usage.c_str(),
                  "LLVM TargetLibraryInfo versus SDK checker");
    outs() << "\nPass @FILE as argument to read options or library names from "
              "FILE.\n";
    return 0;
  }

  TLINames.initialize(Args.getLastArgValue(OPT_triple_EQ));

  // --dump-tli doesn't require any input files.
  if (Args.hasArg(OPT_dump_tli)) {
    TLINames.dump();
    return 0;
  }

  std::vector<std::string> LibList = Args.getAllArgValues(OPT_INPUT);
  if (LibList.empty()) {
    WithColor::error() << "No input files\n";
    exit(EXIT_FAILURE);
  }
  StringRef LibDir = Args.getLastArgValue(OPT_libdir_EQ);
  bool SeparateMode = Args.hasArg(OPT_separate);

  ReportKind ReportLevel =
      SeparateMode ? ReportKind::Summary : ReportKind::Discrepancy;
  if (const opt::Arg *A = Args.getLastArg(OPT_report_EQ)) {
    ReportLevel = StringSwitch<ReportKind>(A->getValue())
                      .Case("summary", ReportKind::Summary)
                      .Case("discrepancy", ReportKind::Discrepancy)
                      .Case("full", ReportKind::Full)
                      .Default(ReportKind::Error);
    if (ReportLevel == ReportKind::Error) {
      WithColor::error() << "invalid option for --report: " << A->getValue();
      exit(EXIT_FAILURE);
    }
  }

  for (size_t I = 0; I < LibList.size(); ++I) {
    // In SeparateMode we report on input libraries individually; otherwise
    // we do one big combined search.  Reading to the end of LibList here
    // will cause the outer while loop to terminate cleanly.
    if (SeparateMode) {
      SDKNames.clear();
      SDKNames.populateFromFile(LibDir, LibList[I]);
      if (SDKNames.empty())
        continue;
    } else {
      do
        SDKNames.populateFromFile(LibDir, LibList[I]);
      while (++I < LibList.size());
      if (SDKNames.empty()) {
        WithColor::error() << "NO symbols found!\n";
        break;
      }
      outs() << "Found a grand total of " << SDKNames.size()
             << " library symbols\n";
    }
    unsigned TLIdoesSDKdoesnt = 0;
    unsigned TLIdoesntSDKdoes = 0;
    unsigned TLIandSDKboth = 0;
    unsigned TLIandSDKneither = 0;
    for (auto &TLIName : TLINames) {
      bool TLIHas = TLIName.second;
      bool SDKHas = SDKNames.count(TLIName.first) == 1;
      int Which = int(TLIHas) * 2 + int(SDKHas);
      switch (Which) {
      case 0: ++TLIandSDKneither; break;
      case 1: ++TLIdoesntSDKdoes; break;
      case 2: ++TLIdoesSDKdoesnt; break;
      case 3: ++TLIandSDKboth;    break;
      }
      // If the results match, report only if user requested a full report.
      ReportKind Threshold =
          TLIHas == SDKHas ? ReportKind::Full : ReportKind::Discrepancy;
      if (Threshold <= ReportLevel) {
        constexpr char YesNo[2][4] = {"no ", "yes"};
        constexpr char Indicator[4][3] = {"!!", ">>", "<<", "=="};
        outs() << Indicator[Which] << " TLI " << YesNo[TLIHas] << " SDK "
               << YesNo[SDKHas] << ": " << PrintableName(TLIName.first) << '\n';
      }
    }

    assert(TLIandSDKboth + TLIandSDKneither + TLIdoesSDKdoesnt +
               TLIdoesntSDKdoes ==
           LibFunc::NumLibFuncs);
    outs() << "<< Total TLI yes SDK no:  " << TLIdoesSDKdoesnt
           << "\n>> Total TLI no  SDK yes: " << TLIdoesntSDKdoes
           << "\n== Total TLI yes SDK yes: " << TLIandSDKboth;
    if (TLIandSDKboth == 0) {
      outs() << " *** NO TLI SYMBOLS FOUND";
      if (SeparateMode)
        outs() << " in '" << LibList[I] << "'";
    }
    outs() << '\n';

    if (!SeparateMode) {
      if (TLIdoesSDKdoesnt == 0 && TLIdoesntSDKdoes == 0)
        outs() << "PASS: LLVM TLI matched SDK libraries successfully.\n";
      else
        outs() << "FAIL: LLVM TLI doesn't match SDK libraries.\n";
    }
  }
}

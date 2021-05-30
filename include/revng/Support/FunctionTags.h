#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <set>

#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/IR/Module.h"

namespace llvm {
class MDNode;
class User;
class Module;
} // namespace llvm

namespace FunctionTags {

class Tag;

class TagsSet {
private:
  std::set<const Tag *> Tags;

public:
  TagsSet() {}
  TagsSet(std::initializer_list<const Tag *> I) : Tags{ I } {}

public:
  static TagsSet from(const llvm::Instruction *I);
  static TagsSet from(const llvm::GlobalObject *G);
  static TagsSet from(const llvm::MDNode *MD);

public:
  bool contains(const Tag &T) const { return Tags.count(&T) != 0; }

public:
  template<typename T>
  void addTo(T *I) const;

public:
  void insert(const Tag *T) { Tags.insert(T); }
};

class Tag {
private:
  llvm::StringRef Name;

public:
  Tag(llvm::StringRef Name);

public:
  void addTo(llvm::Instruction *I) const;
  void addTo(llvm::GlobalObject *G) const;

public:
  template<typename T>
  bool isTagOf(T *I) const {
    return TagsSet::from(I).contains(*this);
  }

  auto functions(llvm::Module *M) const {
    using namespace llvm;
    auto Filter = [this](Function &F) { return isTagOf(&F); };
    return make_filter_range(M->functions(), Filter);
  }

  auto globals(llvm::Module *M) const {
    using namespace llvm;
    auto Filter = [this](GlobalVariable &G) { return isTagOf(&G); };
    return make_filter_range(M->globals(), Filter);
  }
};

template<typename T>
void TagsSet::addTo(T *I) const {
  // TODO: inefficient
  for (const Tag *TheTag : Tags)
    TheTag->addTo(I);
}

extern Tag QEMU;
extern Tag Helper;
extern Tag Lifted;
extern Tag Exceptional;
extern Tag StructInitializer;
extern Tag OpaqueCSVValue;
extern Tag FunctionDispatcher;
extern Tag Root;
extern Tag CSVsAsArgumentsWrapper;
extern Tag Marker;

} // namespace FunctionTags

inline bool isRootOrLifted(const llvm::Function *F) {
  auto Tags = FunctionTags::TagsSet::from(F);
  return Tags.contains(FunctionTags::Root)
         or Tags.contains(FunctionTags::Lifted);
}

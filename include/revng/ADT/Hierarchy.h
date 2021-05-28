#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <iostream>
#include <vector>

#include "llvm/ADT/StringRef.h"
#include "llvm/Support/ManagedStatic.h"

using entry_t = size_t;

template<typename T>
class HierarchyNode {
public:
  HierarchyNode(llvm::StringRef Name, T Value = T()) :
    Value(std::move(Value)), Name(Name.str()) {}
  HierarchyNode(llvm::StringRef Name, HierarchyNode &Parent, T Value = T()) :
    Value(std::move(Value)), Name(Name.str()) {
    Parent.Children->push_back(this);
  }

  HierarchyNode(HierarchyNode &&) = delete;
  HierarchyNode(const HierarchyNode &) = delete;
  HierarchyNode &operator=(HierarchyNode &&) = delete;
  HierarchyNode &operator=(const HierarchyNode &) = delete;
  ~HierarchyNode() = default;

  bool isa(entry_t ID) const { return id() == ID; }

  const T &operator*() const { return Value; }
  T &operator*() { return Value; }

  const T &get() const { return Value; }
  T &get() { return Value; }

  bool ancestorOf(const HierarchyNode &MaybeChild) const {
    return ancestorOf(MaybeChild.id());
  }

public:
  bool ancestorOf(entry_t ID) const { return Start <= ID and ID <= End; }
  entry_t id() const { return Start; }

  entry_t assign(entry_t ID = -1) {

    Start = ++ID;

    for (HierarchyNode *Child : *Children) {
      ID = Child->assign(ID);
      Child->Parent = this;
    }

    End = ID;

    return ID;
  }

  HierarchyNode *getParent() { return Parent; }
  const HierarchyNode *getParent() const { return Parent; }

  HierarchyNode *getRootAncestor() {
    HierarchyNode *LastAncestor = this;
    HierarchyNode *NextAncestor = Parent;
    while (NextAncestor != nullptr) {
      LastAncestor = NextAncestor;
      NextAncestor = NextAncestor->getParent();
    }
    return LastAncestor;
  }

  void dump(size_t Indent = 0) {
    for (size_t I = 0; I < Indent; ++I)
      std::cout << "  ";
    std::cout << "Start " << Start << "\n";

    for (HierarchyNode *Child : *Children) {
      Child->dump(Indent + 1);
    }

    for (size_t I = 0; I < Indent; ++I)
      std::cout << "  ";
    std::cout << "End " << End << "\n";
  }

  llvm::StringRef getName() const { return Name; }

  size_t depth() const {
    size_t ToReturn = 0;
    auto Current = Parent;
    while (Current != nullptr) {
      ToReturn++;
      Current = Current->Parent;
    }
    return ToReturn;
  }

private:
  llvm::ManagedStatic<std::vector<HierarchyNode *>> Children;
  HierarchyNode *Parent;
  entry_t Start;
  entry_t End;
  T Value;
  std::string Name;
};

/// \file Binary.cpp
/// \brief

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/Support/DOTGraphTraits.h"
#include "llvm/Support/GraphWriter.h"
#include "llvm/Support/raw_os_ostream.h"

#include "revng/ADT/GenericGraph.h"
#include "revng/Model/Binary.h"

using namespace llvm;

namespace model {

struct FunctionCFGNodeData {
  FunctionCFGNodeData(MetaAddress Start) : Start(Start) {}
  MetaAddress Start;
};

using FunctionCFGNode = ForwardNode<FunctionCFGNodeData>;

/// Graph data structure to represent the CFG for verification purposes
struct FunctionCFG : public GenericGraph<FunctionCFGNode> {
private:
  MetaAddress Entry;
  std::map<MetaAddress, FunctionCFGNode *> Map;

public:
  FunctionCFG(MetaAddress Entry) : Entry(Entry) {}

public:
  MetaAddress entry() const { return Entry; }
  FunctionCFGNode *entryNode() const { return Map.at(Entry); }

public:
  FunctionCFGNode *get(MetaAddress MA) {
    FunctionCFGNode *Result = nullptr;
    auto It = Map.find(MA);
    if (It == Map.end()) {
      Result = addNode(MA);
      Map[MA] = Result;
    } else {
      Result = It->second;
    }

    return Result;
  }

  bool allNodesAreReachable() const {
    if (Map.size() == 0)
      return true;

    // Ensure all the nodes are reachable from the entry node
    df_iterator_default_set<FunctionCFGNode *> Visited;
    for (auto &Ignore : depth_first_ext(entryNode(), Visited))
      ;
    return Visited.size() == size();
  }

  bool hasOnlyInvalidExits() const {
    for (auto &[Address, Node] : Map)
      if (Address.isValid() and not Node->hasSuccessors())
        return false;
    return true;
  }
};

bool Binary::verify() const {
  for (const Function &F : Functions) {

    // Verify individual functions
    if (not F.verify())
      return false;

    // Ensure all the direct function calls target an existing function
    for (const BasicBlock &Block : F.CFG) {
      for (const auto &Edge : Block.Successors) {
        if (Edge->Type == FunctionEdgeType::FunctionCall
            and Functions.count(Edge->Destination) == 0) {
          return false;
        }
      }
    }
  }

  return model::verifyTypeSystem(Types);
}

static FunctionCFG getGraph(const Function &F) {
  using namespace FunctionEdgeType;

  FunctionCFG Graph(F.Entry);
  for (const BasicBlock &Block : F.CFG) {
    auto *Source = Graph.get(Block.Start);

    for (const auto &Edge : Block.Successors) {
      switch (Edge->Type) {
      case DirectBranch:
      case FakeFunctionCall:
      case FakeFunctionReturn:
      case Return:
      case BrokenReturn:
      case IndirectTailCall:
      case LongJmp:
      case Unreachable:
        Source->addSuccessor(Graph.get(Edge->Destination));
        break;

      case FunctionCall:
      case IndirectCall:
        // TODO: this does not handle noreturn function calls
        Source->addSuccessor(Graph.get(Block.End));
        break;

      case Killer:
        Source->addSuccessor(Graph.get(MetaAddress::invalid()));
        break;

      case Invalid:
        revng_abort();
        break;
      }
    }
  }

  return Graph;
}

void Function::dumpCFG() const {
  FunctionCFG CFG = getGraph(*this);
  raw_os_ostream Stream(dbg);
  WriteGraph(Stream, &CFG);
}

bool Function::verify() const {
  if (Type == FunctionType::Fake)
    return CFG.size() == 0;

  // Verify blocks
  bool HasEntry = false;
  for (const BasicBlock &Block : CFG) {

    if (Block.Start == Entry) {
      if (HasEntry)
        return false;
      HasEntry = true;
    }

    for (const auto &Edge : Block.Successors)
      if (not Edge->verify())
        return false;
  }

  if (not HasEntry)
    return false;

  // Populate graph
  FunctionCFG Graph = getGraph(*this);

  // Ensure all the nodes are reachable from the entry node
  if (not Graph.allNodesAreReachable())
    return false;

  // Ensure the only node with no successors is invalid
  if (not Graph.hasOnlyInvalidExits())
    return false;

  return true;
}

bool FunctionEdge::verify() const {
  using namespace model::FunctionEdgeType;
  return Destination.isValid() == hasDestination(Type);
}

} // namespace model

template<>
struct llvm::DOTGraphTraits<model::FunctionCFG *>
  : public DefaultDOTGraphTraits {
  DOTGraphTraits(bool simple = false) : DefaultDOTGraphTraits(simple) {}

  static std::string
  getNodeLabel(const model::FunctionCFGNode *Node, const model::FunctionCFG *) {
    return Node->Start.toString();
  }

  static std::string getNodeAttributes(const model::FunctionCFGNode *Node,
                                       const model::FunctionCFG *Graph) {
    if (Node->Start == Graph->entry()) {
      return "shape=box,peripheries=2";
    }

    return "";
  }
};

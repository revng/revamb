/// \file GenericGraph.cpp
/// \brief Test the GenericGraph

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#define BOOST_TEST_MODULE GenericGraph
bool init_unit_test();
#include "boost/test/unit_test.hpp"

#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/SCCIterator.h"
#include "llvm/IR/Dominators.h"
#include "llvm/Support/GenericDomTreeConstruction.h"
#include "llvm/Support/GraphWriter.h"
#include "llvm/Support/raw_ostream.h"

#include "revng/ADT/FilteredGraphTraits.h"
#include "revng/ADT/GenericGraph.h"
#include "revng/ADT/SerializableGraph.h"

using namespace llvm;

BOOST_AUTO_TEST_CASE(TestCompile) {
  // Test only it compiles
  if constexpr (false) {
    {
      struct MyForwardNode {
        MyForwardNode(int) {}
        int m;
      };
      using NodeType = ForwardNode<MyForwardNode>;
      GenericGraph<NodeType> Graph;
      auto *Node = Graph.addNode(3);
      Node->addSuccessor(Node);
      Node->addSuccessor(Node, {});
      MyForwardNode *Neighbor = *Node->successors().begin();
      Node->removeSuccessor(Node->successors().begin());
      Node->removeSuccessorEdge(Node->successor_edges().begin());
    }

    {
      struct MyBidirectionalNode {
        MyBidirectionalNode(int) {}
        int m;
      };
      using NodeType = BidirectionalNode<MyBidirectionalNode>;
      GenericGraph<NodeType> Graph;
      auto *Node = Graph.addNode(3);
      Node->addSuccessor(Node);
      Node->addSuccessor(Node, {});
      Node->addPredecessor(Node);
      Node->addPredecessor(Node, {});
      MyBidirectionalNode *Neighbor = *Node->successors().begin();
      Neighbor = *Node->predecessors().begin();
      Node->removePredecessor(Node->predecessors().begin());
      Node->removePredecessorEdge(Node->predecessor_edges().begin());
    }

    struct EdgeLabel {
      int X;
    };

    {
      struct MyForwardNodeWithEdges {
        MyForwardNodeWithEdges(int) {}
        int m;
      };

      using NodeType = ForwardNode<MyForwardNodeWithEdges, EdgeLabel>;

      auto [A, B] = Edge<NodeType, EdgeLabel>{ nullptr };

      GenericGraph<NodeType> Graph;
      auto *Node = Graph.addNode(3);
      Node->addSuccessor(Node);
      Node->addSuccessor(Node, { 99 });
      MyForwardNodeWithEdges *Neighbor = *Node->successors().begin();
    }

    {
      struct MyBidirectionalNodeWithEdges {
        MyBidirectionalNodeWithEdges(int) {}
        int m;
      };
      using NodeType = BidirectionalNode<MyBidirectionalNodeWithEdges,
                                         EdgeLabel>;
      static_assert(std::is_same_v<NodeType::Base::DerivedType, NodeType>);
      GenericGraph<NodeType> Graph;
      auto *Node = Graph.addNode(3);
      Node->addSuccessor(Node);
      Node->addSuccessor(Node, { 99 });
      Node->addPredecessor(Node);
      Node->addPredecessor(Node, { 99 });
      NodeType *Neighbor = *Node->successors().begin();
      Neighbor = *Node->predecessors().begin();
      Graph.removeNode(Graph.nodes().begin());

      using NGT = GraphTraits<NodeType *>;
      NGT::child_begin(Node);
      auto It = NGT::child_edge_begin(Node);

      Neighbor = NGT::edge_dest(*It);

      using INGT = GraphTraits<Inverse<NodeType *>>;
      INGT::child_begin(Node);

      Graph.nodes();
    }
  }
}

//
// Define TestEdgeLabel
//
struct TestEdgeLabel {
  bool operator==(const TestEdgeLabel &) const = default;
  unsigned Weight;
};

INTROSPECTION(TestEdgeLabel, Weight);

template<>
struct llvm::yaml::MappingTraits<TestEdgeLabel>
  : public TupleLikeMappingTraits<TestEdgeLabel> {};

//
// Define TestNodeData
//
struct TestNodeData {
  TestNodeData(unsigned Rank) : Rank(Rank) {}
  bool operator==(const TestNodeData &) const = default;
  unsigned Rank;
};

INTROSPECTION(TestNodeData, Rank);

template<>
struct llvm::yaml::MappingTraits<TestNodeData>
  : public TupleLikeMappingTraits<TestNodeData> {};

template<>
struct KeyedObjectTraits<TestNodeData> {
  static unsigned key(const TestNodeData &Node) { return Node.Rank; }

  static TestNodeData fromKey(const unsigned &Key) { return { Key }; }
};

using TestNode = BidirectionalNode<TestNodeData, TestEdgeLabel>;
using TestGraph = GenericGraph<TestNode>;

SERIALIZABLEGRAPH_INTROSPECTION(TestNodeData, TestEdgeLabel);

static bool
shouldKeepNodePair(TestNode *const &Source, TestNode *const &Destination) {
  return Source->Rank + Destination->Rank <= 2;
}

static bool shouldKeepEdge(Edge<TestNode, TestEdgeLabel> &Edge) {
  return Edge.Weight > 5;
}

struct DiamondGraph {
  TestGraph Graph;
  TestNode *Root;
  TestNode *Then;
  TestNode *Else;
  TestNode *Final;
};

static DiamondGraph createGraph() {
  DiamondGraph DG;
  TestGraph &Graph = DG.Graph;

  // Create nodes
  DG.Root = Graph.addNode(0);
  DG.Then = Graph.addNode(1);
  DG.Else = Graph.addNode(3);
  DG.Final = Graph.addNode(2);

  // Set entry node
  Graph.setEntryNode(DG.Root);

  // Create edges
  DG.Root->addSuccessor(DG.Then, { 7 });
  DG.Root->addSuccessor(DG.Else, { 1 });

  DG.Then->addSuccessor(DG.Final, { 2 });
  DG.Else->addSuccessor(DG.Final, { 3 });

  return DG;
}

BOOST_AUTO_TEST_CASE(TestRPOT) {
  DiamondGraph DG = createGraph();
  ReversePostOrderTraversal<TestGraph *> RPOT(&DG.Graph);
  std::vector<TestNode *> Visited;
  for (TestNode *Node : RPOT)
    Visited.push_back(Node);
  revng_check(Visited.size() == 4);
}

BOOST_AUTO_TEST_CASE(TestDepthFirstVisit) {
  DiamondGraph DG = createGraph();
  std::vector<TestNode *> Visited;
  for (TestNode *Node : depth_first(&DG.Graph))
    Visited.push_back(Node);
  revng_check(Visited.size() == 4);

  Visited.clear();
  for (TestNode *Node : inverse_depth_first(DG.Final))
    Visited.push_back(Node);
  revng_check(Visited.size() == 4);
}

BOOST_AUTO_TEST_CASE(TestDominatorTree) {
  DiamondGraph DG = createGraph();

  DominatorTreeBase<TestNode, false> DT;
  DT.recalculate(DG.Graph);
  revng_check(DT.dominates(DT.getNode(DG.Root), DT.getNode(DG.Then)));
  revng_check(DT.dominates(DT.getNode(DG.Root), DT.getNode(DG.Else)));
  revng_check(DT.dominates(DT.getNode(DG.Root), DT.getNode(DG.Final)));
  revng_check(not DT.dominates(DT.getNode(DG.Then), DT.getNode(DG.Final)));

  DominatorTreeBase<TestNode, true> PDT;
  PDT.recalculate(DG.Graph);
  revng_check(PDT.dominates(PDT.getNode(DG.Final), PDT.getNode(DG.Then)));
  revng_check(PDT.dominates(PDT.getNode(DG.Final), PDT.getNode(DG.Else)));
  revng_check(PDT.dominates(PDT.getNode(DG.Final), PDT.getNode(DG.Root)));
  revng_check(not PDT.dominates(PDT.getNode(DG.Then), PDT.getNode(DG.Root)));
}

BOOST_AUTO_TEST_CASE(TestSCC) {
  DiamondGraph DG = createGraph();
  unsigned SCCCount = 0;
  for (const std::vector<TestNode *> &SCC :
       make_range(scc_begin(&DG.Graph), scc_end(&DG.Graph))) {
    revng_check(SCC.size() == 1);
    ++SCCCount;
  }
  revng_check(SCCCount == 4);
}

BOOST_AUTO_TEST_CASE(TestFilterGraphTraits) {
  DiamondGraph DG = createGraph();
  TestNode *Root = DG.Root;

  {
    using Pair = NodePairFilteredGraph<TestNode *, shouldKeepNodePair>;
    using FGT = GraphTraits<Pair>;
    using fdf_iterator = df_iterator<TestNode *,
                                     df_iterator_default_set<TestNode *>,
                                     false,
                                     FGT>;
    auto Begin = fdf_iterator::begin(Root);
    auto End = fdf_iterator::end(Root);
    revng_check(2 == std::distance(Begin, End));
  }

  {
    using EFGT = GraphTraits<EdgeFilteredGraph<TestNode *, shouldKeepEdge>>;
    using efdf_iterator = df_iterator<TestNode *,
                                      df_iterator_default_set<TestNode *>,
                                      false,
                                      EFGT>;
    auto Begin = efdf_iterator::begin(Root);
    auto End = efdf_iterator::end(Root);
    revng_check(2 == std::distance(Begin, End));
  }
}

BOOST_AUTO_TEST_CASE(TestWriteGraph) {
  DiamondGraph DG = createGraph();
  llvm::raw_null_ostream NullOutput;
  llvm::WriteGraph(NullOutput, &DG.Graph, "lol");
}

BOOST_AUTO_TEST_CASE(TestSerializableGraph) {
  DiamondGraph DG = createGraph();
  auto Serializable = toSerializable(DG.Graph);
  auto Reserializable = toSerializable(Serializable.toGenericGraph<TestNode>());
  revng_check(Reserializable == Serializable);
}

BOOST_AUTO_TEST_CASE(TestSerializeGraph) {
  DiamondGraph DG = createGraph();
  auto Serializable = toSerializable(DG.Graph);

  std::string Buffer;
  {
    llvm::raw_string_ostream Stream(Buffer);
    yaml::Output YAMLOutput(Stream);
    YAMLOutput << Serializable;
  }

  decltype(Serializable) Deserialized;
  llvm::yaml::Input YAMLInput(Buffer);
  YAMLInput >> Deserialized;

  revng_check(Deserialized == Serializable);
}

// MutableEdgeNode tests
struct SomeNode {
  std::string Text;
  SomeNode(std::string NewText) : Text(NewText) {}
};
struct SomeEdge {
  struct PointType {
    float X, Y;
  };
  template<typename... ArgTypes>
  SomeEdge(ArgTypes... Args) : Points{ Args... } {}
  std::vector<PointType> Points;
};

BOOST_AUTO_TEST_CASE(BasicMutableEdgeNodeTest) {
  using Graph = GenericGraph<MutableEdgeNode<SomeNode, SomeEdge>>;

  Graph G;
  auto &A = *G.addNode("A");
  auto &B = *G.addNode("B");

  A.addSuccessor(A, SomeEdge::PointType{ 1.0, 0.1 });
  A.addSuccessor(B, SomeEdge::PointType{ 1.0, 0.2 });
  B.addSuccessor(A, SomeEdge::PointType{ 1.0, 0.3 });

  revng_check(A.successorCount() == 2);
  revng_check(A.predecessorCount() == 2);
  revng_check(B.successorCount() == 1);
  revng_check(B.predecessorCount() == 1);

  for (auto *Node : G.nodes()) {
    revng_check(!Node->Text.empty());
    for (auto [Neighbor, Edge] : Node->successor_edges()) {
      revng_check(!Neighbor.Text.empty());
      for (auto &Point : Edge.Points)
        revng_check(Point.X == 1.0 && Point.Y < 0.4 && Point.Y > 0.0);
    }
  }
  for (auto *Node : G.nodes()) {
    revng_check(!Node->Text.empty());
    for (auto [Neighbor, Edge] : Node->predecessor_edges()) {
      revng_check(!Neighbor.Text.empty());
      for (auto &Point : Edge.Points)
        revng_check(Point.X == 1.0 && Point.Y < 0.4 && Point.Y > 0.0);
    }
  }

  revng_check(A.hasSuccessor(B) && B.hasPredecessor(A));
  revng_check(B.hasSuccessor(A) && A.hasPredecessor(B));
  A.removeSuccessor(B);
  revng_check(!(A.hasSuccessor(B) && B.hasPredecessor(A)));
  revng_check(B.hasSuccessor(A) && A.hasPredecessor(B));

  revng_check(A.hasSuccessor(A) && A.hasPredecessor(A));
  A.removeSuccessor(A);
  revng_check(!(A.hasSuccessor(A) && A.hasPredecessor(A)));
}

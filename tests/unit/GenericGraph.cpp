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

SERIALIZABLEGRAPH_INTROSPECTION(TestNodeData, TestEdgeLabel);

template<typename NodeType>
struct DiamondGraph {
  using Node = NodeType;

  GenericGraph<Node> Graph;
  Node *Root;
  Node *Then;
  Node *Else;
  Node *Final;
};

template<typename NodeType, bool UseRefs = false>
static DiamondGraph<NodeType> createGraph() {
  DiamondGraph<NodeType> DG;
  auto &Graph = DG.Graph;

  // Create nodes
  DG.Root = Graph.addNode(0);
  DG.Then = Graph.addNode(1);
  DG.Else = Graph.addNode(3);
  DG.Final = Graph.addNode(2);

  // Set entry node
  Graph.setEntryNode(DG.Root);

  // Create edges
  if constexpr (UseRefs) {
    DG.Root->addSuccessor(DG.Then, { 7 });
    DG.Root->addSuccessor(DG.Else, { 1 });

    DG.Then->addSuccessor(DG.Final, { 2 });
    DG.Else->addSuccessor(DG.Final, { 3 });
  } else {
    DG.Root->addSuccessor(DG.Then, { 7 });
    DG.Root->addSuccessor(DG.Else, { 1 });

    DG.Then->addSuccessor(DG.Final, { 2 });
    DG.Else->addSuccessor(DG.Final, { 3 });
  }

  return DG;
}

using BidirectionalTestNode = BidirectionalNode<TestNodeData, TestEdgeLabel>;

BOOST_AUTO_TEST_CASE(TestRPOT) {
  auto DG = createGraph<BidirectionalTestNode>();
  ReversePostOrderTraversal<decltype(decltype(DG)::Graph) *> RPOT(&DG.Graph);
  std::vector<typename decltype(DG)::Node *> Visited;
  for (auto *Node : RPOT)
    Visited.push_back(Node);
  revng_check(Visited.size() == 4);
}

BOOST_AUTO_TEST_CASE(TestDepthFirstVisit) {
  auto DG = createGraph<BidirectionalTestNode>();
  std::vector<typename decltype(DG)::Node *> Visited;
  for (auto *Node : depth_first(&DG.Graph))
    Visited.push_back(Node);
  revng_check(Visited.size() == 4);

  Visited.clear();
  for (auto *Node : inverse_depth_first(DG.Final))
    Visited.push_back(Node);
  revng_check(Visited.size() == 4);
}

BOOST_AUTO_TEST_CASE(TestDominatorTree) {
  auto DG = createGraph<BidirectionalTestNode>();

  DominatorTreeBase<typename decltype(DG)::Node, false> DT;
  DT.recalculate(DG.Graph);
  revng_check(DT.dominates(DT.getNode(DG.Root), DT.getNode(DG.Then)));
  revng_check(DT.dominates(DT.getNode(DG.Root), DT.getNode(DG.Else)));
  revng_check(DT.dominates(DT.getNode(DG.Root), DT.getNode(DG.Final)));
  revng_check(not DT.dominates(DT.getNode(DG.Then), DT.getNode(DG.Final)));

  DominatorTreeBase<typename decltype(DG)::Node, true> PDT;
  PDT.recalculate(DG.Graph);
  revng_check(PDT.dominates(PDT.getNode(DG.Final), PDT.getNode(DG.Then)));
  revng_check(PDT.dominates(PDT.getNode(DG.Final), PDT.getNode(DG.Else)));
  revng_check(PDT.dominates(PDT.getNode(DG.Final), PDT.getNode(DG.Root)));
  revng_check(not PDT.dominates(PDT.getNode(DG.Then), PDT.getNode(DG.Root)));
}

BOOST_AUTO_TEST_CASE(TestSCC) {
  auto DG = createGraph<BidirectionalTestNode>();
  unsigned SCCCount = 0;
  for (auto &SCC : make_range(scc_begin(&DG.Graph), scc_end(&DG.Graph))) {
    revng_check(SCC.size() == 1);
    ++SCCCount;
  }
  revng_check(SCCCount == 4);
}

BOOST_AUTO_TEST_CASE(TestFilterGraphTraits) {
  auto DG = createGraph<BidirectionalTestNode>();

  {
    using Node = decltype(DG)::Node;
    using TestType = bool (*)(Node *const &, Node *const &);
    constexpr TestType TestLambda = [](auto *const &From, auto *const &To) {
      return From->Rank + To->Rank <= 2;
    };
    using Pair = NodePairFilteredGraph<Node *, TestLambda>;
    using FGT = GraphTraits<Pair>;
    using fdf_iterator = df_iterator<Node *,
                                     df_iterator_default_set<Node *>,
                                     false,
                                     FGT>;
    auto Begin = fdf_iterator::begin(DG.Root);
    auto End = fdf_iterator::end(DG.Root);
    revng_check(2 == std::distance(Begin, End));
  }

  {
    using Node = decltype(DG)::Node;
    using TestType = bool (*)(Edge<BidirectionalTestNode, TestEdgeLabel> &);
    constexpr TestType TestLambda = [](auto &Edge) { return Edge.Weight > 5; };
    using EFGT = GraphTraits<EdgeFilteredGraph<Node *, TestLambda>>;
    using efdf_iterator = df_iterator<Node *,
                                      df_iterator_default_set<Node *>,
                                      false,
                                      EFGT>;
    auto Begin = efdf_iterator::begin(DG.Root);
    auto End = efdf_iterator::end(DG.Root);
    revng_check(2 == std::distance(Begin, End));
  }
}

BOOST_AUTO_TEST_CASE(TestWriteGraph) {
  auto DG = createGraph<BidirectionalTestNode>();
  llvm::raw_null_ostream NullOutput;
  llvm::WriteGraph(NullOutput, &DG.Graph, "lol");
}

BOOST_AUTO_TEST_CASE(TestSerializableGraph) {
  auto DG = createGraph<BidirectionalTestNode>();
  auto Serializable = toSerializable(DG.Graph);
  using Node = decltype(DG)::Node;
  auto Reserializable = toSerializable(Serializable.toGenericGraph<Node>());
  revng_check(Reserializable == Serializable);
}

BOOST_AUTO_TEST_CASE(TestSerializeGraph) {
  auto DG = createGraph<BidirectionalTestNode>();
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
  auto *A = G.addNode("A");
  auto *B = G.addNode("B");

  A->addSuccessor(A, SomeEdge::PointType{ 1.0, 0.1 });
  A->addSuccessor(B, SomeEdge::PointType{ 1.0, 0.2 });
  B->addSuccessor(A, SomeEdge::PointType{ 1.0, 0.3 });

  revng_check(A->successorCount() == 2);
  revng_check(A->predecessorCount() == 2);
  revng_check(B->successorCount() == 1);
  revng_check(B->predecessorCount() == 1);

  for (auto *Node : G.nodes()) {
    revng_check(!Node->Text.empty());
    for (auto [Neighbor, Edge] : Node->successor_edges()) {
      revng_check(!Neighbor->Text.empty());
      for (auto &Point : Edge->Points)
        revng_check(Point.X == 1.0 && Point.Y < 0.4 && Point.Y > 0.0);
    }
  }
  for (auto *Node : G.nodes()) {
    revng_check(!Node->Text.empty());
    for (auto [Neighbor, Edge] : Node->predecessor_edges()) {
      revng_check(!Neighbor->Text.empty());
      for (auto &Point : Edge->Points)
        revng_check(Point.X == 1.0 && Point.Y < 0.4 && Point.Y > 0.0);
    }
  }

  revng_check(A->hasSuccessor(B) && B->hasPredecessor(A));
  revng_check(B->hasSuccessor(A) && A->hasPredecessor(B));
  A->removeSuccessor(B);
  revng_check(!(A->hasSuccessor(B) && B->hasPredecessor(A)));
  revng_check(B->hasSuccessor(A) && A->hasPredecessor(B));

  revng_check(A->hasSuccessor(A) && A->hasPredecessor(A));
  A->removeSuccessor(A);
  revng_check(!(A->hasSuccessor(A) && A->hasPredecessor(A)));

  A->addSuccessor(B, SomeEdge::PointType{ 1.0, 0.4 });
  A->addSuccessor(A, SomeEdge::PointType{ 1.0, 0.5 });
  B->addSuccessor(B, SomeEdge::PointType{ 1.0, 0.6 });

  revng_check(A->successorCount() == 2);
  revng_check(A->predecessorCount() == 2);
  revng_check(B->successorCount() == 2);
  revng_check(B->predecessorCount() == 2);
  G.removeNode(A);
  revng_check(B->successorCount() == 1);
  revng_check(B->predecessorCount() == 1);
}

BOOST_AUTO_TEST_CASE(MutableEdgeNodeNoEdgeLabelsTest) {
  GenericGraph<MutableEdgeNode<std::string>> Graph;
  auto *A = Graph.addNode("A");
  auto *B = Graph.addNode("B");

  revng_check(!A->hasSuccessor(B) && !B->hasPredecessor(A));
  revng_check(!B->hasSuccessor(A) && !A->hasPredecessor(B));
  A->addSuccessor(B);
  revng_check(A->hasSuccessor(B) && B->hasPredecessor(A));
  revng_check(!B->hasSuccessor(A) && !A->hasPredecessor(B));
  A->removeSuccessor(B);
  revng_check(!A->hasSuccessor(B) && !B->hasPredecessor(A));
  revng_check(!B->hasSuccessor(A) && !A->hasPredecessor(B));
}

using TestMutableEdgeNode = MutableEdgeNode<TestNodeData, TestEdgeLabel>;

BOOST_AUTO_TEST_CASE(MutableEdgeNodeFilterGraphTraitsTest) {
  auto DG = createGraph<TestMutableEdgeNode, true>();

  {
    using Node = decltype(DG)::Node;
    using TestType = bool (*)(Node *const &, Node *const &);
    constexpr TestType TestLambda = [](auto *const &From, auto *const &To) {
      return From->Rank + To->Rank <= 2;
    };
    using FGT = GraphTraits<NodePairFilteredGraph<Node *, TestLambda>>;
    using fdf_iterator = df_iterator<Node *,
                                     df_iterator_default_set<Node *>,
                                     false,
                                     FGT>;
    auto Begin = fdf_iterator::begin(DG.Root);
    auto End = fdf_iterator::end(DG.Root);
    revng_check(2 == std::distance(Begin, End));
  }

  {
    using Node = decltype(DG)::Node;
    using TestType = bool (*)(typename Node::EdgeView const &);
    constexpr TestType TestLambda = [](typename Node::EdgeView const &Edge) {
      return Edge.Label->Weight > 5;
    };
    using EFGT = GraphTraits<EdgeFilteredGraph<Node *, TestLambda>>;
    using efdf_iterator = df_iterator<Node *,
                                      df_iterator_default_set<Node *>,
                                      false,
                                      EFGT>;
    auto Begin = efdf_iterator::begin(DG.Root);
    auto End = efdf_iterator::end(DG.Root);
    revng_check(2 == std::distance(Begin, End));
  }
}

BOOST_AUTO_TEST_CASE(MutableEdgeNodeRemovalTest) {
  GenericGraph<MutableEdgeNode<std::string, double>> Graph;
  auto *A = Graph.addNode("A");
  auto *B = Graph.addNode("B");
  auto *C = Graph.addNode("C");

  revng_check(!A->hasSuccessor(B) && !B->hasPredecessor(A));
  revng_check(!B->hasSuccessor(A) && !A->hasPredecessor(B));
  A->addSuccessor(B);
  revng_check(A->hasSuccessor(B) && B->hasPredecessor(A));
  revng_check(!B->hasSuccessor(A) && !A->hasPredecessor(B));
  A->removeSuccessor(B);
  revng_check(!A->hasSuccessor(B) && !B->hasPredecessor(A));
  revng_check(!B->hasSuccessor(A) && !A->hasPredecessor(B));

  A->addSuccessor(A);
  A->addSuccessor(B);
  A->addSuccessor(C);

  revng_check(A->successorCount() == 3);
  revng_check(A->predecessorCount() == 1);
  revng_check(B->successorCount() == 0);
  revng_check(B->predecessorCount() == 1);
  revng_check(C->successorCount() == 0);
  revng_check(C->predecessorCount() == 1);

  size_t Counter = 0;
  for (auto *From : Graph.nodes()) {
    revng_check(!From->empty());
    for (auto *To : From->successors()) {
      revng_check(!To->empty());
      if (*From == "A" && *To == "C")
        ++Counter;
    }
  }
  revng_check(Counter == 1);

  for (auto Iterator = A->successors().begin();
       Iterator != A->successors().end();) {
    revng_check(!(*Iterator)->empty());
    if (**Iterator != *B)
      Iterator = A->removeSuccessor(Iterator);
    else
      ++Iterator;
  }

  revng_check(A->successorCount() == 1);
  revng_check(A->predecessorCount() == 0);
  revng_check(B->successorCount() == 0);
  revng_check(B->predecessorCount() == 1);
  revng_check(C->successorCount() == 0);
  revng_check(C->predecessorCount() == 0);

  A->addSuccessor(A);
  A->addSuccessor(C);

  for (auto Iterator = A->successor_edges().begin();
       Iterator != A->successor_edges().end();) {
    revng_check(!Iterator->Neighbor->empty());
    if (*Iterator->Neighbor != *C)
      Iterator = A->removeSuccessor(Iterator);
    else
      ++Iterator;
  }

  revng_check(A->successorCount() == 1);
  revng_check(A->predecessorCount() == 0);
  revng_check(B->successorCount() == 0);
  revng_check(B->predecessorCount() == 0);
  revng_check(C->successorCount() == 0);
  revng_check(C->predecessorCount() == 1);
}

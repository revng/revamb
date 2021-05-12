#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/ADT/GraphTraits.h"
#include "llvm/ADT/SmallVector.h"

#include "revng/ADT/Queue.h"

/// \brief Data structure implementing a subgraph of an object providing
///        GraphTraits
///
/// This class represents a "subgraph", i.e., given an object implementing
/// GraphTraits, this class expose a portion of the graph (identified by a
/// whitelist of nodes). The main benefit of such a class is to be able to run
/// graph-oriented algorithms on portion of a graph.
template<typename InnerNodeType>
class SubGraph {
public:
  /// \brief A node of the subgraph
  ///
  /// A node is simply a reference to the original node decorated with a vector
  /// of successor. In theory we could avoid storing the vector and explore the
  /// successors directly of the underlying node (filtering out all the
  /// non-whitelisted ones), but this would require a reference to the graph
  /// object, which is something we'd like to avoid.
  class Node {
  public:
    Node(InnerNodeType Value) : Value(Value) {}

    /// \brief Return the underlying node
    InnerNodeType get() const { return Value; }

  private:
    friend class SubGraph;
    friend struct llvm::GraphTraits<SubGraph<InnerNodeType>>;

    llvm::SmallVector<Node *, 2> Children;
    InnerNodeType Value;
  };
  using NodeType = Node;

private:
  using ParentGraphTraits = llvm::GraphTraits<InnerNodeType>;
  using ChildIteratorType = typename llvm::SmallVector<Node *, 2>::iterator;
  using nodes_iterator = typename std::set<Node>::iterator;

  friend llvm::GraphTraits<SubGraph<InnerNodeType>>;

public:
  /// Construct a subgraph starting from the node \p Entry and considering only
  /// nodes in \p WhiteList.
  ///
  /// Note: this method has a cost proportional to the size of the subgraph,
  /// since it basically creates a copy of it.
  SubGraph(InnerNodeType Entry, const std::set<InnerNodeType> WhiteList) {
    // Create a node for the entry node, and take note of it
    EntryNode = &findOrInsert(Entry);

    // We want to rebuild the graph internally considering only the whitelisted
    // nodes
    OnceQueue<InnerNodeType> Queue;
    Queue.insert(Entry);

    while (!Queue.empty()) {
      InnerNodeType Value = Queue.pop();

      // Get (or create) the current node
      Node &NewNode = findOrInsert(Value);

      // Iterate over successors
      auto Successors = make_range(ParentGraphTraits::child_begin(Value),
                                   ParentGraphTraits::child_end(Value));
      for (InnerNodeType Successor : Successors) {
        // If it's whitelisted, register it as a child and enqueue it
        if (WhiteList.count(Successor) != 0) {
          NewNode.Children.push_back(&findOrInsert(Successor));
          Queue.insert(Successor);
        }
      }
    }
  }

private:
  struct CompareNodes {
    bool operator()(const Node &A, const Node &B) const {
      return A.get() < B.get();
    }
  };

  /// \brief Get the node with value \p Value, of, if absent, insert it in the
  ///        graph
  Node &findOrInsert(InnerNodeType Value) {
    // We can't use find, because it would use the equality comparison operator
    // instead of our CompareNodes. Plus we have to cast away constness, because
    // std::set entries are always const (since you might change the key), but
    // we don't. Users of this function will never change the key, just
    // increment the list of successors.
    auto It = Nodes.lower_bound(Node(Value));
    if (It != Nodes.end() && It->get() == Value)
      return const_cast<Node &>(*It);
    else
      return const_cast<Node &>(*Nodes.emplace_hint(It, Value));
  }

private:
  // We define a custom comparator so that Node still preserves the default
  // comparison operators
  std::set<Node, CompareNodes> Nodes;
  Node *EntryNode;
};

namespace llvm {

/// \brief Specialization of GraphTraits for SubGraph
template<typename InnerNodeType>
struct GraphTraits<SubGraph<InnerNodeType>> {
  using GraphType = SubGraph<InnerNodeType>;
  using NodeRef = typename GraphType::Node *;
  using ChildIteratorType = typename GraphType::ChildIteratorType;
  using nodes_iterator = typename GraphType::nodes_iterator;

  // TODO: here G should be const
  static NodeRef getEntryNode(GraphType &G) { return G.EntryNode; }

  static ChildIteratorType child_begin(NodeRef Parent) {
    return Parent->Children.begin();
  }

  static ChildIteratorType child_end(NodeRef Parent) {
    return Parent->Children.end();
  }

  static nodes_iterator nodes_begin(GraphType *G) { return G->Nodes.begin(); }

  static nodes_iterator nodes_end(GraphType *G) { return G->Nodes.end(); }

  static unsigned size(GraphType *G) { return G->Nodes.size(); }
};

} // namespace llvm

#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <array>
#include <map>

#include "boost/variant.hpp"

#include "revng/ADT/ZipMapIterator.h"
#include "revng/Support/Assert.h"

/// \brief Type-safe wrapper for different iterators sharing value_type
template<typename... Ts>
class Iteratall {
private:
  // Define some helpers
  template<typename... Ps>
  struct are_same : std::false_type {};

  template<typename P, typename... Ps>
  struct are_same<P, P, Ps...> : are_same<P, Ps...> {};

  template<typename P>
  struct are_same<P, P> : std::true_type {};

  template<typename P, typename... Ps>
  struct first {
    using type = P;
  };

  template<typename T>
  using it = std::iterator_traits<T>;
  using itfirst = it<typename first<Ts...>::type>;

  // Assert correct usage
  static_assert(are_same<typename it<Ts>::value_type...>::value,
                "The iterators have different value_type");
  static_assert(are_same<typename it<Ts>::reference...>::value,
                "The iterators have different reference");

public:
  using value_type = typename itfirst::value_type;
  using reference = typename itfirst::reference;
  using pointer = typename itfirst::pointer;
  using difference_type = typename itfirst::difference_type;
  using iterator_category = typename itfirst::iterator_category;

  template<typename T>
  Iteratall(T I) : Iterator(I) {}

private:
  struct PostincrementVisitor : public boost::static_visitor<Iteratall> {
    template<typename T>
    Iteratall operator()(T &It) const {
      return Iteratall(It++);
    }
  };

  struct PreincrementVisitor : public boost::static_visitor<Iteratall> {
    template<typename T>
    Iteratall operator()(T &It) const {
      return Iteratall(++It);
    }
  };

  struct PostdecrementVisitor : public boost::static_visitor<Iteratall> {
    template<typename T>
    Iteratall operator()(T &It) const {
      return Iteratall(It--);
    }
  };

  struct PredecrementVisitor : public boost::static_visitor<Iteratall> {
    template<typename T>
    Iteratall operator()(T &It) const {
      return Iteratall(--It);
    }
  };

  struct DereferenceVisitor : public boost::static_visitor<reference> {
    template<typename T>
    reference operator()(T &It) const {
      return *It;
    }
  };

  struct CompareVisitor : public boost::static_visitor<bool> {
    template<typename T, typename R>
    bool operator()(T &, R &) const {
      // The compared type should always be the same
      revng_abort();
    }

    template<typename T>
    bool operator()(T &It, T &Other) const {
      return It == Other;
    }
  };

public:
  Iteratall() = default;
  Iteratall(const Iteratall &) = default;
  Iteratall(Iteratall &&) = default;
  Iteratall &operator=(const Iteratall &) = default;
  Iteratall &operator=(Iteratall &&) = default;

  Iteratall operator++(int) {
    Iteratall Res(*this);
    boost::apply_visitor(PostincrementVisitor(), Iterator);
    return Res;
  }

  Iteratall &operator++() {
    return *this = boost::apply_visitor(PreincrementVisitor(), Iterator);
  }

  Iteratall &operator+=(int N) {
    for (int I = 0; I < N; ++I)
      *this = boost::apply_visitor(PreincrementVisitor(), Iterator);
    return *this;
  }

  Iteratall operator--(int) {
    Iteratall Res(*this);
    boost::apply_visitor(PostdecrementVisitor(), Iterator);
    return Res;
  }

  Iteratall &operator--() {
    return *this = boost::apply_visitor(PredecrementVisitor(), Iterator);
  }

  Iteratall &operator-=(int N) {
    for (int I = 0; I < N; ++I)
      *this = boost::apply_visitor(PredecrementVisitor(), Iterator);
    return *this;
  }

  bool operator==(const Iteratall &Other) const {
    return boost::apply_visitor(CompareVisitor(), Iterator, Other.Iterator);
  }

  bool operator!=(const Iteratall &Other) const { return !(*this == Other); }

  reference operator*() const {
    return boost::apply_visitor(DereferenceVisitor(), Iterator);
  }

  pointer operator->() const { return &**this; }

private:
  boost::variant<Ts...> Iterator;
};

/// \brief map that usually contains less than N elements
///
/// SmallMap keeps a std::array of pairs inline which are search linearly if
/// size() < N.
///
/// \note Since this data structure internally uses an std::array, expect the
///       default constructor to be used.
///
/// \tparam N number of elements to keep inline.
template<typename K, typename V, unsigned N, typename C = std::less<K>>
class SmallMap {
private:
  // Define some helper types
  using NonConstPair = std::pair<K, V>;
  using NonConstContainer = std::array<NonConstPair, N>;

  using Pair = std::pair<const K, V>;
  using Container = std::array<Pair, N>;

  using VIterator = typename Container::iterator;
  using ConstVIterator = typename Container::const_iterator;

private:
  // These have to be mutable so we can sort() can be a const method.
  // Note that Vector is an array of pairs where the key is *not* const. We use
  // the pair with the const key only to provide iterators compatible with
  // std::map.
  mutable NonConstContainer Vector; ///< Container for inline elements
  mutable bool IsSorted; ///< Is vector sorted?
  unsigned Size; ///< Size of Vector

  // Non-inline version of the container
  std::map<K, V, C> Map;

private:
  VIterator smallBegin() { return reinterpret_cast<VIterator>(Vector.begin()); }

  ConstVIterator smallBegin() const {
    return reinterpret_cast<ConstVIterator>(Vector.begin());
  }

public:
  SmallMap() : IsSorted(true), Size(0) {}

  SmallMap(const SmallMap &) = default;
  SmallMap &operator=(const SmallMap &Other) = default;
  SmallMap(SmallMap &&Other) = default;
  SmallMap &operator=(SmallMap &&Other) = default;

public:
  using iterator = Iteratall<VIterator, typename std::map<K, V, C>::iterator>;
  using const_iterator = Iteratall<ConstVIterator,
                                   typename std::map<K, V, C>::const_iterator>;
  using size_type = size_t;
  using value_type = Pair;
  using pointer = Pair *;
  using const_pointer = const Pair *;
  using key_type = const K;
  using mapped_type = V;

public:
  /// \brief If necessary, sorts the inline vector.
  ///
  /// Call this function in case you need to iterate over the container in order
  void sort() const {
    if (IsSorted || !isSmall() || Size <= 1)
      return;

    auto Compare = [](const Pair &A, const Pair &B) {
      return C()(A.first, B.first);
    };
    std::sort(Vector.begin(), Vector.begin() + Size, Compare);
    IsSorted = true;
  }

  bool empty() const { return Size == 0 && Map.empty(); }

  size_type size() const { return isSmall() ? Size : Map.size(); }

  const_iterator begin() const {
    if (isSmall())
      return const_iterator(smallBegin());
    else
      return const_iterator(Map.begin());
  }

  const_iterator end() const {
    if (isSmall())
      return const_iterator(smallBegin() + Size);
    else
      return const_iterator(Map.end());
  }

  iterator begin() {
    if (isSmall())
      return iterator(smallBegin());
    else
      return iterator(Map.begin());
  }

  iterator end() {
    if (isSmall())
      return iterator(smallBegin() + Size);
    else
      return iterator(Map.end());
  }

  size_type count(const K &Key) const {
    if (isSmall()) {
      return vfind(Key) == smallBegin() + Size ? 0 : 1;
    } else {
      return Map.count(Key);
    }
  }

  std::pair<iterator, bool> insert(const Pair &P) {
    if (!isSmall()) {
      auto Result = Map.insert(P);
      return { iterator(Result.first), Result.second };
    }

    VIterator I = vfind(P.first);
    // Do we have it?
    if (I != smallBegin() + Size)
      return { iterator(I), false };

    if (Size < N) {
      Vector[Size] = P;
      Size++;

      // Check if we're preserving the ordering
      if (Size > 1 && IsSorted)
        IsSorted = !C()(P.first, Vector[Size - 2].first);

      return { iterator(smallBegin() + Size - 1), true };
    }

    // Otherwise, grow from vector to set.
    revng_assert(Map.empty());
    for (unsigned I = 0; I < Size; I++)
      Map.insert(Vector[I]);

    auto Result = Map.insert(P);
    return { iterator(Result.first), Result.second };
  }

  void erase(const K &Key) {
    if (isSmall()) {
      auto I = Vector.begin();
      auto E = Vector.begin() + Size;

      for (; I != E; ++I)
        if (I->first == Key)
          break;

      if (I == E)
        return;

      Size--;
      for (; I < E - 1; I++)
        *I = *(I + 1);

    } else {
      auto It = Map.find(Key);
      if (It != Map.end())
        Map.erase(It);
    }
  }

  iterator find(const K &Key) {
    if (isSmall())
      return iterator(vfind(Key));
    else
      return iterator(Map.find(Key));
  }

  const_iterator find(const K &Key) const {
    if (isSmall())
      return const_iterator(vfind(Key));
    else
      return const_iterator(Map.find(Key));
  }

  iterator lower_bound(const K &Key) {
    if (isSmall())
      return iterator(vlower_bound(Key));
    else
      return iterator(Map.lower_bound(Key));
  }

  const_iterator lower_bound(const K &Key) const {
    if (isSmall())
      return const_iterator(vlower_bound(Key));
    else
      return const_iterator(Map.lower_bound(Key));
  }

  bool contains(const K &Key) const {
    return find(Key) != end();
  }

  void clear() {
    // TODO: we should invoke some destructors at a certain point
    Size = 0;
    Map.clear();
  }

  V &operator[](K &&Key) {
    return insert(std::make_pair(std::move(Key), V())).first->second;
  }

  V &operator[](const K &Key) {
    return insert(std::make_pair(Key, V())).first->second;
  }

  V &at(const K &Key) {
    iterator It = find(Key);
    revng_assert(It != end());
    return It->second;
  }

  const V &at(const K &Key) const {
    const_iterator It = find(Key);
    revng_assert(It != end());
    return It->second;
  }

private:
  bool isSmall() const { return Map.empty(); }

  ConstVIterator vfind(const K &Key) const {
    for (ConstVIterator I = smallBegin(), E = smallBegin() + Size; I != E; ++I)
      if (I->first == Key)
        return I;
    return smallBegin() + Size;
  }

  VIterator vfind(const K &Key) {
    for (VIterator I = smallBegin(), E = smallBegin() + Size; I != E; ++I)
      if (I->first == Key)
        return I;
    return smallBegin() + Size;
  }

  ConstVIterator vlower_bound(const K &Key) const {
    sort();
    for (ConstVIterator I = smallBegin(), E = smallBegin() + Size; I != E; ++I)
      if (!C()(I->first, Key))
        return I;
    return smallBegin() + Size;
  }

  VIterator vlower_bound(const K &Key) {
    sort();
    for (VIterator I = smallBegin(), E = smallBegin() + Size; I != E; ++I)
      if (!C()(I->first, Key))
        return I;
    return smallBegin() + Size;
  }
};

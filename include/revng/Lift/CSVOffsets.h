#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <set>

#include "revng/Support/Assert.h"

template<bool StaticallyEnabled>
class Logger;

/// \brief Different types of accesses to the CPU State Variables (CSVs), with a
///        set of possible offsets.
class CSVOffsets {

private:
  using OffsetSet = std::set<int64_t>;

public:
  using iterator = OffsetSet::iterator;
  using const_iterator = OffsetSet::const_iterator;
  using size_type = OffsetSet::size_type;

public:
  enum Kind {
    Unknown,
    Numeric,
    KnownInPtr,
    OutAndKnownInPtr,
    UnknownInPtr,
    OutAndUnknownInPtr,
  };

  static const char *toString(const Kind K) {
    switch (K) {
    case Unknown:
      return "Unknown";
    case Numeric:
      return "Numeric";
    case KnownInPtr:
      return "KnownInPtr";
    case OutAndKnownInPtr:
      return "OutAndKnownInPtr";
    case UnknownInPtr:
      return "UnknownInPtr";
    case OutAndUnknownInPtr:
      return "OutAndUnknownInPtr";
    default:
      revng_abort();
    }
    return "";
  }

private:
  Kind OffsetKind;
  OffsetSet Offsets;

public:
  CSVOffsets() : OffsetKind(Kind::Numeric), Offsets() {}
  CSVOffsets(Kind K) : OffsetKind(K), Offsets() {
    // Useful for debug revng_assert(isUnknown(K) or isUnknownInPtr(K));
  }
  CSVOffsets(Kind K, int64_t O) : OffsetKind(K), Offsets({ O }) {
    // Useful for debug revng_assert(not isUnknown(K) and not
    // isUnknownInPtr(K));
  }
  CSVOffsets(Kind K, std::set<int64_t> O) : OffsetKind(K), Offsets(O) {
    // Useful for debug revng_assert(not isUnknown(K) and not
    // isUnknownInPtr(K));
  }

public:
  friend void writeToLog(Logger<true> &L, const CSVOffsets &O, int /*Ignore*/);

private:
  explicit operator Kind() const { return OffsetKind; }

public:
  iterator begin() { return Offsets.begin(); }
  iterator end() { return Offsets.end(); }

  const_iterator begin() const { return Offsets.cbegin(); }
  const_iterator end() const { return Offsets.cend(); }

  size_type size() const { return Offsets.size(); }
  size_type empty() const { return Offsets.empty(); }

  static bool isUnknown(const Kind K) { return K == Kind::Unknown; }
  bool isUnknown() const { return isUnknown(OffsetKind); }

  static bool isNumeric(const Kind K) { return K == Kind::Numeric; }
  bool isNumeric() const { return isNumeric(OffsetKind); }

  static bool isPtr(const Kind K) {
    return not isNumeric(K) and not isUnknown(K);
  }
  bool isPtr() const { return isPtr(OffsetKind); }

  static bool isOnlyInPtr(const Kind K) {
    return K == Kind::UnknownInPtr or K == Kind::KnownInPtr;
  }
  bool isOnlyInPtr() const { return isOnlyInPtr(OffsetKind); }

  static bool isInOutPtr(const Kind K) {
    return isPtr(K) and not isOnlyInPtr(K);
  }
  bool isInOutPtr() const { return isInOutPtr(OffsetKind); }

  static bool isUnknownInPtr(const Kind K) {
    return K == Kind::UnknownInPtr or K == Kind::OutAndUnknownInPtr;
  }
  bool isUnknownInPtr() const { return isUnknownInPtr(OffsetKind); }

  static bool isKnownInPtr(const Kind K) {
    return isPtr(K) and not isUnknownInPtr(K);
  }
  bool isKnownInPtr() const { return isKnownInPtr(OffsetKind); }

  static bool hasOffsetSet(const Kind K) {
    return K == Kind::KnownInPtr or K == Kind::OutAndKnownInPtr
           or K == Kind::Numeric;
  }
  bool hasOffsetSet() const { return hasOffsetSet(OffsetKind); }

  enum Kind getKind() const { return OffsetKind; }
  static enum Kind makeUnknown(const Kind K) {
    switch (K) {
    case Kind::Numeric:
      return Kind::Unknown;
    case Kind::KnownInPtr:
      return Kind::UnknownInPtr;
    case Kind::OutAndKnownInPtr:
      return Kind::OutAndUnknownInPtr;
    default:
      revng_abort();
    }
    return K;
  }

  void insert(int64_t O) { Offsets.insert(O); }
  void combine(const CSVOffsets &other) {
    Kind K0 = OffsetKind;
    Kind K1 = other.OffsetKind;
    // For equal kinds just merge the offsets
    if (K0 == K1) {
      Offsets.insert(other.Offsets.begin(), other.Offsets.end());
      return;
    }

    // If one is OutAndUnknownInPtr always return OutAndUnknownInPtr
    if (K0 == Kind::OutAndUnknownInPtr or K1 == Kind::OutAndUnknownInPtr) {
      OffsetKind = Kind::OutAndUnknownInPtr;
      Offsets = {};
      return;
    }

    if (K0 == Kind::UnknownInPtr or K1 == Kind::UnknownInPtr) {
      // If K0 and K1 are equal, or one of them is a OnlyInPtr stay in
      // UnknownInPtr, in all the other cases also access out, hence returning
      // OutAndUnknownInPtr
      if (isOnlyInPtr(K0) or isOnlyInPtr(K1) or K0 == K1)
        OffsetKind = Kind::UnknownInPtr;
      else
        OffsetKind = Kind::OutAndUnknownInPtr;

      Offsets = {};
      return;
    }

    if (K0 == Kind::OutAndKnownInPtr or K1 == Kind::OutAndKnownInPtr) {
      // If one is Unknown it wipes away the knowledge, otherwise we merge the
      // Offsets
      if (isUnknown(K0) or isUnknown(K1)) {
        OffsetKind = Kind::OutAndUnknownInPtr;
        Offsets = {};
      } else {
        OffsetKind = Kind::OutAndKnownInPtr;
        Offsets.insert(other.Offsets.begin(), other.Offsets.end());
      }
      return;
    }

    {
      bool KIP0 = K0 == Kind::KnownInPtr;
      if (KIP0 or K1 == Kind::KnownInPtr) {
        revng_assert(isNumeric(K0) or isNumeric(K1) or isUnknown(K0)
                     or isUnknown(K1));
        OffsetKind = Kind::OutAndKnownInPtr;
        if (not KIP0)
          Offsets = other.Offsets;
        return;
      }
    }
    revng_assert((isNumeric(K0) and isUnknown(K1))
                 or (isNumeric(K1) and isUnknown(K0)));
    OffsetKind = Kind::Unknown;
    Offsets = {};
  }
};

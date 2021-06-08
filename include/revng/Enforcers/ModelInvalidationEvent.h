#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "revng/AutoEnforcer/InvalidationEvent.hpp"
#include "revng/Model/Binary.h"
#include "revng/Model/TupleTreeDiff.h"

namespace AutoEnforcer {
class ModelInvalidationEvent
  : public InvalidationEvent<ModelInvalidationEvent> {
public:
  using Diff = TupleTreeDiff<model::Binary>;
  ModelInvalidationEvent(Diff ModelDiff) : Changes(std::move(ModelDiff)) {}
  static char ID;

  const Diff &getModelDiff() const { return Changes; }

  Diff &getModelDiff() { return Changes; }

private:
  Diff Changes;
};
} // namespace AutoEnforcer

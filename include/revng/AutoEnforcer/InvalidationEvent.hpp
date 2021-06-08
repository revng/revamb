#pragma once

namespace AutoEnforcer {

class InvalidationEventBase {
public:
  InvalidationEventBase(char &ID) : ID(&ID) {}
  virtual ~InvalidationEventBase() = default;

  template<typename Derived>
  bool isA() const {
    return ID == &Derived::ID;
  }

private:
  const char *ID;
};

template<typename Derived>
class InvalidationEvent : InvalidationEventBase {
public:
  InvalidationEvent() : InvalidationEventBase(Derived::ID) {}
};
} // namespace AutoEnforcer

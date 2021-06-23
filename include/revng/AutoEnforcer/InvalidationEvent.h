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
class InvalidationEvent : public InvalidationEventBase {
public:
  InvalidationEvent() : InvalidationEventBase(Derived::ID) {}

  static bool classof(const InvalidationEventBase *Instance) {
    return Instance->isA<Derived>();
  }
};
} // namespace AutoEnforcer

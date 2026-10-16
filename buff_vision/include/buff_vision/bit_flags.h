//
// Created by kaxns on 3/13/22.
//

#pragma once
#include <cstdint>
#include <type_traits>
#define ENABLE_VIRTUALIZATION 1

enum class WindmillRecognizerAnalyzerMode : uint8_t{
  // clang-format off
  TREAT_ALL = 0b10000000,
  TREAT_HAMMER_ONLY = 0b00000001,
  ENABLE_ROI_TRACKING = 0b00000010,
  ENABLE_ROTATION_PREDICTION = 0b00000100,

  EXTRACT_VERTICES_CONVENTIONAL = 0b00001000,
  EXTRACT_VERTICES_DEEP_LEARNING = 0b00010000,
  PNP_MODELING_TARGET_ONLY = 0b00100000,
  PNP_MODELING_ALL = 0b01000000,
  // clang-format on
};

using scope = WindmillRecognizerAnalyzerMode;

template<typename Enum, typename Value,
    typename IsEnum /* not useless, do not remove */ = std::enable_if_t<std::is_enum<Enum>::value, bool>>
class BitFlags {
 public:
  template<typename... Args>
  BitFlags(Args... args) : value_(create<Enum>(args...)) {
  }
  template<typename T, typename... Args>
  Value create(T t, Args... args) {
    return create(t) | create<T>(args...);
  }
  template<typename T>
  Value create(T t) {
    return static_cast<Value>(t);
  }
  template<typename T>
  Value create() {
    return static_cast<Value>(0);
  }
  bool has(Enum e) {
    return (value_ & static_cast<Value>(e)) != 0;
  }
  void add(Enum e) {
    value_ |= static_cast<Value>(e);
  }
  void remove(Enum e) {
    value_ &= ~static_cast<Value>(e);
  }
  void combine(const BitFlags &other) {
    this->value_ |= other.value_;
  }

  int condition() {
    if(mark_ == 1)
      return 1;
    else
      return 0;
  }
//  void onlyToAll() {
//    remove(scope::TREAT_HAMMER_ONLY);
//    add(scope::TREAT_ALL);
//    remove(scope::PNP_MODELING_TARGET_ONLY);
//    add(scope::PNP_MODELING_ALL);
//  }
  void analysisError() {
    remove(scope::TREAT_ALL);
    add(scope::TREAT_HAMMER_ONLY);
    remove(scope::PNP_MODELING_ALL);
    add(scope::PNP_MODELING_TARGET_ONLY);
    mark_ = 0;
  }
 private:
  Value value_;
  int mark_ = 1;
};

class Flags
{
 public:
  static BitFlags<scope, uint8_t> flags_;
};
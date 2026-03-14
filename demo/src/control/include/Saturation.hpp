#pragma once
#include <algorithm>
#include <cstdint>

class Saturation {
public:
  Saturation() = default;
  Saturation(int min_v, int max_v) : min_v_(min_v), max_v_(max_v) {}
  void setLimits(int min_v, int max_v) { min_v_ = min_v; max_v_ = max_v; }
  int clamp(int v) const { return std::clamp(v, min_v_, max_v_); }
  int min() const { return min_v_; }
  int max() const { return max_v_; }
private:
  int min_v_ {-180};
  int max_v_ {180};
};

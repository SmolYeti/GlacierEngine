#pragma once

// GLM
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

// STD
#include <vector>

namespace nurbs {
class Curve2D {
 public:
  Curve2D(glm::dvec2 interval = {0.0, 1.0}) : interval_(interval) {}

  virtual glm::dvec2 EvaluateCurve(double u) const { return {0, 0}; }
  virtual std::vector<glm::dvec2> EvaluateCurvePoints(
      uint32_t point_count) const {
    return {};
  }

  void interval(glm::dvec2 interval) { interval_ = interval; }
  glm::dvec2 interval() { return interval_; }

 protected:
  glm::dvec2 interval_;
};
}  // namespace nurbs
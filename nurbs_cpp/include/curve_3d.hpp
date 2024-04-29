#pragma once

// GLM
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

// STD
#include <vector>

namespace nurbs {
class Curve3D {
public:
  Curve3D(glm::dvec2 interval = {0.0, 1.0}) : interval_(interval) {}

  virtual glm::dvec3 EvaluateCurve(double u) const { return {0.0, 0.0, 0.0}; }

  virtual std::vector<glm::dvec3>
  EvaluateCurvePoints(uint32_t point_count) const {
    std::vector<glm::dvec3> points(point_count);
    const double div =
        (interval_.y - interval_.x) / static_cast<double>(point_count - 1);
    for (uint32_t i = 0; i < point_count; ++i) {
      double u = interval_.x + (static_cast<double>(i) * div);
      points[i] = EvaluateCurve(u);
    }
    return points;
  }

  void interval(glm::dvec2 interval) { interval_ = interval; }
  glm::dvec2 interval() { return interval_; }

public:
  // Utility methods
  double InternalParameter(double param) const {
    // Convert to 0 to 1
    param = (param - interval_.x) / (interval_.y - interval_.x);
    // Convert to internal interval
    param = (param * (internal_interval_.y - internal_interval_.x)) +
            internal_interval_.x;
    return param;
  }

protected:
  glm::dvec2 interval_;
  glm::dvec2 internal_interval_;
};
} // namespace nurbs
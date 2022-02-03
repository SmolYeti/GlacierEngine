#pragma once

// GLM
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

// STD
#include <vector>

namespace nurbs {
class Surface {
 public:
  Surface(glm::dvec2 u_interval = {0.0, 1.0},
          glm::dvec2 v_interval = {0.0, 1.0})
      : u_interval_(u_interval), v_interval_(v_interval) {}

  virtual glm::dvec3 EvaluatePoint(glm::dvec2 uv) const { return {0, 0, 0}; }
  virtual std::vector<glm::dvec3> EvaluatePoints(
      uint32_t u_sample_count, uint32_t v_sample_count) const {
    return {};
  }

  void u_interval(glm::dvec2 interval) { u_interval_ = interval; }
  glm::dvec2 u_interval() { return u_interval_; }

  void v_interval(glm::dvec2 interval) { v_interval_ = interval; }
  glm::dvec2 v_interval() { return v_interval_; }

 protected:
  glm::dvec2 u_interval_;
  glm::dvec2 v_interval_;
};
}  // namespace nurbs
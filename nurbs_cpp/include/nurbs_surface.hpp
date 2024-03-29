#pragma once

#include "include/surface.hpp"

// STD
#include <array>
#include <functional>

namespace nurbs {
class NURBSSurface : public Surface {
 public:
  NURBSSurface(uint32_t u_degree, uint32_t v_degree,
               std::vector<uint32_t> u_knots, std::vector<uint32_t> v_knots,
               std::vector<std::vector<glm::dvec4>> control_polygon,
               glm::dvec2 u_interval = {0.0, 1.0},
               glm::dvec2 v_interval = {0.0, 1.0});

  glm::dvec3 EvaluatePoint(glm::dvec2 uv) const override;

  /* std::vector<glm::dvec3> EvaluatePoints(
       uint32_t u_sample_count, uint32_t v_sample_count) const override;*/
  std::vector<std::vector<glm::dvec3>> Derivatives(
      glm::dvec2 uv, uint32_t max_derivative) const;

 private:
  uint32_t u_degree_;
  uint32_t v_degree_;
  std::vector<uint32_t> u_knots_;
  std::vector<uint32_t> v_knots_;
  std::vector<std::vector<glm::dvec4>> control_polygon_;
};

}  // namespace nurbs

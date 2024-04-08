#pragma once

#include "include/surface.hpp"

namespace nurbs {
// Nonrational B-Spline Surface
class BSplineSurface : public Surface {
public:
  static constexpr double kTolerance = std::numeric_limits<double>::epsilon();

  BSplineSurface(uint32_t u_degree, uint32_t v_degree,
                 const std::vector<uint32_t> &u_knots,
                 const std::vector<uint32_t> &v_knots,
                 const std::vector<std::vector<glm::dvec3>> &control_polygon,
                 glm::dvec2 u_interval = {0.0, 1.0},
                 glm::dvec2 v_interval = {0.0, 1.0});

  glm::dvec3 EvaluatePoint(glm::dvec2 uv) const override;

  std::vector<glm::dvec3> EvaluatePoints(
      uint32_t u_sample_count, uint32_t v_sample_count) const override;

  std::vector<std::vector<glm::dvec3>> Derivative(
      glm::dvec2 uv, uint32_t max_derivative) const;

  // Used for Derivatives2
  std::vector<std::vector<std::vector<std::vector<glm::dvec3>>>>
  SurfaceDerivCpts(uint32_t d, uint32_t r_start, uint32_t r_end,
                   uint32_t s_start, uint32_t s_end) const;
  std::vector<std::vector<glm::dvec3>> Derivatives2(
      glm::dvec2 uv, uint32_t max_derivative) const;

 private:
  uint32_t u_degree_;
  uint32_t v_degree_;
  std::vector<uint32_t> u_knots_;
  std::vector<uint32_t> v_knots_;
  // [u][v]
  std::vector<std::vector<glm::dvec3>> control_polygon_;
};
}  // namespace nurbs
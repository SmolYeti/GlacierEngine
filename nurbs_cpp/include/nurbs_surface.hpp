#pragma once

#include "include/surface.hpp"

// STD
#include <array>
#include <functional>

namespace nurbs {
class NURBSSurface : public Surface {
public:
  static constexpr double kTolerance = std::numeric_limits<double>::epsilon();

  NURBSSurface(uint32_t u_degree, uint32_t v_degree,
               std::vector<double> u_knots, std::vector<double> v_knots,
               std::vector<std::vector<glm::dvec4>> control_polygon,
               glm::dvec2 u_interval = {0.0, 1.0},
               glm::dvec2 v_interval = {0.0, 1.0});

  glm::dvec3 EvaluatePoint(glm::dvec2 uv) const override;

  /* std::vector<glm::dvec3> EvaluatePoints(
       uint32_t u_sample_count, uint32_t v_sample_count) const override;*/
  std::vector<std::vector<glm::dvec3>>
  Derivatives(glm::dvec2 uv, uint32_t max_derivative) const;

  enum SurfaceDirection { kUDir, kVDir };
  NURBSSurface KnotInsert(SurfaceDirection dir, double knot, uint32_t times);

  const std::vector<double> &u_knots() { return u_knots_; }
  const std::vector<double> &v_knots() { return v_knots_; }

  private:
    uint32_t u_degree_;
    uint32_t v_degree_;
    std::vector<double> u_knots_;
    std::vector<double> v_knots_;
    std::vector<std::vector<glm::dvec4>> control_polygon_;

    glm::dvec2 u_internal_interval_;
    glm::dvec2 v_internal_interval_;
  };

} // namespace nurbs

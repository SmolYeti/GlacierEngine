#pragma once

#include "include/surface.hpp"

namespace nurbs {
// Nonrational B-Spline Surface
class BSplineSurface : public Surface {
public:
  static constexpr double kTolerance = std::numeric_limits<double>::epsilon();

  BSplineSurface(uint32_t u_degree, uint32_t v_degree,
                 const std::vector<double> &u_knots,
                 const std::vector<double> &v_knots,
                 const std::vector<std::vector<Point3D>> &control_polygon,
                 Point2D u_interval = {0.0, 1.0},
                 Point2D v_interval = {0.0, 1.0});

  Point3D EvaluatePoint(Point2D uv) const override;

  std::vector<Point3D> EvaluatePoints(
      uint32_t u_sample_count, uint32_t v_sample_count) const override;

  std::vector<std::vector<Point3D>> Derivative(Point2D uv,
                                                uint32_t max_derivative) const;

  // Used for Derivatives2
  std::vector<std::vector<std::vector<std::vector<Point3D>>>>
  SurfaceDerivCpts(uint32_t d, uint32_t r_start, uint32_t r_end,
                   uint32_t s_start, uint32_t s_end) const;
  std::vector<std::vector<Point3D>> Derivatives2(Point2D uv,
                                                 uint32_t max_derivative) const;

 private:
  uint32_t u_degree_;
  uint32_t v_degree_;
  std::vector<double> u_knots_;
  std::vector<double> v_knots_;
  // [u][v]
  std::vector<std::vector<Point3D>> control_polygon_;
};
}  // namespace nurbs
#pragma once

#include "include/bezier_surface.hpp"
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
               std::vector<std::vector<Point4D>> control_polygon,
               Point2D u_interval = {0.0, 1.0},
               Point2D v_interval = {0.0, 1.0});

  Point3D EvaluatePoint(Point2D uv) const override;

  /* std::vector<Point3D> EvaluatePoints(
       uint32_t u_sample_count, uint32_t v_sample_count) const override;*/
  std::vector<std::vector<Point3D>>
  Derivatives(Point2D uv, uint32_t max_derivative) const;

  enum SurfaceDirection { kUDir, kVDir };
  NURBSSurface KnotInsert(SurfaceDirection dir, double knot, int times) const;

  NURBSSurface RefineKnotVect(std::vector<double> knots, SurfaceDirection dir) const;

  std::vector<NURBSSurface> DecomposeU() const;
  std::vector<BezierSurface> DecomposeV() const;

  std::vector<std::vector<BezierSurface>> Decompose() const;

  const std::vector<double> &u_knots() const { return u_knots_; }
  const std::vector<double> &v_knots() const { return v_knots_; }

  const std::vector<std::vector<Point4D>>& control_polygon() const {
    return control_polygon_;
  }

  private:
    uint32_t u_degree_;
    uint32_t v_degree_;
    std::vector<double> u_knots_;
    std::vector<double> v_knots_;
    std::vector<std::vector<Point4D>> control_polygon_;

    Point2D u_internal_interval_;
    Point2D v_internal_interval_;
  };

} // namespace nurbs

#pragma once

#include "curve_2d.hpp"
#include "curve_3d.hpp"

namespace nurbs {
// Nonrational B-Spline Curves
class BSplineCurve2D : public Curve2D {
public:
  static constexpr double kTolerance = std::numeric_limits<double>::epsilon();

  BSplineCurve2D(uint32_t degree, const std::vector<Point2D>& control_points,
                 const std::vector<double> &knots,
                 Point2D interval = {0.0, 1.0});

  Point2D EvaluateCurve(double parameter) const override;

  std::vector<Point2D> Derivatives(double parameter,
                                      uint32_t max_derivative) const;

  // Uses curves points and gives slightly different values than Derivatives
  std::vector<Point2D> Derivatives2(double parameter,
                                       uint32_t max_derivative) const;

 private:
  // Used for Derivatives2
  std::vector<std::vector<Point2D>> DerivativeControlPoints(
      uint32_t max_deriv, uint32_t start, size_t end) const;

  uint32_t degree_;
  std::vector<double> knots_;
  std::vector<Point2D> control_points_;
  double interval_div_ = 1.0;
};

class BSplineCurve3D : public Curve3D {
public:
  static constexpr double kTolerance = std::numeric_limits<double>::epsilon();

  BSplineCurve3D(uint32_t degree, const std::vector<Point3D>& control_points,
                 const std::vector<double> &knots,
                 Point2D interval = {0.0, 1.0});

  Point3D EvaluateCurve(double parameter) const override;

  std::vector<Point3D> Derivatives(double parameter,
                                      uint32_t max_derivative) const;

  // Uses curves points and gives slightly different values than Derivatives
  std::vector<Point3D> Derivatives2(double parameter,
                                       uint32_t max_derivative) const;
  // Used for Derivatives 2 & Surface Derivatives 2
  std::vector<std::vector<Point3D>> DerivativeControlPoints(
      uint32_t max_deriv, uint32_t start, size_t end) const;

 private:
  uint32_t degree_;
   std::vector<double> knots_;
  std::vector<Point3D> control_points_;
  double interval_div_ = 1.0;
};
}  // namespace nurbs
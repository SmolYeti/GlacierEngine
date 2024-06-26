#include "include/parametric_curve.hpp"

namespace nurbs {
ParametricCurve2D::ParametricCurve2D(
    std::array<std::function<double(double)>, 2> functions, Point2D interval)
    : Curve2D(interval), functions_(functions) {}

Point2D ParametricCurve2D::EvaluateCurve(double u) const {
  Point2D point{functions_[0](u), functions_[1](u)};
  return point;
}

ParametricCurve3D::ParametricCurve3D(
    const std::array<std::function<double(double)>, 3>& functions,
    Point2D interval)
    : Curve3D(interval), functions_(functions) {}

Point3D ParametricCurve3D::EvaluateCurve(double u) const {
  Point3D point{functions_[0](u), functions_[1](u), functions_[2](u)};
  return point;
}
}  // namespace nurbs
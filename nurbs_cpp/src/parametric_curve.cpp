#include "include/parametric_curve.hpp"

namespace nurbs {
ParametricCurve2D::ParametricCurve2D(
    std::array<std::function<double(double)>, 2> functions, glm::dvec2 interval)
    : Curve2D(interval), functions_(functions) {}

glm::dvec2 ParametricCurve2D::EvaluateCurve(double u) const {
  glm::dvec2 point{functions_[0](u), functions_[1](u)};
  return point;
}

ParametricCurve3D::ParametricCurve3D(
    const std::array<std::function<double(double)>, 3>& functions,
    glm::dvec2 interval)
    : Curve3D(interval), functions_(functions) {}

glm::dvec3 ParametricCurve3D::EvaluateCurve(double u) const {
  glm::dvec3 point{functions_[0](u), functions_[1](u), functions_[2](u)};
  return point;
}
}  // namespace nurbs
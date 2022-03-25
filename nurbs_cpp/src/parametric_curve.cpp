#include "include/parametric_curve.hpp"

namespace nurbs {
ParametricCurve2D::ParametricCurve2D(
    std::array<std::function<double(double)>, 2> functions, glm::dvec2 interval)
    : Curve2D(interval), functions_(functions) {}

glm::dvec2 ParametricCurve2D::EvaluateCurve(double u) const {
  glm::dvec2 point{functions_[0](u), functions_[1](u)};
  return point;
}

std::vector<glm::dvec2> ParametricCurve2D::EvaluateCurvePoints(
    uint32_t point_count) const {
  std::vector<glm::dvec2> points(point_count);
  double div = (interval_.y - interval_.x) /          //
               static_cast<double>(point_count - 1);  //
  for (uint32_t i = 0; i < point_count; ++i) {
    double u = interval_.x + static_cast<double>(i) * div;
    points[i] = EvaluateCurve(u);
  }
  return points;
}

ParametricCurve3D::ParametricCurve3D(
    const std::array<std::function<double(double)>, 3>& functions,
    glm::dvec2 interval)
    : Curve3D(interval), functions_(functions) {}

glm::dvec3 ParametricCurve3D::EvaluateCurve(double u) const {
  glm::dvec3 point{functions_[0](u), functions_[1](u), functions_[2](u)};
  return point;
}

std::vector<glm::dvec3> ParametricCurve3D::EvaluateCurvePoints(
    uint32_t point_count) const {
  std::vector<glm::dvec3> points(point_count);
  double div = (interval_.y - interval_.x) /          //
               static_cast<double>(point_count - 1);  //
  for (uint32_t i = 0; i < point_count; ++i) {
    double u = interval_.x + static_cast<double>(i) * div;
    points[i] = EvaluateCurve(u);
  }
  return points;
}
}  // namespace nurbs
#include "include/b_spline_curve.hpp"

#include "include/knot_utility_functions.hpp"

namespace nurbs {
BSplineCurve2D::BSplineCurve2D(uint32_t degree,
                               const std::vector<glm::dvec2> &control_points,
                               const std::vector<uint32_t> &knots,
                               glm::dvec2 interval)
    : Curve2D(interval),
      degree_(degree),
      control_points_(control_points),
      knots_(knots) {}

// Chaper 3, ALGORITHM A3.1: CurvePoint p82
glm::dvec2 BSplineCurve2D::EvaluateCurve(double u) const {
  if (u < interval_.x) {
    u = interval_.x;
  }
  if (u > interval_.y) {
    u = interval_.y;
  }
  uint32_t span = knots::FindSpan(degree_, u, knots_);
  std::vector<double> bases = knots::BasisFuns(span, u, degree_, knots_);
  glm::dvec2 point{0.0, 0.0};
  for (uint32_t i = 0; i <= degree_; i++) {
    point += bases[i] * control_points_[span - degree_ + i];
  }
  return point;
}

// Chapter 3, ALGORITHM A3.2: CurveDerivsAlgl p93
std::vector<glm::dvec2> BSplineCurve2D::Derivatives(
    double u, uint32_t max_derivative) const {
  if (u < interval_.x) {
    u = interval_.x;
  }
  if (u > interval_.y) {
    u = interval_.y;
  }
  uint32_t max_deriv = std::min(degree_, max_derivative);
  std::vector<glm::dvec2> derivs(max_deriv + 1, {0, 0});
  uint32_t span = static_cast<uint32_t>(knots::FindSpan(degree_, u, knots_));
  std::vector<std::vector<double>> bases = knots::DersBasisFuns(
      span, u, degree_, static_cast<uint32_t>(control_points_.size() - 1),
      knots_);
  for (uint32_t k = 0; k <= max_deriv; ++k) {
    derivs[k] = {0, 0};
    for (uint32_t j = 0; j <= degree_; ++j) {
      derivs[k] += bases[k][j] * control_points_[span - degree_ + j];
    }
  }
  return derivs;
}

// Chapter 3, ALGORITHM A3.4: CurveDerivsAlg2(n, p, U, P, u, d, CK) p99
std::vector<glm::dvec2> BSplineCurve2D::Derivatives2(
    double u, uint32_t max_derivative) const {
  if (u < interval_.x) {
    u = interval_.x;
  }
  if (u > interval_.y) {
    u = interval_.y;
  }
  uint32_t max_deriv = std::min(degree_, max_derivative);
  std::vector<glm::dvec2> derivs(max_deriv + 1, {0, 0});
  uint32_t span = static_cast<uint32_t>(knots::FindSpan(degree_, u, knots_));
  std::vector<std::vector<double>> bases =
      knots::AllBasisFuns(span, u, degree_, knots_);
  std::vector<std::vector<glm::dvec2>> points =
      DerivativeControlPoints(max_deriv, span - degree_, span);
  for (uint32_t k = 0; k <= max_deriv; ++k) {
    for (uint32_t j = 0; j <= degree_ - k; ++j) {
      derivs[k] += bases[j][degree_ - k] * points[k][j];
    }
  }
  return derivs;
}

// Chapter 3, ALGORITHM A3.3: CurveDerivCpts(n,p,U,P,d,r1,r2,PK)  p98
std::vector<std::vector<glm::dvec2>> BSplineCurve2D::DerivativeControlPoints(
    uint32_t max_deriv, uint32_t start, size_t end) const {
  if (start > control_points_.size() || control_points_.empty()) {
    return {};
  }
  end = std::min(end, control_points_.size() - 1);

  uint32_t total_points = static_cast<uint32_t>(end) - start;
  std::vector<std::vector<glm::dvec2>> points(static_cast<size_t>(degree_ + 1));
  points[0].resize(static_cast<size_t>(total_points + 1));
  for (uint32_t i = 0; i <= total_points; ++i) {
    points[0][i] = control_points_[start + i];
  }
  for (uint32_t k = 1; k <= max_deriv; ++k) {
    points[k].resize(static_cast<size_t>(total_points + 1));
    double temp = static_cast<double>(degree_ - k + 1);
    for (uint32_t i = 0; i <= total_points - k; ++i) {
      points[k][i] = temp * (points[k - 1][i + 1] - points[k - 1][i]) /
                     static_cast<double>(knots_[start + i + degree_ + 1] -
                                         knots_[start + i + k]);
    }
  }
  return points;
}

BSplineCurve3D::BSplineCurve3D(uint32_t degree,
                               const std::vector<glm::dvec3> &control_points,
                               const std::vector<uint32_t> &knots,
                               glm::dvec2 interval)
    : Curve3D(interval),
      degree_(degree),
      control_points_(control_points),
      knots_(knots) {}

// Chaper 3, ALGORITHM A3.1: CurvePoint p82
glm::dvec3 BSplineCurve3D::EvaluateCurve(double u) const {
  if (u < interval_.x) {
    u = interval_.x;
  }
  if (u > interval_.y) {
    u = interval_.y;
  }
  uint32_t span = static_cast<uint32_t>(knots::FindSpan(degree_, u, knots_));
  std::vector<double> bases = knots::BasisFuns(span, u, degree_, knots_);
  glm::dvec3 point{0.0, 0.0, 0.0};
  for (uint32_t i = 0; i <= degree_; i++) {
    point += bases[i] * control_points_[span - degree_ + i];
  }
  return point;
}

// Chapter 3, ALGORITHM A3.2: CurveDerivsAlgl p93
std::vector<glm::dvec3> BSplineCurve3D::Derivatives(
    double u, uint32_t max_derivative) const {
  if (u < interval_.x) {
    u = interval_.x;
  }
  if (u > interval_.y) {
    u = interval_.y;
  }
  uint32_t max_deriv = std::min(degree_, max_derivative);
  std::vector<glm::dvec3> derivs(max_deriv + 1, {0, 0, 0});
  uint32_t span = static_cast<uint32_t>(knots::FindSpan(degree_, u, knots_));
  std::vector<std::vector<double>> bases = knots::DersBasisFuns(
      span, u, degree_, static_cast<uint32_t>(control_points_.size() - 1),
      knots_);
  for (uint32_t k = 0; k <= max_deriv; ++k) {
    derivs[k] = {0, 0, 0};
    for (uint32_t j = 0; j <= degree_; ++j) {
      derivs[k] += bases[k][j] * control_points_[span - degree_ + j];
    }
  }
  return derivs;
}

// Chapter 3, ALGORITHM A3.4: CurveDerivsAlg2(n, p, U, P, u, d, CK) p99
std::vector<glm::dvec3> BSplineCurve3D::Derivatives2(
    double u, uint32_t max_derivative) const {
  if (u < interval_.x) {
    u = interval_.x;
  }
  if (u > interval_.y) {
    u = interval_.y;
  }
  uint32_t max_deriv = std::min(degree_, max_derivative);
  std::vector<glm::dvec3> derivs(max_deriv + 1, {0, 0, 0});
  uint32_t span = static_cast<uint32_t>(knots::FindSpan(degree_, u, knots_));
  std::vector<std::vector<double>> bases =
      knots::AllBasisFuns(span, u, degree_, knots_);
  std::vector<std::vector<glm::dvec3>> points =
      DerivativeControlPoints(max_deriv, span - degree_, span);
  for (uint32_t k = 0; k <= max_deriv; ++k) {
    for (uint32_t j = 0; j <= degree_ - k; ++j) {
      derivs[k] += bases[j][degree_ - k] * points[k][j];
    }
  }
  return derivs;
}

// Chapter 3, ALGORITHM A3.3: CurveDerivCpt p98
std::vector<std::vector<glm::dvec3>> BSplineCurve3D::DerivativeControlPoints(
    uint32_t max_deriv, uint32_t start, size_t end) const {
  if (start > control_points_.size() || control_points_.empty()) {
    return {};
  }
  end = std::min(end, control_points_.size() - 1);

  uint32_t total_points = static_cast<uint32_t>(end) - start;
  std::vector<std::vector<glm::dvec3>> points(static_cast<size_t>(degree_ + 1));
  points[0].resize(static_cast<size_t>(total_points + 1));
  for (uint32_t i = 0; i <= total_points; ++i) {
    points[0][i] = control_points_[start + i];
  }
  for (uint32_t k = 1; k <= max_deriv; ++k) {
    points[k].resize(static_cast<size_t>(total_points + 1));
    double temp = static_cast<double>(degree_ - k + 1);
    for (uint32_t i = 0; i <= total_points - k; ++i) {
      points[k][i] = temp * (points[k - 1][i + 1] - points[k - 1][i]) /
                     static_cast<double>(knots_[start + i + degree_ + 1] -
                                         knots_[start + i + k]);
    }
  }
  return points;
}
}  // namespace nurbs
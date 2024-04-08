#include "include/b_spline_curve.hpp"

#include "include/knot_utility_functions.hpp"

namespace nurbs {
namespace {
double CorrectParameter(double param, double internal_scale,
                        glm::dvec2 interval) {
  double interval_scale = interval.y - interval.x;
  return ((param / interval_scale) + interval.x) * internal_scale;
}
} // namespace

BSplineCurve2D::BSplineCurve2D(uint32_t degree,
                               const std::vector<glm::dvec2> &control_points,
                               const std::vector<uint32_t> &knots,
                               glm::dvec2 interval)
    : Curve2D(interval), degree_(degree), control_points_(control_points),
      knots_(knots) {
  internal_interval_ = interval.y - interval.x;
  if (!knots_.empty()) {
    internal_interval_ = static_cast<double>(knots[knots.size() - 1]);
  }
  if (knots.size() != control_points.size() + degree + 1) {
    throw std::exception("Invalid BSplineCruve2D");
  }
}

// Chaper 3, ALGORITHM A3.1: CurvePoint p82
glm::dvec2 BSplineCurve2D::EvaluateCurve(double param) const {
  if (param < interval_.x) {
    param = interval_.x;
  }
  if (param > interval_.y) {
    param = interval_.y;
  }
  double in_param = CorrectParameter(param, internal_interval_, interval_);
  uint32_t span = knots::FindSpanParam(degree_, knots_, in_param, kTolerance);
  std::vector<double> bases =
      knots::BasisFuns(span, in_param, degree_, knots_, kTolerance);
  glm::dvec2 point{0.0, 0.0};
  for (uint32_t i = 0; i <= degree_; i++) {
    point += bases[i] * control_points_[span - degree_ + i];
  }
  return point;
}

// Chapter 3, ALGORITHM A3.2: CurveDerivsAlgl p93
std::vector<glm::dvec2>
BSplineCurve2D::Derivatives(double param, uint32_t max_derivative) const {
  if (param < interval_.x) {
    param = interval_.x;
  }
  if (param > interval_.y) {
    param = interval_.y;
  }
  double in_param = CorrectParameter(param, internal_interval_, interval_);
  uint32_t max_deriv = std::min(degree_, max_derivative);
  std::vector<glm::dvec2> derivs(max_deriv + 1, {0, 0});
  uint32_t span = knots::FindSpanParam(degree_, knots_, in_param, kTolerance);
  std::vector<std::vector<double>> bases = knots::DersBasisFuns(
      span, in_param, degree_,
      static_cast<uint32_t>(control_points_.size() - 1), knots_);
  for (uint32_t k = 0; k <= max_deriv; ++k) {
    derivs[k] = {0, 0};
    for (uint32_t j = 0; j <= degree_; ++j) {
      derivs[k] += bases[k][j] * control_points_[span - degree_ + j];
    }
  }
  return derivs;
}

// Chapter 3, ALGORITHM A3.4: CurveDerivsAlg2(n, p, U, P, u, d, CK) p99
std::vector<glm::dvec2>
BSplineCurve2D::Derivatives2(double param, uint32_t max_derivative) const {
  if (param < interval_.x) {
    param = interval_.x;
  }
  if (param > interval_.y) {
    param = interval_.y;
  }
  double in_param = CorrectParameter(param, internal_interval_, interval_);
  uint32_t max_deriv = std::min(degree_, max_derivative);
  std::vector<glm::dvec2> derivs(max_deriv + 1, {0, 0});
  uint32_t span = knots::FindSpanParam(degree_, knots_, in_param, kTolerance);
  std::vector<std::vector<double>> bases =
      knots::AllBasisFuns(span, in_param, degree_, knots_);
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
std::vector<std::vector<glm::dvec2>>
BSplineCurve2D::DerivativeControlPoints(uint32_t max_deriv, uint32_t start,
                                        size_t end) const {
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
    : Curve3D(interval), degree_(degree), control_points_(control_points),
      knots_(knots) {
  internal_interval_ = interval.y - interval.x;
  if (!knots_.empty()) {
    internal_interval_ = static_cast<double>(knots[knots.size() - 1]);
  }
  if (knots.size() != control_points.size() + degree + 1) {
    throw std::exception("Invalid BSplineCruve3D");
  }
}

// Chaper 3, ALGORITHM A3.1: CurvePoint p82
glm::dvec3 BSplineCurve3D::EvaluateCurve(double param) const {
  if (param < interval_.x) {
    param = interval_.x;
  }
  if (param > interval_.y) {
    param = interval_.y;
  }
  double in_param = CorrectParameter(param, internal_interval_, interval_);
  uint32_t span = knots::FindSpanParam(degree_, knots_, in_param, kTolerance);
  std::vector<double> bases =
      knots::BasisFuns(span, in_param, degree_, knots_, kTolerance);
  glm::dvec3 point{0.0, 0.0, 0.0};
  for (uint32_t i = 0; i <= degree_; i++) {
    point += bases[i] * control_points_[span - degree_ + i];
  }
  return point;
}

// Chapter 3, ALGORITHM A3.2: CurveDerivsAlgl p93
std::vector<glm::dvec3>
BSplineCurve3D::Derivatives(double param, uint32_t max_derivative) const {
  if (param < interval_.x) {
    param = interval_.x;
  }
  if (param > interval_.y) {
    param = interval_.y;
  }
  double in_param = CorrectParameter(param, internal_interval_, interval_);
  uint32_t max_deriv = std::min(degree_, max_derivative);
  std::vector<glm::dvec3> derivs(max_deriv + 1, {0, 0, 0});
  uint32_t span = knots::FindSpanParam(degree_, knots_, in_param, kTolerance);
  std::vector<std::vector<double>> bases = knots::DersBasisFuns(
      span, in_param, degree_,
      static_cast<uint32_t>(control_points_.size() - 1), knots_);
  for (uint32_t k = 0; k <= max_deriv; ++k) {
    derivs[k] = {0, 0, 0};
    for (uint32_t j = 0; j <= degree_; ++j) {
      derivs[k] += bases[k][j] * control_points_[span - degree_ + j];
    }
  }
  return derivs;
}

// Chapter 3, ALGORITHM A3.4: CurveDerivsAlg2(n, p, U, P, u, d, CK) p99
std::vector<glm::dvec3>
BSplineCurve3D::Derivatives2(double param, uint32_t max_derivative) const {
  if (param < interval_.x) {
    param = interval_.x;
  }
  if (param > interval_.y) {
    param = interval_.y;
  }
  double in_param = CorrectParameter(param, internal_interval_, interval_);
  uint32_t max_deriv = std::min(degree_, max_derivative);
  std::vector<glm::dvec3> derivs(max_deriv + 1, {0, 0, 0});
  uint32_t span = knots::FindSpanParam(degree_, knots_, in_param, kTolerance);
  std::vector<std::vector<double>> bases =
      knots::AllBasisFuns(span, in_param, degree_, knots_);
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
std::vector<std::vector<glm::dvec3>>
BSplineCurve3D::DerivativeControlPoints(uint32_t max_deriv, uint32_t start,
                                        size_t end) const {
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
} // namespace nurbs
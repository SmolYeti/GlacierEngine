#include "include/nurbs_curve.hpp"

#include "include/b_spline_curve.hpp"
#include "include/knot_utility_functions.hpp"

namespace nurbs {
NURBSCurve2D::NURBSCurve2D(uint32_t degree,
                           std::vector<glm::dvec3> control_points,
                           std::vector<uint32_t> knots, glm::dvec2 interval)
    : Curve2D(interval),
      degree_(degree),
      control_points_(control_points),
      knots_(knots) {}

// ALGORITHM A4.1 p.124
glm::dvec2 NURBSCurve2D::EvaluateCurve(double u) const {
  if (u < interval_.x) {
    u = interval_.x;
  }
  if (u > interval_.y) {
    u = interval_.y;
  }
  uint32_t span = static_cast<uint32_t>(knots::FindSpan(degree_, u, knots_));
  std::vector<double> bases = knots::BasisFuns(span, u, degree_, knots_);
  glm::dvec3 temp_point{0.0, 0.0, 0.0};
  for (uint32_t i = 0; i <= degree_; i++) {
    temp_point += bases[i] * control_points_[span - degree_ + i];
  }
  glm::dvec2 point = {temp_point.x / temp_point.z, temp_point.y / temp_point.z};
  return point;
}

// ALGORITHM A4.2 RatCurveDerivs(Aders,wders,d,CK) p.127
//
// The curve point is returned in CK[O] and the kth derivative is returned in
// CK[k].
//
// The array Bin[] [] contains the precomputed binomial coefficients(Bin[n][k]
// (k/i)-> location, not k div i)
// - https://en.wikipedia.org/wiki/Binomial_coefficient
// - ->bin(n, k) = (n * (n - 1) * ... * (n - k + 1) / (k * (k - 1) * ... * 1)
//
// wders is weight derivatives, which I will need to calculate for this.
// - The derivatives A(k)(u) and w^i(u) are obtained using either Eq.(3.3) and
// Algorithm A3.2 or Eq.(3.8) and Algorithm A3 .4.
// - I think this literally means use A3.2 to calculate the weight values and
// then just copy those to the weight vector
//
// ADers should be the derivatives of the point, not the control points, so I
// should just use a b_spline surface to calculate this.

std::vector<double> CurveWeightDerivatives(
    double u, uint32_t degree, const std::vector<uint32_t>& knots,
    const std::vector<double>& weights, uint32_t max_derivative) {
  std::vector<double> derivs(max_derivative + 1, 0.0);
  uint32_t span = static_cast<uint32_t>(knots::FindSpan(degree, u, knots));
  std::vector<std::vector<double>> c_derivs =
      knots::DersBasisFuns(span, u, degree, max_derivative, knots);
  for (uint32_t k = 0; k <= max_derivative; ++k) {
    for (uint32_t j = 0; j <= degree; ++j) {
      derivs[k] += c_derivs[k][j] * weights[span - degree + j];
    }
  }
  return derivs;
}

std::vector<glm::dvec2> NURBSCurve2D::EvaluateDerivative(double u,
                                                         uint32_t d) const {
  if (u < interval_.x) {
    u = interval_.x;
  }
  if (u > interval_.y) {
    u = interval_.y;
  }
  d = std::min(degree_, d);

  std::vector<glm::dvec2> bspl_cpts;
  std::vector<double> weights;
  for (auto& cpt : control_points_) {
    bspl_cpts.push_back({cpt.x, cpt.y});
    weights.push_back(cpt.z);
  }
  BSplineCurve2D b_spline(degree_, bspl_cpts, knots_, interval_);
  auto a_derivs = b_spline.Derivatives(u, d);

  std::vector<std::vector<double>> bin = knots::BinomialCoefficients(d, d);
  // Build the wieght derivatives
  std::vector<double> weight_divs =
      CurveWeightDerivatives(u, degree_, knots_, weights, d);
  // Calculate the derivatives
  std::vector<glm::dvec2> derivs(d + 1);
  for (uint32_t i = 0; i <= d; ++i) {
    glm::dvec2 v = a_derivs[i];

    for (int j = 1; j <= i; ++j) {
      v -= bin[i][j] * weight_divs[j] * derivs[i - j];
    }
    derivs[i] = v / weight_divs[0];
  }

  return derivs;
}

NURBSCurve3D::NURBSCurve3D(uint32_t degree,
                           std::vector<glm::dvec4> control_points,
                           std::vector<uint32_t> knots, glm::dvec2 interval)
    : Curve3D(interval),
      degree_(degree),
      control_points_(control_points),
      knots_(knots) {}

// ALGORITHM A4.1 p.124
glm::dvec3 NURBSCurve3D::EvaluateCurve(double u) const {
  if (u < interval_.x) {
    u = interval_.x;
  }
  if (u > interval_.y) {
    u = interval_.y;
  }
  uint32_t span = static_cast<uint32_t>(knots::FindSpan(degree_, u, knots_));
  std::vector<double> bases = knots::BasisFuns(span, u, degree_, knots_);
  glm::dvec4 temp_point{0.0, 0.0, 0.0, 0.0};
  for (uint32_t i = 0; i <= degree_; i++) {
    temp_point += bases[i] * control_points_[span - degree_ + i];
  }
  glm::dvec3 point = {temp_point.x / temp_point.w, temp_point.y / temp_point.w,
                      temp_point.z / temp_point.w};
  return point;
}

std::vector<glm::dvec3> NURBSCurve3D::EvaluateDerivative(double u,
                                                         uint32_t d) const {
  if (u < interval_.x) {
    u = interval_.x;
  }
  if (u > interval_.y) {
    u = interval_.y;
  }
  d = std::min(degree_, d);

  std::vector<glm::dvec3> bspl_cpts;
  std::vector<double> weights;
  for (auto& cpt : control_points_) {
    bspl_cpts.push_back({cpt.x, cpt.y, cpt.z});
    weights.push_back(cpt.w);
  }
  BSplineCurve3D b_spline(degree_, bspl_cpts, knots_, interval_);
  auto a_derivs = b_spline.Derivatives(u, d);

  std::vector<std::vector<double>> bin = knots::BinomialCoefficients(d, d);
  // Build the wieght derivatives
  std::vector<double> weight_divs =
      CurveWeightDerivatives(u, degree_, knots_, weights, d);
  // Calculate the derivatives
  std::vector<glm::dvec3> derivs(d + 1);
  for (uint32_t i = 0; i <= d; ++i) {
    glm::dvec3 v = a_derivs[i];

    for (int j = 1; j <= i; ++j) {
      v -= bin[i][j] * weight_divs[j] * derivs[i - j];
    }
    derivs[i] = v / weight_divs[0];
  }

  return derivs;
}
}  // namespace nurbs
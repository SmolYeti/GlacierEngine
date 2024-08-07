#include "include/nurbs_curve.hpp"

#include "include/b_spline_curve.hpp"
#include "include/knot_utility_functions.hpp"

namespace nurbs {
NURBSCurve2D::NURBSCurve2D(uint32_t degree, std::vector<Point3D> control_points,
                           std::vector<double> knots, Point2D interval)
    : Curve2D(interval),
      degree_(degree),
      control_points_(control_points),
      knots_(knots) {
  if (knots.size() != control_points.size() + degree + 1) {
    throw std::exception("Invalid BSplineCruve2D");
  }
}

// ALGORITHM A4.1 p.124
Point2D NURBSCurve2D::EvaluateCurve(double param) const {
  double in_param = param;
  if (in_param < interval_.x) {
    in_param = interval_.x;
  }
  if (in_param > interval_.y) {
    in_param = interval_.y;
  }
  uint32_t span = static_cast<uint32_t>(
      knots::FindSpanParam(degree_, knots_, in_param, kTolerance));
  std::vector<double> bases =
      knots::BasisFuns(span, in_param, degree_, knots_, kTolerance);
  Point3D temp_point{0.0, 0.0, 0.0};
  for (uint32_t i = 0; i <= degree_; i++) {
    temp_point += bases[i] * control_points_[span - degree_ + i];
  }
  Point2D point = {temp_point.x / temp_point.z, temp_point.y / temp_point.z};
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

std::vector<double> CurveWeightDerivatives(double param, uint32_t degree,
                                           const std::vector<double> &knots,
                                           const std::vector<double> &weights,
                                           uint32_t max_derivative,
                                           double tolerance) {
  std::vector<double> derivs(max_derivative + 1, 0.0);
  uint32_t span = knots::FindSpanParam(degree, knots, param, tolerance);
  std::vector<std::vector<double>> c_derivs =
      knots::DersBasisFuns(span, param, degree, max_derivative, knots);
  for (uint32_t k = 0; k <= max_derivative; ++k) {
    for (uint32_t j = 0; j <= degree; ++j) {
      derivs[k] += c_derivs[k][j] * weights[span - degree + j];
    }
  }
  return derivs;
}

std::vector<Point2D> NURBSCurve2D::EvaluateDerivative(double param,
                                                      uint32_t d) const {
  double in_param = param;
  if (in_param < interval_.x) {
    in_param = interval_.x;
  }
  if (in_param > interval_.y) {
    in_param = interval_.y;
  }
  d = std::min(degree_, d);

  std::vector<Point2D> bspl_cpts;
  std::vector<double> weights;
  for (auto &cpt : control_points_) {
    bspl_cpts.push_back({cpt.x, cpt.y});
    weights.push_back(cpt.z);
  }
  BSplineCurve2D b_spline(degree_, bspl_cpts, knots_, interval_);
  auto a_derivs = b_spline.Derivatives(param, d);

  std::vector<std::vector<double>> bin = knots::BinomialCoefficients(d, d);
  // Build the wieght derivatives
  std::vector<double> weight_divs =
      CurveWeightDerivatives(in_param, degree_, knots_, weights, d, kTolerance);
  // Calculate the derivatives
  std::vector<Point2D> derivs(d + 1);
  for (uint32_t i = 0; i <= d; ++i) {
    Point2D v = a_derivs[i];

    for (uint32_t j = 1; j <= i; ++j) {
      v -= bin[i][j] * weight_divs[j] * derivs[i - j];
    }
    derivs[i] = v / weight_divs[0];
  }

  return derivs;
}

// ALGORITHM A5.1 CurveKnotins(np, p, UP, Pw, u, k, s, r, nq, UQ, Qw) p.151
NURBSCurve2D NURBSCurve2D::KnotInsertion(double knot, uint32_t times) const {
  // Compute new curve from knot insertion
  // Input: np,p,UP,Pw,u,k,s,r
  // np - Number of control points before insertion
  // p - Degree of the curve
  // UP - Knot vector before insertion
  // Pw - Control points before insertion
  // u - value of knot to insert
  // k - location of knot in insert
  // s - inital knot multiplicity
  // r - times to insert knot
  int p = degree_;
  auto UP = knots_;
  auto Pw = control_points_;
  auto u = knot;
  int k = knots::FindSpanParam(p, UP, knot, kTolerance);
  int s = knots::MultiplicityParam(p, UP, knot, kTolerance);
  int r = times;

  // Max value of r is p - s
  r = std::min(r, p - s);

  // Output: nq,UQ,Qw
  // nq - Number of control points after insertion
  // UQ - Knot vector after insertion
  // Qw - Control points after insertion
  std::vector<double> UQ;
  std::vector<Point3D> Qw;

  UQ.resize(UP.size() + r);
  Qw.resize(Pw.size() + r);

  // Load new knot vector
  for (int i = 0; i <= k; i++) {
    UQ[i] = UP[i];
  }
  for (int i = 1; i <= r; i++) {
    UQ[k + i] = u;
  }
  for (int i = k + 1; i < UP.size(); i++) {
    UQ[i + r] = UP[i];
  }
  std::vector<Point3D> Rw(p + 2);

  // Save unaltered control points
  for (int i = 0; i <= k - p; i++) {
    Qw[i] = Pw[i];
  }
  for (int i = k - s; i < Pw.size(); i++) {
    Qw[i + r] = Pw[i];
  }
  for (int i = 0; i <= p - s; i++) {
    Rw[i] = Pw[k - p + i];
  }

  // Insert the knot r times
  int L = k - p;
  for (int j = 1; j <= r; j++) {
    L = k - p + j;
    for (int i = 0; i <= p - j - s; i++) {
      double alpha = (u - UP[L + i]) / (UP[i + k + 1] - UP[L + i]);
      Rw[i] = (alpha * Rw[i + 1]) + ((1.0 - alpha) * Rw[i]);
    }

    Qw[L] = Rw[0];
    Qw[k + r - j - s] = Rw[p - j - s];
  }

  // Load remaining control points
  for (int i = L + 1; i < k - s; i++) {
    Qw[i] = Rw[i - L];
  }

  return NURBSCurve2D(p, Qw, UQ, interval_);
}

// ALGORITHM A5.2 CurvePntByCornerCut(n, p, U, Pw, u, C) p.153
Point2D NURBSCurve2D::PointByCornerCut(double param) const {
  // Input
  // n - number of control points
  // p - degree of the curve
  // U - knot vector
  // Pw - control points
  auto n = control_points_.size() - 1;
  auto p = degree_;
  const auto &U = knots_;
  const auto &Pw = control_points_;

  // Update u to be in the bounds of the knots
  double in_param = param;
  if (in_param < interval_.x) {
    in_param = interval_.x;
  }
  if (in_param > interval_.y) {
    in_param = interval_.y;
  }

  // Output
  // C - point on the curve
  Point2D C;

  // Handle endpoints
  if (in_param <= static_cast<double>(U[0]) + kTolerance) {
    C = Point2D(Pw[0].x, Pw[0].y) / Pw[0].z;
  } else if (in_param >= static_cast<double>(U[n + p + 1]) - kTolerance) {
    C = Point2D(Pw[n].x, Pw[n].y) / Pw[n].z;
  } else {
    // Local Variables
    int k = knots::FindSpanParam(p, U, in_param, kTolerance);
    int s = knots::MultiplicityParam(p, U, in_param, kTolerance);
    int r = p - s;
    std::vector<Point3D> Rw(r + 1, Point3D(0.0, 0.0, 1.0));

    // Calculate the point
    Rw[0] = Pw[k - p];
    for (uint32_t i = 1; i < r; ++i) {
      Rw[i] = Pw[k - p + i];
    }
    for (uint32_t j = 1; j < r; ++j) {
      int32_t L = k - p + j;
      for (uint32_t i = 0; i < r - j; ++i) {
        // General math is correct, indices may not be though...
        double alpha =
            (in_param - static_cast<double>(U[L + i])) /
            (static_cast<double>(U[i + k + 1]) - static_cast<double>(U[L + i]));
        Rw[i] = (alpha * Rw[i + 1]) + ((1.0 - alpha) * Rw[i]);
      }
    }

    C = Point2D(Rw[0].x, Rw[0].y) / Rw[0].z;
  }
  return C;
}

// ALGORITHM 5.4 RefineKnotVectCurve(n, p, Um Pwm Xm rm Ubar, Qw) p.164
NURBSCurve2D NURBSCurve2D::MergeKnotVect(std::vector<double> knots) const {
  if (knots.empty()) {
    return NURBSCurve2D(degree_, control_points_, knots_, interval_);
  }

  // Input Parameters
  // n - Size of control points
  // p - degree of the curve
  // U - knot vector
  // Pw - control points
  // X - Knot vector to merge
  // r - size of X
  int n = static_cast<int>(control_points_.size()) - 1;
  int p = degree_;
  std::vector<double> U = knots_;
  std::vector<Point3D> Pw = control_points_;
  std::vector<double> X = knots;
  int r = static_cast<int>(X.size()) - 1;

  // Output Parameters
  // Ubar - resulting knot vector
  // Qw - resulting control polygon
  std::vector<Point3D> Qw;
  Qw.resize(Pw.size() + r + 1);
  std::vector<double> Ubar;
  Ubar.resize(U.size() + r + 1);

  int m = n + p + 1;
  int a = knots::FindSpanParam(p, U, X[0], kTolerance);
  int b = knots::FindSpanParam(p, U, X[r], kTolerance);
  b += 1;

  for (int i = 0; i <= a - p; ++i) {
    Qw[i] = Pw[i];
  }
  for (int i = b - 1; i <= n; ++i) {
    Qw[i + r + 1] = Pw[i];
  }
  for (int i = 0; i <= a; ++i) {
    Ubar[i] = U[i];
  }
  for (int i = b + p; i <= m; ++i) {
    Ubar[i + r + 1] = U[i];
  }
  int i = b + p - 1;
  int k = b + p + r;
  for (int j = r; j >= 0; --j) {
    while (X[j] <= U[i] && i > a) {
      Qw[k - p - 1] = Pw[i - p - 1];
      Ubar[k] = U[i];
      k = k - 1;
      i = i - 1;
    }
    Qw[k - p - 1] = Qw[k - p];
    for (int l = 1; l <= p; ++l) {
      int ind = k - p + l;
      double alfa = Ubar[k + l] - X[j];
      if (abs(alfa) < kTolerance) {
        Qw[ind - 1] = Qw[ind];
      } else {
        alfa = alfa / (Ubar[k + l] - U[i - p + l]);
        Qw[ind - 1] = (alfa * Qw[ind - 1]) + ((1.0 - alfa) * Qw[ind]);
      }
    }
    Ubar[k] = X[j];
    k = k - 1;
  }
  return NURBSCurve2D(p, Qw, Ubar, interval_);
}

// ALGORITHM A5.6 DecomposeCurve(n, p, U, Pw, nb, Qw) p.173
std::vector<BezierCurve2D> NURBSCurve2D::Decompose() const {
  // Input:
  // n - Number of points
  // p - degree
  // U - knot vector
  // Pw - Control points
  // Output:
  // nb - number of bezier segments
  // Qw - a vector of points representing nb bezier curves
  auto Pw = control_points_;
  int n = static_cast<int>(Pw.size()) - 1;
  int p = degree_;
  auto U = knots_;

  std::vector<BezierCurve2D> bezier_segments;

  std::vector<std::vector<Point3D>> Qw(2);
  for (auto &vec : Qw) {
    vec.resize(p + 1);
  }

  std::vector<double> alphas(p + 1, 0.0);

  int m = n + p + 1;
  int a = p;
  int b = p + 1;

  for (int i = 0; i <= p; ++i) {
    Qw[0][i] = Pw[i];
  }

  while (b < m) {
    int i = b;
    while (b < m && U[b + 1] == U[b]) {
      ++b;
    }
    int mult = b - i + 1;
    if (mult < p) {
      double numer = U[b] - U[a];  // Numerator of alpha
      // Compute and store alphas
      for (int j = p; j > mult; --j) {
        alphas[j - mult - 1] = numer / (U[a + j] - U[a]);
      }
      int r = p - mult;  // Knot insert r times
      for (int j = 1; j <= r; ++j) {
        int save = r - j;
        int s = mult + j;  // This many new points
        for (int k = p; k >= s; --k) {
          double alpha = alphas[k - s];
          Qw[0][k] = (alpha * Qw[0][k]) + ((1.0 - alpha) * Qw[0][k - 1]);
        }
        if (b < m) {
          // Control point of next segment
          Qw[1][save] = Qw[0][p];
        }
      }
    }

    // Bezier segment complete
    std::vector<Point2D> bezier_points;
    bezier_points.reserve(p + 1);
    for (auto point : Qw[0]) {
      bezier_points.emplace_back(Point2D(point.x, point.y) / point.z);
    }
    bezier_segments.emplace_back(bezier_points);

    if (b < m) {
      Qw[0] = Qw[1];
      Qw[1].clear();
      Qw[1].resize(p + 1);
      // Initialize for next segment
      for (i = std::max(p - mult, 0); i <= p; ++i) {
        Qw[0][i] = Pw[b - p + i];
      }
      a = b;
      b = b + 1;
    }
  }
  return bezier_segments;
}

NURBSCurve3D::NURBSCurve3D(uint32_t degree, std::vector<Point4D> control_points,
                           std::vector<double> knots, Point2D interval)
    : Curve3D(interval),
      degree_(degree),
      control_points_(control_points),
      knots_(knots) {
  if (knots.size() != control_points.size() + degree + 1) {
    throw std::exception("Invalid BSplineCruve2D");
  }
}

// ALGORITHM A4.1 p.124
Point3D NURBSCurve3D::EvaluateCurve(double param) const {
  double in_param = param;
  if (in_param < interval_.x) {
    in_param = interval_.x;
  }
  if (in_param > interval_.y) {
    in_param = interval_.y;
  }
  uint32_t span = static_cast<uint32_t>(
      knots::FindSpanParam(degree_, knots_, in_param, kTolerance));
  std::vector<double> bases =
      knots::BasisFuns(span, in_param, degree_, knots_, kTolerance);
  Point4D temp_point{0.0, 0.0, 0.0, 0.0};
  for (uint32_t i = 0; i <= degree_; i++) {
    temp_point += bases[i] * control_points_[span - degree_ + i];
  }
  Point3D point = {temp_point.x / temp_point.w, temp_point.y / temp_point.w,
                   temp_point.z / temp_point.w};
  return point;
}

std::vector<Point3D> NURBSCurve3D::EvaluateDerivative(double param,
                                                      uint32_t d) const {
  double in_param = param;
  if (in_param < interval_.x) {
    in_param = interval_.x;
  }
  if (in_param > interval_.y) {
    in_param = interval_.y;
  }
  d = std::min(degree_, d);

  std::vector<Point3D> bspl_cpts;
  std::vector<double> weights;
  for (auto &cpt : control_points_) {
    bspl_cpts.push_back({cpt.x, cpt.y, cpt.z});
    weights.push_back(cpt.w);
  }
  BSplineCurve3D b_spline(degree_, bspl_cpts, knots_, interval_);
  auto a_derivs = b_spline.Derivatives(param, d);

  std::vector<std::vector<double>> bin = knots::BinomialCoefficients(d, d);
  // Build the wieght derivatives
  std::vector<double> weight_divs =
      CurveWeightDerivatives(in_param, degree_, knots_, weights, d, kTolerance);
  // Calculate the derivatives
  std::vector<Point3D> derivs(d + 1);
  for (uint32_t i = 0; i <= d; ++i) {
    Point3D v = a_derivs[i];

    for (uint32_t j = 1; j <= i; ++j) {
      v -= bin[i][j] * weight_divs[j] * derivs[i - j];
    }
    derivs[i] = v / weight_divs[0];
  }

  return derivs;
}

// ALGORITHM A5.1 CurveKnotins(np, p, UP, Pw, u, k, s, r, nq, UQ, Qw) p.151
NURBSCurve3D NURBSCurve3D::KnotInsertion(double knot, uint32_t times) const {
  // Compute new curve from knot insertion
  // Input: np,p,UP,Pw,u,k,s,r
  // np - Number of control points before insertion
  // p - Degree of the curve
  // UP - Knot vector before insertion
  // Pw - Control points before insertion
  // u - value of knot to insert
  // k - location of knot in insert
  // s - inital knot multiplicity
  // r - times to insert knot
  auto p = degree_;
  auto UP = knots_;
  auto Pw = control_points_;
  auto u = knot;
  auto k = knots::FindSpanParam(p, UP, knot, kTolerance);
  auto s = knots::MultiplicityParam(p, UP, knot, kTolerance);
  auto r = times;

  // Max value of r is (p + 1) - s
  r = std::min(r, p - s);

  // Output: nq,UQ,Qw
  // nq - Number of control points after insertion
  // UQ - Knot vector after insertion
  // Qw - Control points after insertion
  std::vector<double> UQ;
  std::vector<Point4D> Qw;

  UQ.resize(UP.size() + r);
  Qw.resize(Pw.size() + r);

  // Load new knot vector
  for (uint32_t i = 0; i <= k; i++) {
    UQ[i] = UP[i];
  }
  for (uint32_t i = 1; i <= r; i++) {
    UQ[k + i] = u;
  }
  for (uint32_t i = k + 1; i < UP.size(); i++) {
    UQ[i + r] = UP[i];
  }
  std::vector<Point4D> Rw(p + 1);

  // Save unaltered control points
  for (uint32_t i = 0; i <= k - p; i++) {
    Qw[i] = Pw[i];
  }
  for (uint32_t i = k; i < Pw.size(); i++) {
    Qw[i + r] = Pw[i];
  }
  for (uint32_t i = 0; i <= p; i++) {
    Rw[i] = Pw[k - p + i];
  }

  // Insert the knot r times
  uint32_t L = k - p;
  for (uint32_t j = 1; j <= r; j++) {
    L = k - p + j;
    for (uint32_t i = 0; i <= p - j - s; i++) {
      double alpha =
          (u - static_cast<double>(UP[L + i])) /
          (static_cast<double>(UP[i + k + 1]) - static_cast<double>(UP[L + i]));
      Rw[i] = (alpha * Rw[i + 1]) + ((1.0 - alpha) * Rw[i]);
    }

    Qw[L] = Rw[0];
    Qw[k + r - j] = Rw[p - j];
  }

  // Load remaining control points
  for (uint32_t i = L + 1; i < k; i++) {
    Qw[i] = Rw[i - L];
  }

  return NURBSCurve3D(p, Qw, UQ, interval_);
}

// ALGORITHM A5.2 CurvePntByCornerCut(n, p, U, Pw, u, C) p.153
Point3D NURBSCurve3D::PointByCornerCut(double param) const {
  // Input
  // n - number of control points
  // p - degree of the curve
  // U - knot vector
  // Pw - control points
  auto n = control_points_.size() - 1;
  auto p = degree_;
  const auto &U = knots_;
  const auto &Pw = control_points_;

  // Update u to be in the bounds of the knots
  double in_param = param;
  if (in_param < interval_.x) {
    in_param = interval_.x;
  }
  if (in_param > interval_.y) {
    in_param = interval_.y;
  }

  // Output
  // C - point on the curve
  Point3D C;

  // Handle endpoints
  if (in_param <= static_cast<double>(U[0]) + kTolerance) {
    C = Point3D(Pw[0].x, Pw[0].y, Pw[0].z) / Pw[0].w;
  } else if (in_param >= static_cast<double>(U[n + p + 1]) - kTolerance) {
    C = Point3D(Pw[n].x, Pw[n].y, Pw[n].z) / Pw[n].w;
  } else {
    // Local Variables
    auto k = knots::FindSpanParam(p, U, in_param, kTolerance);
    auto s = knots::MultiplicityParam(p, U, in_param, kTolerance);
    int r = p - s;
    std::vector<Point4D> Rw(r + 1, Point4D(0.0, 0.0, 0.0, 1.0));

    // Calculate the point
    Rw[0] = Pw[k - p];
    for (uint32_t i = 1; i < r; ++i) {
      Rw[i] = Pw[k - p + i];
    }
    for (uint32_t j = 1; j < r; ++j) {
      int32_t L = k - p + j;
      for (uint32_t i = 0; i < r - j; ++i) {
        // General math is correct, indices may not be though...
        double alpha =
            (in_param - static_cast<double>(U[L + i])) /
            (static_cast<double>(U[i + k + 1]) - static_cast<double>(U[L + i]));
        Rw[i] = (alpha * Rw[i + 1]) + ((1.0 - alpha) * Rw[i]);
      }
    }

    C = Point3D(Rw[0].x, Rw[0].y, Rw[0].z) / Rw[0].w;
  }
  return C;
}

// ALGORITHM 5.4 RefineKnotVectCurve(n, p, Um Pwm Xm rm Ubar, Qw) p.164
NURBSCurve3D NURBSCurve3D::MergeKnotVect(std::vector<double> knots) const {
  if (knots.empty()) {
    return NURBSCurve3D(degree_, control_points_, knots_, interval_);
  }

  // Input Parameters
  // n - Size of control points
  // p - degree of the curve
  // U - knot vector
  // Pw - control points
  // X - Knot vector to merge
  // r - size of X
  int n = static_cast<int>(control_points_.size()) - 1;
  int p = degree_;
  std::vector<double> U = knots_;
  std::vector<Point4D> Pw = control_points_;
  std::vector<double> X = knots;
  int r = static_cast<int>(X.size()) - 1;

  // Output Parameters
  // Ubar - resulting knot vector
  // Qw - resulting control polygon
  std::vector<Point4D> Qw;
  Qw.resize(Pw.size() + r + 1);
  std::vector<double> Ubar;
  Ubar.resize(U.size() + r + 1);

  int m = n + p + 1;
  int a = knots::FindSpanParam(p, U, X[0], kTolerance);
  int b = knots::FindSpanParam(p, U, X[r], kTolerance);
  b += 1;

  for (int i = 0; i <= a - p; ++i) {
    Qw[i] = Pw[i];
  }
  for (int i = b - 1; i <= n; ++i) {
    Qw[i + r + 1] = Pw[i];
  }
  for (int i = 0; i <= a; ++i) {
    Ubar[i] = U[i];
  }
  for (int i = b + p; i <= m; ++i) {
    Ubar[i + r + 1] = U[i];
  }
  int i = b + p - 1;
  int k = b + p + r;
  for (int j = r; j >= 0; --j) {
    while (X[j] <= U[i] && i > a) {
      Qw[k - p - 1] = Pw[i - p - 1];
      Ubar[k] = U[i];
      k = k - 1;
      i = i - 1;
    }
    Qw[k - p - 1] = Qw[k - p];
    for (int l = 1; l <= p; ++l) {
      int ind = k - p + l;
      double alfa = Ubar[k + l] - X[j];
      if (abs(alfa) < kTolerance) {
        Qw[ind - 1] = Qw[ind];
      } else {
        alfa = alfa / (Ubar[k + l] - U[i - p + l]);
        Qw[ind - 1] = (alfa * Qw[ind - 1]) + ((1.0 - alfa) * Qw[ind]);
      }
    }
    Ubar[k] = X[j];
    k = k - 1;
  }
  return NURBSCurve3D(p, Qw, Ubar, interval_);
}

// ALGORITHM A5.6 DecomposeCurve(n, p, U, Pw, nb, Qw) p.173
std::vector<BezierCurve3D> NURBSCurve3D::Decompose() const {
  // Input:
  // n - Number of points
  // p - degree
  // U - knot vector
  // Pw - Control points
  // Output:
  // nb - number of bezier segments
  // Qw - a vector of points representing nb bezier curves
  auto Pw = control_points_;
  int n = static_cast<int>(Pw.size()) - 1;
  int p = degree_;
  auto U = knots_;

  std::vector<BezierCurve3D> bezier_segments;

  std::vector<std::vector<Point4D>> Qw(2);
  for (auto &vec : Qw) {
    vec.resize(p + 1);
  }

  std::vector<double> alphas(p + 1, 0.0);

  int m = n + p + 1;
  int a = p;
  int b = p + 1;

  for (int i = 0; i <= p; ++i) {
    Qw[0][i] = Pw[i];
  }

  while (b < m) {
    int i = b;
    while (b < m && U[b + 1] == U[b]) {
      ++b;
    }
    int mult = b - i + 1;
    if (mult < p) {
      double numer = U[b] - U[a];  // Numerator of alpha
      // Compute and store alphas
      for (int j = p; j > mult; --j) {
        alphas[j - mult - 1] = numer / (U[a + j] - U[a]);
      }
      int r = p - mult;  // Knot insert r times
      for (int j = 1; j <= r; ++j) {
        int save = r - j;
        int s = mult + j;  // This many new points
        for (int k = p; k >= s; --k) {
          double alpha = alphas[k - s];
          Qw[0][k] = (alpha * Qw[0][k]) + ((1.0 - alpha) * Qw[0][k - 1]);
        }
        if (b < m) {
          // Control point of next segment
          Qw[1][save] = Qw[0][p];
        }
      }
    }

    // Bezier segment complete
    std::vector<Point3D> bezier_points;
    bezier_points.reserve(p + 1);
    for (auto point : Qw[0]) {
      bezier_points.emplace_back(Point3D(point.x, point.y, point.z) / point.w);
    }
    bezier_segments.emplace_back(bezier_points);

    if (b < m) {
      Qw[0] = Qw[1];
      Qw[1].clear();
      Qw[1].resize(p + 1);
      // Initialize for next segment
      for (i = std::max(p - mult, 0); i <= p; ++i) {
        Qw[0][i] = Pw[b - p + i];
      }
      a = b;
      b = b + 1;
    }
  }
  return bezier_segments;
}

}  // namespace nurbs
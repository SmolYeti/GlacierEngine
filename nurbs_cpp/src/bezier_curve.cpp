#include "nurbs_cpp/include/bezier_curve.hpp"

namespace nurbs {
///
/// Bezier Curve Utility
///

// Chaper 1, Algorithm A1.2 Bernstein, p20
double BezierCurveUtil::Bernstein(size_t i, size_t n, double u) const {
  std::vector<double> temp(n, 0);
  temp[n - i] = 1.0;
  double u_inverse = 1.0 - u;
  for (size_t j = 1; j < n; ++j) {
    for (size_t k = n - 1; k >= j; --k) {
      temp[k] = (u_inverse * temp[k]) + (u * temp[k - 1]);
    }
  }
  return temp[n - 1];
}

// Chaper 1, Algorithm A1.3 AllBernstein, p21
double BezierCurveUtil::AllBernstein(size_t n, double u) const {
  std::vector<double> bernstein(n);

  bernstein[0] = 1.0;
  const double u_inverse = 1.0 - u;
  double saved, temp;

  for (size_t i = 1; i < n, ++i) {
    saved = 0.0;
    for (size_t j = 0; j < i; ++j) {
      temp = bernstein[j];
      bernstein[j] = saved + (u_inverse * temp);
      saved = u * temp;
    }
    bernstein[i] = saved;
  }

  return bernstein;
}

///
/// 2D Bezier Curve
///

BezierCurve2D::BezierCurve2D(std::vector<glm::dvec2> control_points,
                             glm::dvec2 interval)
    : Curve2D(interval), control_points_(control_points) {}

glm::dvec2 BezierCurve2D::EvaluateCurve(double u) const {
  return DeCasteljau(u);
}

std::vector<glm::dvec2> BezierCurve2D::EvaluateCurvePoints(
    uint32_t point_count) const {
  std::vector<glm::dvec2> points(point_count);
  double div = (interval_.y - interval_.x) /          //
               static_cast<double>(point_count - 1);  //
  for (uint32_t i = 0; i < point_count; ++i) {
    double u = interval_.x + (static_cast<double>(i) * div);
    points[i] = EvaluateCurve(u);
  }
  return points;
}

// Chaper 1, Equation 1.9 derivative of a Bezier curve, p22
glm::vec2 BezierCurve2D::Derivative(double u) const {
  const std::vector<double> berstein =
      BezierCurveUtil::AllBernstein(control_points_.size() - 1, u);

  glm::vec2 derivative = {0.0, 0.0};

  for (size_t i = 0; i < berstein.size(); ++i) {
    derivative = berstein[i] * (control_points_[i + 1] - control_points_[i]);
  }

  derivative *= static_cast<double>(berstein.size());

  return derivative;
}

// Chaper 1, Algorithm A1.4 PointOnBezierCurv, p22
glm::dvec2 BezierCurve2D::PointOnBezierCurve(double u) const {
  const std::vector<double> berstein =
      BezierCurveUtil::AllBernstein(control_points_.size(), u);
  glm::dvec2 point = {0.0, 0.0};

  for (size_t i = 0; i < control_points_.size(); ++i) {
    point += berstein[i] * control_points_[i];
  }

  return point;
}

// Chaper 1, Algorithm A1.5 DeCasteljau1, p24
glm::dvec2 BezierCurve2D::DeCasteljau(double u) const {
  std::vector<glm::dvec2> control_points = control_points_;

  const size_t n = control_points.size();
  const u_inverse = 1.0 - u;

  for (size_t i = 1; i < n; ++i) {
    for (size_t j = 0; j < n - i; ++j) {
      control_points[j] =
          (u_inverse * control_points[j]) + (u * control_points[j + 1]);
    }
  }

  return control_points[0];
}

///
/// 3D Bezier Curve
///

BezierCurve3D::BezierCurve3D(std::vector<glm::dvec3> control_points,
                             glm::dvec2 interval)
    : Curve3D(interval), control_points_(control_points) {}

glm::dvec3 BezierCurve3D::EvaluateCurve(double u) const {
  return DeCasteljau(u);
}

std::vector<glm::dvec3> BezierCurve3D::EvaluateCurvePoints(
    uint32_t point_count) const {
  std::vector<glm::dvec3> points(point_count);
  double div = (interval_.y - interval_.x) /          //
               static_cast<double>(point_count - 1);  //
  for (uint32_t i = 0; i < point_count; ++i) {
    double u = interval_.x + (static_cast<double>(i) * div);
    points[i] = EvaluateCurve(u);
  }
  return points;
}

// Chaper 1, Equation 1.9 derivative of a Bezier curve, p22
glm::vec3 BezierCurve3D::Derivative(double u) const {
  const std::vector<double> berstein =
      BezierCurveUtil::AllBernstein(control_points_.size() - 1, u);

  glm::vec3 derivative = {0.0, 0.0, 0.0};

  for (size_t i = 0; i < berstein.size(); ++i) {
    derivative = berstein[i] * (control_points_[i + 1] - control_points_[i]);
  }

  derivative *= static_cast<double>(berstein.size());

  return derivative;
}

// Chaper 1, Algorithm A1.4 PointOnBezierCurv, p22
glm::dvec3 BezierCurve3D::PointOnBezierCurve(double u) const {
  const std::vector<double> berstein =
      BezierCurveUtil::AllBernstein(control_points_.size(), u);
  glm::dvec3 point = {0.0, 0.0};

  for (size_t i = 0; i < control_points_.size(); ++i) {
    point += berstein[i] * control_points_[i];
  }

  return point;
}

// Chaper 1, Algorithm A1.5 DeCasteljau1, p24
glm::dvec3 BezierCurve3D::DeCasteljau(double u) const {
  std::vector<glm::dvec3> control_points = control_points_;

  const size_t n = control_points.size();
  const u_inverse = 1.0 - u;

  for (size_t i = 1; i < n; ++i) {
    for (size_t j = 0; j < n - i; ++j) {
      control_points[j] =
          (u_inverse * control_points[j]) + (u * control_points[j + 1]);
    }
  }

  return control_points[0];
}

}  // namespace nurbs
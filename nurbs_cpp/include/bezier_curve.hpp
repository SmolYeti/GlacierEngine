#pragma once

#include "curve_2d.hpp"
#include "curve_3d.hpp"

namespace nurbs {
class BezierCurveUtil {
 public:
  // Returns:
  // The value of the Bernstein polynomials of i, n - 1 at the value of u
  // B(i,n - 1) (u)
  // Parameters:
  // i - index of berstein polynomial
  // n - degree of berstein polynomial + 1
  // u - input value to berstain polynomial
  static double Bernstein(size_t i, size_t n, double u);

  // Returns:
  // A vector with the values of the Bernstein polynomials of degree n at the
  // fixed value of u B(0,n - 1) (u) to B(n - 1,n - 1)(u) Parameters: n - degree
  // of berstein polynomial + 1 u - input value to berstain polynomial
  static std::vector<double> AllBernstein(size_t n, double u);
};
class BezierCurve2D : public Curve2D {
 public:
  BezierCurve2D(std::vector<glm::dvec2> control_points,
                glm::dvec2 interval = {0.0, 1.0});

  glm::dvec2 EvaluateCurve(double u) const override;
  std::vector<glm::dvec2> EvaluateCurvePoints(
      uint32_t point_count) const override;

  glm::vec2 Derivative(double u)const ;

 private:
  glm::dvec2 PointOnBezierCurve(double u) const;
  glm::dvec2 DeCasteljau(double u) const;

  std::vector<glm::dvec2> control_points_;
};

class BezierCurve3D : public Curve3D {
 public:
  BezierCurve3D(std::vector<glm::dvec3> control_points,
                glm::dvec2 interval = {0.0, 1.0});

  glm::dvec3 EvaluateCurve(double u) const override;
  std::vector<glm::dvec3> EvaluateCurvePoints(
      uint32_t point_count) const override;

  glm::vec3 Derivative(double u) const;

 private:
  glm::dvec3 PointOnBezierCurve(double u) const;
  glm::dvec3 DeCasteljau(double u) const;

  std::vector<glm::dvec3> control_points_;
};
}  // namespace nurbs
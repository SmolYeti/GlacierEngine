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
  BezierCurve2D(std::vector<Point2D> control_points,
                Point2D interval = {0.0, 1.0});

  Point2D EvaluateCurve(double u) const override;

  Point2D Derivative(double u) const;

  Point2D PointOnBezierCurve(double u) const;
  Point2D DeCasteljau(double u) const;

private:
  // Variables
  std::vector<Point2D> control_points_;
};

class BezierCurve3D : public Curve3D {
public:
  BezierCurve3D(std::vector<Point3D> control_points,
                Point2D interval = {0.0, 1.0});

  Point3D EvaluateCurve(double u) const override;

  Point3D Derivative(double u) const;

  Point3D PointOnBezierCurve(double u) const;
  Point3D DeCasteljau(double u) const;

private:
  std::vector<Point3D> control_points_;
};
} // namespace nurbs
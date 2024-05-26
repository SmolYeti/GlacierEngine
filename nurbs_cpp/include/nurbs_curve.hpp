#pragma once

#include "curve_2d.hpp"
#include "curve_3d.hpp"

namespace nurbs {
// Non-Uniform Rational B-Spline Curves
class NURBSCurve2D : public Curve2D {
public:
  static constexpr double kTolerance = std::numeric_limits<double>::epsilon();

  NURBSCurve2D(uint32_t degree, std::vector<Point3D> control_points,
               std::vector<double> knots, Point2D interval = {0.0, 1.0});

  Point2D EvaluateCurve(double parameter) const override;

  std::vector<Point2D> EvaluateDerivative(double parameter,
                                             uint32_t d) const;

  // Method to insert a knot multiple times into the curve and get the resulting
  // curve
  // Parameters:
  // - knot: The value of the knot to insert
  // - time: The number of times to insert the knot
  // Returns:
  // A copy of the curve with the knots inserted
  NURBSCurve2D KnotInsertion(double knot, uint32_t times) const;

  // Method to get a point on the curve by intersecting knots to corner cut the curve
  // Parameters:
  // - parameter: The value to evaluate the curve at
  // Returns:
  // A point on the curve
  Point2D PointByCornerCut(double parameter) const;

  // Merge a knot vector into the curve's knot vector and return the resulting curve
  // Returns:
  // A copy of the curve with the merged in knot vector
  NURBSCurve2D MergeKnotVect(std::vector<double> knots) const;

  const std::vector<double> &knots() const { return knots_; }
  const std::vector<Point3D> &control_points() const { return control_points_; }

private:
  uint32_t degree_;
  std::vector<double> knots_;
  std::vector<Point3D> control_points_;
};

class NURBSCurve3D : public Curve3D {
public:
  static constexpr double kTolerance = std::numeric_limits<double>::epsilon();

  NURBSCurve3D(uint32_t degree, std::vector<Point4D> control_points,
               std::vector<double> knots, Point2D interval = {0.0, 1.0});

  Point3D EvaluateCurve(double parameter) const override;

  std::vector<Point3D> EvaluateDerivative(double parameter,
                                             uint32_t d) const;

  // Method to insert a knot multiple times into the curve and get the resulting
  // curve
  // Parameters:
  // - knot: The value of the knot to insert
  // - time: The number of times to insert the knot
  // Returns:
  // A copy of the curve with the knots inserted
  NURBSCurve3D KnotInsertion(double knot, uint32_t times) const;

  // Method to get a point on the curve by intersecting knots to corner cut the
  // curve Parameters:
  // - u: The value to evaluate the curve at
  // Returns:
  // A point on the curve
  Point3D PointByCornerCut(double parameter) const;

  // Merge a knot vector into the curve's knot vector and return the resulting
  // curve Returns: A copy of the curve with the merged in knot vector
  NURBSCurve3D MergeKnotVect(std::vector<double> knots) const;

  const std::vector<double> &knots() const { return knots_; }
  const std::vector<Point4D> &control_points() const { return control_points_; }

private:
  uint32_t degree_;
  std::vector<double> knots_;
  std::vector<Point4D> control_points_;
};
} // namespace nurbs
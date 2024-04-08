#pragma once

#include "curve_2d.hpp"
#include "curve_3d.hpp"

namespace nurbs {
// Non-Uniform Rational B-Spline Curves
class NURBSCurve2D : public Curve2D {
public:
  static constexpr double kTolerance = std::numeric_limits<double>::epsilon();

  NURBSCurve2D(uint32_t degree, std::vector<glm::dvec3> control_points,
               std::vector<uint32_t> knots, glm::dvec2 interval = {0.0, 1.0});

  glm::dvec2 EvaluateCurve(double parameter) const override;

  std::vector<glm::dvec2> EvaluateDerivative(double parameter,
                                             uint32_t d) const;

  // Method to insert a knot multiple times into the curve and get the resulting
  // curve
  // Parameters:
  // - knot: The value of the knot to insert
  // - time: The number of times to insert the knot
  // Returns:
  // A copy of the curve with the knots inserted
  NURBSCurve2D KnotInsertion(uint32_t knot, uint32_t times) const;

  // Method to get a point on the curve by intersecting knots to corner cut the curve
  // Parameters:
  // - parameter: The value to evaluate the curve at
  // Returns:
  // A point on the curve
  glm::dvec2 PointByCornerCut(double parameter) const;

  const std::vector<uint32_t> &knots() const { return knots_; }

private:
  uint32_t degree_;
  std::vector<uint32_t> knots_;
  std::vector<glm::dvec3> control_points_;
  double internal_interval_;
};

class NURBSCurve3D : public Curve3D {
public:
  static constexpr double kTolerance = std::numeric_limits<double>::epsilon();

  NURBSCurve3D(uint32_t degree, std::vector<glm::dvec4> control_points,
               std::vector<uint32_t> knots, glm::dvec2 interval = {0.0, 1.0});

  glm::dvec3 EvaluateCurve(double parameter) const override;

  std::vector<glm::dvec3> EvaluateDerivative(double parameter,
                                             uint32_t d) const;

  // Method to insert a knot multiple times into the curve and get the resulting
  // curve
  // Parameters:
  // - knot: The value of the knot to insert
  // - time: The number of times to insert the knot
  // Returns:
  // A copy of the curve with the knots inserted
  NURBSCurve3D KnotInsertion(uint32_t knot, uint32_t times) const;

  // Method to get a point on the curve by intersecting knots to corner cut the
  // curve Parameters:
  // - u: The value to evaluate the curve at
  // Returns:
  // A point on the curve
  glm::dvec3 PointByCornerCut(double parameter) const;

  const std::vector<uint32_t> &knots() const { return knots_; }

private:
  uint32_t degree_;
  std::vector<uint32_t> knots_;
  std::vector<glm::dvec4> control_points_;
  double internal_interval_;
};
} // namespace nurbs
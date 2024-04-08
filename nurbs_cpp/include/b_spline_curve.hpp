#pragma once

#include "curve_2d.hpp"
#include "curve_3d.hpp"

namespace nurbs {
// Nonrational B-Spline Curves
class BSplineCurve2D : public Curve2D {
public:
  static constexpr double kTolerance = std::numeric_limits<double>::epsilon();

  BSplineCurve2D(uint32_t degree, const std::vector<glm::dvec2>& control_points,
                 const std::vector<uint32_t>& knots,
                 glm::dvec2 interval = {0.0, 1.0});

  glm::dvec2 EvaluateCurve(double parameter) const override;

  std::vector<glm::dvec2> Derivatives(double parameter,
                                      uint32_t max_derivative) const;

  // Uses curves points and gives slightly different values than Derivatives
  std::vector<glm::dvec2> Derivatives2(double parameter,
                                       uint32_t max_derivative) const;

 private:
  // Used for Derivatives2
  std::vector<std::vector<glm::dvec2>> DerivativeControlPoints(
      uint32_t max_deriv, uint32_t start, size_t end) const;

  uint32_t degree_;
  std::vector<uint32_t> knots_;
  std::vector<glm::dvec2> control_points_;
  double internal_interval_;
};

class BSplineCurve3D : public Curve3D {
public:
  static constexpr double kTolerance = std::numeric_limits<double>::epsilon();

  BSplineCurve3D(uint32_t degree, const std::vector<glm::dvec3>& control_points,
                 const std::vector<uint32_t>& knots,
                 glm::dvec2 interval = {0.0, 1.0});

  glm::dvec3 EvaluateCurve(double parameter) const override;

  std::vector<glm::dvec3> Derivatives(double parameter,
                                      uint32_t max_derivative) const;

  // Uses curves points and gives slightly different values than Derivatives
  std::vector<glm::dvec3> Derivatives2(double parameter,
                                       uint32_t max_derivative) const;
  // Used for Derivatives 2 & Surface Derivatives 2
  std::vector<std::vector<glm::dvec3>> DerivativeControlPoints(
      uint32_t max_deriv, uint32_t start, size_t end) const;

 private:
  uint32_t degree_;
  std::vector<uint32_t> knots_;
  std::vector<glm::dvec3> control_points_;
  double internal_interval_;
};
}  // namespace nurbs
#pragma once

#include "curve_2d.hpp"
#include "curve_3d.hpp"

namespace nurbs {
// Non-Uniform Rational B-Spline Curves
class NURBSCurve2D : public Curve2D {
 public:
  NURBSCurve2D(uint32_t degree, std::vector<glm::dvec3> control_points,
               std::vector<uint32_t> knots, glm::dvec2 interval = {0.0, 1.0});

  glm::dvec2 EvaluateCurve(double u) const override;

  std::vector<glm::dvec2> EvaluateDerivative(double u, uint32_t d) const;

 private:
  uint32_t degree_;
  std::vector<uint32_t> knots_;
  std::vector<glm::dvec3> control_points_;
};

class NURBSCurve3D : public Curve3D {
 public:
  NURBSCurve3D(uint32_t degree, std::vector<glm::dvec4> control_points,
               std::vector<uint32_t> knots, glm::dvec2 interval = {0.0, 1.0});

  glm::dvec3 EvaluateCurve(double u) const override;

  std::vector<glm::dvec3> EvaluateDerivative(double u, uint32_t d) const;

 private:
  uint32_t degree_;
  std::vector<uint32_t> knots_;
  std::vector<glm::dvec4> control_points_;
};
}  // namespace nurbs
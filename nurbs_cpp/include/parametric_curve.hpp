#pragma once

#include "curve_2d.hpp"
#include "curve_3d.hpp"

// STD
#include <array>
#include <functional>

namespace nurbs {
class ParametricCurve2D : public Curve2D {
 public:
  ParametricCurve2D(std::array<std::function<double(double)>, 2> functions,
                    glm::dvec2 interval = {0.0, 1.0});

  glm::dvec2 EvaluateCurve(double u) const override;

 private:
  const std::array<std::function<double(double)>, 2> functions_;
};

class ParametricCurve3D : public Curve3D {
 public:
  ParametricCurve3D(
      const std::array<std::function<double(double)>, 3>& functions,
      glm::dvec2 interval = {0.0, 1.0});

  glm::dvec3 EvaluateCurve(double u) const override;

 private:
  const std::array<std::function<double(double)>, 3> functions_;
};
}  // namespace nurbs
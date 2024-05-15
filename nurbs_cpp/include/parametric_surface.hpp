#pragma once

#include "include/surface.hpp"

// STD
#include <array>
#include <functional>

namespace nurbs {
class ParametricSurface : public Surface {
public:
  ParametricSurface(std::array<std::function<double(Point2D)>, 3> functions,
                    Point2D u_interval = {0.0, 1.0},
                    Point2D v_interval = {0.0, 1.0});

  Point3D EvaluatePoint(Point2D uv) const override;

private:
  const std::array<std::function<double(Point2D)>, 3> functions_;
};
} // namespace nurbs
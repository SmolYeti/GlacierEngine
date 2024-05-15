#include "include/parametric_surface.hpp"

namespace nurbs {
    const double VERY_SMALL_NUMBER = std::numeric_limits<double>::epsilon() * 100.0;

    ParametricSurface::ParametricSurface(std::array<std::function<double(Point2D)>, 3> functions, Point2D u_interval,
        Point2D v_interval) : Surface(u_interval, v_interval), functions_(functions) {}

    Point3D ParametricSurface::EvaluatePoint(Point2D uv) const {
        return { functions_[0](uv), functions_[1](uv), functions_[2](uv) };
    }
}
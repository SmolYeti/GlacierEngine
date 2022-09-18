#include "include/parametric_surface.hpp"

namespace nurbs {
    const double VERY_SMALL_NUMBER = std::numeric_limits<double>::epsilon() * 100.0;

    ParametricSurface::ParametricSurface(std::array<std::function<double(glm::dvec2)>, 3> functions, glm::dvec2 u_interval,
        glm::dvec2 v_interval) : Surface(u_interval, v_interval), functions_(functions) {}

    glm::dvec3 ParametricSurface::EvaluatePoint(glm::dvec2 uv) const {
        return { functions_[0](uv), functions_[1](uv), functions_[2](uv) };
    }
}
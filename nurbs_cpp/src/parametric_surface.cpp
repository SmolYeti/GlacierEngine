#include "include/parametric_surface.hpp"

namespace nurbs {
    const double VERY_SMALL_NUMBER = std::numeric_limits<double>::epsilon() * 100.0;

    ParametricSurface::ParametricSurface(std::array<std::function<double(glm::dvec2)>, 3> functions, glm::dvec2 u_interval,
        glm::dvec2 v_interval) : Surface(u_interval, v_interval), functions_(functions) {}

    glm::dvec3 ParametricSurface::EvaluatePoint(glm::dvec2 uv) const {
        return { functions_[0](uv), functions_[1](uv), functions_[2](uv) };
    }

    std::vector<glm::dvec3> ParametricSurface::EvaluatePoints(
        uint32_t u_sample_count, uint32_t v_sample_count) const {
        std::vector<glm::dvec3> points(u_sample_count * v_sample_count);
        double u_div = (u_interval_.y - u_interval_.x) / static_cast<double>(u_sample_count - 1);
        double v_div = (v_interval_.y - v_interval_.x) / static_cast<double>(v_sample_count - 1);
        for (uint32_t i = 0; i < u_sample_count; ++i) {
            glm::dvec2 uv = { u_interval_.x + static_cast<double>(i) * u_div, 0 };
            for (uint32_t j = 0; j < v_sample_count; ++j) {
                uv.y = v_interval_.x + static_cast<double>(j) * v_div;
                points[(i * v_sample_count) + j] = EvaluatePoint(uv);
            }
        }
        return points;
    }
}
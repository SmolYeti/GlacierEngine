#pragma once

#include "include/surface.hpp"

// STD
#include <array>
#include <functional>

namespace nurbs {
    class ParametricSurface : public Surface {
    public:
        ParametricSurface(std::array<std::function<double(glm::dvec2)>, 3> functions, glm::dvec2 u_interval = { 0.0, 1.0 },
            glm::dvec2 v_interval = { 0.0, 1.0 });

        glm::dvec3 EvaluatePoint(glm::dvec2 uv) const override;

    private:
        const std::array<std::function<double(glm::dvec2)>, 3> functions_;
    };
}
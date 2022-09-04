#pragma once

#include "include/surface.hpp"

namespace nurbs {
    // Nonrational B-Spline Surface
    class BSplineSurface : public Surface {
    public:
        BSplineSurface(glm::dvec2 u_interval = { 0.0, 1.0 },
            glm::dvec2 v_interval = { 0.0, 1.0 });

        glm::dvec3 EvaluatePoint(glm::dvec2 uv) const override;

        std::vector<glm::dvec3> EvaluatePoints(
            uint32_t u_sample_count, uint32_t v_sample_count) const override;

    private:
        uint32_t u_degree_;
        uint32_t v_degree_;
        std::vector<uint32_t> u_knots_;
        std::vector<uint32_t> v_knots_;
        std::vector<glm::dvec3> control_polygon_;
    };
}
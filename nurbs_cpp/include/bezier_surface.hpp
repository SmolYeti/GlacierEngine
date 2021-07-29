#pragma once

#include "include/bezier_curve.hpp"
#include "include/surface.hpp"

// STD
#include <array>
#include <functional>

namespace nurbs {
    class BezierSurface : public Surface {
    public:
        BezierSurface(std::vector<BezierCurve3D> curves, glm::dvec2 u_interval = { 0.0, 1.0 },
            glm::dvec2 v_interval = { 0.0, 1.0 });

        glm::dvec3 EvaluatePoint(glm::dvec2 uv) const override;

        std::vector<glm::dvec3> EvaluatePoints(
            uint32_t u_sample_count, uint32_t v_sample_count) const override;

    private:
        std::vector<BezierCurve3D> curves_;
    };

}
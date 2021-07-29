#pragma once

// GLM
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

// STD
#include <vector>

namespace nurbs {
    class Curve2D {
    public:
        Curve2D(glm::dvec2 interval = { 0.0, 1.0 }) : interval_(interval) {}

        virtual glm::dvec2 EvaluateCurve(double u) const { return { 0, 0 }; }

        virtual std::vector<glm::dvec2> EvaluateCurvePoints(
            uint32_t point_count) const {
            std::vector<glm::dvec2> points(point_count);
            const double div = (interval_.y - interval_.x) /
                static_cast<double>(point_count - 1);
            for (uint32_t i = 0; i < point_count; ++i) {
                double u = interval_.x + (static_cast<double>(i) * div);
                points[i] = EvaluateCurve(u);
            }
            return points;
        }

        void interval(glm::dvec2 interval) { interval_ = interval; }
        glm::dvec2 interval() { return interval_; }

    protected:
        glm::dvec2 interval_;
    };
}  // namespace nurbs
#pragma once

// GLM
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

// STD
#include <vector>

namespace nurbs {
    class Surface {
    public:
        Surface(glm::dvec2 u_interval = { 0.0, 1.0 },
            glm::dvec2 v_interval = { 0.0, 1.0 })
            : u_interval_(u_interval), v_interval_(v_interval) {}

        virtual glm::dvec3 EvaluatePoint(glm::dvec2 uv) const { return { 0, 0, 0 }; }
        virtual std::vector<glm::dvec3> EvaluatePoints(
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

        void u_interval(glm::dvec2 interval) { u_interval_ = interval; }
        glm::dvec2 u_interval() const { return u_interval_; }

        void v_interval(glm::dvec2 interval) { v_interval_ = interval; }
        glm::dvec2 v_interval() const { return v_interval_; }

    protected:
        glm::dvec2 u_interval_;
        glm::dvec2 v_interval_;
    };
}  // namespace nurbs
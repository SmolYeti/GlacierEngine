#pragma once

#include "curve_2d.hpp"
#include "curve_3d.hpp"


namespace nurbs {
    // Nonrational B-Spline Curves
    class BSplineCurve2D : public Curve2D {
    public:
        BSplineCurve2D(uint32_t degree, std::vector<glm::dvec2> control_points, std::vector<uint32_t> knots, glm::dvec2 interval = { 0.0, 1.0 });

        glm::dvec2 EvaluateCurve(double u) const override;
    private:
        uint32_t degree_;
        std::vector<uint32_t> knots_;
        std::vector<glm::dvec2> control_points_;
    };

    class BSplineCurve3D : public Curve3D {
    public:
        BSplineCurve3D(uint32_t degree, std::vector<glm::dvec3> control_points, std::vector<uint32_t> knots, glm::dvec2 interval = { 0.0, 1.0 });

        glm::dvec3 EvaluateCurve(double u) const override;
    private:
        uint32_t degree_;
        std::vector<uint32_t> knots_;
        std::vector<glm::dvec3> control_points_;
    };
}
#pragma once

#include "curve_2d.hpp"
#include "curve_3d.hpp"

namespace nurbs {
    // Nonrational B-Spline Curves
    class BSplineCurve2D : public Curve2D {
    public:
        BSplineCurve2D(uint32_t degree, std::vector<glm::dvec2> control_points, std::vector<uint32_t> knots, glm::dvec2 interval = { 0.0, 1.0 });

        glm::dvec2 EvaluateCurve(double u) const override;

        std::vector<glm::dvec2> Derivatives(double u, uint32_t max_derivative) const;

        // Warning - untested & incorrect method, just implemented for completeness
        std::vector<glm::dvec2> Derivatives2(double u, uint32_t max_derivative) const;
    private:
        // Warning - untested method, just implemented for completeness
        std::vector<std::vector<glm::dvec2>> DerivativeControlPoints(uint32_t max_deriv, uint32_t start, size_t end) const;

        uint32_t degree_;
        std::vector<uint32_t> knots_;
        std::vector<glm::dvec2> control_points_;
    };

    class BSplineCurve3D : public Curve3D {
    public:
        BSplineCurve3D(uint32_t degree, std::vector<glm::dvec3> control_points, std::vector<uint32_t> knots, glm::dvec2 interval = { 0.0, 1.0 });

        glm::dvec3 EvaluateCurve(double u) const override;

        std::vector<glm::dvec3> Derivatives(double u, uint32_t max_derivative) const;
    private:
        uint32_t degree_;
        std::vector<uint32_t> knots_;
        std::vector<glm::dvec3> control_points_;
    };
}
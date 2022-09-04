#include "include/b_spline_curve.hpp"

#include "include/knot_utility_functions.hpp"

namespace nurbs {
    BSplineCurve2D::BSplineCurve2D(uint32_t degree, std::vector<glm::dvec2> control_points,
        std::vector<uint32_t> knots, glm::dvec2 interval)
        : Curve2D(interval), degree_(degree), control_points_(control_points), knots_(knots) {}

    // Chaper 3, ALGORITHM A3.1: CurvePoint p82
    glm::dvec2 BSplineCurve2D::EvaluateCurve(double u) const {
        uint32_t span = static_cast<uint32_t>(knots::FindSpan(degree_, u, knots_));
        if (u > interval_.y || std::abs(u - interval_.y) < std::numeric_limits<double>::epsilon()) {
            --span;
        }
        std::vector<double> bases = knots::BasisFuns(span, u, degree_, knots_);
        glm::dvec2 point{ 0.0, 0.0 };
        for (uint32_t i = 0; i <= degree_; i++) {
            point += bases[i] * control_points_[span - degree_ + i];
        }
        return point;
    }

    BSplineCurve3D::BSplineCurve3D(uint32_t degree, std::vector<glm::dvec3> control_points,
        std::vector<uint32_t> knots, glm::dvec2 interval) :
        Curve3D(interval), degree_(degree), control_points_(control_points), knots_(knots) {
    }

    // Chaper 3, ALGORITHM A3.1: CurvePoint p82
    glm::dvec3 BSplineCurve3D::EvaluateCurve(double u) const {
        int span = knots::FindSpan(degree_, u, knots_);
        if (u > interval_.y || std::abs(u - interval_.y) < std::numeric_limits<double>::epsilon()) {
            --span;
        }
        std::vector<double> bases = knots::BasisFuns(span, u, degree_, knots_);
        glm::dvec3 point{ 0.0, 0.0, 0.0 };
        for (uint32_t i = 0; i <= degree_; i++) {
            point += bases[i] * control_points_[span - degree_ + i];
        }
        return point;
    }
}
#include "include/nurbs_curve.hpp"

#include "include/knot_utility_functions.hpp"

namespace nurbs {
    NURBSCurve2D::NURBSCurve2D(uint32_t degree, std::vector<glm::dvec3> control_points,
        std::vector<uint32_t> knots, glm::dvec2 interval) :
        Curve2D(interval),
        degree_(degree),
        control_points_(control_points),
        knots_(knots) {}


    glm::dvec2 NURBSCurve2D::EvaluateCurve(double u) const {
        if (u < interval_.x) { u = interval_.x; }
        if (u > interval_.y) { u = interval_.y; }
        uint32_t span = static_cast<uint32_t>(knots::FindSpan(degree_, u, knots_));
        std::vector<double> bases = knots::BasisFuns(span, u, degree_, knots_);
        glm::dvec3 temp_point{ 0.0, 0.0, 0.0 };
        for (uint32_t i = 0; i <= degree_; i++) {
            temp_point += bases[i] * control_points_[span - degree_ + i];
        }
        glm::dvec2 point = { temp_point.x / temp_point.z, temp_point.y / temp_point.z };
        return point;
    }

    NURBSCurve3D::NURBSCurve3D(uint32_t degree, std::vector<glm::dvec4> control_points,
        std::vector<uint32_t> knots, glm::dvec2 interval) :
        Curve3D(interval),
        degree_(degree),
        control_points_(control_points),
        knots_(knots) {}

    glm::dvec3 NURBSCurve3D::EvaluateCurve(double u) const {
        if (u < interval_.x) { u = interval_.x; }
        if (u > interval_.y) { u = interval_.y; }
        uint32_t span = static_cast<uint32_t>(knots::FindSpan(degree_, u, knots_));
        std::vector<double> bases = knots::BasisFuns(span, u, degree_, knots_);
        glm::dvec4 temp_point{ 0.0, 0.0, 0.0 , 0.0};
        for (uint32_t i = 0; i <= degree_; i++) {
            temp_point += bases[i] * control_points_[span - degree_ + i];
        }
        glm::dvec3 point = { temp_point.x / temp_point.w,
                             temp_point.y / temp_point.w,
                             temp_point.z / temp_point.w };
        return point;
    }
}
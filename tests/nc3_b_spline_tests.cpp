#include <gtest/gtest.h>

// NURBS_CPP
#include "include/b_spline_curve.hpp"
#include "include/b_spline_surface.hpp"
#include "include/bezier_curve.hpp"
#include "include/knot_utility_functions.hpp"

namespace nurbs {
    TEST(NURBS_Chapter3, BSplineCurveBezierEquivalence) {
        std::vector<glm::dvec2> control_points = { {0,0}, {0,1}, {1,1}, {1, 0} };
        BezierCurve2D bezier(control_points);
        BSplineCurve2D b_spline(3, control_points, { 0,0,0,0,1,1,1,1 });
        double div = 1.0 / 99.0;
        for (uint32_t i = 0; i < 100; ++i) {
            double location = static_cast<double>(i) * div;
            glm::dvec2 point_0 = bezier.EvaluateCurve(location);
            glm::dvec2 point_1 = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point_0.x, point_1.x);
            EXPECT_DOUBLE_EQ(point_0.y, point_1.y);
        }
    }

    TEST(NURBS_Chapter3, BSplineCurveMin) {

    }

    TEST(NURBS_Chapter3, BSplineCurveMid) {

    }

    TEST(NURBS_Chapter3, BSplineCurveMax) {

    }

}
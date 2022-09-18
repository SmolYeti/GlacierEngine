#include <gtest/gtest.h>

// NURBS_CPP
#include "include/b_spline_curve.hpp"
#include "include/b_spline_surface.hpp"
#include "include/nurbs_curve.hpp"


namespace nurbs {
    TEST(NURBS_Chapter4, NURBS_BSpline_Curve2D) {
        uint32_t degree = 3;
        std::vector<uint32_t> knots = { 0,0,0,0,1, 2, 2, 3, 4, 4, 5,5,5,5 };
        // B-Spline Curve
        std::vector<glm::dvec2> control_points = { {0,0}, {0,1}, {1,1}, {1, 0}, {2, 0}, {2, 1}, {3, 1}, {3, 0} };
        BSplineCurve2D b_spline(degree, control_points, knots);
        // NURBS Curves
        std::vector<glm::dvec3> nurbs_pts;
        nurbs_pts.reserve(control_points.size());
        for (auto& pt : control_points) {
            nurbs_pts.push_back({pt.x, pt.y, 1.0});
        }
        NURBSCurve2D nurbs_curve(degree, nurbs_pts, knots);
        // Compare
        double div = 1.0 / 99.0;
        for (int32_t i = -1; i < 101; ++i) {
            double location = static_cast<double>(i) * div;
            glm::dvec2 point_bspl = b_spline.EvaluateCurve(location);
            glm::dvec2 point_nurbs = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point_bspl.x, point_nurbs.x);
            EXPECT_DOUBLE_EQ(point_bspl.y, point_nurbs.y);
        }
    }
}
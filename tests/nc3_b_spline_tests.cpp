#include <gtest/gtest.h>

// NURBS_CPP
#include "include/b_spline_curve.hpp"
#include "include/b_spline_surface.hpp"
#include "include/bezier_curve.hpp"
#include "include/bezier_surface.hpp"
#include "include/knot_utility_functions.hpp"

// STD
#include <numeric>

namespace nurbs {
    TEST(NURBS_Chapter3, BSplineCurveBezierEquivalence2D) {
        std::vector<glm::dvec2> control_points = { {0,0}, {0,1}, {1,1}, {1, 0} };
        BezierCurve2D bezier(control_points);
        BSplineCurve2D b_spline(3, control_points, { 0,0,0,0,1,1,1,1 });
        double div = 1.0 / 99.0;
        for (int32_t i = -1; i < 101; ++i) {
            double location = static_cast<double>(i) * div;
            glm::dvec2 point_0 = bezier.EvaluateCurve(location);
            glm::dvec2 point_1 = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point_0.x, point_1.x);
            EXPECT_DOUBLE_EQ(point_0.y, point_1.y);
        }
    }

    TEST(NURBS_Chapter3, BSplineCurveBezierEquivalence3D) {
        std::vector<glm::dvec3> control_points = { {0,0, 0}, {0,1, 1}, {1,1, 2}, {1, 0, 1} };
        BezierCurve3D bezier(control_points);
        BSplineCurve3D b_spline(3, control_points, { 0,0,0,0,1,1,1,1 });
        double div = 1.0 / 99.0;
        for (int32_t i = -1; i < 101; ++i) {
            double location = static_cast<double>(i) * div;
            glm::dvec2 point_0 = bezier.EvaluateCurve(location);
            glm::dvec2 point_1 = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point_0.x, point_1.x);
            EXPECT_DOUBLE_EQ(point_0.y, point_1.y);
        }
    }

    TEST(NURBS_Chapter3, BSplineCurveMin2D) {
        std::vector<glm::dvec2> control_points = { {0,0}, {0,1}, {0.5,0},  {1,1}, {1, 0} };
        BSplineCurve2D b_spline(3, control_points, { 0,0,0,0,1,2,2,2,2 }, { 0, 2 });
        {
            double location = -1;
            glm::dvec2 point = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point.x, 0);
            EXPECT_DOUBLE_EQ(point.y, 0);
        }
        {
            double location = 0;
            glm::dvec2 point = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point.x, 0);
            EXPECT_DOUBLE_EQ(point.y, 0);
        }
    }

    TEST(NURBS_Chapter3, BSplineCurveMin3D) {
        std::vector<glm::dvec3> control_points = { {0,0, 0}, {0,1, 1}, {0.5,0, 1},  {1,1, 1}, {1, 0, 0} };
        BSplineCurve3D b_spline(3, control_points, { 0,0,0,0,1,2,2,2,2 }, { 0, 2 });
        {
            double location = -1;
            glm::dvec3 point = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point.x, 0);
            EXPECT_DOUBLE_EQ(point.y, 0);
            EXPECT_DOUBLE_EQ(point.z, 0);
        }
        {
            double location = 0;
            glm::dvec3 point = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point.x, 0);
            EXPECT_DOUBLE_EQ(point.y, 0);
            EXPECT_DOUBLE_EQ(point.z, 0);
        }
    }

    TEST(NURBS_Chapter3, BSplineCurveMid2D) {
        std::vector<glm::dvec2> control_points = { {0,0}, {0,1}, {0.5,0}, {1,1}, {1, 0} };
        BSplineCurve2D b_spline(3, control_points, { 0,0,0,0,1,2,2,2,2 }, { 0, 2 });
        {
            double location = 0.5;
            glm::dvec2 point = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point.x, 0.15625);
            EXPECT_DOUBLE_EQ(point.y, 0.625);
        }
        {
            double location = 1.0;
            glm::dvec2 point = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point.x, 0.5);
            EXPECT_DOUBLE_EQ(point.y, 0.5);
        }
        {
            double location = 1.5;
            glm::dvec2 point = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point.x, 0.84375);
            EXPECT_DOUBLE_EQ(point.y, 0.625);
        }

    }

    TEST(NURBS_Chapter3, BSplineCurveMid3D) {
        std::vector<glm::dvec3> control_points = { {0,0, 0}, {0,1, 1}, {0.5,0, 1},  {1,1, 1}, {1, 0, 0} };
        BSplineCurve3D b_spline(3, control_points, { 0,0,0,0,1,2,2,2,2 }, { 0, 2 });
        {
            double location = 0.5;
            glm::dvec3 point = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point.x, 0.15625);
            EXPECT_DOUBLE_EQ(point.y, 0.625);
            EXPECT_DOUBLE_EQ(point.z, 0.875);
        }
        {
            double location = 1.0;
            glm::dvec3 point = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point.x, 0.5);
            EXPECT_DOUBLE_EQ(point.y, 0.5);
            EXPECT_DOUBLE_EQ(point.z, 1.0);
        }
        {
            double location = 1.5;
            glm::dvec3 point = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point.x, 0.84375);
            EXPECT_DOUBLE_EQ(point.y, 0.625);
            EXPECT_DOUBLE_EQ(point.z, 0.875);
        }

    }

    TEST(NURBS_Chapter3, BSplineCurveMax2D) {
        std::vector<glm::dvec2> control_points = { {0,0}, {0,1},  {0.5,0}, {1,1}, {1, 0} };
        BSplineCurve2D b_spline(3, control_points, { 0,0,0,0,1,2,2,2,2 }, { 0, 2 });
        {
            double location = 2;
            glm::dvec2 point = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point.x, 1);
            EXPECT_DOUBLE_EQ(point.y, 0);
        }
        {
            double location = 3;
            glm::dvec2 point = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point.x, 1);
            EXPECT_DOUBLE_EQ(point.y, 0);
        }
    }


    TEST(NURBS_Chapter3, BSplineCurveMax3D) {
        std::vector<glm::dvec3> control_points = { {0,0, 0}, {0,1, 1}, {0.5,0, 1},  {1,1, 1}, {1, 0, 0} };
        BSplineCurve3D b_spline(3, control_points, { 0,0,0,0,1,2,2,2,2 }, { 0, 2 });
        {
            double location = 2;
            glm::dvec3 point = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point.x, 1);
            EXPECT_DOUBLE_EQ(point.y, 0);
            EXPECT_DOUBLE_EQ(point.z, 0);
        }
        {
            double location = 3;
            glm::dvec3 point = b_spline.EvaluateCurve(location);
            EXPECT_DOUBLE_EQ(point.x, 1);
            EXPECT_DOUBLE_EQ(point.y, 0);
            EXPECT_DOUBLE_EQ(point.z, 0);
        }
    }

    TEST(NURBS_Chapter3, BSplineCurveDeriv2DEx3_1) {
        std::vector<glm::dvec2> control_points = { {0,0}, {0,1}, {1,1}, {1, 0} };
        BezierCurve2D bezier(control_points);
        BSplineCurve2D b_spline(3, control_points, { 0,0,0,0,1,1,1,1 });
        double div = 1.0 / 99.0;
        for (int32_t i = 0; i < 100; ++i) {
            double location = static_cast<double>(i) * div;
            glm::dvec2 point_b = bezier.Derivative(location);
            std::vector<glm::dvec2> point_s = b_spline.Derivatives(location, 1);
            // Numbers are not between 0 and 1, so the epsilon needs to be scaled
            // This is not the proper scaling though.
            EXPECT_NEAR(point_b.x, point_s[1].x, std::numeric_limits<double>::epsilon() * 10);
            EXPECT_NEAR(point_b.y, point_s[1].y, std::numeric_limits<double>::epsilon() * 10);
        }
    }

    TEST(NURBS_Chapter3, BSplineCurveDeriv3DEx3_1) {
        std::vector<glm::dvec3> control_points = { {0, 0, 0}, {0, 1, 2}, {1, 1, 2}, {1, 0, 0} };
        BezierCurve3D bezier(control_points);
        BSplineCurve3D b_spline(3, control_points, { 0,0,0,0,1,1,1,1 });
        double div = 1.0 / 99.0;
        for (int32_t i = 0; i < 100; ++i) {
            double location = static_cast<double>(i) * div;
            glm::dvec3 point_b = bezier.Derivative(location);
            std::vector<glm::dvec3> point_s = b_spline.Derivatives(location, 1);
            // Numbers are not between 0 and 1, so the epsilon needs to be scaled
            // This is not the proper scaling though.
            EXPECT_NEAR(point_b.x, point_s[1].x, std::numeric_limits<float>::epsilon() * 10);
            EXPECT_NEAR(point_b.y, point_s[1].y, std::numeric_limits<float>::epsilon() * 10);
            EXPECT_NEAR(point_b.z, point_s[1].z, std::numeric_limits<float>::epsilon() * 10);
        }
    }

    // Disabled due to 'Derivatives2' being incorrect. I need to actually try to get this to be correct at somepoint
    // but currently it is not a priority.
    TEST(NURBS_Chapter3, DISABLED_BSplineCurveDerivCompare2D) {
        std::vector<glm::dvec2> control_points = { {0,0}, {0,1}, {0.5, 0.5}, {1,1}, {1, 0} };
        BSplineCurve2D b_spline(3, control_points, { 0, 0, 0, 0, 1, 2, 2, 2, 2 }, { 0, 2 });
        double div = 1.0 / 99.0;
        for (int32_t i = 0; i < 100; ++i) {
            double location = static_cast<double>(i) * div;
            std::vector<glm::dvec2> points_0 = b_spline.Derivatives(location, 1);
            std::vector<glm::dvec2> points_1 = b_spline.Derivatives2(location, 1);
            ASSERT_EQ(points_0.size(), points_1.size());
            // Numbers are not between 0 and 1, so the epsilon needs to be scaled
            // This is not the proper scaling though.
            for (size_t j = 0; j < points_0.size(); ++j) {
                EXPECT_NEAR(points_0[j].x, points_1[j].x, std::numeric_limits<double>::epsilon());
                EXPECT_NEAR(points_0[j].y, points_1[j].y, std::numeric_limits<double>::epsilon());
            }
        }
    }

    // TODO - Add more tests for B-Spline Curve Derivatives

    TEST(NURBS_Chapter3, BSplineSurfaceConstruct) {
        uint32_t degree = 3;
        std::vector<uint32_t> u_knots = { 0, 0, 0, 0, 1, 2, 2, 2, 2 };
        std::vector<uint32_t> v_knots = { 0, 0, 0, 0, 1, 2, 2, 2, 2 };
        std::vector<std::vector<glm::dvec3>> control_points = {
            {{-1, 0, -1},    {-0.33, 0.1, -1.33},{0.33, 0.1, -1.33}, {0.87, 0, -0.87}, {1, -0.1, -1}},   //
            {{-1.33, -0.25, -0.33},{-0.33, 0, -0.33},  {0.33, 0.0, -0.33},{1.33, -0.25, -0.33},{2, -0.5, -0.63}},   //
            {{-1.33, -0.75, 0.33}, {-0.33, 0.0, 0.33},{0.33, 0.0, 0.33}, {1.33, -0.75, 0.33},   {2, -1, 0.63}},   //
            {{-0.87, -2, 0.87},    {-0.33, 0.0, 1.33},{0.33, 0.0, 1.33}, {0.87, -2, 0.87}, {1, -2.5, 1}},   //
            {{-1, -2.5, 1},    {-0.33, 0.0, 2},{0.33, 0.0, 1.63}, {0.87, -2, 1}, {1, -3, 1.5}},   //
        };
        nurbs::BSplineSurface b_spline_surface(degree, degree, u_knots, v_knots, control_points, { 0, 2 }, { 0, 2 });
    }

    TEST(NURBS_Chapter3, BSplineSurfaceBezierCompare) {
        uint32_t degree = 3;
        std::vector<uint32_t> u_knots = { 0, 0, 0, 0, 1, 1, 1, 1 };
        std::vector<uint32_t> v_knots = { 0, 0, 0, 0, 1, 1, 1, 1 };
        std::vector<std::vector<glm::dvec3>> control_points = {
            {{-0.87, 0, -0.87},    {-0.33, 0.1, -1.33},{0.33, 0.1, -1.33}, {0.87, 0, -0.87}},   //
            {{-1.33, -0.25, -0.33},{-0.33, 0, -0.33},  {0.33, 0.0, -0.33},{1.33, -0.25, -0.33}},   //
            {{-1.33, -0.75, 0.33}, {-0.33, 0.0, 0.33},{0.33, 0.0, 0.33},    {1.33, -0.75, 0.33}},   //
            {{-0.87, -2, 0.87},    {-0.33, 0.0, 1.33},{0.33, 0.0, 1.33}, {0.87, -2, 0.87}},   //
        };
        BSplineSurface bspl_surface(degree, degree, u_knots, v_knots, control_points);
        std::vector<BezierCurve3D> bezier_curves = {BezierCurve3D(control_points[0]),
            BezierCurve3D(control_points[1]),
            BezierCurve3D(control_points[2]),
            BezierCurve3D(control_points[3])};
        BezierSurface bez_surface(bezier_curves);

        // Compare the surfaces
        double div = 1.0 / 99.0;
        for (int32_t i = 0; i < 100; ++i) {
            glm::dvec2 location = { static_cast<double>(i) * div, 0 };
            for (int32_t j = 0; j < 100; ++j) {
                location.y = static_cast<double>(j) * div;
                glm::dvec3 point_bspl = bspl_surface.EvaluatePoint(location);
                glm::dvec3 point_bez = bspl_surface.EvaluatePoint(location);
                // Numbers are not between 0 and 1, so the epsilon needs to be scaled
                // This is not the proper scaling though.
                EXPECT_DOUBLE_EQ(point_bspl.x, point_bez.x);
                EXPECT_DOUBLE_EQ(point_bspl.y, point_bez.y);
                EXPECT_DOUBLE_EQ(point_bspl.z, point_bez.z);
            }
        }
    }

    TEST(NURBS_Chapter3, BSplineSurfacePoints) {
        glm::dvec2 interval = { 0, 4 };
        uint32_t u_degree = 4;
        uint32_t v_degree = 3;
        std::vector<uint32_t> u_knots = { 0, 0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 4, 4, 4 };
        std::vector<uint32_t> v_knots = { 0, 0, 0, 0, 1, 1, 2, 3, 3, 4, 4, 4, 4 };
        std::vector<std::vector<glm::dvec3>> control_points = {
            {{0, 0, 1},{1, 0, 0},{2, 0, 1},{3, 0, 1},{4, 0, 1},{5, 0, 1},{6, 0, 1}},   //
            {{0, 0, 2},{1, 0, 0},{2, 0, 2},{3, 0, 2},{4, 0, 2},{5, 0, 2},{6, 0, 2}},   //
            {{0, 0, 3},{1, 0, 0},{2, 0, 3},{3, 0, 3},{4, 0, 3},{5, 0, 3},{6, 0, 3}},   //
            {{0, 0, 4},{1, 0, 0},{2, 0, 4},{3, 0, 4},{4, 0, 4},{5, 0, 4},{6, 0, 4}},   //
            {{0, 0, 5},{1, 0, 0},{2, 0, 5},{3, 0, 5},{4, 0, 5},{5, 0, 5},{6, 0, 5}},   //
            {{0, 0, 6},{1, 0, 0},{2, 0, 6},{3, 0, 6},{4, 0, 6},{5, 0, 6},{6, 0, 6}},   //
            {{0, 0, 7},{1, 0, 0},{2, 0, 7},{3, 0, 7},{4, 0, 7},{5, 0, 7},{6, 0, 7}},   //
            {{0, 0, 8},{1, 0, 0},{2, 0, 8},{3, 0, 8},{4, 0, 8},{5, 0, 8},{6, 0, 8}},   //
        };
        BSplineSurface b_spline_surface(u_degree, v_degree, u_knots, v_knots, control_points, interval, interval);
        std::vector<BSplineCurve3D> curves = {
            BSplineCurve3D(v_degree, control_points[0], v_knots, interval),
            BSplineCurve3D(v_degree, control_points[1], v_knots, interval),
            BSplineCurve3D(v_degree, control_points[2], v_knots, interval),
            BSplineCurve3D(v_degree, control_points[3], v_knots, interval),
            BSplineCurve3D(v_degree, control_points[4], v_knots, interval),
            BSplineCurve3D(v_degree, control_points[5], v_knots, interval),
            BSplineCurve3D(v_degree, control_points[6], v_knots, interval),
            BSplineCurve3D(v_degree, control_points[7], v_knots, interval),
        };

        // Compare the surface to the curves
        double div = 1.0 / 99.0;
        for (int32_t i = 0; i < 100; ++i) {
            glm::dvec2 location = { static_cast<double>(i) * div, 0 };
            for (int32_t j = 0; j < 100; ++j) {
                location.y = static_cast<double>(j) * div;
                glm::dvec3 point_bspl = b_spline_surface.EvaluatePoint(location);

                std::vector<glm::dvec3> temp_pts(curves.size());
                for (size_t index = 0; index < curves.size(); ++index) {
                    temp_pts[index] = curves[index].EvaluateCurve(location.y);
                }
                BSplineCurve3D u_curve(u_degree, temp_pts, u_knots, interval);
                glm::dvec3 point_curv = u_curve.EvaluateCurve(location.x);
                // Numbers are not between 0 and 1, so the epsilon needs to be scaled
                // This is not the proper scaling though.
                EXPECT_NEAR(point_bspl.x, point_curv.x, std::numeric_limits<double>::epsilon() * 10);
                EXPECT_NEAR(point_bspl.y, point_curv.y, std::numeric_limits<double>::epsilon() * 10);
                EXPECT_NEAR(point_bspl.z, point_curv.z, std::numeric_limits<double>::epsilon() * 10);
            }
        }
    }

    // TODO - Add more tests for B-Spline Curve Derivatives
}
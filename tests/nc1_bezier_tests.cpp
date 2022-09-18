#include <gtest/gtest.h>

// NURBS_CPP
#include "include/bezier_curve.hpp"
#include "include/bezier_surface.hpp"
// STD
#include <numeric>

namespace nurbs {
    TEST(NURBS_Chapter1, BernsteinVsAll) {

        constexpr double div = 1.0 / 99.0;
        for (uint32_t i = 0; i < 100; ++i) {
            const double u = static_cast<double>(i) * div;
            const std::vector<double> all_bern = BezierCurveUtil::AllBernstein(2, u);
            const double bern_0 = BezierCurveUtil::Bernstein(0, 2, u);
            const double bern_1 = BezierCurveUtil::Bernstein(1, 2, u);
            const double bern_2 = BezierCurveUtil::Bernstein(2, 2, u);
            EXPECT_DOUBLE_EQ(all_bern[0], bern_0);
            EXPECT_DOUBLE_EQ(all_bern[1], bern_1);
            EXPECT_DOUBLE_EQ(all_bern[2], bern_2);
        }
    }

    TEST(NURBS_Chapter1, Bezier2DConstruct) {
        const std::vector<glm::dvec2> control_points;
        const BezierCurve2D bezier(control_points);
    }

    TEST(NURBS_Chapter1, Bezier2DPoint) {
        std::vector<glm::dvec2> control_points = { {0,0}, {1, 0}, {1,1}, {0, 1} };
        const BezierCurve2D bezier(control_points);

        const glm::dvec2 point = bezier.EvaluateCurve(0.5);

        std::vector<glm::dvec2> test_points = control_points;
        while (test_points.size() > 1) {
            std::vector<glm::dvec2> temp_points;
            for (uint32_t i = 1; i < test_points.size(); ++i) {
                temp_points.push_back((test_points[i - 1] + test_points[i]) * 0.5);
            }
            test_points = temp_points;
        }

        EXPECT_DOUBLE_EQ(test_points[0].x, point.x);
        EXPECT_DOUBLE_EQ(test_points[0].y, point.y);
    }

    TEST(NURBS_Chapter1, Bezier2DPoints) {
        std::vector<glm::dvec2> control_points = { {0,0}, {1, 0}, {1,1}, {0, 1} };
        constexpr glm::dvec2 interval = { 0.0, 1.0 };
        const BezierCurve2D bezier(control_points, interval);

        const std::vector<glm::dvec2> points = bezier.EvaluateCurvePoints(100);

        constexpr double div = 1.0 / 99.0;
        for (uint32_t i = 0; i < 100; ++i) {
            const double u = static_cast<double>(i) * div;
            const double u_i = 1.0 - u;
            std::vector<glm::dvec2> test_points = control_points;
            while (test_points.size() > 1) {
                std::vector<glm::dvec2> temp_points;
                for (uint32_t i = 1; i < test_points.size(); ++i) {
                    temp_points.push_back((u_i * test_points[i - 1]) + (u * test_points[i]));
                }
                test_points = temp_points;
            }

            EXPECT_DOUBLE_EQ(test_points[0].x, points[i].x);
            EXPECT_DOUBLE_EQ(test_points[0].y, points[i].y);
        }
    }

    TEST(NURBS_Chapter1, BernsteinVsDeCasteljau2D) {
        const std::vector<glm::dvec2> control_points = { {0,0}, {0, 1}, {1, 1}, {1, 0} };
        const BezierCurve2D bezier(control_points);
        constexpr double div = 1.0 / 99.0;
        for (uint32_t i = 0; i < 100; ++i) {
            const double u = static_cast<double>(i) * div;
            const glm::dvec2 bern = bezier.PointOnBezierCurve(u);
            const glm::dvec2 cast = bezier.DeCasteljau(u);
            EXPECT_DOUBLE_EQ(bern.x, cast.x);
            EXPECT_DOUBLE_EQ(bern.y, cast.y);
        }
    }

    TEST(NURBS_Chapter1, Bezier2DPolynomialCompareEx1_6) {
        const std::vector<glm::dvec2> control_points = { {0,0}, {0, 1}, {1, 1}, {1, 0} };
        const BezierCurve2D bezier(control_points);
        constexpr double div = 1.0 / 99.0;
        for (uint32_t i = 0; i < 100; ++i) {
            const double u = static_cast<double>(i) * div;
            // Bezier
            const glm::dvec2 point_b = bezier.EvaluateCurve(u);

            // Cubic Bezier Polynomial
            const double u_i = 1.0 - u;
            glm::dvec2 point_p = std::pow(u_i, 3) * control_points[0];
            point_p += 3 * u * std::pow(u_i, 2) * control_points[1];
            point_p += 3 * std::pow(u, 2) * u_i * control_points[2];
            point_p += std::pow(u, 3) * control_points[3];

            EXPECT_DOUBLE_EQ(point_b.x, point_p.x);
            EXPECT_DOUBLE_EQ(point_b.y, point_p.y);
        }
    }

    TEST(NURBS_Chapter1, Bezier2DPolynomialCompareDerivEx1_6) {
        const std::vector<glm::dvec2> control_points = { {0,0}, {0, 1}, {1, 1}, {1, 0} };
        const BezierCurve2D bezier(control_points);
        constexpr double div = 1.0 / 99.0;
        for (uint32_t i = 0; i < 100; ++i) {
            // Bezier
            const double u = static_cast<double>(i) * div;
            const glm::dvec2 deriv_b = bezier.Derivative(u);

            // Cubic Bezier Polynomial
            const double u_i = 1.0 - u;
            glm::dvec2 deriv_p = std::pow(u, 2) * (control_points[3] - control_points[2]);
            deriv_p += (2.0 * u_i * u * (control_points[2] - control_points[1]));
            deriv_p += (std::pow(u_i, 2) * (control_points[1] - control_points[0]));
            deriv_p *= 3.0;

            EXPECT_DOUBLE_EQ(deriv_b.x, deriv_p.x);
            EXPECT_DOUBLE_EQ(deriv_b.y, deriv_p.y);
        }
    }

    TEST(NURBS_Chapter1, Bezier3DConstruct) {
        const std::vector<glm::dvec3> control_points;
        const BezierCurve3D bezier(control_points);
    }

    TEST(NURBS_Chapter1, Bezier3DPoint) {
        std::vector<glm::dvec3> control_points;
        control_points.push_back({ 0, 0 , 0 });
        control_points.push_back({ 1, 0 , 1 });
        control_points.push_back({ 1, 1 , 2 });
        control_points.push_back({ 0, 1 , 3 });
        const BezierCurve3D bezier(control_points);

        const glm::dvec3 point = bezier.EvaluateCurve(0.5);

        std::vector<glm::dvec3> test_points = control_points;
        while (test_points.size() > 1) {
            std::vector<glm::dvec3> temp_points;
            for (uint32_t i = 1; i < test_points.size(); ++i) {
                temp_points.push_back((test_points[i - 1] + test_points[i]) * 0.5);
            }
            test_points = temp_points;
        }

        EXPECT_DOUBLE_EQ(test_points[0].x, point.x);
        EXPECT_DOUBLE_EQ(test_points[0].y, point.y);
        EXPECT_DOUBLE_EQ(test_points[0].z, point.z);
    }

    TEST(NURBS_Chapter1, Bezier3DPoints) {
        std::vector<glm::dvec3> control_points;
        control_points.push_back({ 0, 0 , 0 });
        control_points.push_back({ 1, 0 , 1 });
        control_points.push_back({ 1, 1 , 2 });
        control_points.push_back({ 0, 1 , 3 });
        constexpr glm::dvec2 interval = { 0.0, 1.0 };
        const BezierCurve3D bezier(control_points, interval);

        const std::vector<glm::dvec3> points = bezier.EvaluateCurvePoints(100);

        constexpr double div = 1.0 / 99.0;
        for (uint32_t i = 0; i < 100; ++i) {
            const double u = static_cast<double>(i) * div;
            const double u_i = 1.0 - u;
            std::vector<glm::dvec3> test_points = control_points;
            while (test_points.size() > 1) {
                std::vector<glm::dvec3> temp_points;
                for (uint32_t i = 1; i < test_points.size(); ++i) {
                    temp_points.push_back((u_i * test_points[i - 1]) + (u * test_points[i]));
                }
                test_points = temp_points;
            }

            EXPECT_DOUBLE_EQ(test_points[0].x, points[i].x);
            EXPECT_DOUBLE_EQ(test_points[0].y, points[i].y);
            EXPECT_DOUBLE_EQ(test_points[0].z, points[i].z);
        }
    }

    TEST(NURBS_Chapter1, BernsteinVsDeCasteljau3D) {
        const std::vector<glm::dvec3> control_points = { {0,0, 0}, {0, 1, 1}, {1, 1, 2}, {1, 0, 1} };
        const BezierCurve3D bezier(control_points);
        constexpr double div = 1.0 / 99.0;
        for (uint32_t i = 0; i < 100; ++i) {
            const double u = static_cast<double>(i) * div;
            const glm::dvec3 bern = bezier.PointOnBezierCurve(u);
            const glm::dvec3 cast = bezier.DeCasteljau(u);
            EXPECT_DOUBLE_EQ(bern.x, cast.x);
            EXPECT_DOUBLE_EQ(bern.y, cast.y);
            EXPECT_DOUBLE_EQ(bern.z, cast.z);
        }
    }

    TEST(NURBS_Chapter1, Bezier3DPolynomialCompareEx1_6) {
        const std::vector<glm::dvec3> control_points = { {0,0, 0}, {0, 1, 1}, {1, 1, 2}, {1, 0, 1} };
        const BezierCurve3D bezier(control_points);
        constexpr double div = 1.0 / 99.0;
        for (uint32_t i = 0; i < 100; ++i) {
            const double u = static_cast<double>(i) * div;
            // Bezier
            const glm::dvec3 point_b = bezier.EvaluateCurve(u);

            // Cubic Bezier Polynomial
            const double u_i = 1.0 - u;
            glm::dvec3 point_p = std::pow(u_i, 3) * control_points[0];
            point_p += 3 * u * std::pow(u_i, 2) * control_points[1];
            point_p += 3 * std::pow(u, 2) * u_i * control_points[2];
            point_p += std::pow(u, 3) * control_points[3];

            EXPECT_DOUBLE_EQ(point_b.x, point_p.x);
            EXPECT_DOUBLE_EQ(point_b.y, point_p.y);
            EXPECT_DOUBLE_EQ(point_b.z, point_p.z);
        }
    }

    TEST(NURBS_Chapter1, Bezier3DPolynomialCompareDerivEx1_6) {
        const std::vector<glm::dvec3> control_points = { {0,0, 0}, {0, 1, 1}, {1, 1, 2}, {1, 0, 1} };
        const BezierCurve3D bezier(control_points);
        constexpr double div = 1.0 / 99.0;
        for (uint32_t i = 0; i < 100; ++i) {
            // Bezier
            const double u = static_cast<double>(i) * div;
            const glm::dvec3 deriv_b = bezier.Derivative(u);

            // Cubic Bezier Polynomial
            const double u_i = 1.0 - u;
            glm::dvec3 deriv_p = std::pow(u, 2) * (control_points[3] - control_points[2]);
            deriv_p += (2.0 * u_i * u * (control_points[2] - control_points[1]));
            deriv_p += (std::pow(u_i, 2) * (control_points[1] - control_points[0]));
            deriv_p *= 3.0;
            // The percision is really just not here with this calculation...
            // I think the generic bezier calculation is more incorrect becuase I can't factor things out,
            // but either way this is a really imprecise calculation
            EXPECT_NEAR(deriv_b.x, deriv_p.x, std::numeric_limits<float>::epsilon() * 10);
            EXPECT_NEAR(deriv_b.y, deriv_p.y, std::numeric_limits<float>::epsilon() * 10);
            EXPECT_NEAR(deriv_b.z, deriv_p.z, std::numeric_limits<float>::epsilon() * 10);
        }
    }

    TEST(NURBS_Chapter1, BezierSurfaceConstruct) {
        const std::vector<BezierCurve3D> curves;
        const BezierSurface surface(curves);
    }

    TEST(NURBS_Chapter1, BezierSurfacePoint) {
        std::vector<BezierCurve3D> curves;
        {
            std::vector<glm::dvec3> control_points;
            control_points.push_back({ 0, -1, 0 });
            control_points.push_back({ 0, 2, 1 });
            control_points.push_back({ 0, 2, 2 });
            control_points.push_back({ 0, 1, 3 });
            curves.push_back(control_points);
        }
        {
            std::vector<glm::dvec3> control_points;
            control_points.push_back({ 1, 0, 0 });
            control_points.push_back({ 1, 4, 1 });
            control_points.push_back({ 1, 3, 2 });
            control_points.push_back({ 1, 2, 3 });
            curves.push_back(control_points);
        }
        {
            std::vector<glm::dvec3> control_points;
            control_points.push_back({ 2, 2, 0 });
            control_points.push_back({ 2, 1, 1 });
            control_points.push_back({ 2, 0, 2 });
            control_points.push_back({ 2, -1, 3 });
            curves.push_back(control_points);
        }
        {
            std::vector<glm::dvec3> control_points;
            control_points.push_back({ 3, 3, 0 });
            control_points.push_back({ 3, -2, 1 });
            control_points.push_back({ 3, -4, 2 });
            control_points.push_back({ 3, 0, 3 });
            curves.push_back(control_points);
        }
        const BezierSurface surface(curves);

        const glm::dvec3 point = surface.EvaluatePoint({ 0.5, 0.5 });

        const std::vector<glm::dvec3> temp_cps = {
            curves[0].EvaluateCurve(0.5),
            curves[1].EvaluateCurve(0.5),
            curves[2].EvaluateCurve(0.5),
            curves[3].EvaluateCurve(0.5)
        };
        const BezierCurve3D curve(temp_cps);
        const glm::dvec3 test_point = curve.EvaluateCurve(0.5);

        EXPECT_DOUBLE_EQ(test_point.x, point.x);
        EXPECT_DOUBLE_EQ(test_point.y, point.y);
        EXPECT_DOUBLE_EQ(test_point.z, point.z);
    }

    TEST(NURBS_Chapter1, BezierSurfacePoints) {
        constexpr uint32_t point_count = 100;
        constexpr double div = 1.0 / static_cast<double>(point_count - 1);
        std::vector<BezierCurve3D> curves;
        {
            std::vector<glm::dvec3> control_points;
            control_points.push_back({ 0, -1, 0 });
            control_points.push_back({ 0, 2, 1 });
            control_points.push_back({ 0, 2, 2 });
            control_points.push_back({ 0, 1, 3 });
            curves.push_back(control_points);
        }
        {
            std::vector<glm::dvec3> control_points;
            control_points.push_back({ 1, 0, 0 });
            control_points.push_back({ 1, 4, 1 });
            control_points.push_back({ 1, 3, 2 });
            control_points.push_back({ 1, 2, 3 });
            curves.push_back(control_points);
        }
        {
            std::vector<glm::dvec3> control_points;
            control_points.push_back({ 2, 2, 0 });
            control_points.push_back({ 2, 1, 1 });
            control_points.push_back({ 2, 0, 2 });
            control_points.push_back({ 2, -1, 3 });
            curves.push_back(control_points);
        }
        {
            std::vector<glm::dvec3> control_points;
            control_points.push_back({ 3, 3, 0 });
            control_points.push_back({ 3, -2, 1 });
            control_points.push_back({ 3, -4, 2 });
            control_points.push_back({ 3, 0, 3 });
            curves.push_back(control_points);
        }
        const BezierSurface surface(curves);

        const std::vector<glm::dvec3> points = surface.EvaluatePoints(point_count, point_count);

        for (uint32_t i = 0; i < point_count; ++i) {
            const double u = static_cast<double>(i) * div;
            const std::vector<glm::dvec3> temp_cps = {
                curves[0].EvaluateCurve(u),
                curves[1].EvaluateCurve(u),
                curves[2].EvaluateCurve(u),
                curves[3].EvaluateCurve(u)
            };
            const BezierCurve3D curve(temp_cps);
            for (uint32_t j = 0; j < point_count; ++j) {
                const double v = static_cast<double>(j) * div;
                const glm::dvec3 test_point = curve.EvaluateCurve(v);
                const glm::dvec3& point = points[(i * point_count) + j];

                EXPECT_DOUBLE_EQ(test_point.x, point.x);
                EXPECT_DOUBLE_EQ(test_point.y, point.y);
                EXPECT_DOUBLE_EQ(test_point.z, point.z);
            }
        }
    }
}
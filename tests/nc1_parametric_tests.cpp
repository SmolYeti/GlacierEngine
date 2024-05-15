#include <gtest/gtest.h>

#include "include/parametric_curve.hpp"
#include "include/parametric_surface.hpp"

#define _USE_MATH_DEFINES
#include <math.h>

namespace nurbs {
    TEST(NURBS_Chapter1, Parametric2DConstruct) {
        std::array<std::function<double(double)>, 2> functions;
        functions[0] = [](double x) { return cos(x); };
        functions[1] = [](double x) { return sin(x); };
        const Point2D interval = {0.0, M_PI * 2};

        const ParametricCurve2D curve(functions, interval);
    }

    TEST(NURBS_Chapter1, Parametric2DPointOnCurve) {
        std::array<std::function<double(double)>, 2> functions;
        functions[0] = [](double x) { return cos(x); };
        functions[1] = [](double x) { return sin(x); };
        const Point2D interval = {0.0, M_PI * 2};

        const ParametricCurve2D curve(functions, interval);

        const Point2D point = curve.EvaluateCurve(M_PI_4);

        EXPECT_DOUBLE_EQ(point.x, cos(M_PI_4));
        EXPECT_DOUBLE_EQ(point.y, sin(M_PI_4));
    }

    TEST(NURBS_Chapter1, Parametric2DPointsOnCurve) {
        std::array<std::function<double(double)>, 2> functions;
        functions[0] = [](double x) { return cos(x); };
        functions[1] = [](double x) { return sin(x); };
        const Point2D interval = {0.0, M_PI * 2};

        const ParametricCurve2D curve(functions, interval);

        const std::vector<Point2D> points = curve.EvaluateCurvePoints(100);

        constexpr double div = (M_PI * 2) / 99.0;
        for (uint32_t i = 0; i < 100; ++i) {
            const double u = static_cast<double>(i) * div;
            EXPECT_DOUBLE_EQ(points[i].x, cos(u));
            EXPECT_DOUBLE_EQ(points[i].y, sin(u));

            const double length = sqrt(pow(points[i].x, 2.0) + pow(points[i].y, 2.0));
            EXPECT_DOUBLE_EQ(length, 1.0);
        }
    }

    TEST(NURBS_Chapter1, Parametric3DConstruct) {
        std::array<std::function<double(double)>, 3> functions;
        functions[0] = [](double x) { return cos(x); };
        functions[1] = [](double x) { return sin(x); };
        functions[2] = [](double x) { return x + 1.5; };
        const Point2D interval = {0.0, M_PI * 2};

        const ParametricCurve3D curve(functions, interval);
    }

    TEST(NURBS_Chapter1, Parametric3DPointOnCurve) {
        std::array<std::function<double(double)>, 3> functions;
        functions[0] = [](double x) { return cos(x); };
        functions[1] = [](double x) { return sin(x); };
        functions[2] = [](double x) { return x + 1.5; };
        const Point2D interval = {0.0, M_PI * 2};

        const ParametricCurve3D curve(functions, interval);

        const Point3D point = curve.EvaluateCurve(M_PI_4);

        EXPECT_DOUBLE_EQ(point.x, cos(M_PI_4));
        EXPECT_DOUBLE_EQ(point.y, sin(M_PI_4));
        EXPECT_DOUBLE_EQ(point.z, M_PI_4 + 1.5);
    }

    TEST(NURBS_Chapter1, Parametric3DPointsOnCurve) {
        std::array<std::function<double(double)>, 3> functions;
        functions[0] = [](double x) { return cos(x); };
        functions[1] = [](double x) { return sin(x); };
        functions[2] = [](double x) { return x + 1.5; };
        const Point2D interval = {0.0, M_PI * 2};

        const ParametricCurve3D curve(functions, interval);

        const std::vector<Point3D> points = curve.EvaluateCurvePoints(100);

        constexpr double div = (M_PI * 2) / 99.0;
        for (uint32_t i = 0; i < 100; ++i) {
            const double u = static_cast<double>(i) * div;
            EXPECT_DOUBLE_EQ(points[i].x, cos(u));
            EXPECT_DOUBLE_EQ(points[i].y, sin(u));
            EXPECT_DOUBLE_EQ(points[i].z, u + 1.5);

            const double length = sqrt(pow(points[i].x, 2.0) + pow(points[i].y, 2.0));
            EXPECT_DOUBLE_EQ(length, 1.0);
        }
    }

    TEST(NURBS_Chapter1, ParametricSurfConstruct) {
        std::array<std::function<double(Point2D)>, 3> functions;
        functions[0] = [](Point2D uv) { return sin(uv.x) * cos(uv.y); };
        functions[1] = [](Point2D uv) { return sin(uv.x) * sin(uv.y); };
        functions[2] = [](Point2D uv) { return cos(uv.x); };
        const Point2D u_interval = {0.0, M_PI * 2};
        const Point2D v_interval = {0.0, M_PI * 2};

        const ParametricSurface surface(functions, u_interval, v_interval);
    }

    TEST(NURBS_Chapter1, ParametricSurfPoint) {
        std::array<std::function<double(Point2D)>, 3> functions;
        functions[0] = [](Point2D uv) { return sin(uv.x) * cos(uv.y); };
        functions[1] = [](Point2D uv) { return sin(uv.x) * sin(uv.y); };
        functions[2] = [](Point2D uv) { return cos(uv.x); };
        const Point2D u_interval = { 0.0, M_PI * 2 };
        const Point2D v_interval = {0.0, M_PI * 2};

        const ParametricSurface surface(functions, u_interval, v_interval);

        const Point3D point = surface.EvaluatePoint({ M_PI, M_PI_4 });

        EXPECT_DOUBLE_EQ(point.x, sin(M_PI) * cos(M_PI_4));
        EXPECT_DOUBLE_EQ(point.y, sin(M_PI) * sin(M_PI_4));
        EXPECT_DOUBLE_EQ(point.z, cos(M_PI));
    }

    TEST(NURBS_Chapter1, ParametricSurfPoints) {
        constexpr uint32_t point_count = 100;
        constexpr double div = (M_PI * 2) / static_cast<double>(point_count - 1);
        std::array<std::function<double(Point2D)>, 3> functions;
        functions[0] = [](Point2D uv) { return sin(uv.x) * cos(uv.y); };
        functions[1] = [](Point2D uv) { return sin(uv.x) * sin(uv.y); };
        functions[2] = [](Point2D uv) { return cos(uv.x); };
        const Point2D u_interval = {0.0, M_PI * 2};
        const Point2D v_interval = {0.0, M_PI * 2};

        const ParametricSurface surface(functions, u_interval, v_interval);

        const std::vector<Point3D> points = surface.EvaluatePoints(point_count, point_count);

        for (uint32_t i = 0; i < point_count; ++i) {
            const double u = static_cast<double>(i) * div;
            for (uint32_t j = 0; j < point_count; ++j) {
                const double v = static_cast<double>(j) * div;
                const Point3D& point = points[(i * point_count) + j];

                EXPECT_DOUBLE_EQ(point.x, sin(u) * cos(v));
                EXPECT_DOUBLE_EQ(point.y, sin(u) * sin(v));
                EXPECT_DOUBLE_EQ(point.z, cos(u));

                const double length = sqrt(pow(points[i].x, 2.0) + pow(points[i].y, 2.0) + pow(points[i].z, 2.0));
                EXPECT_DOUBLE_EQ(length, 1.0);
            }
        }
    }
}
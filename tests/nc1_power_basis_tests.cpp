#include <gtest/gtest.h>

// NURBS_CPP
#include "include/power_basis_curve.hpp"

namespace nurbs {
    TEST(NURBS_Chapter1, PowerBasis2DConstruct) {
        const std::vector<glm::dvec2> bases;
        const PowerBasisCurve2D power_basis(bases);
    }

    TEST(NURBS_Chapter1, PowerBasis2DPoint) {
        std::vector<glm::dvec2> bases;
        bases.push_back({ 0, 0 });
        bases.push_back({ 1, 2 });
        bases.push_back({ 2, 0 });
        const PowerBasisCurve2D power_basis(bases);

        const glm::dvec2 point = power_basis.EvaluateCurve(0.5);

        const glm::dvec2 test_point = bases[0] + (bases[1] * 0.5) + (bases[2] * 0.25);

        EXPECT_DOUBLE_EQ(point.x, test_point.x);
        EXPECT_DOUBLE_EQ(point.y, test_point.y);
    }

    TEST(NURBS_Chapter1, PowerBasis2DPoints) {
        std::vector<glm::dvec2> bases;
        bases.push_back({ 0, 0 });
        bases.push_back({ 1, 2 });
        bases.push_back({ 2, 0 });
        const PowerBasisCurve2D power_basis(bases);

        const std::vector<glm::dvec2> points = power_basis.EvaluateCurvePoints(100);

        constexpr double div = 1.0 / 99.0;
        for (uint32_t i = 0; i < 100; ++i) {
            const double u = static_cast<double>(i) * div;
            const glm::dvec2 test_point = bases[0] + (bases[1] * u) + (bases[2] * (u * u));

            EXPECT_DOUBLE_EQ(points[i].x, test_point.x);
            EXPECT_DOUBLE_EQ(points[i].y, test_point.y);
        }
    }

    TEST(NURBS_Chapter1, PowerBasis3DConstruct) {
        const std::vector<glm::dvec3> bases;
        const PowerBasisCurve3D power_basis(bases);
    }

    TEST(NURBS_Chapter1, PowerBasis3DPoint) {
        std::vector<glm::dvec3> bases;
        bases.push_back({ 0, 0, 0 });
        bases.push_back({ 1, 2, 2 });
        bases.push_back({ 2, 0, 1 });
        const PowerBasisCurve3D power_basis(bases);

        const glm::dvec3 point = power_basis.EvaluateCurve(0.5);

        const glm::dvec3 test_point = bases[0] + (bases[1] * 0.5) + (bases[2] * 0.25);

        EXPECT_DOUBLE_EQ(point.x, test_point.x);
        EXPECT_DOUBLE_EQ(point.y, test_point.y);
        EXPECT_DOUBLE_EQ(point.z, test_point.z);
    }

    TEST(NURBS_Chapter1, PowerBasis3DPoints) {
        std::vector<glm::dvec3> bases;
        bases.push_back({ 0, 0, 0 });
        bases.push_back({ 1, 2, 2 });
        bases.push_back({ 2, 0, 1 });
        const PowerBasisCurve3D power_basis(bases);

        const std::vector<glm::dvec3> points = power_basis.EvaluateCurvePoints(100);

        constexpr double div = 1.0 / 99.0;
        for (uint32_t i = 0; i < 100; ++i) {
            const double u = static_cast<double>(i) * div;
            const glm::dvec3 test_point = bases[0] + (bases[1] * u) + (bases[2] * (u * u));

            EXPECT_DOUBLE_EQ(points[i].x, test_point.x);
            EXPECT_DOUBLE_EQ(points[i].y, test_point.y);
            EXPECT_DOUBLE_EQ(points[i].z, test_point.z);
        }
    }
}
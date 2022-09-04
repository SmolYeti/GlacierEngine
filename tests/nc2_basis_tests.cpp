#include <gtest/gtest.h>

// NURBS_CPP
#include "include/knot_utility_functions.hpp"
#include "include/derived_knot_funcs.hpp"

namespace nurbs {
    namespace knots {

        TEST(NURBS_Chapter2, FindSpanMin) {
            constexpr uint32_t degree = 2;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 2, 2 };
            int ret = FindSpan(degree, -1, knots);
            EXPECT_EQ(ret, 2);
            ret = FindSpan(degree, 0, knots);
            EXPECT_EQ(ret, 2);
        }

        TEST(NURBS_Chapter2, FindSpanMid) {
            constexpr uint32_t degree = 2;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 2, 2 };
            int ret = FindSpan(degree, 0.5, knots);
            EXPECT_EQ(ret, 2);
            ret = FindSpan(degree, 1.0, knots);
            EXPECT_EQ(ret, 3);
            ret = FindSpan(degree, 1.5, knots);
            EXPECT_EQ(ret, 3);
        }

        TEST(NURBS_Chapter2, FindSpanMax) {
            constexpr uint32_t degree = 2;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 2, 2 };
            int ret = FindSpan(degree, 2, knots);
            EXPECT_EQ(ret, 4);
            ret = FindSpan(degree, 3, knots);
            EXPECT_EQ(ret, 4);
        }

        TEST(NURBS_Chapter2, BasisEx2_3) {
            constexpr double u_value = 2.5;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5 };
            constexpr uint32_t degree = 2;
            int span_index = FindSpan(degree, u_value, knots);
            EXPECT_EQ(span_index, 4);
            std::vector<double> bases = BasisFuns(span_index, u_value, degree, knots);
            EXPECT_EQ(bases.size(), 3);
            EXPECT_DOUBLE_EQ(bases[0], 1.0 / 8.0);
            EXPECT_DOUBLE_EQ(bases[1], 6.0 / 8.0);
            EXPECT_DOUBLE_EQ(bases[2], 1.0 / 8.0);
        }

        TEST(NURBS_Chapter2, BasisMin) {
            constexpr double u_value = 0.0;
            constexpr uint32_t degree = 2;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 2, 2 };
            int span_index = FindSpan(degree, u_value, knots);
            std::vector<double> bases = BasisFuns(span_index, u_value, degree, knots);
            EXPECT_EQ(bases.size(), 3);
            EXPECT_DOUBLE_EQ(bases[0], 1.0);
            EXPECT_DOUBLE_EQ(bases[1], 0.0);
            EXPECT_DOUBLE_EQ(bases[2], 0.0);
        }

        TEST(NURBS_Chapter2, BasisMid) {
            constexpr double u_value_0 = 1.0;
            constexpr double u_value_1 = 1.5;
            constexpr uint32_t degree = 2;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 2, 2 };

            int span_index = FindSpan(degree, u_value_0, knots);
            std::vector<double> bases = BasisFuns(span_index, u_value_0, degree, knots);
            EXPECT_EQ(bases.size(), 3);
            EXPECT_DOUBLE_EQ(bases[0], 0.5);
            EXPECT_DOUBLE_EQ(bases[1], 0.5);
            EXPECT_DOUBLE_EQ(bases[2], 0.0);

            span_index = FindSpan(degree, u_value_1, knots);
            bases = BasisFuns(span_index, u_value_1, degree, knots);
            EXPECT_EQ(bases.size(), 3);
            EXPECT_DOUBLE_EQ(bases[0], 0.125);
            EXPECT_DOUBLE_EQ(bases[1], 0.625);
            EXPECT_DOUBLE_EQ(bases[2], 0.25);
        }

        TEST(NURBS_Chapter2, BasisMax) {
            constexpr double u_value_0 = 2.0;
            constexpr double u_value_1 = 2.5;
            constexpr uint32_t degree = 2;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 2, 2 };
            int span_index = FindSpan(degree, u_value_0, knots);
            std::vector<double> bases = BasisFuns(span_index, u_value_0, degree, knots);
            EXPECT_EQ(bases.size(), 3);
            EXPECT_DOUBLE_EQ(bases[0], 0.0);
            EXPECT_DOUBLE_EQ(bases[1], 0.0);
            EXPECT_DOUBLE_EQ(bases[2], 1.0);

            span_index = FindSpan(degree, u_value_1, knots);
            bases = BasisFuns(span_index, u_value_1, degree, knots);
            EXPECT_EQ(bases.size(), 3);
            EXPECT_DOUBLE_EQ(bases[0], 0.0);
            EXPECT_DOUBLE_EQ(bases[1], 0.0);
            EXPECT_DOUBLE_EQ(bases[2], 1.0);
        }
        // Comparison tests between my slightly changed one and origional
        // I'm not going to say optimized until I have floating point and time comparisions
        TEST(NURBS_Chapter2, BasisMinCompare) {
            constexpr double u_value = 0.0;
            constexpr uint32_t degree = 2;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 2, 2 };
            int span_index = FindSpan(degree, u_value, knots);
            std::vector<double> bases_0 = BasisFuns(span_index, u_value, degree, knots);
            std::vector<double> bases_1 = der_knots::BasisFuns(span_index, u_value, degree, knots);
            EXPECT_EQ(bases_0.size(), bases_1.size());
            EXPECT_DOUBLE_EQ(bases_0[0], bases_1[0]);
            EXPECT_DOUBLE_EQ(bases_0[1], bases_1[1]);
            EXPECT_DOUBLE_EQ(bases_0[2], bases_1[2]);
        }

        TEST(NURBS_Chapter2, BasisMidCompare) {
            constexpr double u_value_0 = 1.0;
            constexpr double u_value_1 = 1.5;
            constexpr uint32_t degree = 2;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 2, 2 };

            int span_index = FindSpan(degree, u_value_0, knots);
            std::vector<double> bases_0 = BasisFuns(span_index, u_value_0, degree, knots);
            std::vector<double> bases_1 = der_knots::BasisFuns(span_index, u_value_0, degree, knots);
            EXPECT_EQ(bases_0.size(), bases_1.size());
            EXPECT_DOUBLE_EQ(bases_0[0], bases_1[0]);
            EXPECT_DOUBLE_EQ(bases_0[1], bases_1[1]);
            EXPECT_DOUBLE_EQ(bases_0[2], bases_1[2]);

            span_index = FindSpan(degree, u_value_1, knots);
            bases_0 = BasisFuns(span_index, u_value_1, degree, knots);
            bases_1 = der_knots::BasisFuns(span_index, u_value_1, degree, knots);
            EXPECT_EQ(bases_0.size(), bases_1.size());
            EXPECT_DOUBLE_EQ(bases_0[0], bases_1[0]);
            EXPECT_DOUBLE_EQ(bases_0[1], bases_1[1]);
            EXPECT_DOUBLE_EQ(bases_0[2], bases_1[2]);
        }

        TEST(NURBS_Chapter2, BasisMaxCompare) {
            constexpr double u_value_0 = 2.0;
            constexpr double u_value_1 = 2.5;
            constexpr uint32_t degree = 2;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 2, 2 };
            int span_index = FindSpan(degree, u_value_0, knots);
            std::vector<double> bases_0 = BasisFuns(span_index, u_value_0, degree, knots);
            std::vector<double> bases_1 = der_knots::BasisFuns(span_index, u_value_0, degree, knots);
            EXPECT_EQ(bases_0.size(), bases_1.size());
            EXPECT_DOUBLE_EQ(bases_0[0], bases_1[0]);
            EXPECT_DOUBLE_EQ(bases_0[1], bases_1[1]);

            span_index = FindSpan(degree, u_value_1, knots);
            bases_0 = BasisFuns(span_index, u_value_1, degree, knots);
            bases_1 = der_knots::BasisFuns(span_index, u_value_1, degree, knots);
            EXPECT_EQ(bases_0.size(), bases_1.size());
            EXPECT_DOUBLE_EQ(bases_0[0], bases_1[0]);
            EXPECT_DOUBLE_EQ(bases_0[1], bases_1[1]);
            EXPECT_DOUBLE_EQ(bases_0[2], bases_1[2]);
        }

        // Basis derivative tests
        TEST(NURBS_Chapter2, BasisEx2_4) {
            constexpr double u_value = 2.5;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5 };
            constexpr uint32_t degree = 2;
            int span_index = FindSpan(degree, u_value, knots);
            EXPECT_EQ(span_index, 4);
            std::vector<std::vector<double>> basis_ders = DersBasisFuns(span_index, u_value, degree, 2, knots);
            EXPECT_EQ(basis_ders.size(), 3);
            EXPECT_EQ(basis_ders[2].size(), 3);
            EXPECT_DOUBLE_EQ(basis_ders[1][2], 1.0 / 2.0);
            EXPECT_DOUBLE_EQ(basis_ders[2][2], 1.0);
        }

        TEST(NURBS_Chapter2, BasisDerivativeMin) {
            constexpr double u_value = 0.0;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5 };
            constexpr uint32_t degree = 2;
            int span_index = FindSpan(degree, u_value, knots);
            EXPECT_EQ(span_index, 2);
            std::vector<std::vector<double>> basis_ders = DersBasisFuns(span_index, u_value, degree, 2, knots);
            EXPECT_EQ(basis_ders.size(), 3);
            EXPECT_EQ(basis_ders[2].size(), 3);
            EXPECT_DOUBLE_EQ(basis_ders[0][0], 1.0);
            EXPECT_DOUBLE_EQ(basis_ders[1][0], -2.0);
            EXPECT_DOUBLE_EQ(basis_ders[2][0], 2.0);

            EXPECT_DOUBLE_EQ(basis_ders[0][1], 0.0);
            EXPECT_DOUBLE_EQ(basis_ders[1][1], 2.0);
            EXPECT_DOUBLE_EQ(basis_ders[2][1], -3.0);

            EXPECT_DOUBLE_EQ(basis_ders[0][2], 0.0);
            EXPECT_DOUBLE_EQ(basis_ders[1][2], 0.0);
            EXPECT_DOUBLE_EQ(basis_ders[2][2], 1.0);
        }

        TEST(NURBS_Chapter2, BasisDerivativeMid) {
            constexpr double u_value_0 = 2.5;
            constexpr double u_value_1 = 3.0;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5 };
            constexpr uint32_t degree = 2;
            int span_index = FindSpan(degree, u_value_0, knots);
            EXPECT_EQ(span_index, 4);
            std::vector<std::vector<double>> basis_ders = DersBasisFuns(span_index, u_value_0, degree, 2, knots);
            EXPECT_EQ(basis_ders.size(), 3);
            EXPECT_EQ(basis_ders[2].size(), 3);
            // N^0 i, 2
            EXPECT_DOUBLE_EQ(basis_ders[0][0], 0.125);
            EXPECT_DOUBLE_EQ(basis_ders[0][1], 0.75);
            EXPECT_DOUBLE_EQ(basis_ders[0][2], 0.125);
            // N^1 i, 2
            EXPECT_DOUBLE_EQ(basis_ders[1][0], -0.5);
            EXPECT_DOUBLE_EQ(basis_ders[1][1], 0.0);
            EXPECT_DOUBLE_EQ(basis_ders[1][2], 0.5);
            // N^2 i, 2
            EXPECT_DOUBLE_EQ(basis_ders[2][0], 1.0);
            EXPECT_DOUBLE_EQ(basis_ders[2][1], -2.0);
            EXPECT_DOUBLE_EQ(basis_ders[2][2], 1.0);

            span_index = FindSpan(degree, u_value_1, knots);
            EXPECT_EQ(span_index, 5);
            basis_ders = DersBasisFuns(span_index, u_value_1, degree, 2, knots);
            EXPECT_EQ(basis_ders.size(), 3);
            EXPECT_EQ(basis_ders[2].size(), 3);
            EXPECT_DOUBLE_EQ(basis_ders[0][0], 0.5);
            EXPECT_DOUBLE_EQ(basis_ders[1][0], -1.0);
            EXPECT_DOUBLE_EQ(basis_ders[2][0], 1.0);

            EXPECT_DOUBLE_EQ(basis_ders[0][1], 0.5);
            EXPECT_DOUBLE_EQ(basis_ders[1][1], 1.0);
            EXPECT_DOUBLE_EQ(basis_ders[2][1], -3.0);

            EXPECT_DOUBLE_EQ(basis_ders[0][2], 0.0);
            EXPECT_DOUBLE_EQ(basis_ders[1][2], 0.0);
            EXPECT_DOUBLE_EQ(basis_ders[2][2], 2.0);
        }

        TEST(NURBS_Chapter2, BasisDerivativeMax) {
            constexpr double u_value = 5.0;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5 };
            constexpr uint32_t degree = 2;
            int span_index = FindSpan(degree, u_value, knots);
            EXPECT_EQ(span_index, 8);
            std::vector<std::vector<double>> basis_ders = DersBasisFuns(span_index, u_value, degree, 2, knots);
            EXPECT_EQ(basis_ders.size(), 3);
            EXPECT_EQ(basis_ders[2].size(), 3);
            EXPECT_DOUBLE_EQ(basis_ders[0][0], 0.0);
            EXPECT_DOUBLE_EQ(basis_ders[1][0], 0.0);
            EXPECT_DOUBLE_EQ(basis_ders[2][0], 0.0);

            EXPECT_DOUBLE_EQ(basis_ders[0][1], 0.0);
            EXPECT_DOUBLE_EQ(basis_ders[1][1], 0.0);
            EXPECT_DOUBLE_EQ(basis_ders[2][1], 0.0);

            EXPECT_DOUBLE_EQ(basis_ders[0][2], 0.0);
            EXPECT_DOUBLE_EQ(basis_ders[1][2], 0.0);
            EXPECT_DOUBLE_EQ(basis_ders[2][2], 0.0);
        }

        // Comparison tests between my slightly changed one and origional
        // I'm not going to say optimized until I have floating point and time comparisions
        TEST(NURBS_Chapter2, BasisEx2_4Compare) {
            constexpr double u_value = 2.5;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5 };
            constexpr uint32_t degree = 2;
            int span_index = FindSpan(degree, u_value, knots);
            EXPECT_EQ(span_index, 4);
            std::vector<std::vector<double>> basis_ders_0 = DersBasisFuns(span_index, u_value, degree, 2, knots);
            std::vector<std::vector<double>> basis_ders_1 = der_knots::DersBasisFuns(span_index, u_value, degree, 2, knots);
            EXPECT_EQ(basis_ders_0.size(), basis_ders_1.size());
            EXPECT_EQ(basis_ders_0[2].size(), 3);
            EXPECT_DOUBLE_EQ(basis_ders_0[0][0], basis_ders_1[0][0]);
            EXPECT_DOUBLE_EQ(basis_ders_0[1][0], basis_ders_1[1][0]);
            EXPECT_DOUBLE_EQ(basis_ders_0[2][0], basis_ders_1[2][0]);

            EXPECT_DOUBLE_EQ(basis_ders_0[0][1], basis_ders_1[0][1]);
            EXPECT_DOUBLE_EQ(basis_ders_0[1][1], basis_ders_1[1][1]);
            EXPECT_DOUBLE_EQ(basis_ders_0[2][1], basis_ders_1[2][1]);

            EXPECT_DOUBLE_EQ(basis_ders_0[0][2], basis_ders_1[0][2]);
            EXPECT_DOUBLE_EQ(basis_ders_0[1][2], basis_ders_1[1][2]);
            EXPECT_DOUBLE_EQ(basis_ders_0[2][2], basis_ders_1[2][2]);
        }

        TEST(NURBS_Chapter2, BasisDerivativeMinCompare) {
            constexpr double u_value = 0.0;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5 };
            constexpr uint32_t degree = 2;
            int span_index = FindSpan(degree, u_value, knots);
            EXPECT_EQ(span_index, 2);
            std::vector<std::vector<double>> basis_ders_0 = DersBasisFuns(span_index, u_value, degree, 2, knots);
            std::vector<std::vector<double>> basis_ders_1 = der_knots::DersBasisFuns(span_index, u_value, degree, 2, knots);
            EXPECT_EQ(basis_ders_0.size(), basis_ders_1.size());
            EXPECT_EQ(basis_ders_0[2].size(), 3);
            EXPECT_DOUBLE_EQ(basis_ders_0[0][0], basis_ders_1[0][0]);
            EXPECT_DOUBLE_EQ(basis_ders_0[1][0], basis_ders_1[1][0]);
            EXPECT_DOUBLE_EQ(basis_ders_0[2][0], basis_ders_1[2][0]);

            EXPECT_DOUBLE_EQ(basis_ders_0[0][1], basis_ders_1[0][1]);
            EXPECT_DOUBLE_EQ(basis_ders_0[1][1], basis_ders_1[1][1]);
            EXPECT_DOUBLE_EQ(basis_ders_0[2][1], basis_ders_1[2][1]);

            EXPECT_DOUBLE_EQ(basis_ders_0[0][2], basis_ders_1[0][2]);
            EXPECT_DOUBLE_EQ(basis_ders_0[1][2], basis_ders_1[1][2]);
            EXPECT_DOUBLE_EQ(basis_ders_0[2][2], basis_ders_1[2][2]);
        }

        TEST(NURBS_Chapter2, BasisDerivativeMidCompare) {
            constexpr double u_value_0 = 2.5;
            constexpr double u_value_1 = 3.0;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5 };
            constexpr uint32_t degree = 2;
            int span_index = FindSpan(degree, u_value_0, knots);
            EXPECT_EQ(span_index, 4);
            std::vector<std::vector<double>> basis_ders_0 = DersBasisFuns(span_index, u_value_0, degree, 2, knots);
            std::vector<std::vector<double>> basis_ders_1 = der_knots::DersBasisFuns(span_index, u_value_0, degree, 2, knots);
            EXPECT_EQ(basis_ders_0.size(), basis_ders_1.size());
            EXPECT_EQ(basis_ders_0[2].size(), 3);
            EXPECT_DOUBLE_EQ(basis_ders_0[0][0], basis_ders_1[0][0]);
            EXPECT_DOUBLE_EQ(basis_ders_0[1][0], basis_ders_1[1][0]);
            EXPECT_DOUBLE_EQ(basis_ders_0[2][0], basis_ders_1[2][0]);

            EXPECT_DOUBLE_EQ(basis_ders_0[0][1], basis_ders_1[0][1]);
            EXPECT_DOUBLE_EQ(basis_ders_0[1][1], basis_ders_1[1][1]);
            EXPECT_DOUBLE_EQ(basis_ders_0[2][1], basis_ders_1[2][1]);

            EXPECT_DOUBLE_EQ(basis_ders_0[0][2], basis_ders_1[0][2]);
            EXPECT_DOUBLE_EQ(basis_ders_0[1][2], basis_ders_1[1][2]);
            EXPECT_DOUBLE_EQ(basis_ders_0[2][2], basis_ders_1[2][2]);

            span_index = FindSpan(degree, u_value_1, knots);
            EXPECT_EQ(span_index, 5);
            basis_ders_0 = DersBasisFuns(span_index, u_value_1, degree, 2, knots);
            basis_ders_1 = der_knots::DersBasisFuns(span_index, u_value_1, degree, 2, knots);
            EXPECT_EQ(basis_ders_0.size(), basis_ders_1.size());
            EXPECT_EQ(basis_ders_0[2].size(), 3);
            EXPECT_DOUBLE_EQ(basis_ders_0[0][0], basis_ders_1[0][0]);
            EXPECT_DOUBLE_EQ(basis_ders_0[1][0], basis_ders_1[1][0]);
            EXPECT_DOUBLE_EQ(basis_ders_0[2][0], basis_ders_1[2][0]);

            EXPECT_DOUBLE_EQ(basis_ders_0[0][1], basis_ders_1[0][1]);
            EXPECT_DOUBLE_EQ(basis_ders_0[1][1], basis_ders_1[1][1]);
            EXPECT_DOUBLE_EQ(basis_ders_0[2][1], basis_ders_1[2][1]);

            EXPECT_DOUBLE_EQ(basis_ders_0[0][2], basis_ders_1[0][2]);
            EXPECT_DOUBLE_EQ(basis_ders_0[1][2], basis_ders_1[1][2]);
            EXPECT_DOUBLE_EQ(basis_ders_0[2][2], basis_ders_1[2][2]);
        }

        TEST(NURBS_Chapter2, BasisDerivativeMaxCompare) {
            constexpr double u_value = 5.0;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5 };
            constexpr uint32_t degree = 2;
            int span_index = FindSpan(degree, u_value, knots);
            EXPECT_EQ(span_index, 8);
            std::vector<std::vector<double>> basis_ders_0 = DersBasisFuns(span_index, u_value, degree, 2, knots);
            std::vector<std::vector<double>> basis_ders_1 = der_knots::DersBasisFuns(span_index, u_value, degree, 2, knots);
            EXPECT_EQ(basis_ders_0.size(), basis_ders_1.size());
            EXPECT_EQ(basis_ders_0[2].size(), 3);
            EXPECT_DOUBLE_EQ(basis_ders_0[0][0], basis_ders_1[0][0]);
            EXPECT_DOUBLE_EQ(basis_ders_0[1][0], basis_ders_1[1][0]);
            EXPECT_DOUBLE_EQ(basis_ders_0[2][0], basis_ders_1[2][0]);

            EXPECT_DOUBLE_EQ(basis_ders_0[0][1], basis_ders_1[0][1]);
            EXPECT_DOUBLE_EQ(basis_ders_0[1][1], basis_ders_1[1][1]);
            EXPECT_DOUBLE_EQ(basis_ders_0[2][1], basis_ders_1[2][1]);

            EXPECT_DOUBLE_EQ(basis_ders_0[0][2], basis_ders_1[0][2]);
            EXPECT_DOUBLE_EQ(basis_ders_0[1][2], basis_ders_1[1][2]);
            EXPECT_DOUBLE_EQ(basis_ders_0[2][2], basis_ders_1[2][2]);
        }

        // Test the single function against the multi function
        TEST(NURBS_Chapter2, SingleBasisMin) {
            constexpr double u_value = 0.0;
            constexpr uint32_t degree = 2;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 2, 2 };
            const int span_index = FindSpan(degree, u_value, knots);
            const std::vector<double> bases = BasisFuns(span_index, u_value, degree, knots);

            double basis = OneBasisFun(degree, knots, 0, u_value);
            EXPECT_DOUBLE_EQ(bases[0], basis);

            basis = OneBasisFun(degree, knots, 1, u_value);
            EXPECT_DOUBLE_EQ(bases[1], basis);

            basis = OneBasisFun(degree, knots, 2, u_value);
            EXPECT_DOUBLE_EQ(bases[2], basis);
        }

        TEST(NURBS_Chapter2, SingleBasisMid) {
            constexpr double u_value_0 = 1.0;
            constexpr double u_value_1 = 1.5;
            constexpr uint32_t degree = 2;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 2, 2 };

            int span_index = FindSpan(degree, u_value_0, knots);
            std::vector<double> bases = BasisFuns(span_index, u_value_0, degree, knots);
            double basis = OneBasisFun(degree, knots, 1, u_value_0);
            EXPECT_DOUBLE_EQ(bases[0], basis);
            basis = OneBasisFun(degree, knots, 2, u_value_0);
            EXPECT_DOUBLE_EQ(bases[1], basis);
            basis = OneBasisFun(degree, knots, 3, u_value_0);
            EXPECT_DOUBLE_EQ(bases[2], basis);

            span_index = FindSpan(degree, u_value_1, knots);
            bases = BasisFuns(span_index, u_value_1, degree, knots);
            basis = OneBasisFun(degree, knots, 1, u_value_1);
            EXPECT_DOUBLE_EQ(bases[0], basis);
            basis = OneBasisFun(degree, knots, 2, u_value_1);
            EXPECT_DOUBLE_EQ(bases[1], basis);
            basis = OneBasisFun(degree, knots, 3, u_value_1);
            EXPECT_DOUBLE_EQ(bases[2], basis);
        }

        TEST(NURBS_Chapter2, SingleBasisMax) {
            constexpr double u_value_0 = 2.0;
            constexpr double u_value_1 = 2.5;
            constexpr uint32_t degree = 2;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 2, 2 };
            int span_index = FindSpan(degree, u_value_0, knots);
            std::vector<double> bases = BasisFuns(span_index, u_value_0, degree, knots);
            double basis = OneBasisFun(degree, knots, 1, u_value_0);
            EXPECT_DOUBLE_EQ(bases[0], basis);
            basis = OneBasisFun(degree, knots, 2, u_value_0);
            EXPECT_DOUBLE_EQ(bases[1], basis);
            basis = OneBasisFun(degree, knots, 3, u_value_0);
            EXPECT_DOUBLE_EQ(bases[2], basis);

            span_index = FindSpan(degree, u_value_1, knots);
            bases = BasisFuns(span_index, u_value_1, degree, knots);
            basis = OneBasisFun(degree, knots, 1, u_value_1);
            EXPECT_DOUBLE_EQ(bases[0], basis);
            basis = OneBasisFun(degree, knots, 2, u_value_1);
            EXPECT_DOUBLE_EQ(bases[1], basis);
            basis = OneBasisFun(degree, knots, 3, u_value_1);
            EXPECT_DOUBLE_EQ(bases[2], basis);
        }

        TEST(NURBS_Chapter2, SingleBasisMinCompare) {
            constexpr double u_value = 0.0;
            constexpr uint32_t degree = 2;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 2, 2 };
            const int span_index = FindSpan(degree, u_value, knots);

            double basis_0 = OneBasisFun(degree, knots, 0, u_value);
            double basis_1 = der_knots::OneBasisFun(degree, knots, 0, u_value);
            EXPECT_DOUBLE_EQ(basis_0, basis_1);

            basis_0 = OneBasisFun(degree, knots, 1, u_value);
            basis_1 = der_knots::OneBasisFun(degree, knots, 1, u_value);
            EXPECT_DOUBLE_EQ(basis_0, basis_1);

            basis_0 = OneBasisFun(degree, knots, 2, u_value);
            basis_1 = der_knots::OneBasisFun(degree, knots, 2, u_value);
            EXPECT_DOUBLE_EQ(basis_0, basis_1);
        }

        TEST(NURBS_Chapter2, SingleBasisMidCompare) {
            constexpr double u_value_0 = 1.0;
            constexpr double u_value_1 = 1.5;
            constexpr uint32_t degree = 2;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 2, 2 };

            int span_index = FindSpan(degree, u_value_0, knots);
            double basis_0 = OneBasisFun(degree, knots, 1, u_value_0);
            double basis_1 = der_knots::OneBasisFun(degree, knots, 1, u_value_0);
            EXPECT_DOUBLE_EQ(basis_0, basis_1);
            basis_0 = OneBasisFun(degree, knots, 2, u_value_0);
            basis_1 = der_knots::OneBasisFun(degree, knots, 2, u_value_0);
            EXPECT_DOUBLE_EQ(basis_0, basis_1);
            basis_0 = OneBasisFun(degree, knots, 3, u_value_0);
            basis_1 = der_knots::OneBasisFun(degree, knots, 3, u_value_0);
            EXPECT_DOUBLE_EQ(basis_0, basis_1);

            span_index = FindSpan(degree, u_value_1, knots);
            basis_0 = OneBasisFun(degree, knots, 1, u_value_1);
            basis_1 = der_knots::OneBasisFun(degree, knots, 1, u_value_1);
            EXPECT_DOUBLE_EQ(basis_0, basis_1);
            basis_0 = OneBasisFun(degree, knots, 2, u_value_1);
            basis_1 = der_knots::OneBasisFun(degree, knots, 2, u_value_1);
            EXPECT_DOUBLE_EQ(basis_0, basis_1);
            basis_0 = OneBasisFun(degree, knots, 3, u_value_1);
            basis_1 = der_knots::OneBasisFun(degree, knots, 3, u_value_1);
            EXPECT_DOUBLE_EQ(basis_0, basis_1);
        }

        TEST(NURBS_Chapter2, SingleBasisMaxCompare) {
            constexpr double u_value_0 = 2.0;
            constexpr double u_value_1 = 2.5;
            constexpr uint32_t degree = 2;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 2, 2 };
            int span_index = FindSpan(degree, u_value_0, knots);
            double basis_0 = OneBasisFun(degree, knots, 1, u_value_0);
            double basis_1 = der_knots::OneBasisFun(degree, knots, 1, u_value_0);
            EXPECT_DOUBLE_EQ(basis_0, basis_1);
            basis_0 = OneBasisFun(degree, knots, 2, u_value_0);
            basis_1 = der_knots::OneBasisFun(degree, knots, 2, u_value_0);
            EXPECT_DOUBLE_EQ(basis_0, basis_1);
            basis_0 = OneBasisFun(degree, knots, 3, u_value_0);
            basis_1 = der_knots::OneBasisFun(degree, knots, 3, u_value_0);
            EXPECT_DOUBLE_EQ(basis_0, basis_1);

            span_index = FindSpan(degree, u_value_1, knots);
            basis_0 = OneBasisFun(degree, knots, 1, u_value_1);
            basis_1 = der_knots::OneBasisFun(degree, knots, 1, u_value_1);
            EXPECT_DOUBLE_EQ(basis_0, basis_1);
            basis_0 = OneBasisFun(degree, knots, 2, u_value_1);
            basis_1 = der_knots::OneBasisFun(degree, knots, 2, u_value_1);
            EXPECT_DOUBLE_EQ(basis_0, basis_1);
            basis_0 = OneBasisFun(degree, knots, 3, u_value_1);
            basis_1 = der_knots::OneBasisFun(degree, knots, 3, u_value_1);
            EXPECT_DOUBLE_EQ(basis_0, basis_1);
        }

        // TODO - Test the single function against the multi function
        TEST(NURBS_Chapter2, SingleBasisDerivativeMin) {
            constexpr double u_value = 0.0;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5 };
            constexpr uint32_t degree = 2;
            int span_index = FindSpan(degree, u_value, knots);
            EXPECT_EQ(span_index, 2);
            std::vector<std::vector<double>> bases_der = DersBasisFuns(span_index, u_value, degree, 2, knots);

            std::vector<double> bases = DersOneBasisFun(degree, knots, span_index, u_value, 2);

            EXPECT_DOUBLE_EQ(bases_der[0][2], bases[0]);
            EXPECT_DOUBLE_EQ(bases_der[1][2], bases[1]);
            EXPECT_DOUBLE_EQ(bases_der[2][2], bases[2]);
        }

        TEST(NURBS_Chapter2, SingleBasisDerivativeMid) {
            constexpr double u_value_0 = 2.5;
            constexpr double u_value_1 = 3.0;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5 };
            constexpr uint32_t degree = 2;
            int span_index = FindSpan(degree, u_value_0, knots);
            EXPECT_EQ(span_index, 4);
            std::vector<std::vector<double>> bases_der = DersBasisFuns(span_index, u_value_0, degree, 2, knots);

            std::vector<double> bases = DersOneBasisFun(degree, knots, span_index, u_value_0, 2);

            EXPECT_DOUBLE_EQ(bases_der[0][2], bases[0]);
            EXPECT_DOUBLE_EQ(bases_der[1][2], bases[1]);
            EXPECT_DOUBLE_EQ(bases_der[2][2], bases[2]);

            span_index = FindSpan(degree, u_value_1, knots);
            EXPECT_EQ(span_index, 5);
            bases_der = DersBasisFuns(span_index, u_value_1, degree, 2, knots);

            bases = DersOneBasisFun(degree, knots, span_index, u_value_1, 2);

            EXPECT_DOUBLE_EQ(bases_der[0][2], bases[0]);
            EXPECT_DOUBLE_EQ(bases_der[1][2], bases[1]);
            EXPECT_DOUBLE_EQ(bases_der[2][2], bases[2]);
        }

        TEST(NURBS_Chapter2, SingleBasisDerivativeMax) {
            constexpr double u_value_0 = 4.9;
            constexpr double u_value_1 = 5.0;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5 };
            constexpr uint32_t degree = 2;
            int span_index = FindSpan(degree, u_value_0, knots);
            EXPECT_EQ(span_index, 7);
            std::vector<std::vector<double>> bases_der = DersBasisFuns(span_index, u_value_0, degree, 2, knots);

            std::vector<double> bases = DersOneBasisFun(degree, knots, span_index, u_value_0, 2);

            EXPECT_DOUBLE_EQ(bases_der[0][2], bases[0]);
            EXPECT_DOUBLE_EQ(bases_der[1][2], bases[1]);
            EXPECT_DOUBLE_EQ(bases_der[2][2], bases[2]);

            span_index = FindSpan(degree, u_value_1, knots);
            EXPECT_EQ(span_index, 8);
            bases_der = DersBasisFuns(span_index, u_value_1, degree, 2, knots);

            bases = DersOneBasisFun(degree, knots, span_index, u_value_1, 2);

            EXPECT_DOUBLE_EQ(bases_der[0][2], bases[0]);
            EXPECT_DOUBLE_EQ(bases_der[1][2], bases[1]);
            EXPECT_DOUBLE_EQ(bases_der[2][2], bases[2]);
        }

        TEST(NURBS_Chapter2, SingleBasisDerivativeMinCompare) {
            constexpr double u_value = 0.0;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5 };
            constexpr uint32_t degree = 2;
            int span_index = FindSpan(degree, u_value, knots);
            EXPECT_EQ(span_index, 2);

            std::vector<double> bases_0 = DersOneBasisFun(degree, knots, span_index, u_value, 2);
            std::vector<double> bases_1 = der_knots::DersOneBasisFun(degree, knots, span_index, u_value, 2);

            EXPECT_DOUBLE_EQ(bases_0[0], bases_1[0]);
            EXPECT_DOUBLE_EQ(bases_0[1], bases_1[1]);
            EXPECT_DOUBLE_EQ(bases_0[2], bases_1[2]);
        }

        TEST(NURBS_Chapter2, SingleBasisDerivativeMidCompare) {
            constexpr double u_value_0 = 2.5;
            constexpr double u_value_1 = 3.0;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5 };
            constexpr uint32_t degree = 2;
            int span_index = FindSpan(degree, u_value_0, knots);
            EXPECT_EQ(span_index, 4);

            std::vector<double> bases_0 = DersOneBasisFun(degree, knots, span_index, u_value_0, 2);
            std::vector<double> bases_1 = der_knots::DersOneBasisFun(degree, knots, span_index, u_value_0, 2);

            EXPECT_DOUBLE_EQ(bases_0[0], bases_1[0]);
            EXPECT_DOUBLE_EQ(bases_0[1], bases_1[1]);
            EXPECT_DOUBLE_EQ(bases_0[2], bases_1[2]);

            span_index = FindSpan(degree, u_value_1, knots);
            EXPECT_EQ(span_index, 5);

            bases_0 = DersOneBasisFun(degree, knots, span_index, u_value_1, 2);
            bases_1 = der_knots::DersOneBasisFun(degree, knots, span_index, u_value_1, 2);

            EXPECT_DOUBLE_EQ(bases_0[0], bases_1[0]);
            EXPECT_DOUBLE_EQ(bases_0[1], bases_1[1]);
            EXPECT_DOUBLE_EQ(bases_0[2], bases_1[2]);
        }

        TEST(NURBS_Chapter2, SingleBasisDerivativeMaxCompare) {
            constexpr double u_value_0 = 4.9;
            constexpr double u_value_1 = 5.0;
            const std::vector<uint32_t> knots = { 0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5 };
            constexpr uint32_t degree = 2;
            int span_index = FindSpan(degree, u_value_0, knots);
            EXPECT_EQ(span_index, 7);

            std::vector<double> bases_0 = DersOneBasisFun(degree, knots, span_index, u_value_0, 2);
            std::vector<double> bases_1 = der_knots::DersOneBasisFun(degree, knots, span_index, u_value_0, 2);

            EXPECT_DOUBLE_EQ(bases_0[0], bases_1[0]);
            EXPECT_DOUBLE_EQ(bases_0[1], bases_1[1]);
            EXPECT_DOUBLE_EQ(bases_0[2], bases_1[2]);

            span_index = FindSpan(degree, u_value_1, knots);
            EXPECT_EQ(span_index, 8);

            bases_0 = DersOneBasisFun(degree, knots, span_index, u_value_1, 2);
            bases_1 = der_knots::DersOneBasisFun(degree, knots, span_index, u_value_1, 2);

            EXPECT_DOUBLE_EQ(bases_0[0], bases_1[0]);
            EXPECT_DOUBLE_EQ(bases_0[1], bases_1[1]);
            EXPECT_DOUBLE_EQ(bases_0[2], bases_1[2]);
        }
    }
}
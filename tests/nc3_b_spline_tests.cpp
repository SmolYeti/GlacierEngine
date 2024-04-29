#include <gtest/gtest.h>

// NURBS_CPP
#include "include/b_spline_curve.hpp"
#include "include/b_spline_surface.hpp"
#include "include/bezier_curve.hpp"
#include "include/bezier_surface.hpp"
#include "include/knot_utility_functions.hpp"

// STD
#include <numeric>

constexpr double kTolerance = std::numeric_limits<double>::epsilon();
namespace nurbs {
TEST(NURBS_Chapter3, Curve2DIntervalEqual) {
  std::vector<glm::dvec2> control_points = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};
  std::vector<double> knots = {0, 0, 0, 0, 1, 1, 1, 1};
  BSplineCurve2D b_spline(3, control_points, knots);
  for (double in_param = -0.01; in_param < 1.02; in_param += 0.01) {
    double out_param = b_spline.InternalParameter(in_param);
    EXPECT_DOUBLE_EQ(in_param, out_param);
  }
}

TEST(NURBS_Chapter3, Curve2DIntervalShiftKnots) {
  std::vector<glm::dvec2> control_points = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};
  std::vector<double> knots = {1, 1, 1, 1, 5, 5, 5, 5};
  BSplineCurve2D b_spline(3, control_points, knots);
  for (double in_param = -0.01; in_param < 1.02; in_param += 0.01) {
    double out_param = b_spline.InternalParameter(in_param);
    EXPECT_DOUBLE_EQ(out_param, (in_param * 4.0) + 1.0);
  }
}

TEST(NURBS_Chapter3, Curve2DIntervalShiftInterval) {
  std::vector<glm::dvec2> control_points = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};
  std::vector<double> knots = {1, 1, 1, 1, 5, 5, 5, 5};
  BSplineCurve2D b_spline(3, control_points, knots, {-1.0, 1.0});
  for (double in_param = -1.01; in_param < 1.02; in_param += 0.1) {
    double out_param = b_spline.InternalParameter(in_param);
    EXPECT_DOUBLE_EQ(out_param, ((in_param + 1.0) * 2.0) + 1.0);
  }
}

TEST(NURBS_Chapter3, Curve3DIntervalEqual) {
  std::vector<glm::dvec3> control_points = {
      {0, 0, 0.0}, {0, 1, -1.0}, {1, 1, 2.0}, {1, 0, 0.0}};
  std::vector<double> knots = {0, 0, 0, 0, 1, 1, 1, 1};
  BSplineCurve3D b_spline(3, control_points, knots);
  for (double in_param = -0.01; in_param < 1.02; in_param += 0.01) {
    double out_param = b_spline.InternalParameter(in_param);
    EXPECT_DOUBLE_EQ(in_param, out_param);
  }
}

TEST(NURBS_Chapter3, Curve3DIntervalShiftKnots) {
  std::vector<glm::dvec3> control_points = {
      {0, 0, 0.0}, {0, 1, -1.0}, {1, 1, 2.0}, {1, 0, 0.0}};
  std::vector<double> knots = {1, 1, 1, 1, 5, 5, 5, 5};
  BSplineCurve3D b_spline(3, control_points, knots);
  for (double in_param = -0.01; in_param < 1.02; in_param += 0.01) {
    double out_param = b_spline.InternalParameter(in_param);
    EXPECT_DOUBLE_EQ(out_param, (in_param * 4.0) + 1.0);
  }
}

TEST(NURBS_Chapter3, Curve3DIntervalShiftInterval) {
  std::vector<glm::dvec3> control_points = {
      {0, 0, 0.0}, {0, 1, -1.0}, {1, 1, 2.0}, {1, 0, 0.0}};
  std::vector<double> knots = {1, 1, 1, 1, 5, 5, 5, 5};
  BSplineCurve3D b_spline(3, control_points, knots, {-1.0, 1.0});
  for (double in_param = -1.01; in_param < 1.02; in_param += 0.1) {
    double out_param = b_spline.InternalParameter(in_param);
    EXPECT_DOUBLE_EQ(out_param, ((in_param + 1.0) * 2.0) + 1.0);
  }
}

TEST(NURBS_Chapter3, BSplineCurveBezierEquivalence2D) {
  std::vector<glm::dvec2> control_points = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};
  BezierCurve2D bezier(control_points);
  BSplineCurve2D b_spline(3, control_points, {0, 0, 0, 0, 1, 1, 1, 1});
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
  std::vector<glm::dvec3> control_points = {
      {0, 0, 0}, {0, 1, 1}, {1, 1, 2}, {1, 0, 1}};
  BezierCurve3D bezier(control_points);
  BSplineCurve3D b_spline(3, control_points, {0, 0, 0, 0, 1, 1, 1, 1});
  double div = 1.0 / 99.0;
  for (int32_t i = -1; i < 101; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec2 point_0 = bezier.EvaluateCurve(location);
    glm::dvec2 point_1 = b_spline.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_0.x, point_1.x);
    EXPECT_DOUBLE_EQ(point_0.y, point_1.y);
  }
}

TEST(NURBS_Chapter3, BSplineCurveBasisCompare2D) {
  std::vector<glm::dvec2> control_points = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};
  BezierCurve2D bezier(control_points);
  BSplineCurve2D b_spline(3, control_points, {0, 0, 0, 0, 1, 1, 1, 1});
  double div = 1.0 / 99.0;
  for (int32_t i = -1; i < 101; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec2 point_0 = bezier.EvaluateCurve(location);
    glm::dvec2 point_1 = b_spline.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_0.x, point_1.x);
    EXPECT_DOUBLE_EQ(point_0.y, point_1.y);
  }
}

TEST(NURBS_Chapter3, BSplineCurveBasisCompare3D) {
  std::vector<glm::dvec3> control_points = {
      {0, 0, 0}, {0, 1, 2}, {1, 1, 3}, {1, 0, 1}};
  BezierCurve3D bezier(control_points);
  BSplineCurve3D b_spline(3, control_points, {0, 0, 0, 0, 1, 1, 1, 1});
  double div = 1.0 / 99.0;
  for (int32_t i = -1; i < 101; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec3 point_0 = bezier.EvaluateCurve(location);
    glm::dvec3 point_1 = b_spline.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_0.x, point_1.x);
    EXPECT_DOUBLE_EQ(point_0.y, point_1.y);
    EXPECT_DOUBLE_EQ(point_0.z, point_1.z);
  }
}

TEST(NURBS_Chapter3, BSplineCurveMin2D) {
  std::vector<glm::dvec2> control_points = {
      {0, 0}, {0, 1}, {0.5, 0}, {1, 1}, {1, 0}};
  BSplineCurve2D b_spline(3, control_points, {0, 0, 0, 0, 1, 2, 2, 2, 2},
                          {0, 2});
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
  std::vector<glm::dvec3> control_points = {
      {0, 0, 0}, {0, 1, 1}, {0.5, 0, 1}, {1, 1, 1}, {1, 0, 0}};
  BSplineCurve3D b_spline(3, control_points, {0, 0, 0, 0, 1, 2, 2, 2, 2},
                          {0, 2});
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
  std::vector<glm::dvec2> control_points = {
      {0, 0}, {0, 1}, {0.5, 0}, {1, 1}, {1, 0}};
  BSplineCurve2D b_spline(3, control_points, {0, 0, 0, 0, 1, 2, 2, 2, 2},
                          {0, 2});
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
  std::vector<glm::dvec3> control_points = {
      {0, 0, 0}, {0, 1, 1}, {0.5, 0, 1}, {1, 1, 1}, {1, 0, 0}};
  BSplineCurve3D b_spline(3, control_points, {0, 0, 0, 0, 1, 2, 2, 2, 2},
                          {0, 2});
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
  std::vector<glm::dvec2> control_points = {
      {0, 0}, {0, 1}, {0.5, 0}, {1, 1}, {1, 0}};
  BSplineCurve2D b_spline(3, control_points, {0, 0, 0, 0, 1, 2, 2, 2, 2},
                          {0, 2});
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
  std::vector<glm::dvec3> control_points = {
      {0, 0, 0}, {0, 1, 1}, {0.5, 0, 1}, {1, 1, 1}, {1, 0, 0}};
  BSplineCurve3D b_spline(3, control_points, {0, 0, 0, 0, 1, 2, 2, 2, 2},
                          {0, 2});
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

// p91 - TODO: Calculate and finish
TEST(NURBS_Chapter3, DISABLED_BSplineCurveDeriv) {
  std::vector<glm::dvec2> control_points = {{0, 0}, {0, 1}, {0.5, 0.5},
                                            {1, 1}, {1, 0}, {2, 0}};
  std::vector<double> knots = {0, 0, 0, 1, 2, 4, 4, 5, 5, 5};
  uint32_t degree = 2;
  const double u = 2.5;
  BSplineCurve2D b_spline(degree, control_points, knots, {0, 5});

  // Test basis values
  std::vector<std::vector<double>> derivs = knots::DersBasisFuns(
      knots::FindSpanParam(degree, knots, u, kTolerance), u, degree, 2, knots);
  ASSERT_EQ(derivs.size(), 3);
  EXPECT_DOUBLE_EQ(derivs[0][2], 0.0625);
  EXPECT_DOUBLE_EQ(derivs[1][2], 0.25);
  EXPECT_DOUBLE_EQ(derivs[2][2], 0.5);

  // Test calculated derivative
}

TEST(NURBS_Chapter3, BSplineCurveDeriv2DEx3_1) {
  std::vector<glm::dvec2> control_points = {{0, 0}, {0, 1}, {1, 1}, {1, 0}};
  BezierCurve2D bezier(control_points);
  BSplineCurve2D b_spline(3, control_points, {0, 0, 0, 0, 1, 1, 1, 1});
  double div = 1.0 / 99.0;
  for (int32_t i = 0; i < 100; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec2 point_b = bezier.Derivative(location);
    std::vector<glm::dvec2> point_s = b_spline.Derivatives(location, 1);
    // Numbers are not between 0 and 1, so the epsilon needs to be scaled
    // This is not the proper scaling though.
    EXPECT_NEAR(point_b.x, point_s[1].x,
                std::numeric_limits<double>::epsilon() * 10);
    EXPECT_NEAR(point_b.y, point_s[1].y,
                std::numeric_limits<double>::epsilon() * 10);
  }
}

TEST(NURBS_Chapter3, BSplineCurveDeriv3DEx3_1) {
  std::vector<glm::dvec3> control_points = {
      {0, 0, 0}, {0, 1, 2}, {1, 1, 2}, {1, 0, 0}};
  BezierCurve3D bezier(control_points);
  BSplineCurve3D b_spline(3, control_points, {0, 0, 0, 0, 1, 1, 1, 1});
  double div = 1.0 / 99.0;
  for (int32_t i = 0; i < 100; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec3 point_b = bezier.Derivative(location);
    std::vector<glm::dvec3> point_s = b_spline.Derivatives(location, 1);
    // Numbers are not between 0 and 1, so the epsilon needs to be scaled
    // This is not the proper scaling though.
    EXPECT_NEAR(point_b.x, point_s[1].x,
                std::numeric_limits<float>::epsilon() * 10);
    EXPECT_NEAR(point_b.y, point_s[1].y,
                std::numeric_limits<float>::epsilon() * 10);
    EXPECT_NEAR(point_b.z, point_s[1].z,
                std::numeric_limits<float>::epsilon() * 10);
  }
}

TEST(NURBS_Chapter3, AllBasisFunCompare) {
  constexpr double u_value = 2.5;
  const std::vector<double> knots = {0, 0, 0, 1, 2, 3, 4, 4, 5, 5, 5};
  constexpr uint32_t degree = 2;
  uint32_t span_index =
      knots::FindSpanParam(degree, knots, u_value, kTolerance);
  std::vector<std::vector<double>> all_bases =
      knots::AllBasisFuns(span_index, u_value, degree, knots);
  // Top right triangle check
  for (uint32_t i = 0; i <= degree; ++i) {
    std::vector<double> bases =
        knots::BasisFuns(span_index, u_value, i, knots, kTolerance);
    ASSERT_GE(all_bases.size(), bases.size());
    for (uint32_t j = 0; j <= i; ++j) {
      EXPECT_DOUBLE_EQ(bases[j], all_bases[j][i]);
    }
  }
  // Need to check [1][0], [2][0], and [2][1], which is
  // N(Span + 1, 0), N(Span + 2, 0), and N(Span + 2, 1)
  EXPECT_DOUBLE_EQ(0, all_bases[1][0]);
  EXPECT_DOUBLE_EQ(0, all_bases[2][0]);
  EXPECT_DOUBLE_EQ(0, all_bases[2][1]);
}

TEST(NURBS_Chapter3, BSplineCurveDerivCompare2D) {
  std::vector<glm::dvec2> control_points = {
      {0, 0}, {0, 1}, {0.5, 0.5}, {1, 1}, {1, 0}};
  BSplineCurve2D b_spline(3, control_points, {0, 0, 0, 0, 1, 2, 2, 2, 2},
                          {0, 2});
  double div = 1.0 / 99.0;
  for (int32_t i = 0; i < 100; ++i) {
    double location = static_cast<double>(i) * div;
    std::vector<glm::dvec2> points_0 = b_spline.Derivatives(location, 1);
    std::vector<glm::dvec2> points_1 = b_spline.Derivatives2(location, 1);
    ASSERT_EQ(points_0.size(), points_1.size());
    // Numbers are not between 0 and 1, so the epsilon needs to be scaled
    // This is not the proper scaling though.
    for (size_t j = 0; j < points_0.size(); ++j) {
      EXPECT_NEAR(points_0[j].x, points_1[j].x,
                  std::numeric_limits<double>::epsilon() * 10.0);
      EXPECT_NEAR(points_0[j].y, points_1[j].y,
                  std::numeric_limits<double>::epsilon() * 10.0);
    }
  }
}

TEST(NURBS_Chapter3, BSplineCurveDerivCompare3D) {
  std::vector<glm::dvec3> control_points = {
      {0, 0, 0}, {0, 1, 1}, {0.5, 0.5, 2}, {1, 1, 1}, {1, 0, 0}};
  BSplineCurve3D b_spline(3, control_points, {0, 0, 0, 0, 1, 2, 2, 2, 2},
                          {0, 2});
  double div = 1.0 / 99.0;
  for (int32_t i = 0; i < 100; ++i) {
    double location = static_cast<double>(i) * div;
    std::vector<glm::dvec3> points_0 = b_spline.Derivatives(location, 1);
    std::vector<glm::dvec3> points_1 = b_spline.Derivatives2(location, 1);
    ASSERT_EQ(points_0.size(), points_1.size());
    // Numbers are not between 0 and 1, so the epsilon needs to be scaled
    // This is not the proper scaling though.
    for (size_t j = 0; j < points_0.size(); ++j) {
      EXPECT_NEAR(points_0[j].x, points_1[j].x,
                  std::numeric_limits<double>::epsilon() * 10.0);
      EXPECT_NEAR(points_0[j].y, points_1[j].y,
                  std::numeric_limits<double>::epsilon() * 10.0);
      EXPECT_NEAR(points_0[j].z, points_1[j].z,
                  std::numeric_limits<double>::epsilon() * 10.0);
    }
  }
}

// TODO - Add more tests for B-Spline Curve Derivatives

TEST(NURBS_Chapter3, BSplineSurfaceConstruct) {
  uint32_t degree = 3;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 2, 2};
  std::vector<double> v_knots = {0, 0, 0, 0, 1, 2, 2, 2, 2};
  std::vector<std::vector<glm::dvec3>> control_points = {
      {{-1, 0, -1},
       {-0.33, 0.1, -1.33},
       {0.33, 0.1, -1.33},
       {0.87, 0, -0.87},
       {1, -0.1, -1}}, //
      {{-1.33, -0.25, -0.33},
       {-0.33, 0, -0.33},
       {0.33, 0.0, -0.33},
       {1.33, -0.25, -0.33},
       {2, -0.5, -0.63}}, //
      {{-1.33, -0.75, 0.33},
       {-0.33, 0.0, 0.33},
       {0.33, 0.0, 0.33},
       {1.33, -0.75, 0.33},
       {2, -1, 0.63}}, //
      {{-0.87, -2, 0.87},
       {-0.33, 0.0, 1.33},
       {0.33, 0.0, 1.33},
       {0.87, -2, 0.87},
       {1, -2.5, 1}}, //
      {{-1, -2.5, 1},
       {-0.33, 0.0, 2},
       {0.33, 0.0, 1.63},
       {0.87, -2, 1},
       {1, -3, 1.5}}, //
  };
  nurbs::BSplineSurface b_spline_surface(degree, degree, u_knots, v_knots,
                                         control_points, {0, 2}, {0, 2});
}

TEST(NURBS_Chapter3, BSplineSurfaceBezierCompare) {
  uint32_t degree = 3;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 1, 1, 1};
  std::vector<double> v_knots = {0, 0, 0, 0, 1, 1, 1, 1};
  std::vector<std::vector<glm::dvec3>> control_points = {
      {{-0.87, 0, -0.87},
       {-0.33, 0.1, -1.33},
       {0.33, 0.1, -1.33},
       {0.87, 0, -0.87}}, //
      {{-1.33, -0.25, -0.33},
       {-0.33, 0, -0.33},
       {0.33, 0.0, -0.33},
       {1.33, -0.25, -0.33}}, //
      {{-1.33, -0.75, 0.33},
       {-0.33, 0.0, 0.33},
       {0.33, 0.0, 0.33},
       {1.33, -0.75, 0.33}}, //
      {{-0.87, -2, 0.87},
       {-0.33, 0.0, 1.33},
       {0.33, 0.0, 1.33},
       {0.87, -2, 0.87}}, //
  };
  BSplineSurface bspl_surface(degree, degree, u_knots, v_knots, control_points);
  std::vector<BezierCurve3D> bezier_curves = {
      BezierCurve3D(control_points[0]), BezierCurve3D(control_points[1]),
      BezierCurve3D(control_points[2]), BezierCurve3D(control_points[3])};
  BezierSurface bez_surface(bezier_curves);

  // Compare the surfaces
  double div = 1.0 / 99.0;
  for (int32_t i = 0; i < 100; ++i) {
    glm::dvec2 location = {static_cast<double>(i) * div, 0};
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
  glm::dvec2 interval = {0, 4};
  uint32_t u_degree = 4;
  uint32_t v_degree = 3;
  std::vector<double> u_knots = {0, 0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 4, 4, 4};
  std::vector<double> v_knots = {0, 0, 0, 0, 1, 1, 2, 3, 3, 4, 4, 4, 4};
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  std::vector<std::vector<glm::dvec3>> control_points;
  std::vector<BSplineCurve3D> curves;
  control_points.resize(u_points);
  curves.reserve(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = (u_val * u_val) + (v_val * v_val);
      control_points[u_index][v_index] = {u_index, v_index, dist_0_sq};
    }
    curves.emplace_back(v_degree, control_points[u_index], v_knots, interval);
  }

  BSplineSurface b_spline_surface(u_degree, v_degree, u_knots, v_knots,
                                  control_points, interval, interval);

  // Compare the surface to the curves
  double div = 1.0 / 99.0;
  for (int32_t i = 0; i < 100; ++i) {
    glm::dvec2 location = {static_cast<double>(i) * div, 0};
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
      EXPECT_NEAR(point_bspl.x, point_curv.x,
                  std::numeric_limits<double>::epsilon() * 100);
      EXPECT_NEAR(point_bspl.y, point_curv.y,
                  std::numeric_limits<double>::epsilon() * 100);
      EXPECT_NEAR(point_bspl.z, point_curv.z,
                  std::numeric_limits<double>::epsilon() * 100);
    }
  }
}

// TODO - Add more tests for B-Spline Curve Derivatives
TEST(NURBS_Chapter3, DISABLED_BSplineSurfaceDerivCompare) {
  std::vector<std::vector<glm::dvec3>> control_polygon = {
      {{0, 0, 0}, {1, 1, 0}, {0.5, 0.5, 0}, {1, 1, 0}, {1, 0, 0}, {2, -1, 0}},
      {{0, 1, 1}, {0, 3, 1}, {0.5, 1.5, 1}, {1, 2, 1}, {1, -1, 1}, {2, -2, 1}},
      {{0, 0, 2}, {1, 4, 1}, {0.5, 1.5, 2}, {1, 3, 2}, {1, -2, 2}, {2, -3, 2}},
      {{0, 1, 3}, {0, 5, 3}, {0.5, 2.5, 3}, {1, 4, 3}, {1, -3, 3}, {2, -4, 3}},
      {{0, 0, 4}, {1, 6, 4}, {0.5, 3.5, 4}, {1, 3, 4}, {1, -2, 4}, {2, -4, 4}},
      {{0, 1, 5}, {0, 6, 5}, {0.5, 3.5, 5}, {1, 2, 5}, {1, -1, 5}, {2, -3, 5}},
      {{0, 0, 6}, {1, 4, 6}, {0.5, 2.5, 6}, {1, 1, 6}, {1, 0, 6}, {2, -2, 6}},
      {{0, 1, 7}, {0, 2, 7}, {0.5, 1.5, 7}, {1, 0, 7}, {1, 1, 7}, {2, -2, 7}},
      {{0, 0, 8}, {0, 1, 8}, {0.5, 0.5, 8}, {1, 1, 8}, {1, 0, 8}, {2, -1, 8}},
  };
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 2, 2};
  std::vector<double> v_knots = {0, 0, 0, 0, 1, 1, 2, 3, 3, 3, 4, 4, 4, 4};
  BSplineSurface b_spline(3, 3, u_knots, v_knots, control_polygon, {0, 2},
                          {0, 4});
  size_t errors = 0;
  double div = 1.0 / 99.0;
  for (int32_t i = 0; i < 100; ++i) {
    double u_location = static_cast<double>(i) * div;
    for (int32_t j = 0; j < 100; ++j) {
      double v_location = static_cast<double>(j) * div;
      glm::dvec2 location = {u_location, v_location};
      std::vector<std::vector<glm::dvec3>> points_0 =
          b_spline.Derivative(location, 2);
      std::vector<std::vector<glm::dvec3>> points_1 =
          b_spline.Derivatives2(location, 2);
      ASSERT_EQ(points_0.size(), points_1.size());
      // Numbers are not between 0 and 1, so the epsilon needs to be scaled
      // This is not the proper scaling though.
      for (size_t d_i = 0; d_i < points_0.size(); ++d_i) {
        ASSERT_EQ(points_0[d_i].size(), points_1[d_i].size());
        for (size_t d_j = 0; d_j < points_0[d_i].size(); ++d_j) {
          // This shit is very far off, so still not done. I fucking hate tring
          // to debug this garbage.
          EXPECT_NEAR(points_0[d_i][d_j].x, points_1[d_i][d_j].x, 1.1);
          if (std::abs(points_0[d_i][d_j].x - points_1[d_i][d_j].x) >= 1.1) {
            errors++;
          }
          EXPECT_NEAR(points_0[d_i][d_j].y, points_1[d_i][d_j].y, 1.1);
          if (std::abs(points_0[d_i][d_j].y - points_1[d_i][d_j].y) >= 1.1) {
            errors++;
          }
          EXPECT_NEAR(points_0[d_i][d_j].z, points_1[d_i][d_j].z, 1.1);
          if (std::abs(points_0[d_i][d_j].z - points_1[d_i][d_j].z) >= 1.1) {
            errors++;
          }
        }
      }
    }
  }
  EXPECT_EQ(errors, 0);
}
} // namespace nurbs
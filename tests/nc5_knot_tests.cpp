#include <gtest/gtest.h>

#include "include/knot_utility_functions.hpp"
#include "include/nurbs_curve.hpp"

namespace nurbs {
TEST(NURBS_Chapter5, InsertKnotNone2D) {
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<uint32_t> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  std::vector<glm::dvec3> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}, {2, 0, 1},
      {2, 1, 1}, {3, 1, 1}, {3, 0, 1}, {4, 0, 1}, {4, 1, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots);

  // Knot insert
  NURBSCurve2D insert_curve = nurbs_curve.KnotInsertion(1, 0);

  // Check knot multiplicity
  uint32_t knot_count =
      knots::MultiplicityKnotU(degree, insert_curve.knots(), 1);
  EXPECT_EQ(knot_count, 1);

  // Compare
  double div = 1.0 / 99.0;
  for (int32_t i = -1; i < 101; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec2 point_nurbs = nurbs_curve.EvaluateCurve(location);
    glm::dvec2 point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
  }
}

TEST(NURBS_Chapter5, InsertKnotOnce2D) {
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<uint32_t> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  std::vector<glm::dvec3> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}, {2, 0, 1},
      {2, 1, 1}, {3, 1, 1}, {3, 0, 1}, {4, 0, 1}, {4, 1, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots);

  // Knot insert
  NURBSCurve2D insert_curve = nurbs_curve.KnotInsertion(3, 1);

  // Check knot multiplicity
  uint32_t knot_count =
      knots::MultiplicityKnotU(degree, insert_curve.knots(), 3);
  EXPECT_EQ(knot_count, 2);

  // Compare
  double div = 1.0 / 99.0;
  for (int32_t i = -1; i < 101; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec2 point_nurbs = nurbs_curve.EvaluateCurve(location);
    glm::dvec2 point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
  }
}

TEST(NURBS_Chapter5, InsertKnotTwice2D) {
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<uint32_t> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  std::vector<glm::dvec3> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}, {2, 0, 1},
      {2, 1, 1}, {3, 1, 1}, {3, 0, 1}, {4, 0, 1}, {4, 1, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots);

  // Knot insert
  NURBSCurve2D insert_curve = nurbs_curve.KnotInsertion(2, 2);

  // Check knot multiplicity
  uint32_t knot_count =
      knots::MultiplicityKnotU(degree, insert_curve.knots(), 2);
  EXPECT_EQ(knot_count, 4);

  // Compare
  double div = 1.0 / 99.0;
  for (int32_t i = -1; i < 101; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec2 point_nurbs = nurbs_curve.EvaluateCurve(location);
    glm::dvec2 point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
  }
}

TEST(NURBS_Chapter5, InsertKnotsAtOne2D) {
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<uint32_t> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  std::vector<glm::dvec3> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}, {2, 0, 1},
      {2, 1, 1}, {3, 1, 1}, {3, 0, 1}, {4, 0, 1}, {4, 1, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots);

  // Knot insert
  NURBSCurve2D insert_curve = nurbs_curve.KnotInsertion(1, 3);

  // Check knot multiplicity
  uint32_t knot_count =
      knots::MultiplicityKnotU(degree, insert_curve.knots(), 1);
  EXPECT_EQ(knot_count, 4);

  // Compare
  double div = 1.0 / 99.0;
  for (int32_t i = -1; i < 101; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec2 point_nurbs = nurbs_curve.EvaluateCurve(location);
    glm::dvec2 point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
  }
}

TEST(NURBS_Chapter5, CurveCutPoint2D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon() * 10.0;
  // NURBS Curves
  uint32_t degree = 2;
  std::vector<uint32_t> knots = {0, 0, 0, 1, 2, 2, 2};
  std::vector<glm::dvec3> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots);

  // Knot insert
  NURBSCurve2D insert_curve = nurbs_curve.KnotInsertion(1, 2);

  // Compare
  for (double parameter = -0.1; parameter < 1.15; parameter += 0.1) {
    glm::dvec2 point_nurbs = nurbs_curve.EvaluateCurve(parameter);
    glm::dvec2 point_isrt = insert_curve.EvaluateCurve(parameter);
    glm::dvec2 point_cut = nurbs_curve.PointByCornerCut(parameter);
    glm::dvec2 isrt_cut = insert_curve.PointByCornerCut(parameter);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
    EXPECT_NEAR(point_nurbs.x, point_cut.x, kTestEpsilon);
    EXPECT_NEAR(point_nurbs.y, point_cut.y, kTestEpsilon);
    EXPECT_NEAR(point_isrt.x, isrt_cut.x, kTestEpsilon);
    EXPECT_NEAR(point_isrt.y, isrt_cut.y, kTestEpsilon);
  }
}

TEST(NURBS_Chapter5, CurveCutPoints2D) {
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<uint32_t> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  std::vector<glm::dvec3> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}, {2, 0, 1},
      {2, 1, 1}, {3, 1, 1}, {3, 0, 1}, {4, 0, 1}, {4, 1, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots);

  // Compare
  int32_t splits = 100;
  double div = 1.0 / static_cast<double>(splits - 1);
  for (int32_t i = -1; i < splits + 1; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec2 point_nurbs = nurbs_curve.EvaluateCurve(location);
    glm::dvec2 point_cut = nurbs_curve.PointByCornerCut(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_cut.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_cut.y);
  }
}
TEST(NURBS_Chapter5, InsertKnotNone3D) {
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<uint32_t> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  std::vector<glm::dvec4> control_points = {
      {0, 0, 1, 1}, {0, 1, 2, 1}, {1, 1, 3, 1}, {1, 0, 1, 1}, {2, 0, 3, 1},
      {2, 1, 3, 1}, {3, 1, 5, 1}, {3, 0, 2, 1}, {4, 0, 3, 1}, {4, 1, 1, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots);

  // Knot insert
  NURBSCurve3D insert_curve = nurbs_curve.KnotInsertion(1, 0);

  // Check knot multiplicity
  uint32_t knot_count =
      knots::MultiplicityKnotU(degree, insert_curve.knots(), 1);
  EXPECT_EQ(knot_count, 1);

  // Compare
  double div = 1.0 / 99.0;
  for (int32_t i = -1; i < 101; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec2 point_nurbs = nurbs_curve.EvaluateCurve(location);
    glm::dvec2 point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
  }
}

TEST(NURBS_Chapter5, InsertKnotOnce3D) {
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<uint32_t> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  std::vector<glm::dvec4> control_points = {
      {0, 0, 1, 1}, {0, 1, 2, 1}, {1, 1, 3, 1}, {1, 0, 1, 1}, {2, 0, 3, 1},
      {2, 1, 3, 1}, {3, 1, 5, 1}, {3, 0, 2, 1}, {4, 0, 3, 1}, {4, 1, 1, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots);

  // Knot insert
  NURBSCurve3D insert_curve = nurbs_curve.KnotInsertion(3, 1);

  // Check knot multiplicity
  uint32_t knot_count =
      knots::MultiplicityKnotU(degree, insert_curve.knots(), 3);
  EXPECT_EQ(knot_count, 2);

  // Compare
  double div = 1.0 / 99.0;
  for (int32_t i = -1; i < 101; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec2 point_nurbs = nurbs_curve.EvaluateCurve(location);
    glm::dvec2 point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
  }
}

TEST(NURBS_Chapter5, InsertKnotTwice3D) {
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<uint32_t> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  std::vector<glm::dvec4> control_points = {
      {0, 0, 1, 1}, {0, 1, 2, 1}, {1, 1, 3, 1}, {1, 0, 1, 1}, {2, 0, 3, 1},
      {2, 1, 3, 1}, {3, 1, 5, 1}, {3, 0, 2, 1}, {4, 0, 3, 1}, {4, 1, 1, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots);

  // Knot insert
  NURBSCurve3D insert_curve = nurbs_curve.KnotInsertion(2, 2);

  // Check knot multiplicity
  uint32_t knot_count =
      knots::MultiplicityKnotU(degree, insert_curve.knots(), 2);
  EXPECT_EQ(knot_count, 4);

  // Compare
  double div = 1.0 / 99.0;
  for (int32_t i = -1; i < 101; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec2 point_nurbs = nurbs_curve.EvaluateCurve(location);
    glm::dvec2 point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
  }
}

TEST(NURBS_Chapter5, InsertKnotsAtOne3D) {
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<uint32_t> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  std::vector<glm::dvec4> control_points = {
      {0, 0, 1, 1}, {0, 1, 2, 1}, {1, 1, 3, 1}, {1, 0, 1, 1}, {2, 0, 3, 1},
      {2, 1, 3, 1}, {3, 1, 5, 1}, {3, 0, 2, 1}, {4, 0, 3, 1}, {4, 1, 1, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots);

  // Knot insert
  NURBSCurve3D insert_curve = nurbs_curve.KnotInsertion(1, 3);

  // Check knot multiplicity
  uint32_t knot_count =
      knots::MultiplicityKnotU(degree, insert_curve.knots(), 1);
  EXPECT_EQ(knot_count, 4);

  // Compare
  double div = 1.0 / 99.0;
  for (int32_t i = -1; i < 101; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec2 point_nurbs = nurbs_curve.EvaluateCurve(location);
    glm::dvec2 point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
  }
}

TEST(NURBS_Chapter5, CurveCutPoint3D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon() * 10.0;
  // NURBS Curves
  uint32_t degree = 2;
  std::vector<uint32_t> knots = {0, 0, 0, 1, 2, 2, 2};
  std::vector<glm::dvec4> control_points = {
      {0, 0, 0, 1}, {0, 1, 3, 1}, {1, 1, 1, 1}, {1, 0, -1, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots);

  // Knot insert
  NURBSCurve3D insert_curve = nurbs_curve.KnotInsertion(1, 2);

  // Compare
  for (double parameter = -0.1; parameter < 1.15; parameter += 0.1) {
    glm::dvec2 point_nurbs = nurbs_curve.EvaluateCurve(parameter);
    glm::dvec2 point_isrt = insert_curve.EvaluateCurve(parameter);
    glm::dvec2 point_cut = nurbs_curve.PointByCornerCut(parameter);
    glm::dvec2 isrt_cut = insert_curve.PointByCornerCut(parameter);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
    EXPECT_NEAR(point_nurbs.x, point_cut.x, kTestEpsilon);
    EXPECT_NEAR(point_nurbs.y, point_cut.y, kTestEpsilon);
    EXPECT_NEAR(point_isrt.x, isrt_cut.x, kTestEpsilon);
    EXPECT_NEAR(point_isrt.y, isrt_cut.y, kTestEpsilon);
  }
}

TEST(NURBS_Chapter5, CurveCutPoints3D) {
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<uint32_t> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  std::vector<glm::dvec4> control_points = {
      {0, 0, 1, 1}, {0, 1, 2, 1}, {1, 1, 3, 1}, {1, 0, 1, 1}, {2, 0, 3, 1},
      {2, 1, 3, 1}, {3, 1, 5, 1}, {3, 0, 2, 1}, {4, 0, 3, 1}, {4, 1, 1, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots);

  // Compare
  int32_t splits = 100;
  double div = 1.0 / static_cast<double>(splits - 1);
  for (int32_t i = -1; i < splits + 1; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec2 point_nurbs = nurbs_curve.EvaluateCurve(location);
    glm::dvec2 point_cut = nurbs_curve.PointByCornerCut(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_cut.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_cut.y);
  }
}
} // namespace nurbs
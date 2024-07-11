#include <gtest/gtest.h>

#include "include/knot_utility_functions.hpp"
#include "include/nurbs_curve.hpp"
#include "include/nurbs_surface.hpp"

namespace nurbs {
TEST(NURBS_Chapter5, InsertKnotNone2D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point3D> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}, {2, 0, 1},
      {2, 1, 1}, {3, 1, 1}, {3, 0, 1}, {4, 0, 1}, {4, 1, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots, interval);

  // Knot insert
  NURBSCurve2D insert_curve = nurbs_curve.KnotInsertion(1, 0);

  // Check knot multiplicity
  int knot_count =
      knots::MultiplicityParam(degree, insert_curve.knots(), 1, kTestEpsilon);
  EXPECT_EQ(knot_count, 0);

  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point2D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point2D point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
  }
}

TEST(NURBS_Chapter5, InsertKnotOnce2D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point3D> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}, {2, 0, 1},
      {2, 1, 1}, {3, 1, 1}, {3, 0, 1}, {4, 0, 1}, {4, 1, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots, interval);

  // Knot insert
  NURBSCurve2D insert_curve = nurbs_curve.KnotInsertion(3, 1);

  // Check knot multiplicity
  int knot_count =
      knots::MultiplicityParam(degree, insert_curve.knots(), 3, kTestEpsilon);
  EXPECT_EQ(knot_count, 1);

  // Compare
  int test_points = 10;
  double d_pts = static_cast<double>(test_points);
  double div = (1.0 / d_pts) * (interval.y - interval.x);
  for (int32_t i = -1; i < d_pts + 1; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point2D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point2D point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
  }
}

TEST(NURBS_Chapter5, InsertKnotTwice2D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point3D> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}, {2, 0, 1},
      {2, 1, 1}, {3, 1, 1}, {3, 0, 1}, {4, 0, 1}, {4, 1, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots, interval);

  // Knot insert
  NURBSCurve2D insert_curve = nurbs_curve.KnotInsertion(2, 2);

  // Check knot multiplicity
  int knot_count =
      knots::MultiplicityParam(degree, insert_curve.knots(), 2, kTestEpsilon);
  EXPECT_EQ(knot_count, 3);

  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point2D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point2D point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
  }
}

TEST(NURBS_Chapter5, InsertKnotsAtOne2D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point3D> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}, {2, 0, 1},
      {2, 1, 1}, {3, 1, 1}, {3, 0, 1}, {4, 0, 1}, {4, 1, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots, interval);

  // Knot insert
  NURBSCurve2D insert_curve = nurbs_curve.KnotInsertion(1, 3);

  // Check knot multiplicity
  int knot_count =
      knots::MultiplicityParam(degree, insert_curve.knots(), 1, kTestEpsilon);
  EXPECT_EQ(knot_count, 3);

  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point2D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point2D point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
  }
}

TEST(NURBS_Chapter5, CurveCutPoint2D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon() * 10.0;
  // NURBS Curves
  uint32_t degree = 2;
  std::vector<double> knots = {0, 0, 0, 1, 2, 2, 2};
  Point2D interval = {0.0, 2.0};
  std::vector<Point3D> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots, interval);

  // Knot insert
  NURBSCurve2D insert_curve = nurbs_curve.KnotInsertion(1, 2);

  // Compare
  for (double parameter = -0.1; parameter < 1.15; parameter += 0.1) {
    Point2D point_nurbs = nurbs_curve.EvaluateCurve(parameter);
    Point2D point_isrt = insert_curve.EvaluateCurve(parameter);
    Point2D point_cut = nurbs_curve.PointByCornerCut(parameter);
    Point2D isrt_cut = insert_curve.PointByCornerCut(parameter);
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
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point3D> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}, {2, 0, 1},
      {2, 1, 1}, {3, 1, 1}, {3, 0, 1}, {4, 0, 1}, {4, 1, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots, interval);

  // Compare
  int32_t splits = 100;
  double div =
      (1.0 / static_cast<double>(splits - 1)) * (interval.y - interval.x);
  for (int32_t i = -1; i < splits + 1; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point2D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point2D point_cut = nurbs_curve.PointByCornerCut(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_cut.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_cut.y);
  }
}

TEST(NURBS_Chapter5, InsertKnotNone3D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point4D> control_points = {
      {0, 0, 1, 1}, {0, 1, 2, 1}, {1, 1, 3, 1}, {1, 0, 1, 1}, {2, 0, 3, 1},
      {2, 1, 3, 1}, {3, 1, 5, 1}, {3, 0, 2, 1}, {4, 0, 3, 1}, {4, 1, 1, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots, interval);

  // Knot insert
  NURBSCurve3D insert_curve = nurbs_curve.KnotInsertion(1, 0);

  // Check knot multiplicity
  int knot_count =
      knots::MultiplicityParam(degree, insert_curve.knots(), 1, kTestEpsilon);
  EXPECT_EQ(knot_count, 0);

  // Compare
  int32_t splits = 100;
  double div =
      (1.0 / static_cast<double>(splits - 1)) * (interval.y - interval.x);
  for (int32_t i = -1; i < splits + 1; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point3D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point3D point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
    EXPECT_DOUBLE_EQ(point_nurbs.z, point_isrt.z);
  }
}

TEST(NURBS_Chapter5, InsertKnotOnce3D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point4D> control_points = {
      {0, 0, 1, 1}, {0, 1, 2, 1}, {1, 1, 3, 1}, {1, 0, 1, 1}, {2, 0, 3, 1},
      {2, 1, 3, 1}, {3, 1, 5, 1}, {3, 0, 2, 1}, {4, 0, 3, 1}, {4, 1, 1, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots, interval);

  // Knot insert
  NURBSCurve3D insert_curve = nurbs_curve.KnotInsertion(3, 1);

  // Check knot multiplicity
  int knot_count =
      knots::MultiplicityParam(degree, insert_curve.knots(), 3, kTestEpsilon);
  EXPECT_EQ(knot_count, 1);

  // Compare
  int32_t splits = 100;
  double div =
      (1.0 / static_cast<double>(splits - 1)) * (interval.y - interval.x);
  for (int32_t i = -1; i < splits + 1; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point3D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point3D point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
    EXPECT_DOUBLE_EQ(point_nurbs.z, point_isrt.z);
  }
}

TEST(NURBS_Chapter5, InsertKnotTwice3D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point4D> control_points = {
      {0, 0, 1, 1}, {0, 1, 2, 1}, {1, 1, 3, 1}, {1, 0, 1, 1}, {2, 0, 3, 1},
      {2, 1, 3, 1}, {3, 1, 5, 1}, {3, 0, 2, 1}, {4, 0, 3, 1}, {4, 1, 1, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots, interval);

  // Knot insert
  NURBSCurve3D insert_curve = nurbs_curve.KnotInsertion(2, 2);

  // Check knot multiplicity
  int knot_count =
      knots::MultiplicityParam(degree, insert_curve.knots(), 2, kTestEpsilon);
  EXPECT_EQ(knot_count, 3);

  // Compare
  int32_t splits = 100;
  double div =
      (1.0 / static_cast<double>(splits - 1)) * (interval.y - interval.x);
  for (int32_t i = -1; i < splits + 1; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point3D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point3D point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
    EXPECT_DOUBLE_EQ(point_nurbs.z, point_isrt.z);
  }
}

TEST(NURBS_Chapter5, InsertKnotsAtOne3D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point4D> control_points = {
      {0, 0, 1, 1}, {0, 1, 2, 1}, {1, 1, 3, 1}, {1, 0, 1, 1}, {2, 0, 3, 1},
      {2, 1, 3, 1}, {3, 1, 5, 1}, {3, 0, 2, 1}, {4, 0, 3, 1}, {4, 1, 1, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots, interval);

  // Knot insert
  NURBSCurve3D insert_curve = nurbs_curve.KnotInsertion(1, 3);

  // Check knot multiplicity
  int knot_count =
      knots::MultiplicityParam(degree, insert_curve.knots(), 1, kTestEpsilon);
  EXPECT_EQ(knot_count, 3);

  // Compare
  int32_t splits = 100;
  double div =
      (1.0 / static_cast<double>(splits - 1)) * (interval.y - interval.x);
  for (int32_t i = -1; i < splits + 1; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point3D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point3D point_isrt = insert_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
    EXPECT_DOUBLE_EQ(point_nurbs.z, point_isrt.z);
  }
}

TEST(NURBS_Chapter5, CurveCutPoint3D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon() * 10.0;
  // NURBS Curves
  uint32_t degree = 2;
  std::vector<double> knots = {0, 0, 0, 1, 2, 2, 2};
  Point2D interval = {0.0, 2.0};
  std::vector<Point4D> control_points = {
      {0, 0, 0, 1}, {0, 1, 3, 1}, {1, 1, 1, 1}, {1, 0, -1, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots, interval);

  // Knot insert
  NURBSCurve3D insert_curve = nurbs_curve.KnotInsertion(1, 2);

  // Compare
  for (double parameter = -0.1; parameter < 1.15; parameter += 0.1) {
    Point3D point_nurbs = nurbs_curve.EvaluateCurve(parameter);
    Point3D point_isrt = insert_curve.EvaluateCurve(parameter);
    Point3D point_cut = nurbs_curve.PointByCornerCut(parameter);
    Point3D isrt_cut = insert_curve.PointByCornerCut(parameter);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_isrt.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_isrt.y);
    EXPECT_DOUBLE_EQ(point_nurbs.z, point_isrt.z);
    EXPECT_NEAR(point_nurbs.x, point_cut.x, kTestEpsilon);
    EXPECT_NEAR(point_nurbs.y, point_cut.y, kTestEpsilon);
    EXPECT_NEAR(point_nurbs.z, point_cut.z, kTestEpsilon);
    EXPECT_NEAR(point_isrt.x, isrt_cut.x, kTestEpsilon);
    EXPECT_NEAR(point_isrt.y, isrt_cut.y, kTestEpsilon);
    EXPECT_NEAR(point_isrt.z, isrt_cut.z, kTestEpsilon);
  }
}

TEST(NURBS_Chapter5, CurveCutPoints3D) {
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point4D> control_points = {
      {0, 0, 1, 1}, {0, 1, 2, 1}, {1, 1, 3, 1}, {1, 0, 1, 1}, {2, 0, 3, 1},
      {2, 1, 3, 1}, {3, 1, 5, 1}, {3, 0, 2, 1}, {4, 0, 3, 1}, {4, 1, 1, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots, interval);

  // Compare
  int32_t splits = 100;
  double div =
      (1.0 / static_cast<double>(splits - 1)) * (interval.y - interval.x);
  for (int32_t i = -1; i < splits + 1; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point3D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point3D point_cut = nurbs_curve.PointByCornerCut(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_cut.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_cut.y);
    EXPECT_DOUBLE_EQ(point_nurbs.z, point_cut.z);
  }
}

TEST(NURBS_Chapter5, InsertKnotNoneSurfaceU) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Knot insert
  NURBSSurface insert_surface =
      surface.KnotInsert(NURBSSurface::SurfaceDirection::kUDir, 2, 0);

  // Check knot multiplicity
  int knot_count = knots::MultiplicityParam(u_degree, insert_surface.u_knots(),
                                            2, kTestEpsilon);
  EXPECT_EQ(knot_count, 1);

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = insert_surface.EvaluatePoint(uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, InsertKnotOnceSurfaceU) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Knot insert
  NURBSSurface insert_surface =
      surface.KnotInsert(NURBSSurface::SurfaceDirection::kUDir, 2, 1);

  // Check knot multiplicity
  int knot_count = knots::MultiplicityParam(u_degree, insert_surface.u_knots(),
                                            2, kTestEpsilon);
  EXPECT_EQ(knot_count, 2);

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = insert_surface.EvaluatePoint(uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, InsertKnotOnceSurfaceUBasicInterval) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0,    0,   0,   0,   0.33, 0.66,
                                 0.66, 1.0, 1.0, 1.0, 1.0};
  std::vector<double> v_knots = {0,   0,   0,    0.25, 0.25, 0.5,
                                 0.5, 0.5, 0.75, 1.0,  1.0,  1.0};
  Point2D u_interval = {0.0, 1.0};
  Point2D v_interval = {0.0, 1.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Knot insert
  NURBSSurface insert_surface =
      surface.KnotInsert(NURBSSurface::SurfaceDirection::kUDir, 0.66, 1);

  // Check knot multiplicity
  int knot_count = knots::MultiplicityParam(u_degree, insert_surface.u_knots(),
                                            0.66, kTestEpsilon);
  EXPECT_EQ(knot_count, 2);

  // Compare
  double div = 1.0 / 99.0;
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {static_cast<double>(i) * div, 0.0};
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = static_cast<double>(j) * div;
      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = insert_surface.EvaluatePoint(uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, InsertKnotTwiceSurfaceU) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val = static_cast<double>(u_index);
      double v_val = static_cast<double>(v_index);
      double z_dist = u_val * v_val;
      control_points[u_index][v_index] = {u_val, v_val, z_dist, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Knot insert
  NURBSSurface insert_surface =
      surface.KnotInsert(NURBSSurface::SurfaceDirection::kUDir, 1, 2);

  // Check knot multiplicity
  int knot_count = knots::MultiplicityParam(u_degree, insert_surface.u_knots(),
                                            1, kTestEpsilon);
  EXPECT_EQ(knot_count, 2);

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = insert_surface.EvaluatePoint(uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, InsertKnotsMaxSurfaceU) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Knot insert
  NURBSSurface insert_surface_part =
      surface.KnotInsert(NURBSSurface::SurfaceDirection::kUDir, 1, 3);
  NURBSSurface insert_surface_full =
      surface.KnotInsert(NURBSSurface::SurfaceDirection::kUDir, 2, 2);

  // Check knot multiplicity
  int knot_count = knots::MultiplicityParam(
      u_degree, insert_surface_part.u_knots(), 1, kTestEpsilon);
  EXPECT_EQ(knot_count, 3);
  knot_count = knots::MultiplicityParam(u_degree, insert_surface_full.u_knots(),
                                        2, kTestEpsilon);
  EXPECT_EQ(knot_count, 3);

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = insert_surface_part.EvaluatePoint(uv);
      Point3D point_full = insert_surface_full.EvaluatePoint(uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.x, point_full.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_full.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_full.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, InsertKnotOnceSurfaceV) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon() * 1.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Knot insert
  NURBSSurface insert_surface =
      surface.KnotInsert(NURBSSurface::SurfaceDirection::kVDir, 1, 1);

  // Check knot multiplicity
  int knot_count = knots::MultiplicityParam(v_degree, insert_surface.v_knots(),
                                            1, kTestEpsilon);
  EXPECT_EQ(knot_count, 2);

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = insert_surface.EvaluatePoint(uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, InsertKnotTwiceSurfaceV) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val = static_cast<double>(u_index);
      double v_val = static_cast<double>(v_index);
      double z_dist = u_val * v_val;
      control_points[u_index][v_index] = {u_val, v_val, z_dist, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Knot insert
  NURBSSurface insert_surface =
      surface.KnotInsert(NURBSSurface::SurfaceDirection::kVDir, 3, 2);

  // Check knot multiplicity
  int knot_count = knots::MultiplicityParam(v_degree, insert_surface.v_knots(),
                                            3, kTestEpsilon);
  EXPECT_EQ(knot_count, 2);

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = insert_surface.EvaluatePoint(uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, InsertKnotsMaxSurfaceV) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Knot insert
  NURBSSurface insert_surface_part =
      surface.KnotInsert(NURBSSurface::SurfaceDirection::kVDir, 1, 1);
  NURBSSurface insert_surface_full =
      surface.KnotInsert(NURBSSurface::SurfaceDirection::kVDir, 3, 2);

  // Check knot multiplicity
  int knot_count = knots::MultiplicityParam(
      v_degree, insert_surface_part.v_knots(), 1, kTestEpsilon);
  EXPECT_EQ(knot_count, 2);
  knot_count = knots::MultiplicityParam(v_degree, insert_surface_full.v_knots(),
                                        3, kTestEpsilon);
  EXPECT_EQ(knot_count, 2);

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = insert_surface_part.EvaluatePoint(uv);
      Point3D point_full = insert_surface_full.EvaluatePoint(uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.x, point_full.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_full.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_full.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, MergeKnotNone2D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point3D> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}, {2, 0, 1},
      {2, 1, 1}, {3, 1, 1}, {3, 0, 1}, {4, 0, 1}, {4, 1, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots, interval);

  // Merge Knot
  std::vector<double> merge = {};
  NURBSCurve2D merge_curve = nurbs_curve.MergeKnotVect(merge);

  // Check knot multiplicity
  int knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 1, kTestEpsilon);
  EXPECT_EQ(knot_count, 0);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 2, kTestEpsilon);
  EXPECT_EQ(knot_count, 1);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 3, kTestEpsilon);
  EXPECT_EQ(knot_count, 0);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 4, kTestEpsilon);
  EXPECT_EQ(knot_count, 1);

  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point2D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point2D point_merge = merge_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_merge.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_merge.y);
  }
}

TEST(NURBS_Chapter5, MergeKnotSingles2D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point3D> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}, {2, 0, 1},
      {2, 1, 1}, {3, 1, 1}, {3, 0, 1}, {4, 0, 1}, {4, 1, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots, interval);

  // Merge Knot
  std::vector<double> merge = {1, 2, 3, 4};
  NURBSCurve2D merge_curve = nurbs_curve.MergeKnotVect(merge);

  // Check knot multiplicity
  int knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 1, kTestEpsilon);
  EXPECT_EQ(knot_count, 1);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 2, kTestEpsilon);
  EXPECT_EQ(knot_count, 2);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 3, kTestEpsilon);
  EXPECT_EQ(knot_count, 1);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 4, kTestEpsilon);
  EXPECT_EQ(knot_count, 2);

  // Compare to knot insertion
  NURBSCurve2D insert_curve = nurbs_curve.KnotInsertion(1, 1);
  insert_curve = insert_curve.KnotInsertion(2, 1);
  insert_curve = insert_curve.KnotInsertion(3, 1);
  insert_curve = insert_curve.KnotInsertion(4, 1);

  // Check that the knots are the same
  ASSERT_EQ(merge_curve.knots().size(), insert_curve.knots().size());
  for (size_t i = 0; i < merge_curve.knots().size(); ++i) {
    EXPECT_DOUBLE_EQ(merge_curve.knots()[i], insert_curve.knots()[i]);
  }

  // Check that the resulting control points are the same
  ASSERT_EQ(merge_curve.control_points().size(),
            insert_curve.control_points().size());
  for (size_t i = 0; i < merge_curve.control_points().size(); ++i) {
    EXPECT_DOUBLE_EQ(merge_curve.control_points()[i].x,
                     insert_curve.control_points()[i].x);
    EXPECT_DOUBLE_EQ(merge_curve.control_points()[i].y,
                     insert_curve.control_points()[i].y);
    EXPECT_DOUBLE_EQ(merge_curve.control_points()[i].z,
                     insert_curve.control_points()[i].z);
  }

  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point2D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point2D point_merge = merge_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_merge.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_merge.y);
  }
}

TEST(NURBS_Chapter5, MergeKnotFull2D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point3D> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}, {2, 0, 1},
      {2, 1, 1}, {3, 1, 1}, {3, 0, 1}, {4, 0, 1}, {4, 1, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots, interval);

  // Merge Knot
  std::vector<double> merge = {1, 1, 1, 2, 2, 3, 3, 3, 4, 4};
  NURBSCurve2D merge_curve = nurbs_curve.MergeKnotVect(merge);

  // Check knot multiplicity
  int knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 1, kTestEpsilon);
  EXPECT_EQ(knot_count, 3);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 2, kTestEpsilon);
  EXPECT_EQ(knot_count, 3);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 3, kTestEpsilon);
  EXPECT_EQ(knot_count, 3);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 4, kTestEpsilon);
  EXPECT_EQ(knot_count, 3);

  // Compare to knot insertion
  NURBSCurve2D insert_curve = nurbs_curve.KnotInsertion(1, 3);
  insert_curve = insert_curve.KnotInsertion(2, 2);
  insert_curve = insert_curve.KnotInsertion(3, 3);
  insert_curve = insert_curve.KnotInsertion(4, 2);

  // Check that the knots are the same
  ASSERT_EQ(merge_curve.knots().size(), insert_curve.knots().size());
  for (size_t i = 0; i < merge_curve.knots().size(); ++i) {
    EXPECT_DOUBLE_EQ(merge_curve.knots()[i], insert_curve.knots()[i]);
  }

  // Check that the resulting control points are the same
  ASSERT_EQ(merge_curve.control_points().size(),
            insert_curve.control_points().size());
  for (size_t i = 0; i < merge_curve.control_points().size(); ++i) {
    EXPECT_DOUBLE_EQ(merge_curve.control_points()[i].x,
                     insert_curve.control_points()[i].x);
    EXPECT_DOUBLE_EQ(merge_curve.control_points()[i].y,
                     insert_curve.control_points()[i].y);
    EXPECT_DOUBLE_EQ(merge_curve.control_points()[i].z,
                     insert_curve.control_points()[i].z);
  }

  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point2D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point2D point_merge = merge_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_merge.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_merge.y);
  }
}

TEST(NURBS_Chapter5, MergeKnotNone3D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point4D> control_points = {
      {0, 0, 1, 1}, {0, 1, 2, 1}, {1, 1, 3, 1}, {1, 0, 1, 1}, {2, 0, 3, 1},
      {2, 1, 3, 1}, {3, 1, 5, 1}, {3, 0, 2, 1}, {4, 0, 3, 1}, {4, 1, 1, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots, interval);

  // Merge Knot
  std::vector<double> merge = {};
  NURBSCurve3D merge_curve = nurbs_curve.MergeKnotVect(merge);

  // Check knot multiplicity
  int knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 1, kTestEpsilon);
  EXPECT_EQ(knot_count, 0);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 2, kTestEpsilon);
  EXPECT_EQ(knot_count, 1);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 3, kTestEpsilon);
  EXPECT_EQ(knot_count, 0);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 4, kTestEpsilon);
  EXPECT_EQ(knot_count, 1);

  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point3D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point3D point_merge = merge_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_merge.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_merge.y);
    EXPECT_DOUBLE_EQ(point_nurbs.z, point_merge.z);
  }
}

TEST(NURBS_Chapter5, MergeKnotSingles3D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point4D> control_points = {
      {0, 0, 1, 1}, {0, 1, 2, 1}, {1, 1, 3, 1}, {1, 0, 1, 1}, {2, 0, 3, 1},
      {2, 1, 3, 1}, {3, 1, 5, 1}, {3, 0, 2, 1}, {4, 0, 3, 1}, {4, 1, 1, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots, interval);

  // Merge Knot
  std::vector<double> merge = {1, 2, 3, 4};
  NURBSCurve3D merge_curve = nurbs_curve.MergeKnotVect(merge);

  // Check knot multiplicity
  int knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 1, kTestEpsilon);
  EXPECT_EQ(knot_count, 1);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 2, kTestEpsilon);
  EXPECT_EQ(knot_count, 2);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 3, kTestEpsilon);
  EXPECT_EQ(knot_count, 1);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 4, kTestEpsilon);
  EXPECT_EQ(knot_count, 2);

  // Compare to knot insertion
  NURBSCurve3D insert_curve = nurbs_curve.KnotInsertion(1, 1);
  insert_curve = insert_curve.KnotInsertion(2, 1);
  insert_curve = insert_curve.KnotInsertion(3, 1);
  insert_curve = insert_curve.KnotInsertion(4, 1);

  // Check that the knots are the same
  ASSERT_EQ(merge_curve.knots().size(), insert_curve.knots().size());
  for (size_t i = 0; i < merge_curve.knots().size(); ++i) {
    EXPECT_DOUBLE_EQ(merge_curve.knots()[i], insert_curve.knots()[i]);
  }

  // Check that the resulting control points are the same
  ASSERT_EQ(merge_curve.control_points().size(),
            insert_curve.control_points().size());
  for (size_t i = 0; i < merge_curve.control_points().size(); ++i) {
    EXPECT_DOUBLE_EQ(merge_curve.control_points()[i].x,
                     insert_curve.control_points()[i].x);
    EXPECT_DOUBLE_EQ(merge_curve.control_points()[i].y,
                     insert_curve.control_points()[i].y);
    EXPECT_DOUBLE_EQ(merge_curve.control_points()[i].z,
                     insert_curve.control_points()[i].z);
    EXPECT_DOUBLE_EQ(merge_curve.control_points()[i].z,
                     insert_curve.control_points()[i].z);
  }

  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point3D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point3D point_merge = merge_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_merge.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_merge.y);
    EXPECT_DOUBLE_EQ(point_nurbs.z, point_merge.z);
  }
}

TEST(NURBS_Chapter5, MergeKnotFull3D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point4D> control_points = {
      {0, 0, 1, 1}, {0, 1, 2, 1}, {1, 1, 3, 1}, {1, 0, 1, 1}, {2, 0, 3, 1},
      {2, 1, 3, 1}, {3, 1, 5, 1}, {3, 0, 2, 1}, {4, 0, 3, 1}, {4, 1, 1, 1}};
  const NURBSCurve3D nurbs_curve(degree, control_points, knots, interval);

  // Merge Knot
  std::vector<double> merge = {1, 1, 1, 2, 2, 3, 3, 3, 4, 4};
  const NURBSCurve3D merge_curve = nurbs_curve.MergeKnotVect(merge);

  // Check knot multiplicity
  int knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 1, kTestEpsilon);
  EXPECT_EQ(knot_count, 3);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 2, kTestEpsilon);
  EXPECT_EQ(knot_count, 3);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 3, kTestEpsilon);
  EXPECT_EQ(knot_count, 3);
  knot_count =
      knots::MultiplicityParam(degree, merge_curve.knots(), 4, kTestEpsilon);
  EXPECT_EQ(knot_count, 3);

  // Compare to knot insertion
  NURBSCurve3D insert_curve = nurbs_curve.KnotInsertion(1, 3);
  insert_curve = insert_curve.KnotInsertion(2, 2);
  insert_curve = insert_curve.KnotInsertion(3, 3);
  insert_curve = insert_curve.KnotInsertion(4, 2);

  // Check that the knots are the same
  ASSERT_EQ(merge_curve.knots().size(), insert_curve.knots().size());
  for (size_t i = 0; i < merge_curve.knots().size(); ++i) {
    EXPECT_DOUBLE_EQ(merge_curve.knots()[i], insert_curve.knots()[i]);
  }

  // Check that the resulting control points are the same
  ASSERT_EQ(merge_curve.control_points().size(),
            insert_curve.control_points().size());
  for (size_t i = 0; i < merge_curve.control_points().size(); ++i) {
    EXPECT_DOUBLE_EQ(merge_curve.control_points()[i].x,
                     insert_curve.control_points()[i].x);
    EXPECT_DOUBLE_EQ(merge_curve.control_points()[i].y,
                     insert_curve.control_points()[i].y);
    EXPECT_DOUBLE_EQ(merge_curve.control_points()[i].z,
                     insert_curve.control_points()[i].z);
    EXPECT_DOUBLE_EQ(merge_curve.control_points()[i].z,
                     insert_curve.control_points()[i].z);
  }

  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point3D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point3D point_merge = merge_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_nurbs.x, point_merge.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_merge.y);
    EXPECT_DOUBLE_EQ(point_nurbs.z, point_merge.z);
  }
}

TEST(NURBS_Chapter5, MergeKnotNoneSurfaceU) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Knot Merge
  std::vector<double> merge_knots = {};
  NURBSSurface merge_surface = surface.RefineKnotVect(
      merge_knots, NURBSSurface::SurfaceDirection::kUDir);

  // Check knot multiplicity
  int knot_count = knots::MultiplicityParam(u_degree, merge_surface.u_knots(),
                                            1, kTestEpsilon);
  EXPECT_EQ(knot_count, 0);
  knot_count = knots::MultiplicityParam(u_degree, merge_surface.u_knots(), 2,
                                        kTestEpsilon);
  EXPECT_EQ(knot_count, 1);

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = merge_surface.EvaluatePoint(uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, MergeKnotSinglesSurfaceU) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Merge insert
  std::vector<double> merge_knots = {1, 2};
  NURBSSurface merge_surface = surface.RefineKnotVect(
      merge_knots, NURBSSurface::SurfaceDirection::kUDir);

  // Check knot multiplicity
  int knot_count = knots::MultiplicityParam(u_degree, merge_surface.u_knots(),
                                            1, kTestEpsilon);
  EXPECT_EQ(knot_count, 1);
  knot_count = knots::MultiplicityParam(u_degree, merge_surface.u_knots(), 2,
                                        kTestEpsilon);
  EXPECT_EQ(knot_count, 2);

  // Knot insert
  NURBSSurface insert_surface =
      surface.KnotInsert(NURBSSurface::SurfaceDirection::kUDir, 1, 1);
  insert_surface =
      insert_surface.KnotInsert(NURBSSurface::SurfaceDirection::kUDir, 2, 1);

  // Compare knots
  ASSERT_EQ(merge_surface.u_knots(), insert_surface.u_knots());
  for (size_t idx = 0; idx < merge_surface.u_knots().size(); ++idx) {
    EXPECT_EQ(merge_surface.u_knots()[idx], insert_surface.u_knots()[idx]);
  }
  ASSERT_EQ(merge_surface.v_knots(), insert_surface.v_knots());
  for (size_t idx = 0; idx < merge_surface.v_knots().size(); ++idx) {
    EXPECT_EQ(merge_surface.v_knots()[idx], insert_surface.v_knots()[idx]);
  }

  // Compare control points
  ASSERT_EQ(merge_surface.control_polygon().size(),
            insert_surface.control_polygon().size());
  for (size_t i = 0; i < merge_surface.control_polygon().size(); ++i) {
    ASSERT_EQ(merge_surface.control_polygon()[i].size(),
              insert_surface.control_polygon()[i].size());
    for (size_t j = 0; j < merge_surface.control_polygon()[i].size(); ++j) {
      EXPECT_EQ(merge_surface.control_polygon()[i][j].x,
                insert_surface.control_polygon()[i][j].x);
      EXPECT_EQ(merge_surface.control_polygon()[i][j].y,
                insert_surface.control_polygon()[i][j].y);
      EXPECT_EQ(merge_surface.control_polygon()[i][j].z,
                insert_surface.control_polygon()[i][j].z);
      EXPECT_EQ(merge_surface.control_polygon()[i][j].w,
                insert_surface.control_polygon()[i][j].w);
    }
  }

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = merge_surface.EvaluatePoint(uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, MergeKnotFullSurfaceU) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Merge insert
  std::vector<double> merge_knots = {1, 1, 1, 2, 2};
  NURBSSurface merge_surface = surface.RefineKnotVect(
      merge_knots, NURBSSurface::SurfaceDirection::kUDir);

  // Check knot multiplicity
  int knot_count = knots::MultiplicityParam(u_degree, merge_surface.u_knots(),
                                            1, kTestEpsilon);
  EXPECT_EQ(knot_count, 3);
  knot_count = knots::MultiplicityParam(u_degree, merge_surface.u_knots(), 2,
                                        kTestEpsilon);
  EXPECT_EQ(knot_count, 3);

  // Knot insert
  NURBSSurface insert_surface =
      surface.KnotInsert(NURBSSurface::SurfaceDirection::kUDir, 1, 3);
  insert_surface =
      insert_surface.KnotInsert(NURBSSurface::SurfaceDirection::kUDir, 2, 2);

  // Compare knots
  ASSERT_EQ(merge_surface.u_knots(), insert_surface.u_knots());
  for (size_t idx = 0; idx < merge_surface.u_knots().size(); ++idx) {
    EXPECT_EQ(merge_surface.u_knots()[idx], insert_surface.u_knots()[idx]);
  }
  ASSERT_EQ(merge_surface.v_knots(), insert_surface.v_knots());
  for (size_t idx = 0; idx < merge_surface.v_knots().size(); ++idx) {
    EXPECT_EQ(merge_surface.v_knots()[idx], insert_surface.v_knots()[idx]);
  }

  // Compare control points
  ASSERT_EQ(merge_surface.control_polygon().size(),
            insert_surface.control_polygon().size());
  for (size_t i = 0; i < merge_surface.control_polygon().size(); ++i) {
    ASSERT_EQ(merge_surface.control_polygon()[i].size(),
              insert_surface.control_polygon()[i].size());
    for (size_t j = 0; j < merge_surface.control_polygon()[i].size(); ++j) {
      EXPECT_EQ(merge_surface.control_polygon()[i][j].x,
                insert_surface.control_polygon()[i][j].x);
      EXPECT_EQ(merge_surface.control_polygon()[i][j].y,
                insert_surface.control_polygon()[i][j].y);
      EXPECT_EQ(merge_surface.control_polygon()[i][j].z,
                insert_surface.control_polygon()[i][j].z);
      EXPECT_EQ(merge_surface.control_polygon()[i][j].w,
                insert_surface.control_polygon()[i][j].w);
    }
  }

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = merge_surface.EvaluatePoint(uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, MergeKnotNoneSurfaceV) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Knot Merge
  std::vector<double> merge_knots = {};
  NURBSSurface merge_surface = surface.RefineKnotVect(
      merge_knots, NURBSSurface::SurfaceDirection::kVDir);

  // Check knot multiplicity
  int knot_count = knots::MultiplicityParam(v_degree, merge_surface.v_knots(),
                                            1, kTestEpsilon);
  EXPECT_EQ(knot_count, 1);
  knot_count = knots::MultiplicityParam(v_degree, merge_surface.v_knots(), 2,
                                        kTestEpsilon);
  EXPECT_EQ(knot_count, 2);
  knot_count = knots::MultiplicityParam(v_degree, merge_surface.v_knots(), 3,
                                        kTestEpsilon);
  EXPECT_EQ(knot_count, 0);

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = merge_surface.EvaluatePoint(uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, MergeKnotSinglesSurfaceV) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Merge insert
  std::vector<double> merge_knots = {1, 3};
  NURBSSurface merge_surface = surface.RefineKnotVect(
      merge_knots, NURBSSurface::SurfaceDirection::kVDir);

  // Check knot multiplicity
  int knot_count = knots::MultiplicityParam(v_degree, merge_surface.v_knots(),
                                            1, kTestEpsilon);
  EXPECT_EQ(knot_count, 2);
  knot_count = knots::MultiplicityParam(v_degree, merge_surface.v_knots(), 2,
                                        kTestEpsilon);
  EXPECT_EQ(knot_count, 2);
  knot_count = knots::MultiplicityParam(v_degree, merge_surface.v_knots(), 3,
                                        kTestEpsilon);
  EXPECT_EQ(knot_count, 1);

  // Knot insert
  NURBSSurface insert_surface =
      surface.KnotInsert(NURBSSurface::SurfaceDirection::kVDir, 1, 1);
  insert_surface =
      insert_surface.KnotInsert(NURBSSurface::SurfaceDirection::kVDir, 3, 1);

  // Compare knots
  ASSERT_EQ(merge_surface.u_knots(), insert_surface.u_knots());
  for (size_t idx = 0; idx < merge_surface.u_knots().size(); ++idx) {
    EXPECT_EQ(merge_surface.u_knots()[idx], insert_surface.u_knots()[idx]);
  }
  ASSERT_EQ(merge_surface.v_knots(), insert_surface.v_knots());
  for (size_t idx = 0; idx < merge_surface.v_knots().size(); ++idx) {
    EXPECT_EQ(merge_surface.v_knots()[idx], insert_surface.v_knots()[idx]);
  }

  // Compare control points
  ASSERT_EQ(merge_surface.control_polygon().size(),
            insert_surface.control_polygon().size());
  for (size_t i = 0; i < merge_surface.control_polygon().size(); ++i) {
    ASSERT_EQ(merge_surface.control_polygon()[i].size(),
              insert_surface.control_polygon()[i].size());
    for (size_t j = 0; j < merge_surface.control_polygon()[i].size(); ++j) {
      EXPECT_EQ(merge_surface.control_polygon()[i][j].x,
                insert_surface.control_polygon()[i][j].x);
      EXPECT_EQ(merge_surface.control_polygon()[i][j].y,
                insert_surface.control_polygon()[i][j].y);
      EXPECT_EQ(merge_surface.control_polygon()[i][j].z,
                insert_surface.control_polygon()[i][j].z);
      EXPECT_EQ(merge_surface.control_polygon()[i][j].w,
                insert_surface.control_polygon()[i][j].w);
    }
  }

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = merge_surface.EvaluatePoint(uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, MergeKnotFullSurfaceV) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Merge insert
  std::vector<double> merge_knots = {1, 3, 3};
  NURBSSurface merge_surface = surface.RefineKnotVect(
      merge_knots, NURBSSurface::SurfaceDirection::kVDir);

  // Check knot multiplicity
  int knot_count = knots::MultiplicityParam(v_degree, merge_surface.v_knots(),
                                            1, kTestEpsilon);
  EXPECT_EQ(knot_count, 2);
  knot_count = knots::MultiplicityParam(v_degree, merge_surface.v_knots(), 2,
                                        kTestEpsilon);
  EXPECT_EQ(knot_count, 2);
  knot_count = knots::MultiplicityParam(v_degree, merge_surface.v_knots(), 2,
                                        kTestEpsilon);
  EXPECT_EQ(knot_count, 2);

  // Knot insert
  NURBSSurface insert_surface =
      surface.KnotInsert(NURBSSurface::SurfaceDirection::kVDir, 1, 1);
  insert_surface =
      insert_surface.KnotInsert(NURBSSurface::SurfaceDirection::kVDir, 3, 2);

  // Compare knots
  ASSERT_EQ(merge_surface.u_knots(), insert_surface.u_knots());
  for (size_t idx = 0; idx < merge_surface.u_knots().size(); ++idx) {
    EXPECT_EQ(merge_surface.u_knots()[idx], insert_surface.u_knots()[idx]);
  }
  ASSERT_EQ(merge_surface.v_knots(), insert_surface.v_knots());
  for (size_t idx = 0; idx < merge_surface.v_knots().size(); ++idx) {
    EXPECT_EQ(merge_surface.v_knots()[idx], insert_surface.v_knots()[idx]);
  }

  // Compare control points
  ASSERT_EQ(merge_surface.control_polygon().size(),
            insert_surface.control_polygon().size());
  for (size_t i = 0; i < merge_surface.control_polygon().size(); ++i) {
    ASSERT_EQ(merge_surface.control_polygon()[i].size(),
              insert_surface.control_polygon()[i].size());
    for (size_t j = 0; j < merge_surface.control_polygon()[i].size(); ++j) {
      EXPECT_EQ(merge_surface.control_polygon()[i][j].x,
                insert_surface.control_polygon()[i][j].x);
      EXPECT_EQ(merge_surface.control_polygon()[i][j].y,
                insert_surface.control_polygon()[i][j].y);
      EXPECT_EQ(merge_surface.control_polygon()[i][j].z,
                insert_surface.control_polygon()[i][j].z);
      EXPECT_EQ(merge_surface.control_polygon()[i][j].w,
                insert_surface.control_polygon()[i][j].w);
    }
  }

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = merge_surface.EvaluatePoint(uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, Decompose2DNone) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2,
                               3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point3D> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1},  {1, 0, 1},   {2, 0, 1},
      {2, 1, 1}, {3, 1, 1}, {3, 0, 1},  {4, 0, 1},   {4, 1, 1},
      {5, 1, 1}, {5, 2, 1}, {6, 1, 1},  {7, 3, 1},   {7, 1, 1},
      {8, 2, 1}, {9, 4, 1}, {10, 2, 1}, {10, -1, 1}, {11, 0, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots, interval);

  // Decompose the curve
  std::vector<BezierCurve2D> beziers = nurbs_curve.Decompose();
  ASSERT_EQ(beziers.size(), 5);

  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point2D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point2D point_decomp;
    if (location < 1.0) {
      point_decomp = beziers[0].EvaluateCurve(location);
    } else if (location < 2.0) {
      point_decomp = beziers[1].EvaluateCurve(location - 1.0);
    } else if (location < 3.0) {
      point_decomp = beziers[2].EvaluateCurve(location - 2.0);
    } else if (location < 4.0) {
      point_decomp = beziers[3].EvaluateCurve(location - 3.0);
    } else {
      point_decomp = beziers[4].EvaluateCurve(location - 4.0);
    }

    EXPECT_DOUBLE_EQ(point_nurbs.x, point_decomp.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_decomp.y);
  }
}

TEST(NURBS_Chapter5, Decompose2D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point3D> control_points = {
      {0, 0, 1}, {0, 1, 1}, {1, 1, 1}, {1, 0, 1}, {2, 0, 1},
      {2, 1, 1}, {3, 1, 1}, {3, 0, 1}, {4, 0, 1}, {4, 1, 1}};
  NURBSCurve2D nurbs_curve(degree, control_points, knots, interval);

  // Decompose the curve
  std::vector<BezierCurve2D> beziers = nurbs_curve.Decompose();
  ASSERT_EQ(beziers.size(), 5);

  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point2D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point2D point_decomp;
    if (location < 1.0) {
      point_decomp = beziers[0].EvaluateCurve(location);
    } else if (location < 2.0) {
      point_decomp = beziers[1].EvaluateCurve(location - 1.0);
    } else if (location < 3.0) {
      point_decomp = beziers[2].EvaluateCurve(location - 2.0);
    } else if (location < 4.0) {
      point_decomp = beziers[3].EvaluateCurve(location - 3.0);
    } else {
      point_decomp = beziers[4].EvaluateCurve(location - 4.0);
    }

    EXPECT_DOUBLE_EQ(point_nurbs.x, point_decomp.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_decomp.y);
  }
}

TEST(NURBS_Chapter5, Decompose3DNone) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2,
                               3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point4D> control_points = {
      {0, 0, 1, 1}, {0, 1, 1, 1}, {1, 1, 1, 1},  {1, 0, 1, 1},   {2, 0, 1, 1},
      {2, 1, 1, 1}, {3, 1, 1, 1}, {3, 0, 1, 1},  {4, 0, 1, 1},   {4, 1, 1, 1},
      {5, 1, 1, 1}, {5, 2, 1, 1}, {6, 1, 1, 1},  {7, 3, 1, 1},   {7, 1, 1, 1},
      {8, 2, 1, 1}, {9, 4, 1, 1}, {10, 2, 1, 1}, {10, -1, 1, 1}, {11, 0, 1, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots, interval);

  // Decompose the curve
  std::vector<BezierCurve3D> beziers = nurbs_curve.Decompose();
  ASSERT_EQ(beziers.size(), 5);

  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point3D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point3D point_decomp;
    if (location < 1.0) {
      point_decomp = beziers[0].EvaluateCurve(location);
    } else if (location < 2.0) {
      point_decomp = beziers[1].EvaluateCurve(location - 1.0);
    } else if (location < 3.0) {
      point_decomp = beziers[2].EvaluateCurve(location - 2.0);
    } else if (location < 4.0) {
      point_decomp = beziers[3].EvaluateCurve(location - 3.0);
    } else {
      point_decomp = beziers[4].EvaluateCurve(location - 4.0);
    }

    EXPECT_DOUBLE_EQ(point_nurbs.x, point_decomp.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_decomp.y);
    EXPECT_DOUBLE_EQ(point_nurbs.z, point_decomp.z);
  }
}

TEST(NURBS_Chapter5, Decompose3D) {
  constexpr double kTestEpsilon = std::numeric_limits<double>::epsilon();
  // NURBS Curves
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  std::vector<Point4D> control_points = {
      {0, 0, 1, 1}, {0, 1, 2, 1}, {1, 1, 4, 1}, {1, 0, 3, 1}, {2, 0, 2, 1},
      {2, 1, 0, 1}, {3, 1, 3, 1}, {3, 0, 5, 1}, {4, 0, 5, 1}, {4, 1, 3, 1}};
  NURBSCurve3D nurbs_curve(degree, control_points, knots, interval);

  // Decompose the curve
  std::vector<BezierCurve3D> beziers = nurbs_curve.Decompose();
  ASSERT_EQ(beziers.size(), 5);

  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point3D point_nurbs = nurbs_curve.EvaluateCurve(location);
    Point3D point_decomp;
    if (location < 1.0) {
      point_decomp = beziers[0].EvaluateCurve(location);
    } else if (location < 2.0) {
      point_decomp = beziers[1].EvaluateCurve(location - 1.0);
    } else if (location < 3.0) {
      point_decomp = beziers[2].EvaluateCurve(location - 2.0);
    } else if (location < 4.0) {
      point_decomp = beziers[3].EvaluateCurve(location - 3.0);
    } else {
      point_decomp = beziers[4].EvaluateCurve(location - 4.0);
    }

    EXPECT_DOUBLE_EQ(point_nurbs.x, point_decomp.x);
    EXPECT_DOUBLE_EQ(point_nurbs.y, point_decomp.y);
    EXPECT_DOUBLE_EQ(point_nurbs.z, point_decomp.z);
  }
}

TEST(NURBS_Chapter5, DecomposeSurfaceUNone) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 1, 1, 1,
                                 2, 2, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Merge insert
  std::vector<NURBSSurface> surfaces = surface.DecomposeU();

  // Check surface count
  ASSERT_EQ(surfaces.size(), 3);

  // Compare
  double points = 10.0;
  double div = 1.0 / points;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < static_cast<int32_t>(points + 1.1); ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    Point2D d_uv = {0.0, 0.0};
    size_t u_index = 0;
    if (uv.x < 1.0) {
      u_index = 0;
      d_uv.x = uv.x;
    } else if (uv.x < 2.0) {
      u_index = 1;
      d_uv.x = uv.x - 1.0;
    } else {
      u_index = 2;
      d_uv.x = uv.x - 2.0;
    }
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      d_uv.y = uv.y;

      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_decomp = surfaces[u_index].EvaluatePoint(d_uv);
      EXPECT_NEAR(point_nurbs.x, point_decomp.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_decomp.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_decomp.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, DecomposeSurfaceU) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Merge insert
  std::vector<NURBSSurface> surfaces = surface.DecomposeU();

  // Check surface count
  ASSERT_EQ(surfaces.size(), 3);

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    Point2D d_uv = {0.0, 0.0};
    size_t u_index = 0;
    if (uv.x < 1.0) {
      u_index = 0;
      d_uv.x = uv.x;
    } else if (uv.x < 2.0) {
      u_index = 1;
      d_uv.x = uv.x - 1.0;
    } else {
      u_index = 2;
      d_uv.x = uv.x - 2.0;
    }
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      d_uv.y = uv.y;

      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = surfaces[u_index].EvaluatePoint(d_uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, DecomposeSurfaceVNone) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 1, 1, 1};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 1.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Decompose
  std::vector<BezierSurface> surfaces = surface.DecomposeV();

  // Check surface count
  ASSERT_EQ(surfaces.size(), 4);

  // Compare
  double points = 10.0;
  double div = 1.0 / points;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < static_cast<int32_t>(points + 1.1); ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    Point2D d_uv = uv;
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      size_t v_index = 0;
      if (uv.y < 1.0) {
        v_index = 0;
        d_uv.y = uv.y;
      } else if (uv.y < 2.0) {
        v_index = 1;
        d_uv.y = uv.y - 1.0;
      } else if (uv.y < 3.0) {
        v_index = 2;
        d_uv.y = uv.y - 2.0;
      } else {
        v_index = 3;
        d_uv.y = uv.y - 3.0;
      }

      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_decomp = surfaces[v_index].EvaluatePoint(d_uv);
      EXPECT_NEAR(point_nurbs.x, point_decomp.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_decomp.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_decomp.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, DecomposeSurfaceV) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 1, 1, 1};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Decompose
  std::vector<BezierSurface> surfaces = surface.DecomposeV();

  // Check surface count
  ASSERT_EQ(surfaces.size(), 4);

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    Point2D d_uv = uv;

    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      size_t v_index = 0;
      if (uv.y < 1.0) {
        v_index = 0;
        d_uv.y = uv.y;
      } else if (uv.y < 2.0) {
        v_index = 1;
        d_uv.y = uv.y - 1.0;
      } else if (uv.y < 3.0) {
        v_index = 2;
        d_uv.y = uv.y - 2.0;
      } else {
        v_index = 3;
        d_uv.y = uv.y - 3.0;
      }

      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = surfaces[v_index].EvaluatePoint(d_uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
    }
  }
}

TEST(NURBS_Chapter5, DecomposeSurface) {
  constexpr double kTestEpsilon =
      std::numeric_limits<double>::epsilon() * 100.0;
  // NURBS surface
  uint32_t u_degree = 3;
  uint32_t v_degree = 2;
  std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 3, 3, 3, 3};
  std::vector<double> v_knots = {0, 0, 0, 1, 1, 2, 2, 2, 3, 4, 4, 4};
  Point2D u_interval = {0.0, 3.0};
  Point2D v_interval = {0.0, 4.0};
  std::vector<std::vector<Point4D>> control_points;
  uint32_t u_points = static_cast<uint32_t>(u_knots.size()) - u_degree - 1;
  uint32_t v_points = static_cast<uint32_t>(v_knots.size()) - v_degree - 1;
  control_points.resize(u_points);
  for (uint32_t u_index = 0; u_index < u_points; ++u_index) {
    control_points[u_index].resize(v_points);
    for (uint32_t v_index = 0; v_index < v_points; ++v_index) {
      double u_val =
          static_cast<double>(u_index) - (static_cast<double>(u_points) * 0.5);
      double v_val =
          static_cast<double>(v_index) - (static_cast<double>(v_points) * 0.5);
      double dist_0_sq = ((u_val * u_val) + (v_val * v_val));
      control_points[u_index][v_index] = {u_val, v_val, dist_0_sq, 1.0};
    }
  }

  NURBSSurface surface(u_degree, v_degree, u_knots, v_knots, control_points,
                       u_interval, v_interval);

  // Merge insert
  std::vector<std::vector<BezierSurface>> surfaces = surface.Decompose();

  // Check surface count
  ASSERT_EQ(surfaces.size(), 3);
  ASSERT_EQ(surfaces[0].size(), 4);

  // Compare
  double div = 1.0 / 99.0;
  double u_div = div * (u_interval.y - u_interval.x);
  double v_div = div * (v_interval.y - v_interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    Point2D uv = {(static_cast<double>(i) * u_div) + u_interval.x, 0.0};
    Point2D d_uv = {0.0, 0.0};
    size_t u_index = 0;
    if (uv.x < 1.0) {
      u_index = 0;
      d_uv.x = uv.x;
    } else if (uv.x < 2.0) {
      u_index = 1;
      d_uv.x = uv.x - 1.0;
    } else {
      u_index = 2;
      d_uv.x = uv.x - 2.0;
    }
    for (int32_t j = -1; j < 101; ++j) {
      uv.y = (static_cast<double>(j) * v_div) + v_interval.x;
      size_t v_index = 0;
      if (uv.y < 1.0) {
        v_index = 0;
        d_uv.y = uv.y;
      } else if (uv.y < 2.0) {
        v_index = 1;
        d_uv.y = uv.y - 1.0;
      } else if (uv.y < 3.0) {
        v_index = 2;
        d_uv.y = uv.y - 2.0;
      } else {
        v_index = 3;
        d_uv.y = uv.y - 3.0;
      }
      Point3D point_nurbs = surface.EvaluatePoint(uv);
      Point3D point_isrt = surfaces[u_index][v_index].EvaluatePoint(d_uv);
      EXPECT_NEAR(point_nurbs.x, point_isrt.x, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.y, point_isrt.y, kTestEpsilon);
      EXPECT_NEAR(point_nurbs.z, point_isrt.z, kTestEpsilon);
    }
  }
}
}  // namespace nurbs
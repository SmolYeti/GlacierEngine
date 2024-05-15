#include <gtest/gtest.h>

// NURBS_CPP
#include "include/b_spline_curve.hpp"
#include "include/b_spline_surface.hpp"
#include "include/nurbs_curve.hpp"
#include "include/nurbs_surface.hpp"

namespace nurbs {
TEST(NURBS_Chapter4, NURBS_BSpline_Curve2D) {
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  // B-Spline Curve
  std::vector<Point2D> control_points = {{0, 0}, {0, 1}, {1, 1}, {1, 0},
                                            {2, 0}, {2, 1}, {3, 1}, {3, 0},
                                            {4, 1}, {5, 3}};
  BSplineCurve2D b_spline(degree, control_points, knots, interval);
  // NURBS Curves
  std::vector<Point3D> nurbs_pts;
  nurbs_pts.reserve(control_points.size());
  for (auto &pt : control_points) {
    nurbs_pts.push_back({pt.x, pt.y, 1.0});
  }
  NURBSCurve2D nurbs_curve(degree, nurbs_pts, knots, interval);
  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point2D point_bspl = b_spline.EvaluateCurve(location);
    Point2D point_nurbs = nurbs_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_bspl.x, point_nurbs.x);
    EXPECT_DOUBLE_EQ(point_bspl.y, point_nurbs.y);
  }
}

TEST(NURBS_Chapter4, NURBS_BSpline_Curve3D) {
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  // B-Spline Curve
  std::vector<Point3D> control_points = {
      {0, 0, 0}, {0, 1, 2}, {1, 1, 3}, {1, 0, 2},  {2, 0, 3},
      {2, 1, 5}, {3, 1, 5}, {3, 0, 7}, {5, -1, 8}, {7, 2, 9}};
  BSplineCurve3D b_spline(degree, control_points, knots, interval);
  // NURBS Curves
  std::vector<Point4D> nurbs_pts;
  nurbs_pts.reserve(control_points.size());
  for (auto &pt : control_points) {
    nurbs_pts.push_back({pt.x, pt.y, pt.z, 1.0});
  }
  NURBSCurve3D nurbs_curve(degree, nurbs_pts, knots, interval);
  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    Point3D point_bspl = b_spline.EvaluateCurve(location);
    Point3D point_nurbs = nurbs_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_bspl.x, point_nurbs.x);
    EXPECT_DOUBLE_EQ(point_bspl.y, point_nurbs.y);
    EXPECT_DOUBLE_EQ(point_bspl.z, point_nurbs.z);
  }
}

TEST(NURBS_Chapter4, NURBS_BSplineDerivCompare2D) {
  constexpr double tolerance = std::numeric_limits<double>::epsilon() * 100.0;
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  // B-Spline Curve
  std::vector<Point2D> control_points = {{0, 0}, {0, 1}, {1, 1}, {1, 0},
                                            {2, 0}, {2, 1}, {3, 1}, {3, 0},
                                            {5, 0}, {5, 3}};
  BSplineCurve2D b_spline(degree, control_points, knots, interval);
  // NURBS Curves
  std::vector<Point3D> nurbs_pts;
  nurbs_pts.reserve(control_points.size());
  for (auto &pt : control_points) {
    nurbs_pts.push_back({pt.x, pt.y, 1.0});
  }
  NURBSCurve2D nurbs_curve(degree, nurbs_pts, knots, interval);
  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    std::vector<Point2D> points_bspl = b_spline.Derivatives(location, 2);
    std::vector<Point2D> points_nurbs =
        nurbs_curve.EvaluateDerivative(location, 2);
    ASSERT_EQ(points_bspl.size(), points_nurbs.size());
    for (size_t index = 0; index < points_bspl.size(); ++index) {
      Point2D point_bspl = points_bspl[index];
      Point2D point_nurbs = points_nurbs[index];
      EXPECT_NEAR(point_bspl.x, point_nurbs.x, tolerance);
      EXPECT_NEAR(point_bspl.y, point_nurbs.y, tolerance);
    }
  }
}

TEST(NURBS_Chapter4, NURBS_BSplineDerivCompare3D) {
  constexpr double tolerance = std::numeric_limits<double>::epsilon() * 100.0;
  uint32_t degree = 3;
  std::vector<double> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  Point2D interval = {0.0, 5.0};
  // B-Spline Curve
  std::vector<Point3D> control_points = {
      {0, 0, 0}, {0, 1, 2}, {1, 1, 3}, {1, 0, 2},  {2, 0, 3},
      {2, 1, 5}, {3, 1, 5}, {3, 0, 7}, {5, -1, 8}, {7, 2, 9}};
  BSplineCurve3D b_spline(degree, control_points, knots, interval);
  // NURBS Curves
  std::vector<Point4D> nurbs_pts;
  nurbs_pts.reserve(control_points.size());
  for (auto &pt : control_points) {
    nurbs_pts.push_back({pt.x, pt.y, pt.z, 1.0});
  }
  NURBSCurve3D nurbs_curve(degree, nurbs_pts, knots, interval);
  // Compare
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = -1; i < 101; ++i) {
    double location = (static_cast<double>(i) * div) + interval.x;
    std::vector<Point3D> points_bspl = b_spline.Derivatives(location, 2);
    std::vector<Point3D> points_nurbs =
        nurbs_curve.EvaluateDerivative(location, 2);
    ASSERT_EQ(points_bspl.size(), points_nurbs.size());
    for (size_t index = 0; index < points_bspl.size(); ++index) {
      Point3D point_bspl = points_bspl[index];
      Point3D point_nurbs = points_nurbs[index];
      EXPECT_NEAR(point_bspl.x, point_nurbs.x, tolerance);
      EXPECT_NEAR(point_bspl.y, point_nurbs.y, tolerance);
      EXPECT_NEAR(point_bspl.z, point_nurbs.z, tolerance);
    }
  }
}

TEST(NURBS_Chapter4, NURBS_BSplineSurfaceCompare) {
  constexpr double tolerance = std::numeric_limits<double>::epsilon() * 100.0;
  Point2D interval = {0, 4};
  uint32_t u_degree = 4;
  uint32_t v_degree = 3;
  std::vector<double> u_knots = {0, 0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 4, 4, 4};
  std::vector<double> v_knots = {0, 0, 0, 0, 1, 1, 2, 3, 3, 4, 4, 4, 4};
  std::vector<std::vector<Point3D>> control_points;
  control_points.resize(9);
  for (size_t i = 0; i < 9; ++i) {
    control_points[i].resize(9);
    double i_val = static_cast<double>(i + 1);
    for (size_t j = 0; j < 9; ++j) {
      double j_val = static_cast<double>(j);
      control_points[i][j] = {j_val, i_val - j_val, i_val};
    }
  }
  BSplineSurface b_spline_surface(u_degree, v_degree, u_knots, v_knots,
                                  control_points, interval, interval);

  // NURBS Surface
  std::vector<std::vector<Point4D>> nurbs_pts;
  nurbs_pts.reserve(control_points.size());
  for (auto &vec : control_points) {
    nurbs_pts.push_back({});
    for (auto &pt : vec) {
      nurbs_pts.back().push_back({pt.x, pt.y, pt.z, 1.0});
    }
  }
  NURBSSurface nurbs_surface(u_degree, v_degree, u_knots, v_knots, nurbs_pts,
                             interval, interval);

  // Compare the surface to the curves
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = 0; i < 100; ++i) {
    Point2D location = {(static_cast<double>(i) * div) + interval.x, 0};
    for (int32_t j = 0; j < 100; ++j) {
      location.y = (static_cast<double>(j) * div) + interval.x;
      Point3D point_bspl = b_spline_surface.EvaluatePoint(location);
      Point3D point_nurbs = nurbs_surface.EvaluatePoint(location);
      // Numbers are not between 0 and 1, so the epsilon needs to be scaled
      // This is not the proper scaling though.
      EXPECT_NEAR(point_bspl.x, point_nurbs.x, tolerance);
      EXPECT_NEAR(point_bspl.y, point_nurbs.y, tolerance);
      EXPECT_NEAR(point_bspl.z, point_nurbs.z, tolerance);
    }
  }
}

TEST(NURBS_Chapter4, NURBS_BSplineSurfaceDerivCompare) {
  constexpr double tolerance = std::numeric_limits<double>::epsilon() * 1000.0;
  Point2D interval = {0, 4};
  uint32_t u_degree = 4;
  uint32_t v_degree = 3;
  std::vector<double> u_knots = {0, 0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 4, 4, 4};
  std::vector<double> v_knots = {0, 0, 0, 0, 1, 1, 2, 3, 3, 4, 4, 4, 4};
  std::vector<std::vector<Point3D>> control_points;
  control_points.resize(9);
  for (size_t i = 0; i < 9; ++i) {
    control_points[i].resize(9);
    double i_val = static_cast<double>(i + 1);
    for (size_t j = 0; j < 9; ++j) {
      double j_val = static_cast<double>(j);
      control_points[i][j] = {j_val, i_val - j_val, i_val};
    }
  }
  BSplineSurface b_spline_surface(u_degree, v_degree, u_knots, v_knots,
                                  control_points, interval, interval);

  // NURBS Surface
  std::vector<std::vector<Point4D>> nurbs_pts;
  nurbs_pts.reserve(control_points.size());
  for (auto &vec : control_points) {
    nurbs_pts.push_back({});
    for (auto &pt : vec) {
      nurbs_pts.back().push_back({pt.x, pt.y, pt.z, 1.0});
    }
  }
  NURBSSurface nurbs_surface(u_degree, v_degree, u_knots, v_knots, nurbs_pts,
                             interval, interval);

  // Compare the surface to the curves
  double div = (1.0 / 99.0) * (interval.y - interval.x);
  for (int32_t i = 0; i < 100; ++i) {
    Point2D location = {(static_cast<double>(i) * div) + interval.x, 0};
    for (int32_t j = 0; j < 100; ++j) {
      location.y = (static_cast<double>(j) * div) + interval.x;
      std::vector<std::vector<Point3D>> points_bspl =
          b_spline_surface.Derivative(location, 2);
      std::vector<std::vector<Point3D>> points_nurbs =
          nurbs_surface.Derivatives(location, 2);
      ASSERT_EQ(points_bspl.size(), points_nurbs.size());
      for (size_t index_i = 0; index_i < points_bspl.size(); ++index_i) {
        ASSERT_EQ(points_bspl[index_i].size(), points_nurbs[index_i].size());
        for (size_t index_j = 0; index_j < points_bspl[index_i].size();
             ++index_j) {
          auto &point_bspl = points_bspl[index_i][index_j];
          auto &point_nurbs = points_nurbs[index_i][index_j];
          // Numbers are not between 0 and 1, so the epsilon needs to be scaled
          // This is not the proper scaling though.
          EXPECT_NEAR(point_bspl.x, point_nurbs.x, tolerance);
          EXPECT_NEAR(point_bspl.y, point_nurbs.y, tolerance);
          EXPECT_NEAR(point_bspl.z, point_nurbs.z, tolerance);
        }
      }
    }
  }
}

} // namespace nurbs
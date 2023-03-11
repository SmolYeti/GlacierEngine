#include <gtest/gtest.h>

// NURBS_CPP
#include "include/b_spline_curve.hpp"
#include "include/b_spline_surface.hpp"
#include "include/nurbs_curve.hpp"
#include "include/nurbs_surface.hpp"

namespace nurbs {
TEST(NURBS_Chapter4, NURBS_BSpline_Curve2D) {
  uint32_t degree = 3;
  std::vector<uint32_t> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  // B-Spline Curve
  std::vector<glm::dvec2> control_points = {{0, 0}, {0, 1}, {1, 1}, {1, 0},
                                            {2, 0}, {2, 1}, {3, 1}, {3, 0}};
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
    glm::dvec2 point_nurbs = nurbs_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_bspl.x, point_nurbs.x);
    EXPECT_DOUBLE_EQ(point_bspl.y, point_nurbs.y);
  }
}

TEST(NURBS_Chapter4, NURBS_BSpline_Curve3D) {
  uint32_t degree = 3;
  std::vector<uint32_t> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  // B-Spline Curve
  std::vector<glm::dvec3> control_points = {{0, 0, 0}, {0, 1, 2}, {1, 1, 3},
                                            {1, 0, 2}, {2, 0, 3}, {2, 1, 5},
                                            {3, 1, 5}, {3, 0, 7}};
  BSplineCurve3D b_spline(degree, control_points, knots);
  // NURBS Curves
  std::vector<glm::dvec4> nurbs_pts;
  nurbs_pts.reserve(control_points.size());
  for (auto& pt : control_points) {
    nurbs_pts.push_back({pt.x, pt.y, pt.z, 1.0});
  }
  NURBSCurve3D nurbs_curve(degree, nurbs_pts, knots);
  // Compare
  double div = 1.0 / 99.0;
  for (int32_t i = -1; i < 101; ++i) {
    double location = static_cast<double>(i) * div;
    glm::dvec3 point_bspl = b_spline.EvaluateCurve(location);
    glm::dvec3 point_nurbs = nurbs_curve.EvaluateCurve(location);
    EXPECT_DOUBLE_EQ(point_bspl.x, point_nurbs.x);
    EXPECT_DOUBLE_EQ(point_bspl.y, point_nurbs.y);
    EXPECT_DOUBLE_EQ(point_bspl.z, point_nurbs.z);
  }
}

TEST(NURBS_Chapter4, NURBS_BSplineDerivCompare2D) {
  uint32_t degree = 3;
  std::vector<uint32_t> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  // B-Spline Curve
  std::vector<glm::dvec2> control_points = {{0, 0}, {0, 1}, {1, 1}, {1, 0},
                                            {2, 0}, {2, 1}, {3, 1}, {3, 0}};
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
    std::vector<glm::dvec2> points_bspl = b_spline.Derivatives(location, 2);
    std::vector<glm::dvec2> points_nurbs =
        nurbs_curve.EvaluateDerivative(location, 2);
    ASSERT_EQ(points_bspl.size(), points_nurbs.size());
    for (size_t index = 0; index < points_bspl.size(); ++index) {
      glm::dvec2 point_bspl = points_bspl[index];
      glm::dvec2 point_nurbs = points_nurbs[index];
      EXPECT_NEAR(point_bspl.x, point_nurbs.x,
                  std::numeric_limits<double>::epsilon() * 100);
      EXPECT_NEAR(point_bspl.y, point_nurbs.y,
                  std::numeric_limits<double>::epsilon() * 100);
    }
  }
}

TEST(NURBS_Chapter4, NURBS_BSplineDerivCompare3D) {
  uint32_t degree = 3;
  std::vector<uint32_t> knots = {0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 5, 5, 5, 5};
  // B-Spline Curve
  std::vector<glm::dvec3> control_points = {{0, 0, 0}, {0, 1, 2}, {1, 1, 3},
                                            {1, 0, 2}, {2, 0, 3}, {2, 1, 5},
                                            {3, 1, 5}, {3, 0, 7}};
  BSplineCurve3D b_spline(degree, control_points, knots);
  // NURBS Curves
  std::vector<glm::dvec4> nurbs_pts;
  nurbs_pts.reserve(control_points.size());
  for (auto& pt : control_points) {
    nurbs_pts.push_back({pt.x, pt.y, pt.z, 1.0});
  }
  NURBSCurve3D nurbs_curve(degree, nurbs_pts, knots);
  // Compare
  double div = 1.0 / 99.0;
  for (int32_t i = -1; i < 101; ++i) {
    double location = static_cast<double>(i) * div;
    std::vector<glm::dvec3> points_bspl = b_spline.Derivatives(location, 2);
    std::vector<glm::dvec3> points_nurbs =
        nurbs_curve.EvaluateDerivative(location, 2);
    ASSERT_EQ(points_bspl.size(), points_nurbs.size());
    for (size_t index = 0; index < points_bspl.size(); ++index) {
      glm::dvec3 point_bspl = points_bspl[index];
      glm::dvec3 point_nurbs = points_nurbs[index];
      EXPECT_NEAR(point_bspl.x, point_nurbs.x,
                  std::numeric_limits<double>::epsilon() * 100);
      EXPECT_NEAR(point_bspl.y, point_nurbs.y,
                  std::numeric_limits<double>::epsilon() * 100);
      EXPECT_NEAR(point_bspl.z, point_nurbs.z,
                  std::numeric_limits<double>::epsilon() * 100);
    }
  }
}

TEST(NURBS_Chapter4, NURBS_BSplineSurfaceCompare) {
  glm::dvec2 interval = {0, 4};
  uint32_t u_degree = 4;
  uint32_t v_degree = 3;
  std::vector<uint32_t> u_knots = {0, 0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 4, 4, 4};
  std::vector<uint32_t> v_knots = {0, 0, 0, 0, 1, 1, 2, 3, 3, 4, 4, 4, 4};
  std::vector<std::vector<glm::dvec3>> control_points = {
      {{0, 0, 1},
       {1, 0, 0},
       {2, 0, 1},
       {3, 0, 1},
       {4, 0, 1},
       {5, 0, 1},
       {6, 0, 1}},  //
      {{0, 0, 2},
       {1, 0, 0},
       {2, 0, 2},
       {3, 0, 2},
       {4, 0, 2},
       {5, 0, 2},
       {6, 0, 2}},  //
      {{0, 0, 3},
       {1, 0, 0},
       {2, 0, 3},
       {3, 0, 3},
       {4, 0, 3},
       {5, 0, 3},
       {6, 0, 3}},  //
      {{0, 0, 4},
       {1, 0, 0},
       {2, 0, 4},
       {3, 0, 4},
       {4, 0, 4},
       {5, 0, 4},
       {6, 0, 4}},  //
      {{0, 0, 5},
       {1, 0, 0},
       {2, 0, 5},
       {3, 0, 5},
       {4, 0, 5},
       {5, 0, 5},
       {6, 0, 5}},  //
      {{0, 0, 6},
       {1, 0, 0},
       {2, 0, 6},
       {3, 0, 6},
       {4, 0, 6},
       {5, 0, 6},
       {6, 0, 6}},  //
      {{0, 0, 7},
       {1, 0, 0},
       {2, 0, 7},
       {3, 0, 7},
       {4, 0, 7},
       {5, 0, 7},
       {6, 0, 7}},  //
      {{0, 0, 8},
       {1, 0, 0},
       {2, 0, 8},
       {3, 0, 8},
       {4, 0, 8},
       {5, 0, 8},
       {6, 0, 8}},  //
  };
  BSplineSurface b_spline_surface(u_degree, v_degree, u_knots, v_knots,
                                  control_points, interval, interval);

  // NURBS Surface
  std::vector<std::vector<glm::dvec4>> nurbs_pts;
  nurbs_pts.reserve(control_points.size());
  for (auto& vec : control_points) {
    nurbs_pts.push_back({});
    for (auto& pt : vec) {
      nurbs_pts.back().push_back({pt.x, pt.y, pt.z, 1.0});
    }
  }
  NURBSSurface nurbs_surface(u_degree, v_degree, u_knots, v_knots, nurbs_pts,
                             interval, interval);

  // Compare the surface to the curves
  double div = 1.0 / 99.0;
  for (int32_t i = 0; i < 100; ++i) {
    glm::dvec2 location = {static_cast<double>(i) * div, 0};
    for (int32_t j = 0; j < 100; ++j) {
      location.y = static_cast<double>(j) * div;
      glm::dvec3 point_bspl = b_spline_surface.EvaluatePoint(location);
      glm::dvec3 point_nurbs = nurbs_surface.EvaluatePoint(location);
      // Numbers are not between 0 and 1, so the epsilon needs to be scaled
      // This is not the proper scaling though.
      EXPECT_NEAR(point_bspl.x, point_nurbs.x,
                  std::numeric_limits<double>::epsilon() * 10);
      EXPECT_NEAR(point_bspl.y, point_nurbs.y,
                  std::numeric_limits<double>::epsilon() * 10);
      EXPECT_NEAR(point_bspl.z, point_nurbs.z,
                  std::numeric_limits<double>::epsilon() * 10);
    }
  }
}

TEST(NURBS_Chapter4, NURBS_BSplineSurfaceDerivCompare) {
  glm::dvec2 interval = {0, 4};
  uint32_t u_degree = 4;
  uint32_t v_degree = 3;
  std::vector<uint32_t> u_knots = {0, 0, 0, 0, 0, 1, 2, 2, 3, 4, 4, 4, 4, 4};
  std::vector<uint32_t> v_knots = {0, 0, 0, 0, 1, 1, 2, 3, 3, 4, 4, 4, 4};
  std::vector<std::vector<glm::dvec3>> control_points = {
      {{0, 0, 1},
       {1, 0, 0},
       {2, 0, 1},
       {3, 0, 1},
       {4, 0, 1},
       {5, 0, 1},
       {6, 0, 1}},  //
      {{0, 0, 2},
       {1, 0, 0},
       {2, 0, 2},
       {3, 0, 2},
       {4, 0, 2},
       {5, 0, 2},
       {6, 0, 2}},  //
      {{0, 0, 3},
       {1, 0, 0},
       {2, 0, 3},
       {3, 0, 3},
       {4, 0, 3},
       {5, 0, 3},
       {6, 0, 3}},  //
      {{0, 0, 4},
       {1, 0, 0},
       {2, 0, 4},
       {3, 0, 4},
       {4, 0, 4},
       {5, 0, 4},
       {6, 0, 4}},  //
      {{0, 0, 5},
       {1, 0, 0},
       {2, 0, 5},
       {3, 0, 5},
       {4, 0, 5},
       {5, 0, 5},
       {6, 0, 5}},  //
      {{0, 0, 6},
       {1, 0, 0},
       {2, 0, 6},
       {3, 0, 6},
       {4, 0, 6},
       {5, 0, 6},
       {6, 0, 6}},  //
      {{0, 0, 7},
       {1, 0, 0},
       {2, 0, 7},
       {3, 0, 7},
       {4, 0, 7},
       {5, 0, 7},
       {6, 0, 7}},  //
      {{0, 0, 8},
       {1, 0, 0},
       {2, 0, 8},
       {3, 0, 8},
       {4, 0, 8},
       {5, 0, 8},
       {6, 0, 8}},  //
  };
  BSplineSurface b_spline_surface(u_degree, v_degree, u_knots, v_knots,
                                  control_points, interval, interval);

  // NURBS Surface
  std::vector<std::vector<glm::dvec4>> nurbs_pts;
  nurbs_pts.reserve(control_points.size());
  for (auto& vec : control_points) {
    nurbs_pts.push_back({});
    for (auto& pt : vec) {
      nurbs_pts.back().push_back({pt.x, pt.y, pt.z, 1.0});
    }
  }
  NURBSSurface nurbs_surface(u_degree, v_degree, u_knots, v_knots, nurbs_pts,
                             interval, interval);

  // Compare the surface to the curves
  double div = 1.0 / 99.0;
  for (int32_t i = 0; i < 100; ++i) {
    glm::dvec2 location = {static_cast<double>(i) * div, 0};
    for (int32_t j = 0; j < 100; ++j) {
      location.y = static_cast<double>(j) * div;
      std::vector<std::vector<glm::dvec3>> points_bspl =
          b_spline_surface.Derivative(location, 2);
      std::vector<std::vector<glm::dvec3>> points_nurbs =
          nurbs_surface.Derivatives(location, 2);
      ASSERT_EQ(points_bspl.size(), points_nurbs.size());
      for (size_t index_i = 0; index_i < points_bspl.size(); ++index_i) {
        ASSERT_EQ(points_bspl[index_i].size(), points_nurbs[index_i].size());
        for (size_t index_j = 0; index_j < points_bspl[index_i].size();
             ++index_j) {
          auto& point_bspl = points_bspl[index_i][index_j];
          auto& point_nurbs = points_nurbs[index_i][index_j];
          // Numbers are not between 0 and 1, so the epsilon needs to be scaled
          // This is not the proper scaling though.
          EXPECT_NEAR(point_bspl.x, point_nurbs.x,
                      std::numeric_limits<double>::epsilon() * 1000);
          EXPECT_NEAR(point_bspl.y, point_nurbs.y,
                      std::numeric_limits<double>::epsilon() * 1000);
          EXPECT_NEAR(point_bspl.z, point_nurbs.z,
                      std::numeric_limits<double>::epsilon() * 1000);
        }
      }
    }
  }
}

}  // namespace nurbs
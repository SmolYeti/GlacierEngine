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

TEST(NURBS_Chapter1, BernsteinPartitionOfUnity) {
  constexpr double div = 1.0 / 99.0;
  for (size_t n = 1; n < 5; ++n) {
    for (uint32_t i = 0; i < 100; ++i) {
      const double u = static_cast<double>(i) * div;
      const std::vector<double> all_bern = BezierCurveUtil::AllBernstein(n, u);
      double sum = 0;
      for (double bern : all_bern) {
        sum += bern;
      }
      EXPECT_DOUBLE_EQ(sum, 1.0);
    }
  }
}

TEST(NURBS_Chapter1, BernsteinStartEnd) {
  const double start_u = 0.0;
  const double end_u = 1.0;
  for (size_t n = 1; n < 10; ++n) {
    const double bern_start = BezierCurveUtil::Bernstein(0, n, start_u);
    const double bern_end = BezierCurveUtil::Bernstein(n, n, end_u);
    EXPECT_DOUBLE_EQ(bern_start, 1.0);
    EXPECT_DOUBLE_EQ(bern_end, 1.0);
  }
}

TEST(NURBS_Chapter1, BernsteinMax) {
  constexpr double div = 1.0 / 99.0;
  for (size_t n = 1; n < 5; ++n) {
    std::vector<double> maxes = {};
    std::vector<double> u_vals = {};
    // Build the max berstein list
    double n_1 = 1.0 / static_cast<double>(n);
    maxes.reserve(n);
    u_vals.reserve(n);
    for (uint32_t i = 0; i <= n; ++i) {
      u_vals.push_back(static_cast<double>(i) * n_1);
      maxes.push_back(BezierCurveUtil::Bernstein(i, n, u_vals.back()));
    }

    // Check to make sure the max is never exceeded
    for (uint32_t i = 0; i < 100; ++i) {
      const double u = static_cast<double>(i) * div;
      const std::vector<double> all_bern = BezierCurveUtil::AllBernstein(n, u);

      for (uint32_t j = 0; j <= n; ++j) {
        if (std::abs(u_vals[j] - u) < std::numeric_limits<double>::epsilon()) {
          EXPECT_DOUBLE_EQ(all_bern[j], maxes[j]);
        } else {
          EXPECT_LT(all_bern[j], maxes[j]);
        }
      }
    }
  }
}

TEST(NURBS_Chapter1, Bezier2DConstruct) {
  const std::vector<Point2D> control_points;
  const BezierCurve2D bezier(control_points);
}

TEST(NURBS_Chapter1, Bezier2DPoint) {
  std::vector<Point2D> control_points = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  const BezierCurve2D bezier(control_points);

  const Point2D point = bezier.EvaluateCurve(0.5);

  std::vector<Point2D> test_points = control_points;
  while (test_points.size() > 1) {
    std::vector<Point2D> temp_points;
    for (uint32_t i = 1; i < test_points.size(); ++i) {
      temp_points.push_back((test_points[i - 1] + test_points[i]) * 0.5);
    }
    test_points = temp_points;
  }

  EXPECT_DOUBLE_EQ(test_points[0].x, point.x);
  EXPECT_DOUBLE_EQ(test_points[0].y, point.y);
}

TEST(NURBS_Chapter1, Bezier2DPointInterval) {
  std::vector<Point2D> control_points = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  const BezierCurve2D bezier(control_points, {0.0, 10.0});

  const Point2D point = bezier.EvaluateCurve(5.0);

  std::vector<Point2D> test_points = control_points;
  while (test_points.size() > 1) {
    std::vector<Point2D> temp_points;
    for (uint32_t i = 1; i < test_points.size(); ++i) {
      temp_points.push_back((test_points[i - 1] + test_points[i]) * 0.5);
    }
    test_points = temp_points;
  }

  EXPECT_DOUBLE_EQ(test_points[0].x, point.x);
  EXPECT_DOUBLE_EQ(test_points[0].y, point.y);
}

TEST(NURBS_Chapter1, Bezier2DPoints) {
  std::vector<Point2D> control_points = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  const Point2D interval = {0.0, 1.0};
  const BezierCurve2D bezier(control_points, interval);

  const std::vector<Point2D> points = bezier.EvaluateCurvePoints(100);

  constexpr double div = 1.0 / 99.0;
  for (uint32_t i = 0; i < 100; ++i) {
    const double u = static_cast<double>(i) * div;
    const double u_i = 1.0 - u;
    std::vector<Point2D> test_points = control_points;
    while (test_points.size() > 1) {
      std::vector<Point2D> temp_points;
      for (uint32_t i = 1; i < test_points.size(); ++i) {
        temp_points.push_back((u_i * test_points[i - 1]) +
                              (u * test_points[i]));
      }
      test_points = temp_points;
    }

    EXPECT_DOUBLE_EQ(test_points[0].x, points[i].x);
    EXPECT_DOUBLE_EQ(test_points[0].y, points[i].y);
  }
}

TEST(NURBS_Chapter1, Bezier2DPointsInterval) {
  double tolerance = std::numeric_limits<double>::epsilon() * 10;
  std::vector<Point2D> control_points = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
  const Point2D interval = {0.0, 10.0};
  const BezierCurve2D bezier(control_points, interval);

  const std::vector<Point2D> points = bezier.EvaluateCurvePoints(100);

  constexpr double div = 1.0 / 99.0;
  for (uint32_t i = 0; i < 100; ++i) {
    const double u = static_cast<double>(i) * div;
    const double u_i = 1.0 - u;
    std::vector<Point2D> test_points = control_points;
    while (test_points.size() > 1) {
      std::vector<Point2D> temp_points;
      for (uint32_t i = 1; i < test_points.size(); ++i) {
        temp_points.push_back((u_i * test_points[i - 1]) +
                              (u * test_points[i]));
      }
      test_points = temp_points;
    }

    EXPECT_NEAR(test_points[0].x, points[i].x, tolerance);
    EXPECT_NEAR(test_points[0].y, points[i].y, tolerance);
  }
}

TEST(NURBS_Chapter1, Bezier2DEndDerivatives) {
  std::vector<Point2D> control_points = {{0, 0}, {1, 1}, {2, 1}, {3, 0}};
  const BezierCurve2D bezier(control_points);

  const Point2D start_d = bezier.Derivative(0.0);
  const Point2D end_d = bezier.Derivative(1.0);

  const double n = static_cast<double>(control_points.size() - 1);
  // The start derivative should be n(P1 - P0)
  const Point2D start_calc = n * (control_points[1] - control_points[0]);
  EXPECT_DOUBLE_EQ(start_calc.x, start_d.x);
  EXPECT_DOUBLE_EQ(start_calc.y, start_d.y);
  // The end derivative should be n(Pn - Pn-1)
  const Point2D end_calc = n * (control_points[3] - control_points[2]);
  EXPECT_DOUBLE_EQ(end_calc.x, end_d.x);
  EXPECT_DOUBLE_EQ(end_calc.y, end_d.y);
}

TEST(NURBS_Chapter1, Bezier2DEndDerivativesInterval) {
  std::vector<Point2D> control_points = {{0, 0}, {1, 1}, {2, 1}, {3, 0}};
  const BezierCurve2D bezier(control_points, {0.0, 17.5});

  const Point2D start_d = bezier.Derivative(0.0);
  const Point2D end_d = bezier.Derivative(17.5);

  const double n = static_cast<double>(control_points.size() - 1);
  // The start derivative should be n(P1 - P0)
  const Point2D start_calc = n * (control_points[1] - control_points[0]);
  EXPECT_DOUBLE_EQ(start_calc.x, start_d.x);
  EXPECT_DOUBLE_EQ(start_calc.y, start_d.y);
  // The end derivative should be n(Pn - Pn-1)
  const Point2D end_calc = n * (control_points[3] - control_points[2]);
  EXPECT_DOUBLE_EQ(end_calc.x, end_d.x);
  EXPECT_DOUBLE_EQ(end_calc.y, end_d.y);
}

TEST(NURBS_Chapter1, BernsteinVsDeCasteljau2D) {
  const std::vector<Point2D> control_points = {
      {0, 0}, {0, 1}, {1, 1}, {1, 0}};
  const BezierCurve2D bezier(control_points);
  constexpr double div = 1.0 / 99.0;
  for (uint32_t i = 0; i < 100; ++i) {
    const double u = static_cast<double>(i) * div;
    const Point2D bern = bezier.PointOnBezierCurve(u);
    const Point2D cast = bezier.DeCasteljau(u);
    EXPECT_DOUBLE_EQ(bern.x, cast.x);
    EXPECT_DOUBLE_EQ(bern.y, cast.y);
  }
}

TEST(NURBS_Chapter1, Bezier2DPolynomialCompareEx1_6) {
  const std::vector<Point2D> control_points = {
      {0, 0}, {0, 1}, {1, 1}, {1, 0}};
  const BezierCurve2D bezier(control_points);
  constexpr double div = 1.0 / 99.0;
  for (uint32_t i = 0; i < 100; ++i) {
    const double u = static_cast<double>(i) * div;
    // Bezier
    const Point2D point_b = bezier.EvaluateCurve(u);

    // Cubic Bezier Polynomial
    const double u_i = 1.0 - u;
    Point2D point_p = std::pow(u_i, 3) * control_points[0];
    point_p += 3 * u * std::pow(u_i, 2) * control_points[1];
    point_p += 3 * std::pow(u, 2) * u_i * control_points[2];
    point_p += std::pow(u, 3) * control_points[3];

    EXPECT_DOUBLE_EQ(point_b.x, point_p.x);
    EXPECT_DOUBLE_EQ(point_b.y, point_p.y);
  }
}

TEST(NURBS_Chapter1, Bezier2DPolynomialCompareDerivEx1_6) {
  const std::vector<Point2D> control_points = {
      {0, 0}, {0, 1}, {1, 1}, {1, 0}};
  const BezierCurve2D bezier(control_points);
  constexpr double div = 1.0 / 99.0;
  for (uint32_t i = 0; i < 100; ++i) {
    // Bezier
    const double u = static_cast<double>(i) * div;
    const Point2D deriv_b = bezier.Derivative(u);

    // Cubic Bezier Polynomial
    const double u_i = 1.0 - u;
    Point2D deriv_p =
        std::pow(u, 2) * (control_points[3] - control_points[2]);
    deriv_p += (2.0 * u_i * u * (control_points[2] - control_points[1]));
    deriv_p += (std::pow(u_i, 2) * (control_points[1] - control_points[0]));
    deriv_p *= 3.0;

    EXPECT_DOUBLE_EQ(deriv_b.x, deriv_p.x);
    EXPECT_DOUBLE_EQ(deriv_b.y, deriv_p.y);
  }
}

TEST(NURBS_Chapter1, Bezier3DConstruct) {
  const std::vector<Point3D> control_points;
  const BezierCurve3D bezier(control_points);
}

TEST(NURBS_Chapter1, Bezier3DPoint) {
  std::vector<Point3D> control_points;
  control_points.push_back({0, 0, 0});
  control_points.push_back({1, 0, 1});
  control_points.push_back({1, 1, 2});
  control_points.push_back({0, 1, 3});
  const BezierCurve3D bezier(control_points);

  const Point3D point = bezier.EvaluateCurve(0.5);

  std::vector<Point3D> test_points = control_points;
  while (test_points.size() > 1) {
    std::vector<Point3D> temp_points;
    for (uint32_t i = 1; i < test_points.size(); ++i) {
      temp_points.push_back((test_points[i - 1] + test_points[i]) * 0.5);
    }
    test_points = temp_points;
  }

  EXPECT_DOUBLE_EQ(test_points[0].x, point.x);
  EXPECT_DOUBLE_EQ(test_points[0].y, point.y);
  EXPECT_DOUBLE_EQ(test_points[0].z, point.z);
}

TEST(NURBS_Chapter1, Bezier3DPointInterval) {
  std::vector<Point3D> control_points;
  control_points.push_back({0, 0, 0});
  control_points.push_back({1, 0, 1});
  control_points.push_back({1, 1, 2});
  control_points.push_back({0, 1, 3});
  const BezierCurve3D bezier(control_points, {-10.0, 12.0});

  const Point3D point = bezier.EvaluateCurve(1.0);

  std::vector<Point3D> test_points = control_points;
  while (test_points.size() > 1) {
    std::vector<Point3D> temp_points;
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
  std::vector<Point3D> control_points;
  control_points.push_back({0, 0, 0});
  control_points.push_back({1, 0, 1});
  control_points.push_back({1, 1, 2});
  control_points.push_back({0, 1, 3});
  const Point2D interval = {0.0, 1.0};
  const BezierCurve3D bezier(control_points, interval);

  const std::vector<Point3D> points = bezier.EvaluateCurvePoints(100);

  constexpr double div = 1.0 / 99.0;
  for (uint32_t i = 0; i < 100; ++i) {
    const double u = static_cast<double>(i) * div;
    const double u_i = 1.0 - u;
    std::vector<Point3D> test_points = control_points;
    while (test_points.size() > 1) {
      std::vector<Point3D> temp_points;
      for (uint32_t i = 1; i < test_points.size(); ++i) {
        temp_points.push_back((u_i * test_points[i - 1]) +
                              (u * test_points[i]));
      }
      test_points = temp_points;
    }

    EXPECT_DOUBLE_EQ(test_points[0].x, points[i].x);
    EXPECT_DOUBLE_EQ(test_points[0].y, points[i].y);
    EXPECT_DOUBLE_EQ(test_points[0].z, points[i].z);
  }
}

TEST(NURBS_Chapter1, Bezier3DPointsInterval) {
  double tolerance = std::numeric_limits<double>::epsilon() * 10;
  std::vector<Point3D> control_points;
  control_points.push_back({0, 0, 0});
  control_points.push_back({1, 0, 1});
  control_points.push_back({1, 1, 2});
  control_points.push_back({0, 1, 3});
  const Point2D interval = {5.0, 55.0};
  const BezierCurve3D bezier(control_points, interval);

  const std::vector<Point3D> points = bezier.EvaluateCurvePoints(100);

  constexpr double div = 1.0 / 99.0;
  for (uint32_t i = 0; i < 100; ++i) {
    const double u = static_cast<double>(i) * div;
    const double u_i = 1.0 - u;
    std::vector<Point3D> test_points = control_points;
    while (test_points.size() > 1) {
      std::vector<Point3D> temp_points;
      for (uint32_t i = 1; i < test_points.size(); ++i) {
        temp_points.push_back((u_i * test_points[i - 1]) +
                              (u * test_points[i]));
      }
      test_points = temp_points;
    }

    EXPECT_NEAR(test_points[0].x, points[i].x, tolerance);
    EXPECT_NEAR(test_points[0].y, points[i].y, tolerance);
    EXPECT_NEAR(test_points[0].z, points[i].z, tolerance);
  }
}

TEST(NURBS_Chapter1, Bezier3DEndDerivatives) {
  std::vector<Point3D> control_points;
  control_points.push_back({0, 0, 0});
  control_points.push_back({1, 0, 1});
  control_points.push_back({1, 1, 2});
  control_points.push_back({0, 1, 3});
  const Point2D interval = {0.0, 1.0};
  const BezierCurve3D bezier(control_points, interval);

  const Point3D start_d = bezier.Derivative(0.0);
  const Point3D end_d = bezier.Derivative(1.0);

  const double n = static_cast<double>(control_points.size() - 1);
  // The start derivative should be n(P1 - P0)
  const Point3D start_calc = n * (control_points[1] - control_points[0]);
  EXPECT_DOUBLE_EQ(start_calc.x, start_d.x);
  EXPECT_DOUBLE_EQ(start_calc.y, start_d.y);
  // The end derivative should be n(Pn - Pn-1)
  const Point3D end_calc = n * (control_points[3] - control_points[2]);
  EXPECT_DOUBLE_EQ(end_calc.x, end_d.x);
  EXPECT_DOUBLE_EQ(end_calc.y, end_d.y);
}

TEST(NURBS_Chapter1, Bezier3DEndDerivativesInterval) {
  std::vector<Point3D> control_points;
  control_points.push_back({0, 0, 0});
  control_points.push_back({1, 0, 1});
  control_points.push_back({1, 1, 2});
  control_points.push_back({0, 1, 3});
  const Point2D interval = {26.0, 27.0};
  const BezierCurve3D bezier(control_points, interval);

  const Point3D start_d = bezier.Derivative(26.0);
  const Point3D end_d = bezier.Derivative(27.0);

  const double n = static_cast<double>(control_points.size() - 1);
  // The start derivative should be n(P1 - P0)
  const Point3D start_calc = n * (control_points[1] - control_points[0]);
  EXPECT_DOUBLE_EQ(start_calc.x, start_d.x);
  EXPECT_DOUBLE_EQ(start_calc.y, start_d.y);
  // The end derivative should be n(Pn - Pn-1)
  const Point3D end_calc = n * (control_points[3] - control_points[2]);
  EXPECT_DOUBLE_EQ(end_calc.x, end_d.x);
  EXPECT_DOUBLE_EQ(end_calc.y, end_d.y);
}

TEST(NURBS_Chapter1, BernsteinVsDeCasteljau3D) {
  const std::vector<Point3D> control_points = {
      {0, 0, 0}, {0, 1, 1}, {1, 1, 2}, {1, 0, 1}};
  const BezierCurve3D bezier(control_points);
  constexpr double div = 1.0 / 99.0;
  for (uint32_t i = 0; i < 100; ++i) {
    const double u = static_cast<double>(i) * div;
    const Point3D bern = bezier.PointOnBezierCurve(u);
    const Point3D cast = bezier.DeCasteljau(u);
    EXPECT_DOUBLE_EQ(bern.x, cast.x);
    EXPECT_DOUBLE_EQ(bern.y, cast.y);
    EXPECT_DOUBLE_EQ(bern.z, cast.z);
  }
}

TEST(NURBS_Chapter1, Bezier3DPolynomialCompareEx1_6) {
  const std::vector<Point3D> control_points = {
      {0, 0, 0}, {0, 1, 1}, {1, 1, 2}, {1, 0, 1}};
  const BezierCurve3D bezier(control_points);
  constexpr double div = 1.0 / 99.0;
  for (uint32_t i = 0; i < 100; ++i) {
    const double u = static_cast<double>(i) * div;
    // Bezier
    const Point3D point_b = bezier.EvaluateCurve(u);

    // Cubic Bezier Polynomial
    const double u_i = 1.0 - u;
    Point3D point_p = std::pow(u_i, 3) * control_points[0];
    point_p += 3 * u * std::pow(u_i, 2) * control_points[1];
    point_p += 3 * std::pow(u, 2) * u_i * control_points[2];
    point_p += std::pow(u, 3) * control_points[3];

    EXPECT_DOUBLE_EQ(point_b.x, point_p.x);
    EXPECT_DOUBLE_EQ(point_b.y, point_p.y);
    EXPECT_DOUBLE_EQ(point_b.z, point_p.z);
  }
}

TEST(NURBS_Chapter1, Bezier3DPolynomialCompareDerivEx1_6) {
  const std::vector<Point3D> control_points = {
      {0, 0, 0}, {0, 1, 1}, {1, 1, 2}, {1, 0, 1}};
  const BezierCurve3D bezier(control_points);
  constexpr double div = 1.0 / 99.0;
  for (uint32_t i = 0; i < 100; ++i) {
    // Bezier
    const double u = static_cast<double>(i) * div;
    const Point3D deriv_b = bezier.Derivative(u);

    // Cubic Bezier Polynomial
    const double u_i = 1.0 - u;
    Point3D deriv_p =
        std::pow(u, 2) * (control_points[3] - control_points[2]);
    deriv_p += (2.0 * u_i * u * (control_points[2] - control_points[1]));
    deriv_p += (std::pow(u_i, 2) * (control_points[1] - control_points[0]));
    deriv_p *= 3.0;
    // The percision is really just not here with this calculation...
    // I think the generic bezier calculation is more incorrect becuase I can't
    // factor things out, but either way this is a really imprecise calculation
    EXPECT_NEAR(deriv_b.x, deriv_p.x,
                std::numeric_limits<float>::epsilon() * 10);
    EXPECT_NEAR(deriv_b.y, deriv_p.y,
                std::numeric_limits<float>::epsilon() * 10);
    EXPECT_NEAR(deriv_b.z, deriv_p.z,
                std::numeric_limits<float>::epsilon() * 10);
  }
}

TEST(NURBS_Chapter1, BezierSurfaceConstruct) {
  const std::vector<BezierCurve3D> curves;
  const BezierSurface surface(curves);
}

TEST(NURBS_Chapter1, BezierSurfacePoint) {
  std::vector<BezierCurve3D> curves;
  {
    std::vector<Point3D> control_points;
    control_points.push_back({0, -1, 0});
    control_points.push_back({0, 2, 1});
    control_points.push_back({0, 2, 2});
    control_points.push_back({0, 1, 3});
    curves.push_back(control_points);
  }
  {
    std::vector<Point3D> control_points;
    control_points.push_back({1, 0, 0});
    control_points.push_back({1, 4, 1});
    control_points.push_back({1, 3, 2});
    control_points.push_back({1, 2, 3});
    curves.push_back(control_points);
  }
  {
    std::vector<Point3D> control_points;
    control_points.push_back({2, 2, 0});
    control_points.push_back({2, 1, 1});
    control_points.push_back({2, 0, 2});
    control_points.push_back({2, -1, 3});
    curves.push_back(control_points);
  }
  {
    std::vector<Point3D> control_points;
    control_points.push_back({3, 3, 0});
    control_points.push_back({3, -2, 1});
    control_points.push_back({3, -4, 2});
    control_points.push_back({3, 0, 3});
    curves.push_back(control_points);
  }
  const BezierSurface surface(curves);

  const Point3D point = surface.EvaluatePoint({0.5, 0.5});

  const std::vector<Point3D> temp_cps = {
      curves[0].EvaluateCurve(0.5), curves[1].EvaluateCurve(0.5),
      curves[2].EvaluateCurve(0.5), curves[3].EvaluateCurve(0.5)};
  const BezierCurve3D curve(temp_cps);
  const Point3D test_point = curve.EvaluateCurve(0.5);

  EXPECT_DOUBLE_EQ(test_point.x, point.x);
  EXPECT_DOUBLE_EQ(test_point.y, point.y);
  EXPECT_DOUBLE_EQ(test_point.z, point.z);
}

TEST(NURBS_Chapter1, BezierSurfacePointInterval) {
  Point2D u_interval = {0.0, 10.0};
  Point2D v_interval = {-12.0, -10.0};
  double u_mid = (u_interval.x + u_interval.y) * 0.5;
  double v_mid = (v_interval.x + v_interval.y) * 0.5;
  std::vector<BezierCurve3D> curves;
  {
    std::vector<Point3D> control_points;
    control_points.push_back({0, -1, 0});
    control_points.push_back({0, 2, 1});
    control_points.push_back({0, 2, 2});
    control_points.push_back({0, 1, 3});
    curves.emplace_back(control_points, u_interval);
  }
  {
    std::vector<Point3D> control_points;
    control_points.push_back({1, 0, 0});
    control_points.push_back({1, 4, 1});
    control_points.push_back({1, 3, 2});
    control_points.push_back({1, 2, 3});
    curves.emplace_back(control_points, u_interval);
  }
  {
    std::vector<Point3D> control_points;
    control_points.push_back({2, 2, 0});
    control_points.push_back({2, 1, 1});
    control_points.push_back({2, 0, 2});
    control_points.push_back({2, -1, 3});
    curves.emplace_back(control_points, u_interval);
  }
  {
    std::vector<Point3D> control_points;
    control_points.push_back({3, 3, 0});
    control_points.push_back({3, -2, 1});
    control_points.push_back({3, -4, 2});
    control_points.push_back({3, 0, 3});
    curves.emplace_back(control_points, u_interval);
  }
  const BezierSurface surface(curves, u_interval, v_interval);

  const Point3D point = surface.EvaluatePoint({u_mid, v_mid});

  const std::vector<Point3D> temp_cps = {
      curves[0].EvaluateCurve(u_mid), curves[1].EvaluateCurve(u_mid),
      curves[2].EvaluateCurve(u_mid), curves[3].EvaluateCurve(u_mid)};
  const BezierCurve3D curve(temp_cps, v_interval);
  const Point3D test_point = curve.EvaluateCurve(v_mid);

  EXPECT_DOUBLE_EQ(test_point.x, point.x);
  EXPECT_DOUBLE_EQ(test_point.y, point.y);
  EXPECT_DOUBLE_EQ(test_point.z, point.z);
}

TEST(NURBS_Chapter1, BezierSurfacePoints) {
  constexpr uint32_t point_count = 100;
  constexpr double div = 1.0 / static_cast<double>(point_count - 1);
  std::vector<BezierCurve3D> curves;
  {
    std::vector<Point3D> control_points;
    control_points.push_back({0, -1, 0});
    control_points.push_back({0, 2, 1});
    control_points.push_back({0, 2, 2});
    control_points.push_back({0, 1, 3});
    curves.push_back(control_points);
  }
  {
    std::vector<Point3D> control_points;
    control_points.push_back({1, 0, 0});
    control_points.push_back({1, 4, 1});
    control_points.push_back({1, 3, 2});
    control_points.push_back({1, 2, 3});
    curves.push_back(control_points);
  }
  {
    std::vector<Point3D> control_points;
    control_points.push_back({2, 2, 0});
    control_points.push_back({2, 1, 1});
    control_points.push_back({2, 0, 2});
    control_points.push_back({2, -1, 3});
    curves.push_back(control_points);
  }
  {
    std::vector<Point3D> control_points;
    control_points.push_back({3, 3, 0});
    control_points.push_back({3, -2, 1});
    control_points.push_back({3, -4, 2});
    control_points.push_back({3, 0, 3});
    curves.push_back(control_points);
  }
  const BezierSurface surface(curves);

  const std::vector<Point3D> points =
      surface.EvaluatePoints(point_count, point_count);

  for (uint32_t i = 0; i < point_count; ++i) {
    const double u = static_cast<double>(i) * div;
    const std::vector<Point3D> temp_cps = {
        curves[0].EvaluateCurve(u), curves[1].EvaluateCurve(u),
        curves[2].EvaluateCurve(u), curves[3].EvaluateCurve(u)};
    const BezierCurve3D curve(temp_cps);
    for (uint32_t j = 0; j < point_count; ++j) {
      const double v = static_cast<double>(j) * div;
      const Point3D test_point = curve.EvaluateCurve(v);
      const Point3D &point = points[(i * point_count) + j];

      EXPECT_DOUBLE_EQ(test_point.x, point.x);
      EXPECT_DOUBLE_EQ(test_point.y, point.y);
      EXPECT_DOUBLE_EQ(test_point.z, point.z);
    }
  }
}

TEST(NURBS_Chapter1, BezierSurfacePointsInterval) {
  constexpr double tolerance = std::numeric_limits<double>::epsilon() * 100.0;
  const Point2D u_interval = {-10.0, 10.0};
  const Point2D v_interval = {22.0, 30.0};
  const double u_mid = (u_interval.x + u_interval.y) * 0.5;
  const double v_mid = (v_interval.x + v_interval.y) * 0.5;
  constexpr uint32_t point_count = 100;
  constexpr double div = 1.0 / static_cast<double>(point_count - 1);
  const double u_div = (div * (u_interval.y - u_interval.x));
  const double v_div = (div * (v_interval.y - v_interval.x));
  std::vector<BezierCurve3D> curves;
  {
    std::vector<Point3D> control_points;
    control_points.push_back({0, -1, 0});
    control_points.push_back({0, 2, 1});
    control_points.push_back({0, 2, 2});
    control_points.push_back({0, 1, 3});
    curves.emplace_back(control_points, u_interval);
  }
  {
    std::vector<Point3D> control_points;
    control_points.push_back({1, 0, 0});
    control_points.push_back({1, 4, 1});
    control_points.push_back({1, 3, 2});
    control_points.push_back({1, 2, 3});
    curves.emplace_back(control_points, u_interval);
  }
  {
    std::vector<Point3D> control_points;
    control_points.push_back({2, 2, 0});
    control_points.push_back({2, 1, 1});
    control_points.push_back({2, 0, 2});
    control_points.push_back({2, -1, 3});
    curves.emplace_back(control_points, u_interval);
  }
  {
    std::vector<Point3D> control_points;
    control_points.push_back({3, 3, 0});
    control_points.push_back({3, -2, 1});
    control_points.push_back({3, -4, 2});
    control_points.push_back({3, 0, 3});
    curves.emplace_back(control_points, u_interval);
  }
  const BezierSurface surface(curves, u_interval, v_interval);

  const std::vector<Point3D> points =
      surface.EvaluatePoints(point_count, point_count);

  for (uint32_t i = 0; i < point_count; ++i) {
    const double u = (static_cast<double>(i) * u_div) + u_interval.x;
    const std::vector<Point3D> temp_cps = {
        curves[0].EvaluateCurve(u), curves[1].EvaluateCurve(u),
        curves[2].EvaluateCurve(u), curves[3].EvaluateCurve(u)};
    const BezierCurve3D curve(temp_cps, v_interval);
    for (uint32_t j = 0; j < point_count; ++j) {
      const double v = (static_cast<double>(j) * v_div) + v_interval.x;
      const Point3D test_point = curve.EvaluateCurve(v);
      const Point3D &point = points[(i * point_count) + j];

      EXPECT_NEAR(test_point.x, point.x, tolerance);
      EXPECT_NEAR(test_point.y, point.y, tolerance);
      EXPECT_NEAR(test_point.z, point.z, tolerance);
    }
  }
}
} // namespace nurbs
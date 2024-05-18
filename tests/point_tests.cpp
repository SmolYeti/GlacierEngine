#include <gtest/gtest.h>

#include "include/point_types.hpp"

namespace nurbs {
// Point 2D
TEST(Point2DTest, ConstructEmpty) {
  const Point2D point;
  EXPECT_DOUBLE_EQ(point.x, 0.0);
  EXPECT_DOUBLE_EQ(point.y, 0.0);
}

TEST(Point2DTest, ConstructVals) {
  const Point2D point = {0.3, 1.1};
  EXPECT_DOUBLE_EQ(point.x, 0.3);
  EXPECT_DOUBLE_EQ(point.y, 1.1);
}

// Addition
TEST(Point2DTest, AddPointDouble) {
  const Point2D point = {0.3, 1.1};
  constexpr double val = 2.0;
  Point2D test_point = point + val;
  EXPECT_DOUBLE_EQ(test_point.x, 2.3);
  EXPECT_DOUBLE_EQ(test_point.y, 3.1);
}

TEST(Point2DTest, AddDoublePoint) {
  const Point2D point = {0.3, 1.1};
  constexpr double val = 2.0;
  const Point2D test_point = val + point;
  EXPECT_DOUBLE_EQ(test_point.x, 2.3);
  EXPECT_DOUBLE_EQ(test_point.y, 3.1);
}

TEST(Point2DTest, AddPointPoint) {
  const Point2D point_0 = {0.3, 1.1};
  const Point2D point_1 = {0.4, 5.2};
  const Point2D test_point = point_0 + point_1;
  EXPECT_DOUBLE_EQ(test_point.x, 0.7);
  EXPECT_DOUBLE_EQ(test_point.y, 6.3);
}

TEST(Point2DTest, AddEqualsDouble) {
  Point2D point = {0.3, 1.1};
  constexpr double val = 2.0;
  point += val;
  EXPECT_DOUBLE_EQ(point.x, 2.3);
  EXPECT_DOUBLE_EQ(point.y, 3.1);
}

TEST(Point2DTest, AddEqualsPoint) {
  Point2D point = {0.3, 1.1};
  point += point;
  EXPECT_DOUBLE_EQ(point.x, 0.6);
  EXPECT_DOUBLE_EQ(point.y, 2.2);
}

// Subtraction
TEST(Point2DTest, SubPointDouble) {
  const Point2D point = {0.3, 1.1};
  constexpr double val = 2.0;
  Point2D test_point = point - val;
  EXPECT_DOUBLE_EQ(test_point.x, -1.7);
  EXPECT_DOUBLE_EQ(test_point.y, -0.9);
}

TEST(Point2DTest, SubDoublePoint) {
  const Point2D point = {0.3, 1.1};
  constexpr double val = 2.0;
  const Point2D test_point = val - point;
  EXPECT_DOUBLE_EQ(test_point.x, 1.7);
  EXPECT_DOUBLE_EQ(test_point.y, 0.9);
}

TEST(Point2DTest, SubPointPoint) {
  const Point2D point_0 = {0.3, 1.1};
  const Point2D point_1 = {0.4, 5.2};
  const Point2D test_point = point_0 - point_1;
  EXPECT_DOUBLE_EQ(test_point.x, -0.1);
  EXPECT_DOUBLE_EQ(test_point.y, -4.1);
}

TEST(Point2DTest, SubEqualsDouble) {
  Point2D point = {0.3, 1.1};
  constexpr double val = 2.0;
  point -= val;
  EXPECT_DOUBLE_EQ(point.x, -1.7);
  EXPECT_DOUBLE_EQ(point.y, -0.9);
}

TEST(Point2DTest, SubEqualsPoint) {
  Point2D point_0 = {0.3, 1.1};
  const Point2D point_1 = {0.4, 5.2};
  point_0 -= point_1;
  EXPECT_DOUBLE_EQ(point_0.x, -0.1);
  EXPECT_DOUBLE_EQ(point_0.y, -4.1);
}

// Multiplication
TEST(Point2DTest, MulPointDouble) {
  const Point2D point = {0.3, 1.1};
  constexpr double val = 2.0;
  Point2D test_point = point * val;
  EXPECT_DOUBLE_EQ(test_point.x, 0.6);
  EXPECT_DOUBLE_EQ(test_point.y, 2.2);
}

TEST(Point2DTest, MulDoublePoint) {
  const Point2D point = {0.3, 1.1};
  constexpr double val = 2.0;
  const Point2D test_point = val * point;
  EXPECT_DOUBLE_EQ(test_point.x, 0.6);
  EXPECT_DOUBLE_EQ(test_point.y, 2.2);
}

TEST(Point2DTest, MulEqualsDouble) {
  Point2D point = {0.3, 1.1};
  constexpr double val = 2.0;
  point *= val;
  EXPECT_DOUBLE_EQ(point.x, 0.6);
  EXPECT_DOUBLE_EQ(point.y, 2.2);
}

// Division
TEST(Point2DTest, DivPointDouble) {
  const Point2D point = {0.3, 1.1};
  constexpr double val = 2.0;
  Point2D test_point = point / val;
  EXPECT_DOUBLE_EQ(test_point.x, 0.15);
  EXPECT_DOUBLE_EQ(test_point.y, 0.55);
}

TEST(Point2DTest, DivDoublePoint) {
  const Point2D point = {0.5, 2.0};
  constexpr double val = 2.0;
  const Point2D test_point = val / point;
  EXPECT_DOUBLE_EQ(test_point.x, 4.0);
  EXPECT_DOUBLE_EQ(test_point.y, 1.0);
}

TEST(Point2DTest, DivEqualsDouble) {
  Point2D point = {0.3, 1.1};
  constexpr double val = 2.0;
  point /= val;
  EXPECT_DOUBLE_EQ(point.x, 0.15);
  EXPECT_DOUBLE_EQ(point.y, 0.55);
}

// Point 3D
TEST(Point3DTest, ConstructEmpty) {
  const Point3D point;
  EXPECT_DOUBLE_EQ(point.x, 0.0);
  EXPECT_DOUBLE_EQ(point.y, 0.0);
  EXPECT_DOUBLE_EQ(point.z, 0.0);
}

TEST(Point3DTest, ConstructVals) {
  const Point3D point = {0.3, 1.1, 4.4};
  EXPECT_DOUBLE_EQ(point.x, 0.3);
  EXPECT_DOUBLE_EQ(point.y, 1.1);
  EXPECT_DOUBLE_EQ(point.z, 4.4);
}

// Addition
TEST(Point3DTest, AddPointDouble) {
  const Point3D point = {0.3, 1.1, 4.4};
  constexpr double val = 2.0;
  Point3D test_point = point + val;
  EXPECT_DOUBLE_EQ(test_point.x, 2.3);
  EXPECT_DOUBLE_EQ(test_point.y, 3.1);
  EXPECT_DOUBLE_EQ(test_point.z, 6.4);
}

TEST(Point3DTest, AddDoublePoint) {
  const Point3D point = {0.3, 1.1, 4.4};
  constexpr double val = 2.0;
  const Point3D test_point = val + point;
  EXPECT_DOUBLE_EQ(test_point.x, 2.3);
  EXPECT_DOUBLE_EQ(test_point.y, 3.1);
  EXPECT_DOUBLE_EQ(test_point.z, 6.4);
}

TEST(Point3DTest, AddPointPoint) {
  const Point3D point_0 = {0.3, 1.1, 2.7};
  const Point3D point_1 = {0.4, 5.2, 9.0};
  const Point3D test_point = point_0 + point_1;
  EXPECT_DOUBLE_EQ(test_point.x, 0.7);
  EXPECT_DOUBLE_EQ(test_point.y, 6.3);
  EXPECT_DOUBLE_EQ(test_point.z, 11.7);
}

TEST(Point3DTest, AddEqualsDouble) {
  Point3D point = {0.3, 1.1, 2.2};
  constexpr double val = 2.0;
  point += val;
  EXPECT_DOUBLE_EQ(point.x, 2.3);
  EXPECT_DOUBLE_EQ(point.y, 3.1);
  EXPECT_DOUBLE_EQ(point.z, 4.2);
}

TEST(Point3DTest, AddEqualsPoint) {
  Point3D point = {0.3, 1.1, 2.2};
  point += point;
  EXPECT_DOUBLE_EQ(point.x, 0.6);
  EXPECT_DOUBLE_EQ(point.y, 2.2);
  EXPECT_DOUBLE_EQ(point.z, 4.4);
}

// Subtraction
TEST(Point3DTest, SubPointDouble) {
  const Point3D point = {0.3, 1.1, 2.21};
  constexpr double val = 2.0;
  Point3D test_point = point - val;
  EXPECT_DOUBLE_EQ(test_point.x, -1.7);
  EXPECT_DOUBLE_EQ(test_point.y, -0.9);
  EXPECT_DOUBLE_EQ(test_point.z, 0.21);
}

TEST(Point3DTest, SubDoublePoint) {
  const Point3D point = {0.3, 1.1, 2.21};
  constexpr double val = 2.0;
  const Point3D test_point = val - point;
  EXPECT_DOUBLE_EQ(test_point.x, 1.7);
  EXPECT_DOUBLE_EQ(test_point.y, 0.9);
  EXPECT_DOUBLE_EQ(test_point.z, -0.21);
}

TEST(Point3DTest, SubPointPoint) {
  const Point3D point_0 = {0.3, 1.1, 2.2};
  const Point3D point_1 = {0.4, 5.2, 3.7};
  const Point3D test_point = point_0 - point_1;
  EXPECT_DOUBLE_EQ(test_point.x, -0.1);
  EXPECT_DOUBLE_EQ(test_point.y, -4.1);
  EXPECT_DOUBLE_EQ(test_point.z, -1.5);
}

TEST(Point3DTest, SubEqualsDouble) {
  Point3D point = {0.3, 1.1, 2.21};
  constexpr double val = 2.0;
  point -= val;
  EXPECT_DOUBLE_EQ(point.x, -1.7);
  EXPECT_DOUBLE_EQ(point.y, -0.9);
  EXPECT_DOUBLE_EQ(point.z, 0.21);
}

TEST(Point3DTest, SubEqualsPoint) {
  Point3D point_0 = {0.3, 1.1, 2.2};
  const Point3D point_1 = {0.4, 5.2, 3.7};
  point_0 -= point_1;
  EXPECT_DOUBLE_EQ(point_0.x, -0.1);
  EXPECT_DOUBLE_EQ(point_0.y, -4.1);
  EXPECT_DOUBLE_EQ(point_0.z, -1.5);
}

// Multiplication
TEST(Point3DTest, MulPointDouble) {
  const Point3D point = {0.3, 1.1, 2.2};
  constexpr double val = 2.0;
  Point3D test_point = point * val;
  EXPECT_DOUBLE_EQ(test_point.x, 0.6);
  EXPECT_DOUBLE_EQ(test_point.y, 2.2);
  EXPECT_DOUBLE_EQ(test_point.z, 4.4);
}

TEST(Point3DTest, MulDoublePoint) {
  const Point3D point = {0.3, 1.1, 2.4};
  constexpr double val = 2.0;
  const Point3D test_point = val * point;
  EXPECT_DOUBLE_EQ(test_point.x, 0.6);
  EXPECT_DOUBLE_EQ(test_point.y, 2.2);
  EXPECT_DOUBLE_EQ(test_point.z, 4.8);
}

TEST(Point3DTest, MulEqualsDouble) {
  Point3D point = {0.3, 1.1, 2.4};
  constexpr double val = 2.0;
  point *= val;
  EXPECT_DOUBLE_EQ(point.x, 0.6);
  EXPECT_DOUBLE_EQ(point.y, 2.2);
  EXPECT_DOUBLE_EQ(point.z, 4.8);
}

// Division
TEST(Point3DTest, DivPointDouble) {
  const Point3D point = {0.3, 1.1, 2.4};
  constexpr double val = 2.0;
  Point3D test_point = point / val;
  EXPECT_DOUBLE_EQ(test_point.x, 0.15);
  EXPECT_DOUBLE_EQ(test_point.y, 0.55);
  EXPECT_DOUBLE_EQ(test_point.z, 1.2);
}

TEST(Point3DTest, DivDoublePoint) {
  const Point3D point = {0.5, 2.0, 4.0};
  constexpr double val = 2.0;
  const Point3D test_point = val / point;
  EXPECT_DOUBLE_EQ(test_point.x, 4.0);
  EXPECT_DOUBLE_EQ(test_point.y, 1.0);
  EXPECT_DOUBLE_EQ(test_point.z, 0.5);
}

TEST(Point3DTest, DivEqualsDouble) {
  Point3D point = {0.3, 1.1, 3.5};
  constexpr double val = 2.0;
  point /= val;
  EXPECT_DOUBLE_EQ(point.x, 0.15);
  EXPECT_DOUBLE_EQ(point.y, 0.55);
  EXPECT_DOUBLE_EQ(point.z, 1.75);
}

// Point 4D
TEST(Point4DTest, ConstructEmpty) {
  const Point4D point;
  EXPECT_DOUBLE_EQ(point.x, 0.0);
  EXPECT_DOUBLE_EQ(point.y, 0.0);
  EXPECT_DOUBLE_EQ(point.z, 0.0);
  EXPECT_DOUBLE_EQ(point.w, 0.0);
}

TEST(Point4DTest, ConstructVals) {
  const Point4D point = {0.3, 1.1, 4.4, 5.6};
  EXPECT_DOUBLE_EQ(point.x, 0.3);
  EXPECT_DOUBLE_EQ(point.y, 1.1);
  EXPECT_DOUBLE_EQ(point.z, 4.4);
  EXPECT_DOUBLE_EQ(point.w, 5.6);
}

// Addition
TEST(Point4DTest, AddPointDouble) {
  const Point4D point = {0.3, 1.1, 4.4, 5.6};
  constexpr double val = 2.0;
  Point4D test_point = point + val;
  EXPECT_DOUBLE_EQ(test_point.x, 2.3);
  EXPECT_DOUBLE_EQ(test_point.y, 3.1);
  EXPECT_DOUBLE_EQ(test_point.z, 6.4);
  EXPECT_DOUBLE_EQ(test_point.w, 7.6);
}

TEST(Point4DTest, AddDoublePoint) {
  const Point4D point = {0.3, 1.1, 4.4, 5.6};
  constexpr double val = 2.0;
  const Point4D test_point = val + point;
  EXPECT_DOUBLE_EQ(test_point.x, 2.3);
  EXPECT_DOUBLE_EQ(test_point.y, 3.1);
  EXPECT_DOUBLE_EQ(test_point.z, 6.4);
  EXPECT_DOUBLE_EQ(test_point.w, 7.6);
}

TEST(Point4DTest, AddPointPoint) {
  const Point4D point_0 = {0.3, 1.1, 2.7, 5.6};
  const Point4D point_1 = {0.4, 5.2, 9.0, 3.9};
  const Point4D test_point = point_0 + point_1;
  EXPECT_DOUBLE_EQ(test_point.x, 0.7);
  EXPECT_DOUBLE_EQ(test_point.y, 6.3);
  EXPECT_DOUBLE_EQ(test_point.z, 11.7);
  EXPECT_DOUBLE_EQ(test_point.w, 9.5);
}

TEST(Point4DTest, AddEqualsDouble) {
  Point4D point = {0.3, 1.1, 2.2, 5.6};
  constexpr double val = 2.0;
  point += val;
  EXPECT_DOUBLE_EQ(point.x, 2.3);
  EXPECT_DOUBLE_EQ(point.y, 3.1);
  EXPECT_DOUBLE_EQ(point.z, 4.2);
  EXPECT_DOUBLE_EQ(point.w, 7.6);
}

TEST(Point4DTest, AddEqualsPoint) {
  Point4D point = {0.3, 1.1, 2.2, 5.6};
  point += point;
  EXPECT_DOUBLE_EQ(point.x, 0.6);
  EXPECT_DOUBLE_EQ(point.y, 2.2);
  EXPECT_DOUBLE_EQ(point.z, 4.4);
  EXPECT_DOUBLE_EQ(point.w, 11.2);
}

// Subtraction
TEST(Point4DTest, SubPointDouble) {
  const Point4D point = {0.3, 1.1, 2.21, 5.6};
  constexpr double val = 2.0;
  Point4D test_point = point - val;
  EXPECT_DOUBLE_EQ(test_point.x, -1.7);
  EXPECT_DOUBLE_EQ(test_point.y, -0.9);
  EXPECT_DOUBLE_EQ(test_point.z, 0.21);
  EXPECT_DOUBLE_EQ(test_point.w, 3.6);
}

TEST(Point4DTest, SubDoublePoint) {
  const Point4D point = {0.3, 1.1, 2.21, 5.6};
  constexpr double val = 2.0;
  const Point4D test_point = val - point;
  EXPECT_DOUBLE_EQ(test_point.x, 1.7);
  EXPECT_DOUBLE_EQ(test_point.y, 0.9);
  EXPECT_DOUBLE_EQ(test_point.z, -0.21);
  EXPECT_DOUBLE_EQ(test_point.w, -3.6);
}

TEST(Point4DTest, SubPointPoint) {
  const Point4D point_0 = {0.3, 1.1, 2.2, 5.61};
  const Point4D point_1 = {0.4, 5.2, 3.7, 4.9};
  const Point4D test_point = point_0 - point_1;
  EXPECT_DOUBLE_EQ(test_point.x, -0.1);
  EXPECT_DOUBLE_EQ(test_point.y, -4.1);
  EXPECT_DOUBLE_EQ(test_point.z, -1.5);
  EXPECT_DOUBLE_EQ(test_point.w, 0.71);
}

TEST(Point4DTest, SubEqualsDouble) {
  Point4D point = {0.3, 1.1, 2.21, 5.6};
  constexpr double val = 2.0;
  point -= val;
  EXPECT_DOUBLE_EQ(point.x, -1.7);
  EXPECT_DOUBLE_EQ(point.y, -0.9);
  EXPECT_DOUBLE_EQ(point.z, 0.21);
  EXPECT_DOUBLE_EQ(point.w, 3.6);
}

TEST(Point4DTest, SubEqualsPoint) {
  Point4D point_0 = {0.3, 1.1, 2.2, 5.61};
  const Point4D point_1 = {0.4, 5.2, 3.7, 4.9};
  point_0 -= point_1;
  EXPECT_DOUBLE_EQ(point_0.x, -0.1);
  EXPECT_DOUBLE_EQ(point_0.y, -4.1);
  EXPECT_DOUBLE_EQ(point_0.z, -1.5);
  EXPECT_DOUBLE_EQ(point_0.w, 0.71);
}

// Multiplication
TEST(Point4DTest, MulPointDouble) {
  const Point4D point = {0.3, 1.1, 2.2, 5.6};
  constexpr double val = 2.0;
  Point4D test_point = point * val;
  EXPECT_DOUBLE_EQ(test_point.x, 0.6);
  EXPECT_DOUBLE_EQ(test_point.y, 2.2);
  EXPECT_DOUBLE_EQ(test_point.z, 4.4);
  EXPECT_DOUBLE_EQ(test_point.w, 11.2);
}

TEST(Point4DTest, MulDoublePoint) {
  const Point4D point = {0.3, 1.1, 2.4, 5.6};
  constexpr double val = 2.0;
  const Point4D test_point = val * point;
  EXPECT_DOUBLE_EQ(test_point.x, 0.6);
  EXPECT_DOUBLE_EQ(test_point.y, 2.2);
  EXPECT_DOUBLE_EQ(test_point.z, 4.8);
  EXPECT_DOUBLE_EQ(test_point.w, 11.2);
}

TEST(Point4DTest, MulEqualsDouble) {
  Point4D point = {0.3, 1.1, 2.4, 5.6};
  constexpr double val = 2.0;
  point *= val;
  EXPECT_DOUBLE_EQ(point.x, 0.6);
  EXPECT_DOUBLE_EQ(point.y, 2.2);
  EXPECT_DOUBLE_EQ(point.z, 4.8);
  EXPECT_DOUBLE_EQ(point.w, 11.2);
}

// Division
TEST(Point4DTest, DivPointDouble) {
  const Point4D point = {0.3, 1.1, 2.4, 5.6};
  constexpr double val = 2.0;
  Point4D test_point = point / val;
  EXPECT_DOUBLE_EQ(test_point.x, 0.15);
  EXPECT_DOUBLE_EQ(test_point.y, 0.55);
  EXPECT_DOUBLE_EQ(test_point.z, 1.2);
  EXPECT_DOUBLE_EQ(test_point.w, 2.8);
}

TEST(Point4DTest, DivDoublePoint) {
  const Point4D point = {0.5, 2.0, 4.0, 8.0};
  constexpr double val = 2.0;
  const Point4D test_point = val / point;
  EXPECT_DOUBLE_EQ(test_point.x, 4.0);
  EXPECT_DOUBLE_EQ(test_point.y, 1.0);
  EXPECT_DOUBLE_EQ(test_point.z, 0.5);
  EXPECT_DOUBLE_EQ(test_point.w, 0.25);
}

TEST(Point4DTest, DivEqualsDouble) {
  Point4D point = {0.3, 1.1, 3.5, 5.6};
  constexpr double val = 2.0;
  point /= val;
  EXPECT_DOUBLE_EQ(point.x, 0.15);
  EXPECT_DOUBLE_EQ(point.y, 0.55);
  EXPECT_DOUBLE_EQ(point.z, 1.75);
  EXPECT_DOUBLE_EQ(point.w, 2.8);
}
} // namespace nurbs
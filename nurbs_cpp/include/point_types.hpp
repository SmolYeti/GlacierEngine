#pragma once

namespace nurbs {
struct Point2D {
  Point2D() : x(0.0), y(0.0) {}
  Point2D(double _x, double _y) : x(_x), y(_y) {}
  double x, y;

  // Operators
  Point2D operator+(double rhs) const;
  Point2D operator+(const Point2D &rhs) const;
  void operator+=(double rhs);
  void operator+=(const Point2D &rhs);

  Point2D operator-(double rhs) const;
  Point2D operator-(const Point2D &rhs) const;
  void operator-=(double rhs);
  void operator-=(const Point2D &rhs);

  Point2D operator*(double rhs) const;
  void operator*=(double rhs);

  Point2D operator/(double rhs) const;
  void operator/=(double rhs);
};

struct Point3D {
  Point3D() : x(0.0), y(0.0), z(0.0) {}
  Point3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
  double x, y, z;

  // Operators
  Point3D operator+(double rhs) const;
  Point3D operator+(const Point3D &rhs) const;
  void operator+=(double rhs);
  void operator+=(const Point3D &rhs);

  Point3D operator-(double rhs) const;
  Point3D operator-(const Point3D &rhs) const;
  void operator-=(double rhs);
  void operator-=(const Point3D &rhs);

  Point3D operator*(double rhs) const;
  void operator*=(double rhs);

  Point3D operator/(double rhs) const;
  void operator/=(double rhs);
};

struct Point4D {
  Point4D() : x(0.0), y(0.0), z(0.0), w(0.0) {}
  Point4D(double _x, double _y, double _z, double _w)
      : x(_x), y(_y), z(_z), w(_w) {}
  double x, y, z, w;

  // Operators
  Point4D operator+(double rhs) const;
  Point4D operator+(const Point4D &rhs) const;
  void operator+=(double rhs);
  void operator+=(const Point4D &rhs);

  Point4D operator-(double rhs) const;
  Point4D operator-(const Point4D &rhs) const;
  void operator-=(double rhs);
  void operator-=(const Point4D &rhs);

  Point4D operator*(double rhs) const;
  void operator*=(double rhs);

  Point4D operator/(double rhs) const;
  void operator/=(double rhs);
};

// Point 2D Operators
static Point2D operator+(double lhs, const Point2D &rhs) {
  return {rhs.x + lhs, rhs.y + lhs};
}

static Point2D operator-(double lhs, const Point2D &rhs) {
  return {lhs - rhs.x, lhs - rhs.y};
}

static Point2D operator*(double lhs, const Point2D &rhs) {
  return {rhs.x * lhs, rhs.y * lhs};
}

static Point2D operator/(double lhs, const Point2D &rhs) {
  return {lhs / rhs.x, lhs / rhs.y};
}

// Point 3D Operators
static Point3D operator+(double lhs, const Point3D &rhs) {
  return {rhs.x + lhs, rhs.y + lhs, rhs.z + lhs};
}

static Point3D operator-(double lhs, const Point3D &rhs) {
  return {lhs - rhs.x, lhs - rhs.y, lhs - rhs.z};
}

static Point3D operator*(double lhs, const Point3D &rhs) {
  return {lhs * rhs.x, lhs * rhs.y, lhs * rhs.z};
}

static Point3D operator/(double lhs, const Point3D &rhs) {
  return {lhs / rhs.x, lhs / rhs.y, lhs / rhs.z};
}

// Point 4D Operators

static Point4D operator+(double lhs, const Point4D &rhs) {
  return {rhs.x + lhs, rhs.y + lhs, rhs.z + lhs, rhs.w + lhs};
}

static Point4D operator-(double lhs, const Point4D &rhs) {
  return {lhs - rhs.x, lhs - rhs.y, lhs - rhs.z, lhs - rhs.z};
}

static Point4D operator*(double lhs, const Point4D &rhs) {
  return {rhs.x * lhs, rhs.y * lhs, rhs.z * lhs, rhs.w * lhs};
}

static Point4D operator/(double lhs, const Point4D &rhs) {
  return {lhs / rhs.x, lhs / rhs.y, lhs / rhs.z, lhs / rhs.w};
}
} // namespace nurbs
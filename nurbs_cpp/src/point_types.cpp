#include "include/point_types.hpp"

namespace nurbs {
// Point 2D Operators

Point2D Point2D::operator+(double rhs) const { return {x + rhs, y + rhs}; }

Point2D Point2D::operator+(const Point2D &rhs) const {
  return {x + rhs.x, y + rhs.x};
}

void Point2D::operator+=(double rhs) {
  x += rhs;
  y += rhs;
}

void Point2D::operator+=(const Point2D &rhs) {
  x += rhs.x;
  y += rhs.y;
}

Point2D Point2D::operator-(double rhs) const { return {x - rhs, y - rhs}; }

Point2D Point2D::operator-(const Point2D &rhs) const {
  return {x - rhs.x, y - rhs.y};
}

void Point2D::operator-=(double rhs) {
  x -= rhs;
  y -= rhs;
}

void Point2D::operator-=(const Point2D &rhs) {
  x -= rhs.x;
  y -= rhs.y;
}

Point2D Point2D::operator*(double rhs) const { return {x * rhs, y * rhs}; }

void Point2D::operator*=(double rhs) {
  x *= rhs;
  y *= rhs;
}

Point2D Point2D::operator/(double rhs) const { return {x / rhs, y / rhs}; }

void Point2D::operator/=(double rhs) {
  x /= rhs;
  y /= rhs;
}

// Point 3D Operators

Point3D Point3D::operator+(double rhs) const {
  return {x + rhs, y + rhs, z + rhs};
}

Point3D Point3D::operator+(const Point3D &rhs) const {
  return {x + rhs.x, y + rhs.x, z + rhs.z};
}

void Point3D::operator+=(double rhs) {
  x += rhs;
  y += rhs;
  z += rhs;
}

void Point3D::operator+=(const Point3D &rhs) {
  x += rhs.x;
  y += rhs.y;
  z += rhs.z;
}

Point3D Point3D::operator-(double rhs) const {
  return {x - rhs, y - rhs, z - rhs};
}

Point3D Point3D::operator-(const Point3D &rhs) const {
  return {x - rhs.x, y - rhs.y, z - rhs.z};
}

void Point3D::operator-=(double rhs) {
  x -= rhs;
  y -= rhs;
  z -= rhs;
}

void Point3D::operator-=(const Point3D &rhs) {
  x -= rhs.x;
  y -= rhs.y;
  z -= rhs.z;
}

Point3D Point3D::operator*(double rhs) const {
  return {x * rhs, y * rhs, z * rhs};
}

void Point3D::operator*=(double rhs) {
  x *= rhs;
  y *= rhs;
  z *= rhs;
}

Point3D Point3D::operator/(double rhs) const {
  return {x / rhs, y / rhs, z / rhs};
}

void Point3D::operator/=(double rhs) {
  x /= rhs;
  y /= rhs;
  z /= rhs;
}

// Point 4D Operators

Point4D Point4D::operator+(double rhs) const {
  return {x + rhs, y + rhs, z + rhs, w + rhs};
}

Point4D Point4D::operator+(const Point4D &rhs) const {
  return {x + rhs.x, y + rhs.x, z + rhs.z, w + rhs.w};
}

void Point4D::operator+=(double rhs) {
  x += rhs;
  y += rhs;
  z += rhs;
  w += rhs;
}

void Point4D::operator+=(const Point4D &rhs) {
  x += rhs.x;
  y += rhs.y;
  z += rhs.z;
  w += rhs.w;
}

Point4D Point4D::operator-(double rhs) const {
  return {x - rhs, y - rhs, z - rhs, w - rhs};
}

Point4D Point4D::operator-(const Point4D &rhs) const {
  return {x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w};
}

void Point4D::operator-=(double rhs) {
  x -= rhs;
  y -= rhs;
  z -= rhs;
  w -= rhs;
}

void Point4D::operator-=(const Point4D &rhs) {
  x -= rhs.x;
  y -= rhs.y;
  z -= rhs.z;
  w -= rhs.w;
}

Point4D Point4D::operator*(double rhs) const {
  return {x * rhs, y * rhs, z * rhs, w * rhs};
}

void Point4D::operator*=(double rhs) {
  x *= rhs;
  y *= rhs;
  z *= rhs;
  w *= rhs;
}

Point4D Point4D::operator/(double rhs) const {
  return {x / rhs, y / rhs, z / rhs, w / rhs};
}

void Point4D::operator/=(double rhs) {
  x /= rhs;
  y /= rhs;
  z /= rhs;
  w /= rhs;
}
} // namespace nurbs
#pragma once

// NURBS
#include "include/point_types.hpp"

// STD
#include <vector>

namespace nurbs {
class Surface {
public:
  Surface(Point2D u_interval = {0.0, 1.0}, Point2D v_interval = {0.0, 1.0})
      : u_interval_(u_interval), v_interval_(v_interval) {}

  virtual Point3D EvaluatePoint(Point2D uv) const { return {0, 0, 0}; }
  virtual std::vector<Point3D> EvaluatePoints(uint32_t u_sample_count,
                                              uint32_t v_sample_count) const {
    std::vector<Point3D> points(u_sample_count * v_sample_count);
    double u_div = (u_interval_.y - u_interval_.x) /
                   static_cast<double>(u_sample_count - 1);
    double v_div = (v_interval_.y - v_interval_.x) /
                   static_cast<double>(v_sample_count - 1);
    for (uint32_t i = 0; i < u_sample_count; ++i) {
      Point2D uv = {u_interval_.x + static_cast<double>(i) * u_div, 0};
      for (uint32_t j = 0; j < v_sample_count; ++j) {
        uv.y = v_interval_.x + static_cast<double>(j) * v_div;
        points[(i * v_sample_count) + j] = EvaluatePoint(uv);
      }
    }
    return points;
  }

  void u_interval(Point2D interval) { u_interval_ = interval; }
  Point2D u_interval() const { return u_interval_; }

  void v_interval(Point2D interval) { v_interval_ = interval; }
  Point2D v_interval() const { return v_interval_; }

protected:
  Point2D u_interval_;
  Point2D v_interval_;
};
} // namespace nurbs
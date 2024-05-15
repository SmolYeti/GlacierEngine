#pragma once

// NURBS
#include "include/point_types.hpp"

// STD
#include <vector>

namespace nurbs {
class Curve2D {
public:
  Curve2D(Point2D interval = {0.0, 1.0}) : interval_(interval) {
    interval_div_ = 1.0 / (interval_.y - interval_.x);
  }

  virtual Point2D EvaluateCurve(double u) const { return {0, 0}; }

  virtual std::vector<Point2D>
  EvaluateCurvePoints(uint32_t point_count) const {
    std::vector<Point2D> points(point_count);
    const double div =
        (interval_.y - interval_.x) / static_cast<double>(point_count - 1);
    for (uint32_t i = 0; i < point_count; ++i) {
      double u = interval_.x + (static_cast<double>(i) * div);
      points[i] = EvaluateCurve(u);
    }
    return points;
  }

  void interval(Point2D interval) {
    interval_ = interval;
    interval_div_ = 1.0 / (interval_.y - interval_.x);
  }
  Point2D interval() const { return interval_; }

protected:
  // Helper methods
  double ClampInterval(double u) const {
    if (u < interval_.x) {
      u = interval_.x;
    }
    if (u > interval_.y) {
      u = interval_.y;
    }
    return u;
  }
  double LocalizeClampInterval(double u) const {
    if (u < interval_.x) {
      u = interval_.x;
    }
    if (u > interval_.y) {
      u = interval_.y;
    }
    return (u - interval_.x) * interval_div_;
  }

protected:
  Point2D interval_;
  double interval_div_ = 1.0;
};
} // namespace nurbs
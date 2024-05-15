#include "include/bezier_surface.hpp"

namespace nurbs {
    BezierSurface::BezierSurface(std::vector<BezierCurve3D> curves, Point2D u_interval,
        Point2D v_interval) : Surface(u_interval, v_interval), curves_(curves) {}


    Point3D BezierSurface::EvaluatePoint(Point2D uv) const {
        std::vector<Point3D> control_points;
        for (const auto& curve : curves_) {
            control_points.push_back(curve.EvaluateCurve(uv.x));
        }
        const BezierCurve3D curve(control_points, v_interval_);
        return curve.EvaluateCurve(uv.y);
    }

    std::vector<Point3D> BezierSurface::EvaluatePoints(
        uint32_t u_sample_count, uint32_t v_sample_count) const {
        std::vector<Point3D> points(u_sample_count * v_sample_count);
        double u_div = (u_interval_.y - u_interval_.x) / static_cast<double>(u_sample_count - 1);
        double v_div = (v_interval_.y - v_interval_.x) / static_cast<double>(v_sample_count - 1);
        for (uint32_t i = 0; i < u_sample_count; ++i) {
            double u = u_interval_.x + static_cast<double>(i) * u_div;
            std::vector<Point3D> control_points;
            for (const auto& curve : curves_) {
                control_points.push_back(curve.EvaluateCurve(u));
            }
            const BezierCurve3D curve(control_points, v_interval_);
            for (uint32_t j = 0; j < v_sample_count; ++j) {
                const double v = v_interval_.x + static_cast<double>(j) * v_div;
                points[(i * v_sample_count) + j] = curve.EvaluateCurve(v);
            }
        }
        return points;
    }
}
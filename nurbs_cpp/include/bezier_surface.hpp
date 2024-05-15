#pragma once

#include "include/bezier_curve.hpp"
#include "include/surface.hpp"

// STD
#include <array>
#include <functional>

namespace nurbs {
    class BezierSurface : public Surface {
    public:
        BezierSurface(std::vector<BezierCurve3D> curves, Point2D u_interval = { 0.0, 1.0 },
            Point2D v_interval = { 0.0, 1.0 });

        Point3D EvaluatePoint(Point2D uv) const override;

        std::vector<Point3D> EvaluatePoints(
            uint32_t u_sample_count, uint32_t v_sample_count) const override;

    private:
        std::vector<BezierCurve3D> curves_;
    };

}
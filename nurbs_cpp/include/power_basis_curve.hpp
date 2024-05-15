#pragma once

#include "curve_2d.hpp"
#include "curve_3d.hpp"

namespace nurbs {
class PowerBasisCurve2D : public Curve2D {
   public:
    PowerBasisCurve2D(std::vector<Point2D> bases,
                      Point2D interval = {0.0, 1.0});

    Point2D EvaluateCurve(double u) const override;

   private:
    Point2D Horner(double u) const;

    std::vector<Point2D> bases_;
};

class PowerBasisCurve3D : public Curve3D {
   public:
    PowerBasisCurve3D(std::vector<Point3D> bases,
                      Point2D interval = {0.0, 1.0});

    Point3D EvaluateCurve(double u) const override;

   private:
    Point3D Horner(double u) const;

    std::vector<Point3D> bases_;
};
}  // namespace nurbs
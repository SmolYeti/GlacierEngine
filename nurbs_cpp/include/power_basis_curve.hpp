#pragma once

#include "curve_2d.hpp"
#include "curve_3d.hpp"

namespace nurbs {
class PowerBasisCurve2D : public Curve2D {
   public:
    PowerBasisCurve2D(std::vector<glm::dvec2> bases,
                      glm::dvec2 interval = {0.0, 1.0});

    glm::dvec2 EvaluateCurve(double u) const override;
    std::vector<glm::dvec2> EvaluateCurvePoints(uint32_t point_count) const override;

   private:
    glm::dvec2 Horner(double u) const;

    std::vector<glm::dvec2> bases_;
};

class PowerBasisCurve3D : public Curve3D {
   public:
    PowerBasisCurve3D(std::vector<glm::dvec3> bases,
                      glm::dvec2 interval = {0.0, 1.0});

    glm::dvec3 EvaluateCurve(double u) const override;
    std::vector<glm::dvec3> EvaluateCurvePoints(uint32_t point_count) const override;

   private:
    glm::dvec3 Horner(double u) const;

    std::vector<glm::dvec3> bases_;
};
}  // namespace nurbs
#include "include/power_basis_curve.hpp"

namespace nurbs {
PowerBasisCurve2D::PowerBasisCurve2D(std::vector<Point2D> bases,
                                     Point2D interval)
    : Curve2D(interval), bases_(bases) {}

Point2D PowerBasisCurve2D::EvaluateCurve(double u) const {
    return Horner(u);
}

// Chaper 1, Algorithm 1.1 Horner 1, p7
Point2D PowerBasisCurve2D::Horner(double u) const {
    Point2D point = {0.0, 0.0};
    if (!bases_.empty()) {
        point = bases_[bases_.size() - 1];
    }
    for (int i = static_cast<int>(bases_.size()) - 2; i >= 0; --i) {
        point = (u * point) + bases_[i];
    }
    return point;
}

PowerBasisCurve3D::PowerBasisCurve3D(std::vector<Point3D> bases,
                                     Point2D interval)
    : Curve3D(interval), bases_(bases) {}

Point3D PowerBasisCurve3D::EvaluateCurve(double u) const {
    return Horner(u);
}

// Chaper 1, Algorithm 1.1 Horner 1, p7
Point3D PowerBasisCurve3D::Horner(double u) const {
    Point3D point = {0.0, 0.0, 0.0};
    if (!bases_.empty()) {
        point = bases_[bases_.size() - 1];
    }
    for (int i = static_cast<int>(bases_.size()) - 2; i >= 0; --i) {
        point = (u * point) + bases_[i];
    }
    return point;
}
}  // namespace nurbs
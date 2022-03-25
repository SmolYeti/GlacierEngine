#include "include/power_basis_curve.hpp"

namespace nurbs {
PowerBasisCurve2D::PowerBasisCurve2D(std::vector<glm::dvec2> bases,
                                     glm::dvec2 interval)
    : Curve2D(interval), bases_(bases) {}

glm::dvec2 PowerBasisCurve2D::EvaluateCurve(double u) const {
    return Horner(u);
}

std::vector<glm::dvec2> PowerBasisCurve2D::EvaluateCurvePoints(
    uint32_t point_count) const {
    std::vector<glm::dvec2> points(point_count);
    double div = (interval_.y - interval_.x) /          //
                 static_cast<double>(point_count - 1);  //
    for (uint32_t i = 0; i < point_count; ++i) {
        double u = interval_.x + (static_cast<double>(i) * div);
        points[i] = EvaluateCurve(u);
    }
    return points;
}

// Chaper 1, Algorithm 1.1 Horner 1, p7
glm::dvec2 PowerBasisCurve2D::Horner(double u) const {
    glm::dvec2 point = {0.0, 0.0};
    if (!bases_.empty()) {
        point = bases_[bases_.size() - 1];
    }
    for (int i = static_cast<int>(bases_.size()) - 2; i >= 0; --i) {
        point = (u * point) + bases_[i];
    }
    return point;
}

PowerBasisCurve3D::PowerBasisCurve3D(std::vector<glm::dvec3> bases,
                                     glm::dvec2 interval)
    : Curve3D(interval), bases_(bases) {}

glm::dvec3 PowerBasisCurve3D::EvaluateCurve(double u) const {
    return Horner(u);
}

std::vector<glm::dvec3> PowerBasisCurve3D::EvaluateCurvePoints(
    uint32_t point_count) const {
    std::vector<glm::dvec3> points(point_count);
    double div = (interval_.y - interval_.x) /          //
                 static_cast<double>(point_count - 1);  //
    for (uint32_t i = 0; i < point_count; ++i) {
        double u = interval_.x + (static_cast<double>(i) * div);
        points[i] = EvaluateCurve(u);
    }
    return points;
}

// Chaper 1, Algorithm 1.1 Horner 1, p7
glm::dvec3 PowerBasisCurve3D::Horner(double u) const {
    glm::dvec3 point = {0.0, 0.0, 0.0};
    if (!bases_.empty()) {
        point = bases_[bases_.size() - 1];
    }
    for (int i = static_cast<int>(bases_.size()) - 2; i >= 0; --i) {
        point = (u * point) + bases_[i];
    }
    return point;
}
}  // namespace nurbs
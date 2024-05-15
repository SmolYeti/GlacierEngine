#include "curve_model.hpp"

namespace vulkeng {

CurveModel::CurveModel(VulkanDevice* device,
                       const std::vector<Vertex>& vertices)
    : LineModel(device, vertices) {}

std::shared_ptr<CurveModel> CurveModel::ModelFromCurve2D(
    VulkanDevice* device, const nurbs::Curve2D& curve) {
    const std::vector<nurbs::Point2D> points = curve.EvaluateCurvePoints(POINT_COUNT);

    std::vector<LineModel::Vertex> vertices = {};
    vertices.reserve(points.size());
    for (const auto& point : points) {
        LineModel::Vertex v;
        v.pos = {static_cast<float>(point.x), static_cast<float>(point.y), 0};
        v.color = {1.0f, 0.0f, 0.0f};
        vertices.push_back(v);
    }
    return std::make_shared<CurveModel>(device, vertices);
}

std::shared_ptr<CurveModel> CurveModel::ModelFromCurve3D(
    VulkanDevice* device, const nurbs::Curve3D& curve) {
    const std::vector<nurbs::Point3D> points = curve.EvaluateCurvePoints(POINT_COUNT);

    std::vector<LineModel::Vertex> vertices = {};
    vertices.reserve(points.size());
    for (const auto& point : points) {
        LineModel::Vertex v;
        v.pos = {point.x, point.y, point.y};
        v.color = {1.0, 0.0, 0.0};
        vertices.push_back(v);
    }
    return std::make_shared<CurveModel>(device, vertices);
}
}  // namespace vulkeng
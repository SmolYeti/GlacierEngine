#include "vulkeng/experiment/surface_model.hpp"

namespace vulkeng {

SurfaceModel::SurfaceModel(VulkanDevice* device,
                           const TriangleModel::Builder& builder)
    : TriangleModel(device, builder) {}

std::shared_ptr<SurfaceModel> SurfaceModel::ModelFromSurface(
      VulkanDevice* device, const nurbs::Surface& surface) {
  const std::vector<glm::dvec3> points =
      surface.EvaluatePoints(POINT_COUNT, POINT_COUNT);

  TriangleModel::Builder builder;
  builder.vertices.reserve(points.size());
  for (const auto& point : points) {
    TriangleModel::Vertex v;
    v.pos = {static_cast<float>(point.x), static_cast<float>(point.y),
             static_cast<float>(point.z)};
    v.color = {1.0f, 1.0f, 1.0f};
    builder.vertices.push_back(v);
  }

  // Indicies
  for (uint32_t i = 0; i < POINT_COUNT - 1; ++i) {    // y
    for (uint32_t j = 0; j < POINT_COUNT - 1; ++j) {  // x
      uint32_t index = i * POINT_COUNT + j;
      builder.indices.push_back(index);
      builder.indices.push_back(index + POINT_COUNT);
      builder.indices.push_back(index + 1);

      builder.indices.push_back(index + 1);
      builder.indices.push_back(index + POINT_COUNT);
      builder.indices.push_back(index + POINT_COUNT + 1);
    }
  }
  return std::make_shared<SurfaceModel>(device, builder);
}
}  // namespace vulkeng
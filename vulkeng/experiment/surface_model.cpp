#include "vulkeng/experiment/surface_model.hpp"

namespace vulkeng {

SurfaceModel::SurfaceModel(VulkanDevice *device,
                           const TriangleModel::Builder &builder)
    : TriangleModel(device, builder) {}

std::shared_ptr<SurfaceModel>
SurfaceModel::ModelFromSurface(VulkanDevice *device,
                               const nurbs::Surface &surface) {
  const std::vector<glm::dvec3> points =
      surface.EvaluatePoints(POINT_COUNT, POINT_COUNT);

  TriangleModel::Builder builder;
  builder.vertices.reserve(points.size());
  double u_div = 1 / static_cast<double>(POINT_COUNT - 1);
  double v_div = 1 / static_cast<double>(POINT_COUNT - 1);

  for (size_t index = 0; index < points.size(); ++index) {
    auto const &point = points[index];
    size_t i = index / POINT_COUNT;
    size_t j = index % POINT_COUNT;
    TriangleModel::Vertex v;
    v.pos = {static_cast<float>(point.x), static_cast<float>(point.y),
             static_cast<float>(point.z)};
    v.color = {1.0f, 1.0f, 1.0f};
    v.uv = {static_cast<double>(i) * u_div, static_cast<double>(j) * v_div};
    builder.vertices.push_back(v);
  }

  // Indicies
  for (uint32_t i = 0; i < POINT_COUNT - 1; ++i) {   // y
    for (uint32_t j = 0; j < POINT_COUNT - 1; ++j) { // x
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
} // namespace vulkeng
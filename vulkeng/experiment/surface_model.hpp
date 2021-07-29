#pragma once

#include "nurbs_cpp/include/surface.hpp"

#include "vulkeng/include/triangle_model.hpp"

namespace vulkeng {
class SurfaceModel : public TriangleModel {
  static const uint32_t POINT_COUNT = 100;

 public:
  SurfaceModel(VulkanDevice* device, const TriangleModel::Builder& builder);

  SurfaceModel(const SurfaceModel&) = delete;
  SurfaceModel& operator=(const SurfaceModel&) = delete;

  static std::shared_ptr<SurfaceModel> ModelFromSurface(
      VulkanDevice* device, const nurbs::Surface& surface);
};
}  // namespace vulkeng
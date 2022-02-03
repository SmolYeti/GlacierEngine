#pragma once

#include "nurbs_cpp/include/curve_2d.hpp"
#include "nurbs_cpp/include/curve_3d.hpp"

#include "vulkeng/include/line_model.hpp"

namespace vulkeng {
class CurveModel : public LineModel {
  static const uint32_t POINT_COUNT = 100;

 public:
  CurveModel(VulkanDevice* device, const std::vector<Vertex>& vertices);

  CurveModel(const CurveModel&) = delete;
  CurveModel& operator=(const CurveModel&) = delete;

  static std::shared_ptr<CurveModel> ModelFromCurve2D(
      VulkanDevice* device, const nurbs::Curve2D& curve);

  static std::shared_ptr<CurveModel> ModelFromCurve3D(
      VulkanDevice* device, const nurbs::Curve3D& curve);
};
}  // namespace vulkeng
#pragma once

#include "callback_manager.hpp"
#include "vulkan_camera.hpp"

namespace vulkeng {
class CameraController {
 public:
  CameraController(VulkanCamera camera);

  VulkanCamera GetUpdatedCamera(float time_change);


 private:
  VulkanCamera camera_;
};
}  // namespace vulkeng
#pragma once

// vulkeng
#include "vulkan_descriptors.hpp"
#include "vulkan_device.hpp"
#include "vulkan_game_object.hpp"
#include "vulkan_renderer.hpp"
#include "vulkan_window.hpp"

// std
#include <memory>
#include <vector>

namespace vulkeng {
class VulkanApplication {
 public:
  VulkanApplication(int width, int height, std::string app_name);
  ~VulkanApplication();

  VulkanApplication(const VulkanApplication&) = delete;
  VulkanApplication& operator=(const VulkanApplication&) = delete;

  void Run();

 protected:
  void LoadGameObjects();

  // Variables
  std::unique_ptr<VulkanWindow> window_;
  std::unique_ptr<VulkanDevice> device_;
  std::unique_ptr<VulkanRenderer> renderer_;

  std::unique_ptr<VulkanDescriptorPool> global_pool_;
  VulkanGameObject::Map game_objects_;
};
}  // namespace vulkeng
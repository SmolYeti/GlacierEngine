#pragma once

#include "vulkan_camera.hpp"
#include "vulkan_device.hpp"
#include "vulkan_frame_info.hpp"
#include "vulkan_game_object.hpp"
#include "vulkan_pipeline.hpp"

// std
#include <memory>
#include <vector>

namespace vulkeng {
class SimpleRenderSystem {
 public:
  SimpleRenderSystem(VulkanDevice* device, VkRenderPass render_pass, VkDescriptorSetLayout global_set_layout);
  ~SimpleRenderSystem();

  SimpleRenderSystem(const SimpleRenderSystem&) = delete;
  SimpleRenderSystem& operator=(const SimpleRenderSystem&) = delete;

  void RenderGameObjects(const FrameInfo& frame_info);

 protected:
  void CreatePipelineLayout(VkDescriptorSetLayout global_set_layout);
  void CreatePipeline(VkRenderPass render_pass);

  // Variables
  VulkanDevice* device_ = nullptr;

  std::unique_ptr<VulkanPipeline> pipeline_;
  VkPipelineLayout pipeline_layout_ = nullptr;
};
}  // namespace vulkeng
#pragma once

#include "vulkeng/include/vulkan_camera.hpp"
#include "vulkeng/include/vulkan_device.hpp"
#include "vulkeng/include/vulkan_frame_info.hpp"
#include "vulkeng/include/vulkan_game_object.hpp"
#include "vulkeng/include/vulkan_pipeline.hpp"

// std
#include <memory>
#include <vector>

namespace vulkeng {
class LineRenderSystem {
  const float LINE_WIDTH = 2.0;
 public:
  LineRenderSystem(VulkanDevice* device, VkRenderPass render_pass,
                     VkDescriptorSetLayout global_set_layout);
  ~LineRenderSystem();

  LineRenderSystem(const LineRenderSystem&) = delete;
  LineRenderSystem& operator=(const LineRenderSystem&) = delete;

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
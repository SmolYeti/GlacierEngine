#pragma once

#include "vulkan_device.hpp"
#include "vulkan_swap_chain.hpp"

#include "vulkan_window.hpp"

#include <cassert>
#include <memory>
#include <vector>

namespace vulkeng {
class VulkanRenderer {
 public:
  VulkanRenderer(VulkanWindow* window, VulkanDevice* device);
  ~VulkanRenderer();

  VulkanRenderer(const VulkanRenderer&) = delete;
  VulkanRenderer& operator=(const VulkanRenderer&) = delete;

  VkRenderPass GetSwapChainRenderPass() const {
    return swap_chain_->render_pass();
  }

  float GetAspectRatio() const { return swap_chain_->ExtentAspectRatio(); } 

  bool IsFrameInProgress() const { return is_frame_started_; }

  VkCommandBuffer GetCurrentCommandBuffer() const {
    assert(is_frame_started_ &&
           "Cannot get command buffer when frame is not in progress");
    return command_buffers_[current_frame_index_];
  }

  int FrameIndex() const {
    assert(is_frame_started_ &&
           "Cannot get frame index when frame is not in progress");
    return current_frame_index_;
  }

  VkCommandBuffer BeginFrame();
  void EndFrame();

  void BeginSwapChainRenderPass(VkCommandBuffer command_buffer);
  void EndSwapChainRenderPass(VkCommandBuffer command_buffer);

 private:
  // Private Methods
  void CreateCommandBuffers();
  void FreeCommandBuffers();
  void RecreateSwapChain();

  // Variables
  VulkanWindow* window_ = nullptr;
  VulkanDevice* device_ = nullptr;
  std::unique_ptr<VulkanSwapChain> swap_chain_;
  std::vector<VkCommandBuffer> command_buffers_;

  uint32_t current_image_index_ = 0;
  int current_frame_index_ = 0;
  bool is_frame_started_ = false;
};
}  // namespace vulkeng
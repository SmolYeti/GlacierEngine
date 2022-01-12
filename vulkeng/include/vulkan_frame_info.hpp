#pragma once

#include "vulkan_camera.hpp"
#include "vulkan_game_object.hpp"

#include <vulkan/vulkan.h>

namespace vulkeng {
struct FrameInfo {
  int frame_index;
  float frame_time;
  VkCommandBuffer command_buffer;
  VulkanCamera& camera;
  VkDescriptorSet global_descriptor_set;
  VulkanGameObject::Map& game_objects;
};
}  // namespace vulkeng
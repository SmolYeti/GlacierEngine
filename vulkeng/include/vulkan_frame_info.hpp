#pragma once

#include "vulkan_camera.hpp"
#include "vulkan_game_object.hpp"

#include <vulkan/vulkan.h>

namespace vulkeng {

#define MAX_LIGHTS 10

struct PointLight {
  glm::vec4 position{}; // ignore w
  glm::vec4 color{}; // w is intensity
};

struct GlobalUbo {
  glm::mat4 projection{1.f};
  glm::mat4 view{1.f};
  glm::mat4 inv_view{1.f};
  glm::vec4 ambient_light_color{1.f, 1.f, 1.f, 0.1f};
  PointLight lights[MAX_LIGHTS];
  float offset{0.0f};
  int active_lights{0};
};

struct FrameInfo {
  int frame_index;
  float frame_time;
  VkCommandBuffer command_buffer;
  VulkanCamera& camera;
  VkDescriptorSet global_descriptor_set;
  VulkanGameObject::Map& game_objects;
};
}  // namespace vulkeng
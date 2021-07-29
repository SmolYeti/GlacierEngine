#pragma once

#include "vulkan_buffer.hpp"
#include "vulkan_device.hpp"

// GLM
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

// STD
#include <memory>
#include <vector>

namespace vulkeng {
class LineModel {
 public:
  struct Vertex {
    glm::vec3 pos;
    glm::vec3 color;

    static std::vector<VkVertexInputBindingDescription> BindingDescription();
    static std::vector<VkVertexInputAttributeDescription>
    AttributeDescriptions();

    bool operator==(const Vertex& other) const {
      return pos == other.pos && color == other.color;
    }
  };

  LineModel(VulkanDevice* device);
  LineModel(VulkanDevice* device, const std::vector<Vertex>& vertices);
  ~LineModel();

  LineModel(const LineModel&) = delete;
  LineModel& operator=(const LineModel&) = delete;

  void Bind(VkCommandBuffer command_buffer);
  void Draw(VkCommandBuffer command_buffer);

 private:
  void CreateVertexBuffers(const std::vector<Vertex>& vertices);

  VulkanDevice* device_ = nullptr;

  std::unique_ptr<VulkanBuffer> vertex_buffer_ = nullptr;
  uint32_t vertex_count_ = 0;
};
}  // namespace vulkeng
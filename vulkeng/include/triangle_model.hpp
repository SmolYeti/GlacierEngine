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
class TriangleModel {
 public:
  struct Vertex {
    glm::vec3 pos;
    glm::vec3 color;
    glm::vec3 normal;
    glm::vec2 uv;

    static std::vector<VkVertexInputBindingDescription> BindingDescription();
    static std::vector<VkVertexInputAttributeDescription>
    AttributeDescriptions();

    bool operator==(const Vertex& other) const {
      return pos == other.pos && color == other.color &&
             normal == other.normal && uv == other.uv;
    }
  };

  struct Builder {
    std::vector<Vertex> vertices{};
    std::vector<uint32_t> indices{};

    void LoadModel(const std::string& file_path);
  };

  TriangleModel(VulkanDevice* device, const Builder& builder);
  ~TriangleModel();

  TriangleModel(const TriangleModel&) = delete;
  TriangleModel& operator=(const TriangleModel&) = delete;

  static std::unique_ptr<TriangleModel> CreateModelFromFile(
      VulkanDevice* device, const std::string& file_path);

  void Bind(VkCommandBuffer command_buffer);
  void Draw(VkCommandBuffer command_buffer);

 private:
  void CreateVertexBuffers(const std::vector<Vertex>& vertices);
  void CreateIndexBuffers(const std::vector<uint32_t>& indices);

  VulkanDevice* device_ = nullptr;

  std::unique_ptr<VulkanBuffer> vertex_buffer_ = nullptr;
  uint32_t vertex_count_ = 0;

  std::unique_ptr<VulkanBuffer> index_buffer_ = nullptr;
  uint32_t index_count_ = 0;
};
}  // namespace vulkeng
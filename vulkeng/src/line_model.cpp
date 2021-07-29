#include "vulkeng/include/line_model.hpp"

#include "vulkeng/include/vulkan_utils.hpp"

// Libs
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/hash.hpp>

// STD
#include <cassert>
#include <cstring>
#include <unordered_map>

namespace vulkeng {
LineModel::LineModel(VulkanDevice* device) : device_(device) {
  std::vector<Vertex> vertices{};
  Vertex v_0 = {{0.f, 0.f, 0.f}, {1.f, 1.f, 1.f}};
  Vertex v_1 = {{0.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}};
  Vertex v_2 = {{1.0f, 1.0f, 0.0f}, {0.0f, 1.0f, 0.0f}};
  Vertex v_3 = {{1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
  Vertex v_4 = {{0.f, 0.f, 0.f}, {1.f, 1.f, 1.f}};
  CreateVertexBuffers({v_0, v_1, v_2, v_3, v_4});
}
LineModel::LineModel(VulkanDevice* device, const std::vector<Vertex>& vertices)
    : device_(device) {
  CreateVertexBuffers(vertices);
}

LineModel::~LineModel() {}

void LineModel::CreateVertexBuffers(const std::vector<Vertex>& vertices) {
  vertex_count_ = static_cast<uint32_t>(vertices.size());
  assert(vertex_count_ >= 2 && "Vertex count must be at least 2");
  VkDeviceSize buffer_size = sizeof(vertices[0]) * vertex_count_;
  VkDeviceSize vertex_size = sizeof(vertices[0]);

  VulkanBuffer staging_buffer{
      device_,
      vertex_size,
      vertex_count_,
      VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
          VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
  };

  staging_buffer.Map();
  staging_buffer.WriteToBuffer((void*)vertices.data());

  vertex_buffer_ = std::make_unique<VulkanBuffer>(
      device_, vertex_size, vertex_count_,
      VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

  device_->CopyBuffer(staging_buffer.buffer(), vertex_buffer_->buffer(),
                      buffer_size);
}

void LineModel::Draw(VkCommandBuffer command_buffer) {
  vkCmdDraw(command_buffer, vertex_count_, 1, 0, 0);
}

void LineModel::Bind(VkCommandBuffer command_buffer) {
  VkBuffer buffers[] = {vertex_buffer_->buffer()};
  VkDeviceSize offsets[] = {0};
  vkCmdBindVertexBuffers(command_buffer, 0, 1, buffers, offsets);
}

std::vector<VkVertexInputBindingDescription>
LineModel::Vertex::BindingDescription() {
  std::vector<VkVertexInputBindingDescription> binding_description(1);
  binding_description[0].binding = 0;
  binding_description[0].stride = sizeof(Vertex);
  binding_description[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

  return binding_description;
}

std::vector<VkVertexInputAttributeDescription>
LineModel::Vertex::AttributeDescriptions() {
  std::vector<VkVertexInputAttributeDescription> attributed_descriptions{};

  // Position
  attributed_descriptions.push_back(
      {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, pos)});
  // Color
  attributed_descriptions.push_back(
      {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, color)});

  return attributed_descriptions;
}
}  // namespace vulkeng

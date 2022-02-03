#include "vulkeng/include/triangle_model.hpp"

#include "vulkeng/include/vulkan_utils.hpp"

// Libs
#define TINYOBJLOADER_IMPLEMENTATION
#include <external/tiny_obj_loader.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/hash.hpp>

// STD
#include <cassert>
#include <cstring>
#include <unordered_map>

namespace std {
template <>
struct hash<vulkeng::TriangleModel::Vertex> {
  size_t operator()(vulkeng::TriangleModel::Vertex const& vertex) const {
    size_t seed = 0;
    vulkeng::HashCombine(seed, vertex.pos, vertex.color, vertex.normal,
                         vertex.uv);
    return seed;
  }
};
}  // namespace std

namespace vulkeng {
TriangleModel::TriangleModel(VulkanDevice* device, const Builder& builder)
    : device_(device) {
  CreateVertexBuffers(builder.vertices);
  CreateIndexBuffers(builder.indices);
}

TriangleModel::~TriangleModel() {}

std::unique_ptr<TriangleModel> TriangleModel::CreateModelFromFile(
    VulkanDevice* device, const std::string& file_path) {
  Builder builder{};
  builder.LoadModel(file_path);

  return std::make_unique<TriangleModel>(device, builder);
}

void TriangleModel::CreateVertexBuffers(const std::vector<Vertex>& vertices) {
  vertex_count_ = static_cast<uint32_t>(vertices.size());
  assert(vertex_count_ >= 3 && "Vertex count must be at least 3");
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

void TriangleModel::CreateIndexBuffers(const std::vector<uint32_t>& indices) {
  index_count_ = static_cast<uint32_t>(indices.size());

  if (indices.empty()) {
    return;
  }

  VkDeviceSize buffer_size = sizeof(uint32_t) * index_count_;
  VkDeviceSize index_size = sizeof(uint32_t);

  VulkanBuffer staging_buffer{
      device_,
      index_size,
      index_count_,
      VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
          VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
  };

  staging_buffer.Map();
  staging_buffer.WriteToBuffer((void*)indices.data());

  index_buffer_ = std::make_unique<VulkanBuffer>(
      device_, index_size, index_count_,
      VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

  device_->CopyBuffer(staging_buffer.buffer(), index_buffer_->buffer(),
                      buffer_size);
}

void TriangleModel::Draw(VkCommandBuffer command_buffer) {
  if (index_buffer_) {
    vkCmdDrawIndexed(command_buffer, index_count_, 1, 0, 0, 0);
  } else {
    vkCmdDraw(command_buffer, vertex_count_, 1, 0, 0);
  }
}

void TriangleModel::Bind(VkCommandBuffer command_buffer) {
  VkBuffer buffers[] = {vertex_buffer_->buffer()};
  VkDeviceSize offsets[] = {0};
  vkCmdBindVertexBuffers(command_buffer, 0, 1, buffers, offsets);

  if (index_buffer_) {
    vkCmdBindIndexBuffer(command_buffer, index_buffer_->buffer(), 0,
                         VK_INDEX_TYPE_UINT32);
  }
}

std::vector<VkVertexInputBindingDescription>
TriangleModel::Vertex::BindingDescription() {
  std::vector<VkVertexInputBindingDescription> binding_description(1);
  binding_description[0].binding = 0;
  binding_description[0].stride = sizeof(Vertex);
  binding_description[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

  return binding_description;
}

std::vector<VkVertexInputAttributeDescription>
TriangleModel::Vertex::AttributeDescriptions() {
  std::vector<VkVertexInputAttributeDescription> attributed_descriptions{};

  // Position
  attributed_descriptions.push_back(
      {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, pos)});
  // Color
  attributed_descriptions.push_back(
      {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, color)});
  // Normal
  attributed_descriptions.push_back(
      {2, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, normal)});
  // UVs
  attributed_descriptions.push_back(
      {3, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(Vertex, uv)});

  return attributed_descriptions;
}

void TriangleModel::Builder::LoadModel(const std::string& file_path) {
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warn, err;

  if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                        file_path.c_str())) {
    throw std::runtime_error(warn + err);
  }

  vertices.clear();
  indices.clear();

  std::unordered_map<Vertex, uint32_t> unique_vertices{};
  for (const auto& shape : shapes) {
    for (const auto& index : shape.mesh.indices) {
      Vertex vertex{};

      if (index.vertex_index >= 0) {
        vertex.pos = {
            attrib.vertices[3 * index.vertex_index + 0],
            attrib.vertices[3 * index.vertex_index + 1],
            attrib.vertices[3 * index.vertex_index + 2],
        };

        vertex.color = {
            attrib.colors[3 * index.vertex_index + 0],
            attrib.colors[3 * index.vertex_index + 1],
            attrib.colors[3 * index.vertex_index + 2],
        };
      }

      if (index.normal_index >= 0) {
        vertex.normal = {
            attrib.normals[3 * index.normal_index + 0],
            attrib.normals[3 * index.normal_index + 1],
            attrib.normals[3 * index.normal_index + 2],
        };
      }

      if (index.texcoord_index >= 0) {
        vertex.uv = {
            attrib.texcoords[2 * index.texcoord_index + 0],
            attrib.texcoords[2 * index.texcoord_index + 1],
        };
      }

      if (unique_vertices.count(vertex) == 0) {
        unique_vertices[vertex] = static_cast<uint32_t>(vertices.size());
        vertices.push_back(vertex);
      }
      indices.push_back(unique_vertices[vertex]);
    }
  }
}
}  // namespace vulkeng

#include "vulkeng/include/vulkan_model.hpp"

#include <cassert>
#include <cstring>

namespace vulkeng {
    VulkanModel::VulkanModel(VulkanDevice* device, const std::vector<Vertex>& vertices) : device_(device) {
        CreateVertexBuffers(vertices);
    }

    VulkanModel::~VulkanModel() {
        vkDestroyBuffer(device_->device(), vertex_buffer_, nullptr);
        vkFreeMemory(device_->device(), vertex_buffer_memory_, nullptr);
    }


    void VulkanModel::Bind(VkCommandBuffer command_buffer) {
        VkBuffer buffers[] = { vertex_buffer_ };
        VkDeviceSize offsets[] = { 0 };
        vkCmdBindVertexBuffers(command_buffer, 0, 1, buffers, offsets);
    }

    void VulkanModel::Draw(VkCommandBuffer command_buffer) {
        vkCmdDraw(command_buffer, vertex_count_, 1, 0, 0);
    }

    void VulkanModel::CreateVertexBuffers(const std::vector<Vertex>& vertices) {
        vertex_count_ = static_cast<uint32_t>(vertices.size());
        assert(vertex_count_ >= 3 && "Vertex count must be at least 3");
        VkDeviceSize buffer_size = sizeof(vertices[0]) * vertex_count_;

        device_->CreateBuffer(buffer_size, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            vertex_buffer_, vertex_buffer_memory_);

        void* data;
        vkMapMemory(device_->device(), vertex_buffer_memory_, 0, buffer_size, 0, &data);
        memcpy(data, vertices.data(), static_cast<size_t>(buffer_size));
        vkUnmapMemory(device_->device(), vertex_buffer_memory_);
    }


    std::vector<VkVertexInputBindingDescription> VulkanModel::Vertex::BindingDescription() {
        std::vector<VkVertexInputBindingDescription> binding_description(1);
        binding_description[0].binding = 0;
        binding_description[0].stride = sizeof(Vertex);
        binding_description[0].inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

        return binding_description;
    }

    std::vector<VkVertexInputAttributeDescription> VulkanModel::Vertex::AttributeDescriptions() {
        std::vector<VkVertexInputAttributeDescription> attributed_descriptions(2);
        attributed_descriptions[0].binding = 0;
        attributed_descriptions[0].location = 0;
        attributed_descriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
        attributed_descriptions[0].offset = offsetof(Vertex, pos);

        // I think I only need one (color) or the other (texture),
        // but we will likely want different shader programs for each
        attributed_descriptions[1].binding = 0;
        attributed_descriptions[1].location = 1;
        attributed_descriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
        attributed_descriptions[1].offset = offsetof(Vertex, color);

        return attributed_descriptions;
    }
}


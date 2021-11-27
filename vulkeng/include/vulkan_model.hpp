#pragma once

#include "vulkan_device.hpp"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

// STD
#include <vector>

namespace vulkeng {
    class VulkanModel {
    public:
        struct Vertex {
            glm::vec3 pos;
            glm::vec3 color;

            static std::vector<VkVertexInputBindingDescription> BindingDescription();
            static std::vector<VkVertexInputAttributeDescription> AttributeDescriptions();
        };

        VulkanModel(VulkanDevice* device, const std::vector<Vertex>& vertices);
        ~VulkanModel();

        VulkanModel(const VulkanModel&) = delete;
        VulkanModel& operator=(const VulkanModel&) = delete;

        void Bind(VkCommandBuffer command_buffer);
        void Draw(VkCommandBuffer command_buffer);
    private:
        void CreateVertexBuffers(const std::vector<Vertex>& vertices);

        VulkanDevice* device_ = nullptr;

        VkBuffer vertex_buffer_ = nullptr;
        VkDeviceMemory vertex_buffer_memory_ = nullptr;
        uint32_t vertex_count_ = 0;

    };
}
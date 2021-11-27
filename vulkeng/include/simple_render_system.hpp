#pragma once

#include "vulkan_device.hpp"
#include "vulkan_game_object.hpp"
#include "vulkan_pipeline.hpp"

// std
#include <memory>
#include <vector>

namespace vulkeng {
    class SimpleRenderSystem {
    public:
        SimpleRenderSystem(VulkanDevice* device, VkRenderPass render_pass);
        ~SimpleRenderSystem();

        SimpleRenderSystem(const SimpleRenderSystem&) = delete;
        SimpleRenderSystem& operator=(const SimpleRenderSystem&) = delete;

        void RenderGameObjects(VkCommandBuffer command_buffer, std::vector<VulkanGameObject>& game_objects);
    protected:
        void CreatePipelineLayout();
        void CreatePipeline(VkRenderPass render_pass);


        // Variables
        VulkanDevice* device_ = nullptr;

        std::unique_ptr<VulkanPipeline> pipeline_;
        VkPipelineLayout pipeline_layout_ = nullptr;
    };
}
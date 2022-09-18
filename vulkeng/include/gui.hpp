#pragma once

// Vulkeng
#include "vulkan_descriptors.hpp"
#include "vulkan_device.hpp"
#include "vulkan_renderer.hpp"
#include "vulkan_window.hpp"

// ImGUI
#include "imgui.h"

namespace vulkeng {
    class GUI {
    public:
        GUI(VulkanWindow* window, VulkanDevice* device, VulkanRenderer* render);
        ~GUI();

        void NewFrame();

        void Render(VkCommandBuffer command_buffer);

        bool show_demo_window = false;
        bool show_another_window = false;
        ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
        void runExample();
    private:
        void Setup(VulkanWindow* window, VulkanDevice* device, VulkanRenderer* render);

        VulkanDevice* device_;

        // Is this needed?
        VkDescriptorPool descriptor_pool_;
    };

}
#pragma once
// TODO - Rename Vulkan App
// vulkeng
#include "include/vulkan_device.hpp"
#include "include/vulkan_game_object.hpp"
#include "include/vulkan_renderer.hpp"
#include "include/window.hpp"

// std
#include <memory>
#include <vector>

namespace vulkeng {
    class VulkanApplication {
    public:
        VulkanApplication(int width, int height, std::string app_name);
        ~VulkanApplication();

        VulkanApplication(const VulkanApplication&) = delete;
        VulkanApplication& operator=(const VulkanApplication&) = delete;

        void Run();

    protected:
        void LoadGameObjects();

        // Variables
        std::unique_ptr<VulkanWindow> window_;
        std::unique_ptr<VulkanDevice> device_;
        std::unique_ptr<VulkanRenderer> renderer_;

        std::vector<VulkanGameObject> game_objects_;
    };
}
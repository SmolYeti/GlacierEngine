#pragma once

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/hash.hpp>

#include <iostream>
#include <mutex>
#include <optional>
#include <set>

const std::vector<const char*> validationLayers = {
    "VK_LAYER_KHRONOS_validation" };

const std::vector<const char*> deviceExtensions = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME };

#ifdef NDEBUG
const bool enableValidationLayers = false;
#else
const bool enableValidationLayers = true;
#endif

VkResult CreateDebugUtilsMessengerEXT(
    VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo,
    const VkAllocationCallbacks* pAllocator,
    VkDebugUtilsMessengerEXT* pDebugMessenger) {
    auto func = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(
        instance, "vkCreateDebugUtilsMessengerEXT");
    if (func != nullptr) {
        return func(instance, pCreateInfo, pAllocator, pDebugMessenger);
    }
    else {
        return VK_ERROR_EXTENSION_NOT_PRESENT;
    }
}

void DestroyDebugUtilsMessengerEXT(VkInstance instance,
    VkDebugUtilsMessengerEXT debugMessenger,
    const VkAllocationCallbacks* pAllocator) {
    auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(
        instance, "vkDestroyDebugUtilsMessengerEXT");
    if (func != nullptr) {
        func(instance, debugMessenger, pAllocator);
    }
}

// Structs
struct QueueFamilyIndices {
    std::optional<uint32_t> graphicsFamily;
    std::optional<uint32_t> presentFamily;

    bool isComplete() {
        return graphicsFamily.has_value() && presentFamily.has_value();
    }
};

struct SwapChainSupportDetails {
    VkSurfaceCapabilitiesKHR capabilities;
    std::vector<VkSurfaceFormatKHR> formats;
    std::vector<VkPresentModeKHR> presentModes;
};

class VulkanWindow {
public:
    using KeyCallback = std::function<void(GLFWwindow* window, int key, int scancode, int action, int mods)>;
    using CursorPositionCallback = std::function<void(GLFWwindow* window, double xpos, double ypos)>;
    using MouseButtonCallback = std::function<void(GLFWwindow* window, int button, int action, int mods)>;
private:
    class CallbackManager {
    public:
        void AddCursorPositionCallback(const CursorPositionCallback& callback) {
            const std::lock_guard<std::mutex> lock(callback_lock_);
            cursor_position_callback_ = callback;
        }

        void AddKeyCallback(const KeyCallback& callback)
        {
            const std::lock_guard<std::mutex> lock(callback_lock_);
            key_callbacks_.push_back(callback);
        }

        void AddMouseButtonCallback(const MouseButtonCallback& callback) {
            const std::lock_guard<std::mutex> lock(callback_lock_);
            mouse_button_callbacks_.push_back(callback);
        }

        void OnCursorPosition(GLFWwindow* window, double xpos, double ypos) {
            cursor_position_callback_(window, xpos, ypos);
        }

        void OnKeyPress(GLFWwindow* window, int key, int scancode, int action, int mods)
        {
            const std::lock_guard<std::mutex> lock(callback_lock_);
            for (KeyCallback& callback : key_callbacks_)
            {
                callback(window, key, scancode, action, mods);
            }
        }

        void OnMouseButtonPress(GLFWwindow* window, int button, int action, int mods) {
            const std::lock_guard<std::mutex> lock(callback_lock_);
            for (MouseButtonCallback& callback : mouse_button_callbacks_)
            {
                callback(window, button, action, mods);
            }
        }

    private:
        std::mutex callback_lock_;
        std::vector<KeyCallback> key_callbacks_;
        std::vector<MouseButtonCallback> mouse_button_callbacks_;
        CursorPositionCallback cursor_position_callback_ = [](GLFWwindow* window, double xpos, double ypos) {};
    };
    CallbackManager callback_manager_;

public:
    VulkanWindow(int width, int height) :width_(width), height_(height) {}

    void Run();

    virtual void init() {};
    virtual void frame() {};
    virtual void cleanup() {};

    void AddCursorPositionCallback(const CursorPositionCallback& callback) {
        callback_manager_.AddCursorPositionCallback(callback);
    }

    void AddKeyCallback(const KeyCallback& callback) {
        callback_manager_.AddKeyCallback(callback);
    }

    void AddMouseButtonCallback(const MouseButtonCallback& callback) {
        callback_manager_.AddMouseButtonCallback(callback);
    }

protected:
    void InitWindow() {
        glfwInit();

        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

        window_ = glfwCreateWindow(width_, height_, "Vulkan", nullptr,
            nullptr);
        glfwSetWindowUserPointer(window_, this);
        glfwSetFramebufferSizeCallback(window_, framebufferResizeCallback);

        glfwSetKeyCallback(window_, [](GLFWwindow* window, int key, int scancode, int action, int mode) {
            VulkanWindow* vulkan_window = (VulkanWindow*)glfwGetWindowUserPointer(window);
            vulkan_window->callback_manager_.OnKeyPress(window, key, scancode, action, mode);
        });
        glfwSetCursorPosCallback(window_, [](GLFWwindow* window, double xpos, double ypos) {
            VulkanWindow* vulkan_window = (VulkanWindow*)glfwGetWindowUserPointer(window);
            vulkan_window->callback_manager_.OnCursorPosition(window, xpos, ypos);
        });
        glfwSetMouseButtonCallback(window_, [](GLFWwindow* window, int button, int action, int mode) {
            VulkanWindow* vulkan_window = (VulkanWindow*)glfwGetWindowUserPointer(window);
            vulkan_window->callback_manager_.OnMouseButtonPress(window, button, action, mode);
        });
    }

    static void framebufferResizeCallback(GLFWwindow* window, int width,
        int height) {
        auto app = reinterpret_cast<VulkanWindow*>(
            glfwGetWindowUserPointer(window));
        app->framebuffer_resized_ = true;
    }

    QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device) {
        QueueFamilyIndices indices;

        uint32_t queueFamilyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount,
            nullptr);

        std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount,
            queueFamilies.data());

        int i = 0;
        for (const auto& queueFamily : queueFamilies) {
            VkBool32 presentSupport = false;
            vkGetPhysicalDeviceSurfaceSupportKHR(device, i, surface_,
                &presentSupport);

            if (presentSupport) {
                indices.presentFamily = i;
            }

            if (queueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT) {
                indices.graphicsFamily = i;
            }

            if (indices.isComplete()) {
                break;
            }

            i++;
        }

        return indices;
    }

    bool checkDeviceExtensionSupport(VkPhysicalDevice device) {
        uint32_t extensionCount;
        vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount,
            nullptr);

        std::vector<VkExtensionProperties> availableExtensions(extensionCount);
        vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount,
            availableExtensions.data());

        std::set<std::string> requiredExtensions(deviceExtensions.begin(),
            deviceExtensions.end());

        for (const auto& extension : availableExtensions) {
            requiredExtensions.erase(extension.extensionName);
        }

        return requiredExtensions.empty();
    }

    SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device) {
        SwapChainSupportDetails details;

        vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, surface_,
            &details.capabilities);

        uint32_t formatCount;
        vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface_, &formatCount,
            nullptr);

        if (formatCount != 0) {
            details.formats.resize(formatCount);
            vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface_, &formatCount,
                details.formats.data());
        }

        uint32_t presentModeCount;
        vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface_,
            &presentModeCount, nullptr);

        if (presentModeCount != 0) {
            details.presentModes.resize(presentModeCount);
            vkGetPhysicalDeviceSurfacePresentModesKHR(
                device, surface_, &presentModeCount, details.presentModes.data());
        }

        return details;
    }

    bool isDeviceSuitable(VkPhysicalDevice device) {
        QueueFamilyIndices indices = findQueueFamilies(device);

        bool extensionsSupported = checkDeviceExtensionSupport(device);

        bool swapChainAdequate = false;
        if (extensionsSupported) {
            SwapChainSupportDetails swapChainSupport = querySwapChainSupport(device);
            swapChainAdequate = !swapChainSupport.formats.empty() &&
                !swapChainSupport.presentModes.empty();
        }

        VkPhysicalDeviceFeatures supportedFeatures;
        vkGetPhysicalDeviceFeatures(device, &supportedFeatures);

        return indices.isComplete() && extensionsSupported && swapChainAdequate && supportedFeatures.samplerAnisotropy;
    }

    VkSampleCountFlagBits getMaxUsableSampleCount() {
        VkPhysicalDeviceProperties physicalDeviceProperties;
        vkGetPhysicalDeviceProperties(physical_device_, &physicalDeviceProperties);

        VkSampleCountFlags counts = physicalDeviceProperties.limits.framebufferColorSampleCounts & physicalDeviceProperties.limits.framebufferDepthSampleCounts;
        if (counts & VK_SAMPLE_COUNT_64_BIT) { return VK_SAMPLE_COUNT_64_BIT; }
        if (counts & VK_SAMPLE_COUNT_32_BIT) { return VK_SAMPLE_COUNT_32_BIT; }
        if (counts & VK_SAMPLE_COUNT_16_BIT) { return VK_SAMPLE_COUNT_16_BIT; }
        if (counts & VK_SAMPLE_COUNT_8_BIT) { return VK_SAMPLE_COUNT_8_BIT; }
        if (counts & VK_SAMPLE_COUNT_4_BIT) { return VK_SAMPLE_COUNT_4_BIT; }
        if (counts & VK_SAMPLE_COUNT_2_BIT) { return VK_SAMPLE_COUNT_2_BIT; }

        return VK_SAMPLE_COUNT_1_BIT;
    }


    void pickPhysicalDevice() {
        uint32_t deviceCount = 0;
        vkEnumeratePhysicalDevices(vk_instance_, &deviceCount, nullptr);

        if (deviceCount == 0) {
            throw std::runtime_error("failed to find GPUs with Vulkan support!");
        }

        std::vector<VkPhysicalDevice> devices(deviceCount);
        vkEnumeratePhysicalDevices(vk_instance_, &deviceCount, devices.data());

        for (const auto& device : devices) {
            if (isDeviceSuitable(device)) {
                physical_device_ = device;
                msaaSamples = getMaxUsableSampleCount();
                break;
            }
        }

        if (physical_device_ == VK_NULL_HANDLE) {
            throw std::runtime_error("failed to find a suitable GPU!");
        }
    }

    void cleanupSwapChain() {
        vkDestroyImageView(device_, colorImageView, nullptr);
        vkDestroyImage(device_, colorImage, nullptr);
        vkFreeMemory(device_, colorImageMemory, nullptr);
        vkDestroyImageView(device_, depth_image_view_, nullptr);
        vkDestroyImage(device_, depth_image_, nullptr);
        vkFreeMemory(device_, depth_image_memory_, nullptr);

        for (auto framebuffer : swap_chain_framebuffers_) {
            vkDestroyFramebuffer(device_, framebuffer, nullptr);
        }

        vkFreeCommandBuffers(device_, command_pool_,
            static_cast<uint32_t>(command_buffers_.size()),
            command_buffers_.data());

        vkDestroyPipeline(device_, graphics_pipeline_, nullptr);
        vkDestroyPipelineLayout(device_, pipeline_layout_, nullptr);
        vkDestroyRenderPass(device_, render_pass_, nullptr);

        for (auto imageView : swap_chain_image_views_) {
            vkDestroyImageView(device_, imageView, nullptr);
        }

        vkDestroySwapchainKHR(device_, swap_chain_, nullptr);

        for (size_t i = 0; i < swap_chain_images_.size(); i++) {
            vkDestroyBuffer(device_, uniform_buffers_[i], nullptr);
            vkFreeMemory(device_, uniform_buffers_memory_[i], nullptr);
        }

        vkDestroyDescriptorPool(device_, descriptor_pool_, nullptr);
    }

    void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
    {
        if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
            std::cout << "Space" << std::endl;
    }

protected:
    // Variables
    const int width_, height_;
    GLFWwindow* window_;
    VkSurfaceKHR surface_;

    VkInstance vk_instance_;
    VkDebugUtilsMessengerEXT debug_messenger_;

    VkPhysicalDevice physical_device_ = VK_NULL_HANDLE;
    VkDevice device_;

    VkQueue graphics_queue_;
    VkQueue present_queue_;

    VkSwapchainKHR swap_chain_;
    std::vector<VkImage> swap_chain_images_;
    VkFormat swap_chain_image_format_;
    VkExtent2D swap_chain_extent_;
    std::vector<VkImageView> swap_chain_image_views_;
    std::vector<VkFramebuffer> swap_chain_framebuffers_;

    VkRenderPass render_pass_;
    VkDescriptorSetLayout descriptor_set_layout_;
    VkPipelineLayout pipeline_layout_;

    VkPipeline graphics_pipeline_;

    VkCommandPool command_pool_;
    std::vector<VkCommandBuffer> command_buffers_;

    std::vector<VkSemaphore> image_available_semaphores_;
    std::vector<VkSemaphore> render_finished_semaphores_;
    std::vector<VkFence> in_flight_fences_;
    std::vector<VkFence> images_in_flight_;
    size_t current_frame_ = 0;

    bool framebuffer_resized_ = false;

    std::vector<VkBuffer> uniform_buffers_;
    std::vector<VkDeviceMemory> uniform_buffers_memory_;

    VkDescriptorPool descriptor_pool_;
    std::vector<VkDescriptorSet> descriptor_sets_;

    uint32_t mip_levels_;

    VkImageView texture_image_view_;
    VkSampler texture_sampler_;

    VkImage depth_image_;
    VkDeviceMemory depth_image_memory_;
    VkImageView depth_image_view_;

    VkSampleCountFlagBits msaaSamples = VK_SAMPLE_COUNT_1_BIT;
    VkImage colorImage;
    VkDeviceMemory colorImageMemory;
    VkImageView colorImageView;
};
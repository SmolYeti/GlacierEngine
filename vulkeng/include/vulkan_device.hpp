#pragma once

#include "vulkan_window.hpp"

#include <optional>
#include <string>
#include <vector>

namespace vulkeng {
const std::vector<const char*> validationLayers = {
    "VK_LAYER_KHRONOS_validation"};

const std::vector<const char*> deviceExtensions = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME};

// Structs
struct SwapChainSupportDetails {
  VkSurfaceCapabilitiesKHR capabilities;
  std::vector<VkSurfaceFormatKHR> formats;
  std::vector<VkPresentModeKHR> presentModes;
};

struct QueueFamilyIndices {
  std::optional<uint32_t> graphicsFamily;
  std::optional<uint32_t> presentFamily;

  bool isComplete() {
    return graphicsFamily.has_value() && presentFamily.has_value();
  }
};

class VulkanDevice {
 public:
#ifdef NDEBUG
  const bool enableValidationLayers = false;
#else
  //const bool enableValidationLayers = true;
  const bool enableValidationLayers = false;
#endif

  VulkanDevice(VulkanWindow* window);
  ~VulkanDevice();

  VulkanDevice(const VulkanDevice&) = delete;
  VulkanDevice& operator=(const VulkanDevice&) = delete;
  VulkanDevice(VulkanDevice&&) = delete;
  VulkanDevice& operator=(VulkanDevice&&) = delete;

  VkDevice& device() { return device_; }
  VkCommandPool command_pool() { return command_pool_; }
  VkSurfaceKHR surface() { return surface_; }
  VkQueue graphics_queue() { return graphics_queue_; }
  VkQueue present_queue() { return present_queue_; }

  VkPhysicalDevice physical_device() { return physical_device_; }

  SwapChainSupportDetails GetSwapChainSupport() {
    return QuerySwapChainSupport(physical_device_);
  }
  QueueFamilyIndices FindPhysicalQueueFamilies() {
    return FindQueueFamilies(physical_device_);
  }
  uint32_t FindMemoryType(uint32_t typeFilter,
                          VkMemoryPropertyFlags properties);
  VkFormat FindSupportedFormat(const std::vector<VkFormat>& candidates,
                               VkImageTiling tiling,
                               VkFormatFeatureFlags features);

  // Buffer Helper Functions
  void CreateBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                    VkMemoryPropertyFlags properties, VkBuffer& buffer,
                    VkDeviceMemory& buffer_memory);
  VkCommandBuffer BeginSingleTimeCommands();
  void EndSingleTimeCommands(VkCommandBuffer commandBuffer);
  void CopyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size);
  void CopyBufferToImage(VkBuffer buffer, VkImage image, uint32_t width,
                         uint32_t height, uint32_t layerCount);

  void CreateImageWithInfo(const VkImageCreateInfo& imageInfo,
                           VkMemoryPropertyFlags properties, VkImage& image,
                           VkDeviceMemory& imageMemory);

 private:
  void CreateInstance();
  void SetupDebugMessenger();
  void CreateSurface();
  void PickPhysicalDevice();
  void CreateLogicalDevice();
  void CreateCommandPool();

  bool IsDeviceSuitable(VkPhysicalDevice device);
  QueueFamilyIndices FindQueueFamilies(VkPhysicalDevice device);
  SwapChainSupportDetails QuerySwapChainSupport(VkPhysicalDevice device);
  VkSampleCountFlagBits GetMaxUsableSampleCount();
  bool CheckValidationLayerSupport();
  void PopulateDebugMessengerCreateInfo(
      VkDebugUtilsMessengerCreateInfoEXT& createInfo);
  std::vector<const char*> GetRequiredExtensions();
  bool CheckDeviceExtensionSupport(VkPhysicalDevice device);

  void HasGflwRequiredInstanceExtensions();

  // Variables
  VulkanWindow* window_ = nullptr;

  VkInstance vk_instance_ = VK_NULL_HANDLE;
  VkDebugUtilsMessengerEXT debug_messenger_ = VK_NULL_HANDLE;

  VkPhysicalDevice physical_device_ = VK_NULL_HANDLE;
  VkDevice device_ = VK_NULL_HANDLE;

  VkSurfaceKHR surface_ = nullptr;
  VkCommandPool command_pool_ = nullptr;

  VkQueue graphics_queue_ = nullptr;
  VkQueue present_queue_ = nullptr;

  const std::vector<const char*> validationLayers = {
      "VK_LAYER_KHRONOS_validation"};
  const std::vector<const char*> deviceExtensions = {
      VK_KHR_SWAPCHAIN_EXTENSION_NAME};
};
}  // namespace vulkeng

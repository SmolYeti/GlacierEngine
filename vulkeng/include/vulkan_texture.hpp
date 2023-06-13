#pragma once

#include <string>

#include "vulkan_device.hpp"

namespace vulkeng {
class VulkanTexture {
public:
  VulkanTexture(VulkanDevice *device, const std::string &filepath);
  ~VulkanTexture();

  VulkanTexture(const VulkanTexture &) = delete;
  VulkanTexture &operator=(const VulkanTexture &) = delete;
  VulkanTexture(const VulkanTexture &&) = delete;
  VulkanTexture &operator=(const VulkanTexture &&) = delete;

  VkSampler sampler() { return sampler_; }
  VkImageView image_view() { return image_view_; }
  VkImageLayout image_layout() { return image_layout_; }

private:
  void TransitionImageLayout(VkImageLayout old_layout,
                             VkImageLayout new_layout);

  void GenerateMipMaps();

  int width_, height_;
  int mip_levels_;
  VulkanDevice *device_;
  VkImage image_;
  VkDeviceMemory image_memory_;
  VkImageView image_view_;
  VkSampler sampler_;
  VkFormat image_format_;
  VkImageLayout image_layout_;
};
} // namespace vulkeng

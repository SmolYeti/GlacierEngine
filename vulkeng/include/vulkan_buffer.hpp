#pragma once

#include "vulkan_device.hpp"

namespace vulkeng {

class VulkanBuffer {
 public:
  VulkanBuffer(VulkanDevice* device, VkDeviceSize instance_size,
               uint32_t instance_count, VkBufferUsageFlags usage_flags,
               VkMemoryPropertyFlags memory_property_flags,
               VkDeviceSize min_offfset_alignment = 1);
  ~VulkanBuffer();

  VulkanBuffer(const VulkanBuffer&) = delete;
  VulkanBuffer& operator=(const VulkanBuffer&) = delete;

  VkResult Map(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0);
  void Unmap();

  void WriteToBuffer(void* data, VkDeviceSize size = VK_WHOLE_SIZE,
                     VkDeviceSize offset = 0);
  VkResult Flush(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0);
  VkDescriptorBufferInfo DescriptorInfo(VkDeviceSize size = VK_WHOLE_SIZE,
                                        VkDeviceSize offset = 0);
  VkResult Invalidate(VkDeviceSize size = VK_WHOLE_SIZE,
                      VkDeviceSize offset = 0);

  void WriteToIndex(void* data, int index);
  VkResult FlushIndex(int index);
  VkDescriptorBufferInfo DescriptorInfoForIndex(int index);
  VkResult InvalidateIndex(int index);

  VkBuffer buffer() const { return buffer_; }
  void* mapped_memory() const { return mapped_; }
  uint32_t Instance_count() const { return instance_count_; }
  VkDeviceSize instance_size() const { return instance_size_; }
  VkDeviceSize alignment_size() const { return instance_size_; }
  VkBufferUsageFlags usage_flags() const { return usage_flags_; }
  VkMemoryPropertyFlags memory_property_flags() const {
    return memory_property_flags_;
  }
  VkDeviceSize buffer_size() const { return buffer_size_; }

 private:
  static VkDeviceSize Alignment(VkDeviceSize instance_size,
                                   VkDeviceSize min_offset_alignment);

  VulkanDevice* device_;
  void* mapped_ = nullptr;
  VkBuffer buffer_ = VK_NULL_HANDLE;
  VkDeviceMemory memory_ = VK_NULL_HANDLE;

  VkDeviceSize buffer_size_;
  uint32_t instance_count_;
  VkDeviceSize instance_size_;
  VkDeviceSize alignment_size_;
  VkBufferUsageFlags usage_flags_;
  VkMemoryPropertyFlags memory_property_flags_;
};

}  // namespace vulkeng
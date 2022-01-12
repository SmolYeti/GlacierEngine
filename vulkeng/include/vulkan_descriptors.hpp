#pragma once

#include "vulkan_device.hpp"

// std
#include <memory>
#include <unordered_map>
#include <vector>

namespace vulkeng {

class VulkanDescriptorSetLayout {
 public:
  class Builder {
   public:
    Builder(VulkanDevice *device) : device_{device} {}

    Builder &AddBinding(uint32_t binding, VkDescriptorType descriptorType,
                        VkShaderStageFlags stageFlags, uint32_t count = 1);
    std::unique_ptr<VulkanDescriptorSetLayout> Build() const;

   private:
    VulkanDevice *device_;
    std::unordered_map<uint32_t, VkDescriptorSetLayoutBinding> bindings_{};
  };

  VulkanDescriptorSetLayout(
      VulkanDevice *device,
      std::unordered_map<uint32_t, VkDescriptorSetLayoutBinding> bindings);
  ~VulkanDescriptorSetLayout();
  VulkanDescriptorSetLayout(const VulkanDescriptorSetLayout &) = delete;
  VulkanDescriptorSetLayout &operator=(const VulkanDescriptorSetLayout &) =
      delete;

  VkDescriptorSetLayout DescriptorSetLayout() const {
    return descriptor_set_layout_;
  }

 private:
  VulkanDevice *device_;
  VkDescriptorSetLayout descriptor_set_layout_;
  std::unordered_map<uint32_t, VkDescriptorSetLayoutBinding> bindings_;

  friend class VulkanDescriptorWriter;
};

class VulkanDescriptorPool {
 public:
  class Builder {
   public:
    Builder(VulkanDevice *device) : device_{device} {}

    Builder &AddPoolSize(VkDescriptorType descriptorType, uint32_t count);
    Builder &SetPoolFlags(VkDescriptorPoolCreateFlags flags);
    Builder &SetMaxSets(uint32_t count);
    std::unique_ptr<VulkanDescriptorPool> Build() const;

   private:
    VulkanDevice *device_;
    std::vector<VkDescriptorPoolSize> pool_sizes_{};
    uint32_t max_sets_ = 1000;
    VkDescriptorPoolCreateFlags pool_flags_ = 0;
  };

  VulkanDescriptorPool(VulkanDevice *device, uint32_t max_sets,
                       VkDescriptorPoolCreateFlags pool_flags,
                       const std::vector<VkDescriptorPoolSize> &pool_sizes);
  ~VulkanDescriptorPool();
  VulkanDescriptorPool(const VulkanDescriptorPool &) = delete;
  VulkanDescriptorPool &operator=(const VulkanDescriptorPool &) = delete;

  bool AllocateDescriptorSet(const VkDescriptorSetLayout descriptorSetLayout,
                          VkDescriptorSet &descriptor) const;

  void FreeDescriptors(std::vector<VkDescriptorSet> &descriptors) const;

  void ResetPool();

 private:
  VulkanDevice *device_;
  VkDescriptorPool descriptor_pool_;

  friend class VulkanDescriptorWriter;
};

class VulkanDescriptorWriter {
 public:
  VulkanDescriptorWriter(VulkanDescriptorSetLayout &setLayout,
                         VulkanDescriptorPool &pool);

  VulkanDescriptorWriter &WriteBuffer(uint32_t binding,
                                   VkDescriptorBufferInfo *bufferInfo);
  VulkanDescriptorWriter &WriteImage(uint32_t binding,
                                  VkDescriptorImageInfo *imageInfo);

  bool Build(VkDescriptorSet &set);
  void Overwrite(VkDescriptorSet &set);

 private:
  VulkanDescriptorSetLayout &set_layout_;
  VulkanDescriptorPool &pool_;
  std::vector<VkWriteDescriptorSet> writes_;
};
}  // namespace vulkeng
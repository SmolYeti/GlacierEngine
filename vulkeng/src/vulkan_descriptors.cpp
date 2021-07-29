#include "vulkeng/include/vulkan_descriptors.hpp"

// std
#include <cassert>
#include <stdexcept>

namespace vulkeng {

// *************** Descriptor Set Layout Builder *********************

VulkanDescriptorSetLayout::Builder &
VulkanDescriptorSetLayout::Builder::AddBinding(uint32_t binding,
                                               VkDescriptorType descriptorType,
                                               VkShaderStageFlags stageFlags,
                                               uint32_t count) {
  assert(bindings_.count(binding) == 0 && "Binding already in use");
  VkDescriptorSetLayoutBinding layoutBinding{};
  layoutBinding.binding = binding;
  layoutBinding.descriptorType = descriptorType;
  layoutBinding.descriptorCount = count;
  layoutBinding.stageFlags = stageFlags;
  bindings_[binding] = layoutBinding;
  return *this;
}

std::unique_ptr<VulkanDescriptorSetLayout>
VulkanDescriptorSetLayout::Builder::Build() const {
  return std::make_unique<VulkanDescriptorSetLayout>(device_, bindings_);
}

// *************** Descriptor Set Layout *********************
VulkanDescriptorSetLayout::VulkanDescriptorSetLayout(
    VulkanDevice *device,
    std::unordered_map<uint32_t, VkDescriptorSetLayoutBinding> bindings)
    : device_{device}, bindings_{bindings} {
  std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings{};
  for (auto kv : bindings) {
    setLayoutBindings.push_back(kv.second);
  }

  VkDescriptorSetLayoutCreateInfo descriptorSetLayoutInfo{};
  descriptorSetLayoutInfo.sType =
      VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  descriptorSetLayoutInfo.bindingCount =
      static_cast<uint32_t>(setLayoutBindings.size());
  descriptorSetLayoutInfo.pBindings = setLayoutBindings.data();

  if (vkCreateDescriptorSetLayout(device_->device(), &descriptorSetLayoutInfo,
                                  nullptr,
                                  &descriptor_set_layout_) != VK_SUCCESS) {
    throw std::runtime_error("failed to create descriptor set layout!");
  }
}

VulkanDescriptorSetLayout::~VulkanDescriptorSetLayout() {
  vkDestroyDescriptorSetLayout(device_->device(), descriptor_set_layout_,
                               nullptr);
}

// *************** Descriptor Pool Builder *********************

VulkanDescriptorPool::Builder &VulkanDescriptorPool::Builder::AddPoolSize(
    VkDescriptorType descriptor_type, uint32_t count) {
  pool_sizes_.push_back({descriptor_type, count});
  return *this;
}

VulkanDescriptorPool::Builder &VulkanDescriptorPool::Builder::SetPoolFlags(
    VkDescriptorPoolCreateFlags flags) {
  pool_flags_ = flags;
  return *this;
}

VulkanDescriptorPool::Builder &VulkanDescriptorPool::Builder::SetMaxSets(
    uint32_t count) {
  max_sets_ = count;
  return *this;
}

std::unique_ptr<VulkanDescriptorPool> VulkanDescriptorPool::Builder::Build()
    const {
  return std::make_unique<VulkanDescriptorPool>(device_, max_sets_, pool_flags_,
                                                pool_sizes_);
}

// *************** Descriptor Pool *********************

VulkanDescriptorPool::VulkanDescriptorPool(
    VulkanDevice *device, uint32_t max_sets,
    VkDescriptorPoolCreateFlags pool_flags,
    const std::vector<VkDescriptorPoolSize> &pool_sizes)
    : device_{device} {
  VkDescriptorPoolCreateInfo descriptor_pool_info{};
  descriptor_pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  descriptor_pool_info.poolSizeCount = static_cast<uint32_t>(pool_sizes.size());
  descriptor_pool_info.pPoolSizes = pool_sizes.data();
  descriptor_pool_info.maxSets = max_sets;
  descriptor_pool_info.flags = pool_flags;

  if (vkCreateDescriptorPool(device_->device(), &descriptor_pool_info, nullptr,
                             &descriptor_pool_) != VK_SUCCESS) {
    throw std::runtime_error("failed to create descriptor pool!");
  }
}

VulkanDescriptorPool::~VulkanDescriptorPool() {
  vkDestroyDescriptorPool(device_->device(), descriptor_pool_, nullptr);
}

bool VulkanDescriptorPool::AllocateDescriptorSet(
    const VkDescriptorSetLayout descriptor_set_layout,
    VkDescriptorSet &descriptor) const {
  VkDescriptorSetAllocateInfo alloc_info{};
  alloc_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  alloc_info.descriptorPool = descriptor_pool_;
  alloc_info.pSetLayouts = &descriptor_set_layout;
  alloc_info.descriptorSetCount = 1;

  // Might want to create a "DescriptorPoolManager" class that handles this
  // case, and builds a new pool whenever an old pool fills up. But this is
  // beyond our current scope
  if (vkAllocateDescriptorSets(device_->device(), &alloc_info, &descriptor) !=
      VK_SUCCESS) {
    return false;
  }
  return true;
}

void VulkanDescriptorPool::FreeDescriptors(
    std::vector<VkDescriptorSet> &descriptors) const {
  vkFreeDescriptorSets(device_->device(), descriptor_pool_,
                       static_cast<uint32_t>(descriptors.size()),
                       descriptors.data());
}

void VulkanDescriptorPool::ResetPool() {
  vkResetDescriptorPool(device_->device(), descriptor_pool_, 0);
}

// *************** Descriptor Writer *********************

VulkanDescriptorWriter::VulkanDescriptorWriter(
    VulkanDescriptorSetLayout &set_layout, VulkanDescriptorPool &pool)
    : set_layout_{set_layout}, pool_{pool} {}

VulkanDescriptorWriter &VulkanDescriptorWriter::WriteBuffer(
    uint32_t binding, VkDescriptorBufferInfo *bufferInfo) {
  assert(set_layout_.bindings_.count(binding) == 1 &&
         "Layout does not contain specified binding");

  auto &bindingDescription = set_layout_.bindings_[binding];

  assert(bindingDescription.descriptorCount == 1 &&
         "Binding single descriptor info, but binding expects multiple");

  VkWriteDescriptorSet write{};
  write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  write.descriptorType = bindingDescription.descriptorType;
  write.dstBinding = binding;
  write.pBufferInfo = bufferInfo;
  write.descriptorCount = 1;

  writes_.push_back(write);
  return *this;
}

VulkanDescriptorWriter &VulkanDescriptorWriter::WriteImage(
    uint32_t binding, VkDescriptorImageInfo *imageInfo) {
  assert(set_layout_.bindings_.count(binding) == 1 &&
         "Layout does not contain specified binding");

  auto &bindingDescription = set_layout_.bindings_[binding];

  assert(bindingDescription.descriptorCount == 1 &&
         "Binding single descriptor info, but binding expects multiple");

  VkWriteDescriptorSet write{};
  write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  write.descriptorType = bindingDescription.descriptorType;
  write.dstBinding = binding;
  write.pImageInfo = imageInfo;
  write.descriptorCount = 1;

  writes_.push_back(write);
  return *this;
}

bool VulkanDescriptorWriter::Build(VkDescriptorSet &set) {
  bool success =
      pool_.AllocateDescriptorSet(set_layout_.DescriptorSetLayout(), set);
  if (!success) {
    return false;
  }
  Overwrite(set);
  return true;
}

void VulkanDescriptorWriter::Overwrite(VkDescriptorSet &set) {
  for (auto &write : writes_) {
    write.dstSet = set;
  }
  vkUpdateDescriptorSets(pool_.device_->device(), static_cast<uint32_t>(writes_.size()),
                         writes_.data(), 0, nullptr);
}

}  // namespace vulkeng
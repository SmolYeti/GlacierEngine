/*
 * Encapsulates a vulkan buffer
 *
 * Initially based off VulkanBuffer by Sascha Willems -
 * https://github.com/SaschaWillems/Vulkan/blob/master/base/VulkanBuffer.h
 */

#include "vulkeng/include/vulkan_buffer.hpp"

// std
#include <cassert>
#include <cstring>

namespace vulkeng {

/**
 * Returns the minimum instance size required to be compatible with devices
 * minOffsetAlignment
 *
 * @param instanceSize The size of an instance
 * @param minOffsetAlignment The minimum required alignment, in bytes, for the
 * offset member (eg minUniformBufferOffsetAlignment)
 *
 * @return VkResult of the buffer mapping call
 */
VkDeviceSize VulkanBuffer::Alignment(VkDeviceSize instanceSize,
                                     VkDeviceSize minOffsetAlignment) {
    if (minOffsetAlignment > 0) {
        return (instanceSize + minOffsetAlignment - 1) & ~(minOffsetAlignment - 1);
    }
    return instanceSize;
}

VulkanBuffer::VulkanBuffer(VulkanDevice *device, VkDeviceSize instance_size,
                           uint32_t instance_count,
                           VkBufferUsageFlags usage_flags,
                           VkMemoryPropertyFlags memory_property_flags,
                           VkDeviceSize min_offset_alignment)
    : device_{device},
      instance_size_{instance_size},
      instance_count_{instance_count},
      usage_flags_{usage_flags},
      memory_property_flags_{memory_property_flags} {
    alignment_size_ = Alignment(instance_size_, min_offset_alignment);
    buffer_size_ = alignment_size_ * instance_count_;
    device_->CreateBuffer(buffer_size_, usage_flags_, memory_property_flags_, buffer_,
                          memory_);
}

VulkanBuffer::~VulkanBuffer() {
    Unmap();
    vkDestroyBuffer(device_->device(), buffer_, nullptr);
    vkFreeMemory(device_->device(), memory_, nullptr);
}

/**
 * Map a memory range of this buffer. If successful, mapped points to the
 * specified buffer range.
 *
 * @param size (Optional) Size of the memory range to map. Pass VK_WHOLE_SIZE to
 * map the complete buffer range.
 * @param offset (Optional) Byte offset from beginning
 *
 * @return VkResult of the buffer mapping call
 */
VkResult VulkanBuffer::Map(VkDeviceSize size, VkDeviceSize offset) {
    assert(buffer_ && memory_ && "Called map on buffer before create");
    return vkMapMemory(device_->device(), memory_, offset, size, 0, &mapped_);
}

/**
 * Unmap a mapped memory range
 *
 * @note Does not return a result as vkUnmapMemory can't fail
 */
void VulkanBuffer::Unmap() {
    if (mapped_) {
        vkUnmapMemory(device_->device(), memory_);
        mapped_ = nullptr;
    }
}

/**
 * Copies the specified data to the mapped buffer. Default value writes whole
 * buffer range
 *
 * @param data Pointer to the data to copy
 * @param size (Optional) Size of the data to copy. Pass VK_WHOLE_SIZE to flush
 * the complete buffer range.
 * @param offset (Optional) Byte offset from beginning of mapped region
 *
 */
void VulkanBuffer::WriteToBuffer(void *data, VkDeviceSize size,
                                 VkDeviceSize offset) {
    assert(mapped_ && "Cannot copy to unmapped buffer");

    if (size == VK_WHOLE_SIZE) {
        memcpy(mapped_, data, static_cast<size_t>(buffer_size_));
    } else {
        char *mem_offset = (char *)mapped_;
        mem_offset += offset;
        memcpy(mem_offset, data, static_cast<size_t>(size));
    }
}

/**
 * Flush a memory range of the buffer to make it visible to the device
 *
 * @note Only required for non-coherent memory
 *
 * @param size (Optional) Size of the memory range to flush. Pass VK_WHOLE_SIZE
 * to flush the complete buffer range.
 * @param offset (Optional) Byte offset from beginning
 *
 * @return VkResult of the flush call
 */
VkResult VulkanBuffer::Flush(VkDeviceSize size, VkDeviceSize offset) {
    VkMappedMemoryRange mapped_range = {};
    mapped_range.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    mapped_range.memory = memory_;
    mapped_range.offset = offset;
    mapped_range.size = size;
    return vkFlushMappedMemoryRanges(device_->device(), 1, &mapped_range);
}

/**
 * Invalidate a memory range of the buffer to make it visible to the host
 *
 * @note Only required for non-coherent memory
 *
 * @param size (Optional) Size of the memory range to invalidate. Pass
 * VK_WHOLE_SIZE to invalidate the complete buffer range.
 * @param offset (Optional) Byte offset from beginning
 *
 * @return VkResult of the invalidate call
 */
VkResult VulkanBuffer::Invalidate(VkDeviceSize size, VkDeviceSize offset) {
    VkMappedMemoryRange mapped_range = {};
    mapped_range.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    mapped_range.memory = memory_;
    mapped_range.offset = offset;
    mapped_range.size = size;
    return vkInvalidateMappedMemoryRanges(device_->device(), 1, &mapped_range);
}

/**
 * Create a buffer info descriptor
 *
 * @param size (Optional) Size of the memory range of the descriptor
 * @param offset (Optional) Byte offset from beginning
 *
 * @return VkDescriptorBufferInfo of specified offset and range
 */
VkDescriptorBufferInfo VulkanBuffer::DescriptorInfo(VkDeviceSize size,
                                                    VkDeviceSize offset) {
    return VkDescriptorBufferInfo{
        buffer_,
        offset,
        size,
    };
}

/**
 * Copies "instanceSize" bytes of data to the mapped buffer at an offset of
 * index * alignmentSize
 *
 * @param data Pointer to the data to copy
 * @param index Used in offset calculation
 *
 */
void VulkanBuffer::WriteToIndex(void *data, int index) {
    WriteToBuffer(data, instance_size_, index * alignment_size_);
}

/**
 *  Flush the memory range at index * alignmentSize of the buffer to make it
 * visible to the device
 *
 * @param index Used in offset calculation
 *
 */
VkResult VulkanBuffer::FlushIndex(int index) {
    return Flush(alignment_size_, index * alignment_size_);
}

/**
 * Create a buffer info descriptor
 *
 * @param index Specifies the region given by index * alignmentSize
 *
 * @return VkDescriptorBufferInfo for instance at index
 */
VkDescriptorBufferInfo VulkanBuffer::DescriptorInfoForIndex(int index) {
    return DescriptorInfo(alignment_size_, index * alignment_size_);
}

/**
 * Invalidate a memory range of the buffer to make it visible to the host
 *
 * @note Only required for non-coherent memory
 *
 * @param index Specifies the region to invalidate: index * alignmentSize
 *
 * @return VkResult of the invalidate call
 */
VkResult VulkanBuffer::InvalidateIndex(int index) {
    return Invalidate(alignment_size_, index * alignment_size_);
}

}  // namespace vulkeng
#include "vulkeng/include/vulkan_renderer.hpp"

// STD
#include <array>
#include <cassert>
#include <stdexcept>

namespace vulkeng {
    VulkanRenderer::VulkanRenderer(VulkanWindow* window, VulkanDevice* device) :window_(window), device_(device) {
        RecreateSwapChain();
        CreateCommandBuffers();
    }

    VulkanRenderer::~VulkanRenderer() {
        FreeCommandBuffers();
    }

    void VulkanRenderer::CreateCommandBuffers() {
        // Create the command buffers
        command_buffers_.resize(VulkanSwapChain::MAX_FRAMES_IN_FLIGHT);

        VkCommandBufferAllocateInfo allocInfo = {};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandPool = device_->command_pool();
        allocInfo.commandBufferCount = static_cast<uint32_t>(command_buffers_.size());

        if (vkAllocateCommandBuffers(device_->device(), &allocInfo,
            command_buffers_.data()) != VK_SUCCESS) {
            throw std::runtime_error("Failed to Allocate Command Buffers!");
        }
    }

    void VulkanRenderer::FreeCommandBuffers() {
        vkFreeCommandBuffers(device_->device(), device_->command_pool(),
            static_cast<uint32_t>(command_buffers_.size()), command_buffers_.data());
        command_buffers_.clear();
    }

    void VulkanRenderer::RecreateSwapChain() {
        auto window_extent = window_->extent();
        while (window_extent.width == 0 || window_extent.height == 0) {
            window_extent = window_->extent();
            glfwWaitEvents();
        }

        vkDeviceWaitIdle(device_->device());

        if (swap_chain_ == nullptr) {
            swap_chain_ = std::make_unique<VulkanSwapChain>(device_, window_extent);
        }
        else {
            std::shared_ptr<VulkanSwapChain> old_swap_chain = std::move(swap_chain_);
            swap_chain_ = std::make_unique<VulkanSwapChain>(device_, window_extent, old_swap_chain);

            if (!old_swap_chain->CompareSwapFormats(swap_chain_.get())) {
                throw std::runtime_error("Swap chain image(or depth) format has changed");
            }
        }
    }

    VkCommandBuffer VulkanRenderer::BeginFrame() {
        assert(!is_frame_started_ && "Can't call BeginFrame() with frame in progress!");

        VkResult result = swap_chain_->AcquireNextImage(&current_image_index_);

        if (result == VK_ERROR_OUT_OF_DATE_KHR) {
            RecreateSwapChain();
            return nullptr;
        }
        else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
            throw std::runtime_error("failed to acquire swap chain image!");
        }

        is_frame_started_ = true;

        auto command_bufffer = GetCurrentCommandBuffer();

        // Begin the command buffer
        VkCommandBufferBeginInfo begin_info = {};
        begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        begin_info.flags = 0;                   // Optional
        begin_info.pInheritanceInfo = nullptr;  // Optional

        if (vkBeginCommandBuffer(command_bufffer, &begin_info) != VK_SUCCESS) {
            throw std::runtime_error("Failed to Begin Recording Command Buffer!");
        }

        return command_bufffer;
    }

    void VulkanRenderer::EndFrame() {
        assert(is_frame_started_ && "Can't call EndFrame() when frame is not in progress!");

        auto command_buffer = GetCurrentCommandBuffer();

        // End the command buffer
        if (vkEndCommandBuffer(command_buffer) != VK_SUCCESS) {
            throw std::runtime_error("Failed to Record Command Buffer!");
        }

        VkResult result = swap_chain_->SubmitCommandBuffers(&command_buffer, &current_image_index_);
        if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || window_->WasFrameBufferResized()) {
            window_->ResetFrameBufferResized();
            RecreateSwapChain();
        }
        else if (result != VK_SUCCESS) {
            throw std::runtime_error("failed to present swap chain image!");
        }

        is_frame_started_ = false;
        current_frame_index_ = (current_frame_index_ + 1) % VulkanSwapChain::MAX_FRAMES_IN_FLIGHT;
    }

    void VulkanRenderer::BeginSwapChainRenderPass(VkCommandBuffer command_buffer) {
        assert(is_frame_started_ && "Can't call BeginSwapChainRenderPass() when frame is not in progress!");
        assert(command_buffer == GetCurrentCommandBuffer() && "Can't begin render pass on command buffer from a different frame");

        // Begin a render pass for each command buffer
        VkRenderPassBeginInfo render_pass_info = {};
        render_pass_info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        render_pass_info.renderPass = swap_chain_->render_pass();
        render_pass_info.framebuffer =
            swap_chain_->framebuffer(current_image_index_);

        render_pass_info.renderArea.offset = { 0, 0 };
        render_pass_info.renderArea.extent = swap_chain_->extent();

        std::array<VkClearValue, 2> clear_values{};
        clear_values[0].color = { 0.01f, 0.01f, 0.01f, 1.0f };
        clear_values[1].depthStencil = { 1.0f, 0 };

        render_pass_info.clearValueCount = static_cast<uint32_t>(clear_values.size());
        render_pass_info.pClearValues = clear_values.data();

        vkCmdBeginRenderPass(command_buffer, &render_pass_info, VK_SUBPASS_CONTENTS_INLINE);

        VkViewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<float>(swap_chain_->extent().width);
        viewport.height = static_cast<float>(swap_chain_->extent().height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        VkRect2D scissor{ {0, 0}, swap_chain_->extent() };
        vkCmdSetViewport(command_buffer, 0, 1, &viewport);
        vkCmdSetScissor(command_buffer, 0, 1, &scissor);
    }

    void VulkanRenderer::EndSwapChainRenderPass(VkCommandBuffer command_buffer) {
        assert(is_frame_started_ && "Can't call EndSwapChainRenderPass() when frame is not in progress!");
        assert(command_buffer == GetCurrentCommandBuffer() && "Can't end render pass on command buffer from a different frame");

        // End the render pass
        vkCmdEndRenderPass(command_buffer);
    }
}
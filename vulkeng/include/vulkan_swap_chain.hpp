#pragma once

//#include <vulkan/vulkan.h>

#include <memory>
#include <vector>

#include "vulkan_device.hpp"

namespace vulkeng {
class VulkanSwapChain {
   public:
    static constexpr uint32_t MAX_FRAMES_IN_FLIGHT = 2;

    VulkanSwapChain(VulkanDevice* device, VkExtent2D extent);
    VulkanSwapChain(VulkanDevice* device, VkExtent2D extent,
                    std::shared_ptr<VulkanSwapChain> previous);
    ~VulkanSwapChain();

    VulkanSwapChain(const VulkanSwapChain&) = delete;
    VulkanSwapChain& operator=(const VulkanSwapChain&) = delete;

    VkFramebuffer framebuffer(int index) const {
        return swap_chain_framebuffers_[index];
    }
    VkRenderPass render_pass() const { return render_pass_; }
    VkImageView image_view(int index) const {
        return swap_chain_image_views_[index];
    }
    size_t image_count() const { return swap_chain_images_.size(); }
    VkFormat image_format() const { return swap_chain_image_format_; }
    VkExtent2D extent() const { return swap_chain_extent_; }
    uint32_t width() const { return swap_chain_extent_.width; }
    uint32_t height() const { return swap_chain_extent_.height; }

    float ExtentAspectRatio() const {
        return static_cast<float>(swap_chain_extent_.width) /
               static_cast<float>(swap_chain_extent_.height);
    }
    VkFormat FindDepthFormat();

    VkResult AcquireNextImage(uint32_t* image_index);
    VkResult SubmitCommandBuffers(const VkCommandBuffer* buffers,
                                  uint32_t* image_index);

    bool CompareSwapFormats(const VulkanSwapChain* swap_chain) const {
        return swap_chain->swap_chain_depth_format_ ==
                   this->swap_chain_depth_format_ &&
               swap_chain->swap_chain_image_format_ ==
                   this->swap_chain_image_format_;
    }

   private:
    void Init();
    void CreateSwapChain();
    void CreateImageViews();
    void CreateRenderPass();
    void CreateDepthResources();
    void CreateFramebuffers();
    void CreateSyncObjects();

    // Helper Functions
    VkSurfaceFormatKHR ChooseSwapSurfaceFormat(
        const std::vector<VkSurfaceFormatKHR>& available_formats);
    VkPresentModeKHR ChooseSwapPresentMode(
        const std::vector<VkPresentModeKHR>& available_present_modes);
    VkExtent2D ChooseSwapExtent(const VkSurfaceCapabilitiesKHR& capabilities);

   private:
    // Variables
    VkFormat swap_chain_image_format_{};
    VkFormat swap_chain_depth_format_{};
    VkExtent2D swap_chain_extent_{};

    std::vector<VkFramebuffer> swap_chain_framebuffers_;
    VkRenderPass render_pass_ = nullptr;

    std::vector<VkImage> depth_images_;
    std::vector<VkDeviceMemory> depth_image_memories_;
    std::vector<VkImageView> depth_image_views_;
    std::vector<VkImage> swap_chain_images_;
    std::vector<VkImageView> swap_chain_image_views_;

    VulkanDevice* device_ = nullptr;
    VkExtent2D window_extent_{};

    VkSwapchainKHR swap_chain_ = nullptr;
    std::shared_ptr<VulkanSwapChain> old_swap_chain_;

    std::vector<VkSemaphore> image_available_semaphores_;
    std::vector<VkSemaphore> render_finished_semaphores_;
    std::vector<VkFence> in_flight_fences_;
    std::vector<VkFence> images_in_flight_;
    size_t current_frame_ = 0;
};
}  // namespace vulkeng
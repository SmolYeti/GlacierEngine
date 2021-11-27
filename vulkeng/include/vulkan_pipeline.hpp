#pragma once

#include "vulkan_device.hpp"

// STD
#include <string>
#include <vector>

namespace vulkeng {
    class VulkanDevice;

    struct PipelineConfigInfo {
        PipelineConfigInfo() = default;
        PipelineConfigInfo(const PipelineConfigInfo&) = delete;
        PipelineConfigInfo& operator=(const PipelineConfigInfo&) = delete;

        VkPipelineViewportStateCreateInfo viewport_info;
        VkPipelineInputAssemblyStateCreateInfo input_assembly_info;
        VkPipelineRasterizationStateCreateInfo rasterization_info;
        VkPipelineMultisampleStateCreateInfo multisample_info;
        VkPipelineColorBlendAttachmentState color_blend_attachment;
        VkPipelineColorBlendStateCreateInfo color_blend_info;
        VkPipelineDepthStencilStateCreateInfo depth_stencil_info;
        std::vector<VkDynamicState> dynamic_state_enables;
        VkPipelineDynamicStateCreateInfo dynamic_state_info;
        VkPipelineLayout pipeline_layout = nullptr;
        VkRenderPass render_pass = nullptr;
        uint32_t subpass = 0;
    };

    class VulkanPipeline {
    public:
        VulkanPipeline(VulkanDevice* device,      //
            const std::string& vertex_filepath,   //
            const std::string& fragment_filepath, //
            const PipelineConfigInfo& config);    //
        ~VulkanPipeline();

        VulkanPipeline(const VulkanPipeline&) = delete;
        VulkanPipeline& operator=(const VulkanPipeline&) = delete;

        void Bind(VkCommandBuffer command_buffer);

        static void DefaultPipelineConfigInfo(PipelineConfigInfo& config_info);

    private:
        static std::vector<char> ReadFile(const std::string& file_path);

        void CreateDescriptorSetLayout();

        void CreateGraphicsPipeline(const std::string& vertex_filepath,
            const std::string& fragment_filepath,
            const PipelineConfigInfo& config);

        void CreateShaderModule(const std::vector<char>& code, VkShaderModule* shader_module);

        // Variables
        VulkanDevice* device_ = nullptr;
        VkPipeline graphics_pipeline_ = nullptr;
        VkShaderModule vert_shader_module_ = nullptr;
        VkShaderModule frag_shader_module_ = nullptr;
    };
}
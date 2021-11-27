#include "vulkeng/include/simple_render_system.hpp"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

// STD
#include <stdexcept>

// TODO - Don't make these hardcoded paths
const std::string VERTEX_PATH = "C:/Users/WJSSn/Documents/GitRepos/VulkanGrowProject/shaders/vert.spv";
const std::string FRAGMENT_PATH = "C:/Users/WJSSn/Documents/GitRepos/VulkanGrowProject/shaders/frag.spv";

namespace vulkeng {
    struct SimplePushConstantData {
        glm::mat4 transform{ 1.f };
        alignas(16) glm::vec3 color;
    };

    SimpleRenderSystem::SimpleRenderSystem(VulkanDevice* device, VkRenderPass render_pass) : device_(device) {
        CreatePipelineLayout();
        CreatePipeline(render_pass);
    }

    SimpleRenderSystem::~SimpleRenderSystem() {
        vkDestroyPipelineLayout(device_->device(), pipeline_layout_, nullptr);
    }

    void SimpleRenderSystem::CreatePipelineLayout() {
        VkPushConstantRange push_constant_range;
        push_constant_range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        push_constant_range.offset = 0;
        push_constant_range.size = sizeof(SimplePushConstantData);

        VkPipelineLayoutCreateInfo pipeline_layout_info{};
        pipeline_layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pipeline_layout_info.setLayoutCount = 0;
        pipeline_layout_info.pSetLayouts = nullptr;
        pipeline_layout_info.pushConstantRangeCount = 1;
        pipeline_layout_info.pPushConstantRanges = &push_constant_range;

        if (vkCreatePipelineLayout(device_->device(), &pipeline_layout_info, nullptr, &pipeline_layout_) != VK_SUCCESS) {
            throw std::runtime_error("Failed to Create Pipeline Layout");
        }
    }

    void SimpleRenderSystem::CreatePipeline(VkRenderPass render_pass) {
        assert(pipeline_layout_ != nullptr && "Can't create pipeline before pipeline layout");

        PipelineConfigInfo pipeline_config = {};
        VulkanPipeline::DefaultPipelineConfigInfo(pipeline_config);
        pipeline_config.render_pass = render_pass;
        pipeline_config.pipeline_layout = pipeline_layout_;

        pipeline_ = std::make_unique<VulkanPipeline>(device_, VERTEX_PATH, FRAGMENT_PATH, pipeline_config);
    }

    void SimpleRenderSystem::RenderGameObjects(VkCommandBuffer command_buffer, std::vector<VulkanGameObject>& game_objects) {
        pipeline_->Bind(command_buffer);

        for (auto& object : game_objects) {
            object.transform_.rotation.y = glm::mod(object.transform_.rotation.y + 0.01f, glm::two_pi<float>());
            object.transform_.rotation.x = glm::mod(object.transform_.rotation.x + 0.005f, glm::two_pi<float>());

            // PUSH CONSTANTS Video 10:
            SimplePushConstantData push{};
            push.color = object.color_;
            push.transform = object.transform_.mat4();

            vkCmdPushConstants(command_buffer, pipeline_layout_,
                VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(SimplePushConstantData), &push);

            object.model_->Bind(command_buffer);
            object.model_->Draw(command_buffer);
        }
    }
}
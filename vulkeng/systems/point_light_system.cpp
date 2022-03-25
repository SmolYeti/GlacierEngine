#include "vulkeng/systems/point_light_system.hpp"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

// STD
#include <filesystem>
#include <iostream>
#include <stdexcept>

// TODO - Don't make these hardcoded paths
const std::string PL_VERTEX_PATH =
    "C:/Users/WJSSn/Documents/GitRepos/VulkanGrowProject/shaders/"
    "point_light.vert.spv";
const std::string PL_FRAGMENT_PATH =
    "C:/Users/WJSSn/Documents/GitRepos/VulkanGrowProject/shaders/"
    "point_light.frag.spv";

namespace vulkeng {

struct PointLightPushContant {
    glm::vec4 position{};
    glm::vec4 color;
    float radius{};
};

PointLightSystem::PointLightSystem(VulkanDevice* device,
                                   VkRenderPass render_pass,
                                   VkDescriptorSetLayout global_set_layout)
    : device_(device) {
    std::cout << std::filesystem::current_path() << std::endl;
    CreatePipelineLayout(global_set_layout);
    CreatePipeline(render_pass);
}

PointLightSystem::~PointLightSystem() {
    vkDestroyPipelineLayout(device_->device(), pipeline_layout_, nullptr);
}

void PointLightSystem::CreatePipelineLayout(
    VkDescriptorSetLayout global_set_layout) {
    VkPushConstantRange push_constant_range;
    push_constant_range.stageFlags =
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    push_constant_range.offset = 0;
    push_constant_range.size = sizeof(PointLightPushContant);

    std::vector<VkDescriptorSetLayout> descriptor_set_layouts{global_set_layout};

    VkPipelineLayoutCreateInfo pipeline_layout_info{};
    pipeline_layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipeline_layout_info.setLayoutCount =
        static_cast<uint32_t>(descriptor_set_layouts.size());
    pipeline_layout_info.pSetLayouts = descriptor_set_layouts.data();
    pipeline_layout_info.pushConstantRangeCount = 1;
    pipeline_layout_info.pPushConstantRanges = &push_constant_range;

    if (vkCreatePipelineLayout(device_->device(), &pipeline_layout_info, nullptr,
                               &pipeline_layout_) != VK_SUCCESS) {
        throw std::runtime_error("Failed to Create Pipeline Layout");
    }
}

void PointLightSystem::CreatePipeline(VkRenderPass render_pass) {
    assert(pipeline_layout_ != nullptr &&
           "Can't create pipeline before pipeline layout");

    PipelineConfigInfo pipeline_config = {};
    VulkanPipeline::DefaultPipelineConfigInfo(pipeline_config);
    pipeline_config.attribute_descriptions.clear();
    pipeline_config.binding_descriptions.clear();
    pipeline_config.render_pass = render_pass;
    pipeline_config.pipeline_layout = pipeline_layout_;

    pipeline_ = std::make_unique<VulkanPipeline>(
        device_, PL_VERTEX_PATH, PL_FRAGMENT_PATH, pipeline_config);
}

void PointLightSystem::Update(const FrameInfo& frame_info, GlobalUbo& ubo) {
    auto rotate_light =
        glm::rotate(glm::mat4(1.f), frame_info.frame_time / 10.0f, {0.0f, -1.0f, 0.0f});

    int light_index = 0;
    for (auto& key_value : frame_info.game_objects) {
        auto& obj = key_value.second;
        if (obj.point_light.has_value()) {
            assert(light_index < MAX_LIGHTS &&
                   "Point lights exceed maximum specified");

            // update position
            obj.transform_.translation =
                glm::vec3(rotate_light * glm::vec4(obj.transform_.translation, 1.f));

            // copy light to ubo
            ubo.lights[light_index].position =
                glm::vec4(obj.transform_.translation, 1.f);
            ubo.lights[light_index].color =
                glm::vec4(obj.color_, obj.point_light.value().light_intensity);
            ++light_index;
        }
    }
    ubo.active_lights = light_index;
}

void PointLightSystem::Render(const FrameInfo& frame_info) {
    pipeline_->Bind(frame_info.command_buffer);

    vkCmdBindDescriptorSets(frame_info.command_buffer,          //
                            VK_PIPELINE_BIND_POINT_GRAPHICS,    //
                            pipeline_layout_,                  //
                            0,                                  //
                            1,                                  //
                            &frame_info.global_descriptor_set,  //
                            0,                                  //
                            nullptr);                           //

    for (auto& key_value : frame_info.game_objects) {
        auto& obj = key_value.second;
        if (obj.point_light.has_value()) {
            PointLightPushContant push{};
            push.position = glm::vec4(obj.transform_.translation, 1.f);
            push.color =
                glm::vec4(obj.color_, obj.point_light.value().light_intensity);
            push.radius = obj.transform_.scale.x;

            vkCmdPushConstants(
                frame_info.command_buffer, pipeline_layout_,
                VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                sizeof(PointLightPushContant), &push);

            vkCmdDraw(frame_info.command_buffer, 6, 1, 0, 0);
        }
    }
}
}  // namespace vulkeng
#include "vulkeng/include/simple_render_system.hpp"

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

// STD
#include <stdexcept>

// TODO - Don't make these hardcoded paths
const std::string VERTEX_PATH =
    "C:/Users/WJSSn/Documents/GitRepos/VulkanGrowProject/shaders/vert.spv";
const std::string FRAGMENT_PATH =
    "C:/Users/WJSSn/Documents/GitRepos/VulkanGrowProject/shaders/frag.spv";

namespace vulkeng {
struct SimplePushConstantData {
  glm::mat4 model_matrix{1.f};
  glm::mat4 normal_matrix{1.f};
};

SimpleRenderSystem::SimpleRenderSystem(VulkanDevice* device,
                                       VkRenderPass render_pass,
                                       VkDescriptorSetLayout global_set_layout)
    : device_(device) {
  CreatePipelineLayout(global_set_layout);
  CreatePipeline(render_pass);
}

SimpleRenderSystem::~SimpleRenderSystem() {
  vkDestroyPipelineLayout(device_->device(), pipeline_layout_, nullptr);
}

void SimpleRenderSystem::CreatePipelineLayout(
    VkDescriptorSetLayout global_set_layout) {
  VkPushConstantRange push_constant_range;
  push_constant_range.stageFlags =
      VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
  push_constant_range.offset = 0;
  push_constant_range.size = sizeof(SimplePushConstantData);

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

void SimpleRenderSystem::CreatePipeline(VkRenderPass render_pass) {
  assert(pipeline_layout_ != nullptr &&
         "Can't create pipeline before pipeline layout");

  PipelineConfigInfo pipeline_config = {};
  VulkanPipeline::DefaultPipelineConfigInfo(pipeline_config);
  pipeline_config.render_pass = render_pass;
  pipeline_config.pipeline_layout = pipeline_layout_;

  pipeline_ = std::make_unique<VulkanPipeline>(device_, VERTEX_PATH,
                                               FRAGMENT_PATH, pipeline_config);
}

void SimpleRenderSystem::RenderGameObjects(const FrameInfo& frame_info) {
  pipeline_->Bind(frame_info.command_buffer);

  vkCmdBindDescriptorSets(frame_info.command_buffer,
                          VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_layout_, 0,
                          1, &frame_info.global_descriptor_set, 0, nullptr);

  for (auto& kv : frame_info.game_objects) {
    auto& obj = kv.second;
    SimplePushConstantData push{};
    push.model_matrix = obj.transform_.mat4();
    push.normal_matrix = obj.transform_.NormalMatrix();

    vkCmdPushConstants(
        frame_info.command_buffer, pipeline_layout_,
        VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0,
        sizeof(SimplePushConstantData), &push);

    obj.model_->Bind(frame_info.command_buffer);
    obj.model_->Draw(frame_info.command_buffer);
  }
}
}  // namespace vulkeng
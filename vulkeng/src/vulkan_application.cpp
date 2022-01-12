#include "vulkeng/include/vulkan_application.hpp"

// VulkEng
#include "vulkeng/include/keyboard_movement_controller.hpp"
#include "vulkeng/include/simple_render_system.hpp"
#include "vulkeng/include/vulkan_buffer.hpp"
#include "vulkeng/include/vulkan_camera.hpp"

// GLM
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

// STD
#include <array>
#include <cassert>
#include <chrono>
#include <numeric>
#include <stdexcept>
#include <unordered_map>

namespace vulkeng {

struct GlobalUbo {
  glm::mat4 projection_view{1.f};
  glm::vec4 ambient_light_color{1.f, 1.f, 1.f, 0.02f};
  glm::vec3 light_position{-1.f};
  alignas(16) glm::vec4 light_color{1.f};
};

VulkanApplication::VulkanApplication(int width, int height,
                                     std::string app_name) {
  window_.reset(new VulkanWindow(width, height, app_name));
  device_.reset(new VulkanDevice(window_.get()));
  renderer_.reset(new VulkanRenderer(window_.get(), device_.get()));

  global_pool_ = VulkanDescriptorPool::Builder(device_.get())
                     .SetMaxSets(VulkanSwapChain::MAX_FRAMES_IN_FLIGHT)
                     .AddPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                                  VulkanSwapChain::MAX_FRAMES_IN_FLIGHT)
                     .Build();
  LoadGameObjects();
}

void VulkanApplication::Run() {
  std::vector<std::unique_ptr<VulkanBuffer>> uniform_buffers(
      VulkanSwapChain::MAX_FRAMES_IN_FLIGHT);
  for (size_t i = 0; i < uniform_buffers.size(); ++i) {
    uniform_buffers[i] = std::make_unique<VulkanBuffer>(
        device_.get(), sizeof(GlobalUbo), 1, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
    uniform_buffers[i]->Map();
  }

  auto global_set_layout = VulkanDescriptorSetLayout::Builder(device_.get())
                               .AddBinding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                                           VK_SHADER_STAGE_ALL_GRAPHICS)
                               .Build();

  std::vector<VkDescriptorSet> global_descriptor_sets(
      VulkanSwapChain::MAX_FRAMES_IN_FLIGHT);
  for (int i = 0; i < global_descriptor_sets.size(); ++i) {
    auto buffer_info = uniform_buffers[i]->DescriptorInfo();
    VulkanDescriptorWriter(*global_set_layout, *global_pool_)
        .WriteBuffer(0, &buffer_info)
        .Build(global_descriptor_sets[i]);
  }

  SimpleRenderSystem render_system{device_.get(),
                                   renderer_->GetSwapChainRenderPass(),
                                   global_set_layout->DescriptorSetLayout()};
  VulkanCamera camera{};
  camera.SetViewTarget(glm::vec3{-1.f, -2.f, 3.f}, glm::vec3{0.0f, 0.f, 2.5f});

  auto viewer_object = VulkanGameObject::CreateVulkanGameObject();
  viewer_object.transform_.translation.z = -2.5f;
  KeyboardMovementController movement_controller{};

  auto current_time = std::chrono::high_resolution_clock::now();

  while (!window_->ShouldClose()) {
    glfwPollEvents();

    auto new_time = std::chrono::high_resolution_clock::now();
    float frame_time =
        std::chrono::duration<float, std::chrono::seconds::period>(new_time -
                                                                   current_time)
            .count();
    current_time = new_time;

    movement_controller.MoveInPlane(window_->window(), frame_time,
                                    viewer_object);
    camera.SetViewYXZ(viewer_object.transform_.translation,
                      viewer_object.transform_.rotation);

    float aspect = renderer_->GetAspectRatio();
    // camera.SetOrthographicProjection(-aspect, aspect, -1, 1, -1, 1);
    camera.SetPerspectiveProjection(glm::radians(50.f), aspect, 0.1f, 100.f);

    if (auto command_buffer = renderer_->BeginFrame()) {
      int frame_index = renderer_->FrameIndex();
      FrameInfo frame_info{frame_index, frame_time, command_buffer, camera,
                           global_descriptor_sets[frame_index], game_objects_};
      // Update
      GlobalUbo ubo;
      ubo.projection_view = camera.projection_matrix() * camera.view_matrix();
      uniform_buffers[frame_index]->WriteToBuffer(&ubo);
      uniform_buffers[frame_index]->Flush();

      // Render
      renderer_->BeginSwapChainRenderPass(command_buffer);
      render_system.RenderGameObjects(frame_info);
      renderer_->EndSwapChainRenderPass(command_buffer);
      renderer_->EndFrame();
    }
  }

  vkDeviceWaitIdle(device_->device());
}

VulkanApplication::~VulkanApplication() {
  game_objects_.clear();
  global_pool_.reset();
  renderer_.reset();
  device_.reset();
  window_.reset();
}

void VulkanApplication::LoadGameObjects() {
  std::shared_ptr<VulkanModel> model = VulkanModel::CreateModelFromFile(
      device_.get(),
      "C:/Users/WJSSn/Documents/GitRepos/VulkanGrowProject/models/"
      "flat_vase.obj");

  auto flat_vase = VulkanGameObject::CreateVulkanGameObject();
  flat_vase.model_ = model;
  flat_vase.transform_.translation = {0.f, 0.5f, 0.f};
  flat_vase.transform_.scale = {3.0f, 1.f, 3.f};
  game_objects_.emplace(flat_vase.id(), std::move(flat_vase));

  model = VulkanModel::CreateModelFromFile(
      device_.get(),
      "C:/Users/WJSSn/Documents/GitRepos/VulkanGrowProject/models/"
      "smooth_vase.obj");

  auto smooth_vase = VulkanGameObject::CreateVulkanGameObject();
  smooth_vase.model_ = model;
  smooth_vase.transform_.translation = {1.f, 0.5f, 0.f};
  smooth_vase.transform_.scale = {3.0f, 1.f, 3.f};
  game_objects_.emplace(smooth_vase.id(), std::move(smooth_vase));

  model = VulkanModel::CreateModelFromFile(
      device_.get(),
      "C:/Users/WJSSn/Documents/GitRepos/VulkanGrowProject/models/"
      "quad.obj");

  auto floor = VulkanGameObject::CreateVulkanGameObject();
  floor.model_ = model;
  floor.transform_.translation = {0.f, 0.5f, 0.f};
  floor.transform_.scale = {3.0f, 1.f, 3.f};
  game_objects_.emplace(floor.id(), std::move(floor));
}
}  // namespace vulkeng
#include "vulkeng/include/vulkan_application.hpp"

// VulkEng
#include "vulkeng/experiment/perlin_noise_gen.hpp"
#include "vulkeng/experiment/perlin_noise_render_system.hpp"
#include "vulkeng/include/keyboard_movement_controller.hpp"
#include "vulkeng/include/vulkan_buffer.hpp"
#include "vulkeng/include/vulkan_camera.hpp"
#include "vulkeng/include/vulkan_texture.hpp"
#include "vulkeng/systems/line_render_system.hpp"
#include "vulkeng/systems/point_light_system.hpp"
#include "vulkeng/systems/simple_render_system.hpp"

// NURBS
#include "nurbs_cpp/include/b_spline_curve.hpp"
#include "nurbs_cpp/include/b_spline_surface.hpp"
#include "nurbs_cpp/include/bezier_curve.hpp"
#include "nurbs_cpp/include/bezier_surface.hpp"
#include "nurbs_cpp/include/parametric_curve.hpp"
#include "nurbs_cpp/include/parametric_surface.hpp"
#include "vulkeng/experiment/curve_model.hpp"
#include "vulkeng/experiment/surface_model.hpp"

// Math Constants
#define _USE_MATH_DEFINES
#include <math.h>

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

VulkanApplication::VulkanApplication(int width, int height,
                                     std::string app_name) {
  window_.reset(new VulkanWindow(width, height, app_name));
  device_.reset(new VulkanDevice(window_.get()));
  renderer_.reset(new VulkanRenderer(window_.get(), device_.get()));

  global_pool_ = VulkanDescriptorPool::Builder(device_.get())
                     .SetMaxSets(VulkanSwapChain::MAX_FRAMES_IN_FLIGHT)
                     .AddPoolSize(VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                                  VulkanSwapChain::MAX_FRAMES_IN_FLIGHT)
                     .AddPoolSize(VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                                  VulkanSwapChain::MAX_FRAMES_IN_FLIGHT)
                     .Build();

  LoadGameObjects();
}

void VulkanApplication::Run() {
  GUI gui(window_.get(), device_.get(), renderer_.get());

  std::vector<std::unique_ptr<VulkanBuffer>> uniform_buffers(
      VulkanSwapChain::MAX_FRAMES_IN_FLIGHT);
  for (size_t i = 0; i < uniform_buffers.size(); ++i) {
    uniform_buffers[i] = std::make_unique<VulkanBuffer>(
        device_.get(), sizeof(GlobalUbo), 1, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT);
    uniform_buffers[i]->Map();
  }

  VulkanTexture texture(device_.get(), "../textures/image.png");

  VkDescriptorImageInfo image_info;
  image_info.sampler = texture.sampler();
  image_info.imageView = texture.image_view();
  image_info.imageLayout = texture.image_layout();

  auto global_set_layout =
      VulkanDescriptorSetLayout::Builder(device_.get())
          .AddBinding(0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,
                      VK_SHADER_STAGE_ALL_GRAPHICS)
          .AddBinding(1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
                      VK_SHADER_STAGE_FRAGMENT_BIT)
          .Build();

  std::vector<VkDescriptorSet> global_descriptor_sets(
      VulkanSwapChain::MAX_FRAMES_IN_FLIGHT);
  for (size_t i = 0; i < global_descriptor_sets.size(); ++i) {
    auto buffer_info = uniform_buffers[i]->DescriptorInfo();
    VulkanDescriptorWriter(*global_set_layout, *global_pool_)
        .WriteBuffer(0, &buffer_info)
        .WriteImage(1, &image_info)
        .Build(global_descriptor_sets[i]);
  }
  LineRenderSystem line_render_system{device_.get(),
                                      renderer_->GetSwapChainRenderPass(),
                                      global_set_layout->DescriptorSetLayout()};
  SimpleRenderSystem render_system{device_.get(),
                                   renderer_->GetSwapChainRenderPass(),
                                   global_set_layout->DescriptorSetLayout()};
  // PerlinNoiseRenderSystem perlin_render_system{
  //    device_.get(), renderer_->GetSwapChainRenderPass(),
  //    global_set_layout->DescriptorSetLayout()};
  PointLightSystem point_light_system{device_.get(),
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
      // tell imgui that we're starting a new frame
      gui.NewFrame();

      int frame_index = renderer_->FrameIndex();
      FrameInfo frame_info{frame_index,
                           frame_time,
                           command_buffer,
                           camera,
                           global_descriptor_sets[frame_index],
                           game_objects_};
      // Update
      GlobalUbo ubo;
      ubo.projection = camera.projection_matrix();
      ubo.view = camera.view_matrix();
      ubo.inv_view = camera.inverse_view_matrix();
      point_light_system.Update(frame_info, ubo);
      // perlin_render_system.Update(frame_info, ubo);
      uniform_buffers[frame_index]->WriteToBuffer(&ubo);
      uniform_buffers[frame_index]->Flush();

      // Render
      renderer_->BeginSwapChainRenderPass(command_buffer);

      // perlin_render_system.Render(frame_info);
      render_system.RenderGameObjects(frame_info);
      line_render_system.RenderGameObjects(frame_info);
      point_light_system.Render(
          frame_info); // Must be rendered last for transparency
      gui.runExample();
      gui.Render(command_buffer);
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
  // Curve Model
  {
    std::vector<nurbs::Point2D> control_points = {
        {0, 0}, {0, 1}, {0.5, -1}, {1, 1}, {1, 0}};
    nurbs::BSplineCurve2D b_spline(3, control_points,
                                   {0, 0, 0, 0, 1, 2, 2, 2, 2}, {0, 2});
    auto curve = CurveModel::ModelFromCurve2D(device_.get(), b_spline);

    auto parametric_curve = VulkanGameObject::CreateVulkanGameObject();
    parametric_curve.line_model_ = curve;
    parametric_curve.transform_.translation = {0.f, -1.f, 0.f};
    parametric_curve.transform_.scale = {0.75f, 0.75f, 0.75f};
    game_objects_.emplace(parametric_curve.id(), std::move(parametric_curve));
  }
  // Surface Model
  {
    uint32_t degree = 3;
    std::vector<double> u_knots = {0, 0, 0, 0, 1, 2, 2, 2, 2};
    std::vector<double> v_knots = {0, 0, 0, 0, 1, 2, 2, 2, 2};
    std::vector<std::vector<nurbs::Point3D>> control_points = {
        {{-1, 0, -1},
         {-0.33, 0.1, -1.33},
         {0.33, 0.1, -1.33},
         {0.87, 0, -0.87},
         {1, -0.1, -1}}, //
        {{-1.33, -0.25, -0.33},
         {-0.33, 0, -0.33},
         {0.33, 0.0, -0.33},
         {1.33, -0.25, -0.33},
         {2, -0.5, -0.63}}, //
        {{-1.33, -0.75, 0.33},
         {-0.33, 0.0, 0.33},
         {0.33, 0.0, 0.33},
         {1.33, -0.75, 0.33},
         {2, -1, 0.63}}, //
        {{-0.87, -2, 0.87},
         {-0.33, 0.0, 1.33},
         {0.33, 0.0, 1.33},
         {0.87, -2, 0.87},
         {1, -2.5, 1}}, //
        {{-1, -2.5, 1},
         {-0.33, 0.0, 2},
         {0.33, 0.0, 1.63},
         {0.87, -2, 1},
         {1, -3, 1.5}}, //
    };
    nurbs::BSplineSurface b_surface(degree, degree, u_knots, v_knots,
                                    control_points, {0, 2}, {0, 2});
    auto surface = SurfaceModel::ModelFromSurface(device_.get(), b_surface);

    auto surf_obj = VulkanGameObject::CreateVulkanGameObject();
    surf_obj.model_ = surface;
    surf_obj.transform_.translation = {0.0f, 0.5f, 0.0f};
    surf_obj.transform_.scale = {3.0f, 1.0f, 3.0f};
    game_objects_.emplace(surf_obj.id(), std::move(surf_obj));
  }

  // Current Tutorial Objects:
  {
    std::shared_ptr<TriangleModel> model = TriangleModel::CreateModelFromFile(
        device_.get(), "C:/Users/WJSSn/Documents/GitRepos/GlacierEngine/models/"
                       "flat_vase.obj");

    auto flat_vase = VulkanGameObject::CreateVulkanGameObject();
    flat_vase.model_ = model;
    flat_vase.transform_.translation = {-0.5f, 0.35f, 0.f};
    flat_vase.transform_.scale = {3.0f, 1.f, 3.f};
    game_objects_.emplace(flat_vase.id(), std::move(flat_vase));
  }

  {
    std::shared_ptr<TriangleModel> model = TriangleModel::CreateModelFromFile(
        device_.get(), "C:/Users/WJSSn/Documents/GitRepos/GlacierEngine/models/"
                       "smooth_vase.obj");

    auto smooth_vase = VulkanGameObject::CreateVulkanGameObject();
    smooth_vase.model_ = model;
    smooth_vase.transform_.translation = {0.5f, 0.35f, 0.f};
    smooth_vase.transform_.scale = {3.0f, 1.f, 3.f};
    game_objects_.emplace(smooth_vase.id(), std::move(smooth_vase));
  }
  /*{
      std::shared_ptr<TriangleModel> model = TriangleModel::CreateModelFromFile(
          device_.get(),
          "C:/Users/WJSSn/Documents/GitRepos/GlacierEngine/models/"
          "quad.obj");

      auto floor = VulkanGameObject::CreateVulkanGameObject();
      floor.model_ = model;
      floor.transform_.translation = { 0.f, 0.5f, 0.f };
      floor.transform_.scale = { 3.0f, 1.f, 3.f };
      game_objects_.emplace(floor.id(), std::move(floor));
  }*/

  {
    std::vector<glm::vec3> light_colors = {{1.0f, 0.1f, 0.1f},  //
                                           {0.1f, 0.1f, 1.0f},  //
                                           {0.1f, 1.0f, 0.1f},  //
                                           {1.0f, 1.0f, 0.1f},  //
                                           {0.1f, 1.0f, 1.0f},  //
                                           {1.0f, 0.1f, 1.0f}}; //
    for (size_t i = 0; i < light_colors.size(); ++i) {
      auto point_light = VulkanGameObject::MakePointLight(0.5f);
      point_light.color_ = light_colors[i];
      auto rotate_light =
          glm::rotate(glm::mat4(1.f),
                      (static_cast<float>(i) * glm::two_pi<float>()) /
                          static_cast<float>(light_colors.size()),
                      {0.0f, -1.0f, 0.0f});
      point_light.transform_.translation =
          glm::vec3(rotate_light * glm::vec4(2.5f, -0.5f, 0.f, 1.f));
      game_objects_.emplace(point_light.id(), std::move(point_light));
    }
  }

  /*PerlinNoiseGen noise(device_.get());

  auto perlin_model = VulkanGameObject::CreateVulkanGameObject();
  //perlin_model.model_ = noise.GetModel({0, 0}, {100, 100});
  perlin_model.model_ = noise.GetModelFlat( {1000, 1000});
  perlin_model.transform_.translation = {0.5f, 0.5f, 0.f};
  perlin_model.transform_.scale = {3.0f, 1.f, 3.f};
  game_objects_.emplace(perlin_model.id(), std::move(perlin_model));*/
}
} // namespace vulkeng
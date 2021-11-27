#include "vulkeng/include/vulkan_application.hpp"

// VulkEng
#include "vulkeng/include/simple_render_system.hpp"

#define TINYOBJLOADER_IMPLEMENTATION
#include "include/tiny_obj_loader.h"

// GLM
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

// STD
#include <array>
#include <cassert>
#include <stdexcept>
#include <unordered_map>

namespace vulkeng {
VulkanApplication::VulkanApplication(int width, int height, std::string app_name) {
  window_.reset(new VulkanWindow(width, height, app_name));
  device_.reset(new VulkanDevice(window_.get()));
  renderer_.reset(new VulkanRenderer(window_.get(), device_.get()));

  LoadGameObjects();
}

void VulkanApplication::Run() {
  SimpleRenderSystem render_system{device_.get(),
                                   renderer_->GetSwapChainRenderPass()};

  // glfwWindowShouldClose(window_->window() should be moved into a call to the
  // window
  while (!window_->ShouldClose()) {
    glfwPollEvents();

    if (auto command_buffer = renderer_->BeginFrame()) {
      renderer_->BeginSwapChainRenderPass(command_buffer);
      render_system.RenderGameObjects(command_buffer, game_objects_);
      renderer_->EndSwapChainRenderPass(command_buffer);
      renderer_->EndFrame();
    }
  }

  vkDeviceWaitIdle(device_->device());
}

VulkanApplication::~VulkanApplication() {
  game_objects_.clear();
  renderer_.reset();
  device_.reset();
  window_.reset();
}

// temporary helper function, creates a 1x1x1 cube centered at offset
std::unique_ptr<VulkanModel> createCubeModel(VulkanDevice* device,
                                             glm::vec3 offset) {
  std::vector<VulkanModel::Vertex> vertices{

      // left face (white)
      {{-.5f, -.5f, -.5f}, {.9f, .9f, .9f}},
      {{-.5f, .5f, .5f}, {.9f, .9f, .9f}},
      {{-.5f, -.5f, .5f}, {.9f, .9f, .9f}},
      {{-.5f, -.5f, -.5f}, {.9f, .9f, .9f}},
      {{-.5f, .5f, -.5f}, {.9f, .9f, .9f}},
      {{-.5f, .5f, .5f}, {.9f, .9f, .9f}},

      // right face (yellow)
      {{.5f, -.5f, -.5f}, {.8f, .8f, .1f}},
      {{.5f, .5f, .5f}, {.8f, .8f, .1f}},
      {{.5f, -.5f, .5f}, {.8f, .8f, .1f}},
      {{.5f, -.5f, -.5f}, {.8f, .8f, .1f}},
      {{.5f, .5f, -.5f}, {.8f, .8f, .1f}},
      {{.5f, .5f, .5f}, {.8f, .8f, .1f}},

      // top face (orange, remember y axis points down)
      {{-.5f, -.5f, -.5f}, {.9f, .6f, .1f}},
      {{.5f, -.5f, .5f}, {.9f, .6f, .1f}},
      {{-.5f, -.5f, .5f}, {.9f, .6f, .1f}},
      {{-.5f, -.5f, -.5f}, {.9f, .6f, .1f}},
      {{.5f, -.5f, -.5f}, {.9f, .6f, .1f}},
      {{.5f, -.5f, .5f}, {.9f, .6f, .1f}},

      // bottom face (red)
      {{-.5f, .5f, -.5f}, {.8f, .1f, .1f}},
      {{.5f, .5f, .5f}, {.8f, .1f, .1f}},
      {{-.5f, .5f, .5f}, {.8f, .1f, .1f}},
      {{-.5f, .5f, -.5f}, {.8f, .1f, .1f}},
      {{.5f, .5f, -.5f}, {.8f, .1f, .1f}},
      {{.5f, .5f, .5f}, {.8f, .1f, .1f}},

      // nose face (blue)
      {{-.5f, -.5f, 0.5f}, {.1f, .1f, .8f}},
      {{.5f, .5f, 0.5f}, {.1f, .1f, .8f}},
      {{-.5f, .5f, 0.5f}, {.1f, .1f, .8f}},
      {{-.5f, -.5f, 0.5f}, {.1f, .1f, .8f}},
      {{.5f, -.5f, 0.5f}, {.1f, .1f, .8f}},
      {{.5f, .5f, 0.5f}, {.1f, .1f, .8f}},

      // tail face (green)
      {{-.5f, -.5f, -0.5f}, {.1f, .8f, .1f}},
      {{.5f, .5f, -0.5f}, {.1f, .8f, .1f}},
      {{-.5f, .5f, -0.5f}, {.1f, .8f, .1f}},
      {{-.5f, -.5f, -0.5f}, {.1f, .8f, .1f}},
      {{.5f, -.5f, -0.5f}, {.1f, .8f, .1f}},
      {{.5f, .5f, -0.5f}, {.1f, .8f, .1f}},

  };
  for (auto& v : vertices) {
    v.pos += offset;
  }
  return std::make_unique<VulkanModel>(device, vertices);
}

void VulkanApplication::LoadGameObjects() {
  std::shared_ptr<VulkanModel> cube_model =
      createCubeModel(device_.get(), {0.f, 0.f, 0.f});

  auto cube = VulkanGameObject::CreateVulkanGameObject();
  cube.model_ = cube_model;
  cube.transform_.translation = {0.f, 0.f, 0.5f};
  cube.transform_.scale = {0.5f, 0.5f, 0.5f};

  game_objects_.push_back(std::move(cube));
}

// void VulkanApplication::LoadModel() {
/*tinyobj::attrib_t attrib;
std::vector<tinyobj::shape_t> shapes;
std::vector<tinyobj::material_t> materials;
std::string warn, err;

if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
MODEL_PATH.c_str())) { throw std::runtime_error(warn + err);
}

std::unordered_map<VulkanModel::Vertex, uint32_t> unique_vertices{};
std::vector<VulkanModel::Vertex> vertices{};
std::vector<uint32_t> indices{};

for (const auto& shape : shapes) {
    for (const auto& index : shape.mesh.indices) {
        VulkanModel::Vertex vertex{};

        vertex.pos = {
            attrib.vertices[3 * index.vertex_index + 0],
            attrib.vertices[3 * index.vertex_index + 1],
            attrib.vertices[3 * index.vertex_index + 2]
        };

        vertex.texCoord = {
            attrib.texcoords[2 * index.texcoord_index + 0],
            1.0f - attrib.texcoords[2 * index.texcoord_index + 1]
        };

        vertex.color = { 1.0f, 1.0f, 1.0f };

        if (unique_vertices.count(vertex) == 0) {
            unique_vertices[vertex] = static_cast<uint32_t>(vertices.size());
            vertices.push_back(vertex);
        }

        indices.push_back(unique_vertices[vertex]);
    }
}
model_ = std::make_unique<VulkanModel>(device_.get(), vertices);*/
//}
}  // namespace vulkeng
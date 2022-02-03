#pragma once

#include "callback_manager.hpp"

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <string>

namespace vulkeng {
class VulkanWindow {
 public:
  CallbackManager callback_manager_;

 public:
  VulkanWindow(int width, int height, std::string name = "Glacier Engine");
  ~VulkanWindow();

  bool ShouldClose() { return glfwWindowShouldClose(window_); }
  int width() { return width_; }
  int height() { return height_; }
  VkExtent2D extent() {
    return {static_cast<uint32_t>(width_), static_cast<uint32_t>(height_)};
  }

  virtual void init(){};
  virtual void frame(){};
  virtual void cleanup(){};

  GLFWwindow* window() { return window_; }

  // Frame Buffer Resized info
  bool WasFrameBufferResized() { return framebuffer_resized_; }
  void ResetFrameBufferResized() { framebuffer_resized_ = false; }

  void CreateWindowSurface(VkInstance instance, VkSurfaceKHR* surface);

  // Callback
  void SetCursorPositionCallback(
      const CallbackManager::CursorPositionCallback& callback) {
    callback_manager_.SetCursorPositionCallback(callback);
  }

  void AddKeyCallback(const CallbackManager::KeyCallback& callback) {
    callback_manager_.AddKeyCallback(callback);
  }

  void AddMouseButtonCallback(
      const CallbackManager::MouseButtonCallback& callback) {
    callback_manager_.AddMouseButtonCallback(callback);
  }

 private:
  static void framebufferResizeCallback(GLFWwindow* window, int width,
                                        int height);
  void InitWindow();

  int width_ = 0;
  int height_ = 0;
  bool framebuffer_resized_ = false;

  std::string name_;
  GLFWwindow* window_ = nullptr;
};
}  // namespace vulkeng
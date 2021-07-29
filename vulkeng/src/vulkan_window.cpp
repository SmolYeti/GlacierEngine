#include "vulkeng/include/vulkan_window.hpp"

// STD
#include <stdexcept>

namespace vulkeng {
VulkanWindow::VulkanWindow(int width, int height, std::string name)
    : width_(width), height_(height), name_(name) {
  InitWindow();
}

VulkanWindow::~VulkanWindow() {

  glfwDestroyWindow(window_);
  glfwTerminate();
}

void VulkanWindow::InitWindow() {
  glfwInit();

  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

  window_ = glfwCreateWindow(width_, height_, name_.c_str(), nullptr, nullptr);
  glfwSetWindowUserPointer(window_, this);
  glfwSetFramebufferSizeCallback(window_, framebufferResizeCallback);

  // Input callback setup
  glfwSetKeyCallback(window_, [](GLFWwindow* window, int key, int scancode,
                                 int action, int mode) {
    VulkanWindow* temp_window = (VulkanWindow*)glfwGetWindowUserPointer(window);
    temp_window->callback_manager_.OnKeyPress(window, key, scancode, action,
                                              mode);
  });

  glfwSetCursorPosCallback(
      window_, [](GLFWwindow* window, double xpos, double ypos) {
        VulkanWindow* temp_window = (VulkanWindow*)glfwGetWindowUserPointer(window);
        temp_window->callback_manager_.OnCursorPosition(window, xpos, ypos);
      });

  glfwSetMouseButtonCallback(
      window_, [](GLFWwindow* window, int button, int action, int mode) {
        VulkanWindow* temp_window = (VulkanWindow*)glfwGetWindowUserPointer(window);
        temp_window->callback_manager_.OnMouseButtonPress(window, button,
                                                          action, mode);
      });
}

void VulkanWindow::CreateWindowSurface(VkInstance instance, VkSurfaceKHR* surface) {
  if (glfwCreateWindowSurface(instance, window_, nullptr, surface) !=
      VK_SUCCESS) {
    throw std::runtime_error("failed to craete window surface");
  }
}

void VulkanWindow::framebufferResizeCallback(GLFWwindow* window, int width,
                                       int height) {
  auto app = reinterpret_cast<VulkanWindow*>(glfwGetWindowUserPointer(window));
  app->framebuffer_resized_ = true;
  app->width_ = width;
  app->height_ = height;
}
}  // namespace vulkeng
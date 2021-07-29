#pragma once

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

// STD
#include <functional>
#include <mutex>

namespace vulkeng {
class CallbackManager {
 public:
  using KeyCallback = std::function<void(GLFWwindow* window, int key,
                                         int scancode, int action, int mods)>;
  using CursorPositionCallback =
      std::function<void(GLFWwindow* window, double xpos, double ypos)>;
  using MouseButtonCallback =
      std::function<void(GLFWwindow* window, int button, int action, int mods)>;

  // Add Callbacks
  void SetCursorPositionCallback(const CursorPositionCallback& callback) {
    const std::lock_guard<std::mutex> lock(callback_lock_);
    cursor_position_callback_ = callback;
  }

  void AddKeyCallback(const KeyCallback& callback) {
    const std::lock_guard<std::mutex> lock(callback_lock_);
    key_callbacks_.push_back(callback);
  }

  void AddMouseButtonCallback(const MouseButtonCallback& callback) {
    const std::lock_guard<std::mutex> lock(callback_lock_);
    mouse_button_callbacks_.push_back(callback);
  }

  void OnCursorPosition(GLFWwindow* window, double xpos, double ypos) {
    cursor_position_callback_(window, xpos, ypos);
  }

  void OnKeyPress(GLFWwindow* window, int key, int scancode, int action,
                  int mods) {
    const std::lock_guard<std::mutex> lock(callback_lock_);
    for (KeyCallback& callback : key_callbacks_) {
      callback(window, key, scancode, action, mods);
    }
  }

  void OnMouseButtonPress(GLFWwindow* window, int button, int action,
                          int mods) {
    const std::lock_guard<std::mutex> lock(callback_lock_);
    for (MouseButtonCallback& callback : mouse_button_callbacks_) {
      callback(window, button, action, mods);
    }
  }

 private:
  std::mutex callback_lock_;
  std::vector<KeyCallback> key_callbacks_;
  std::vector<MouseButtonCallback> mouse_button_callbacks_;
  CursorPositionCallback cursor_position_callback_ =
      [](GLFWwindow* window, double xpos, double ypos) {};
};

}  // namespace vulkeng
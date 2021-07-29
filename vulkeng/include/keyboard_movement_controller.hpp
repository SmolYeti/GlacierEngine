#pragma once

#include "vulkan_game_object.hpp"
#include "vulkan_window.hpp"

namespace vulkeng {
class KeyboardMovementController {
 public:
  struct KeyMappings {
    int kMoveLeft = GLFW_KEY_A;
    int kMoveRight = GLFW_KEY_D;
    int kMoveForward = GLFW_KEY_W;
    int kMoveBackward = GLFW_KEY_S;
    int kMoveUp = GLFW_KEY_E;
    int kMoveDown = GLFW_KEY_Q;
    int kLookLeft = GLFW_KEY_LEFT;
    int kLookRight = GLFW_KEY_RIGHT;
    int kLookUp = GLFW_KEY_UP;
    int kLookDown = GLFW_KEY_DOWN;
  };

  void MoveInPlane(GLFWwindow* window, float dt, VulkanGameObject& game_object);

  KeyMappings keys{};
  float move_speed_{3.f};
  float look_speed_{1.5f};
};
}  // namespace vulkeng

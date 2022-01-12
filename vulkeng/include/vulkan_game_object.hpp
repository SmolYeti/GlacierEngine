#pragma once

#include "vulkan_model.hpp"

// libs
#include <glm/gtc/matrix_transform.hpp>

// std
#include <memory>
#include <unordered_map>

namespace vulkeng {
struct TransformComponent {
  glm::vec3 translation{};
  glm::vec3 scale{1.f, 1.f, 1.f};
  glm::vec3 rotation{};

  // Matrix corrsponds to Translate * Ry * Rx * Rz * Scale
  // Rotations correspond to Tait-bryan angles of Y(1), X(2), Z(3)
  // https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix
  glm::mat4 mat4();

  glm::mat3 NormalMatrix();
};

class VulkanGameObject {
 public:
  using id_t = unsigned int;
  using Map = std::unordered_map<id_t, VulkanGameObject>;

  static VulkanGameObject CreateVulkanGameObject() {
    static id_t current_id = 0;
    return VulkanGameObject(current_id++);
  }

  VulkanGameObject(const VulkanGameObject&) = delete;
  VulkanGameObject& operator=(const VulkanGameObject&) = delete;
  VulkanGameObject(VulkanGameObject&&) = default;
  VulkanGameObject& operator=(VulkanGameObject&&) = default;

  id_t id() const { return id_; }

  std::shared_ptr<VulkanModel> model_{};
  glm::vec3 color_{};
  TransformComponent transform_{};

 private:
  VulkanGameObject(id_t id) : id_(id) {}
  id_t id_ = 0;
};
}  // namespace vulkeng
#pragma once

#include "line_model.hpp"
#include "triangle_model.hpp"

// libs
#include <glm/gtc/matrix_transform.hpp>

// std
#include <memory>
#include <optional>
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

struct PointLightComponent {
  float light_intensity = 1.0f;
};

class VulkanGameObject {
 public:
  using id_t = unsigned int;
  using Map = std::unordered_map<id_t, VulkanGameObject>;

  static VulkanGameObject CreateVulkanGameObject() {
    static id_t current_id = 0;
    return VulkanGameObject(current_id++);
  }

  static VulkanGameObject MakePointLight(float intensity = 10.0f, float radius = 0.1f, glm::vec3 color = glm::vec3(1.f));

  VulkanGameObject(const VulkanGameObject&) = delete;
  VulkanGameObject& operator=(const VulkanGameObject&) = delete;
  VulkanGameObject(VulkanGameObject&&) = default;
  VulkanGameObject& operator=(VulkanGameObject&&) = default;

  id_t id() const { return id_; }

  glm::vec3 color_{};
  TransformComponent transform_{};

  // optional?
  std::shared_ptr<TriangleModel> model_{};
  std::shared_ptr<LineModel> line_model_{};
  std::optional<PointLightComponent> point_light = std::nullopt;

 private:
  VulkanGameObject(id_t id) : id_(id) {}
  id_t id_ = 0;
};
}  // namespace vulkeng
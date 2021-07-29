#pragma once

// GLM
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

namespace vulkeng {
    class VulkanCamera {
    public:
        void SetOrthographicProjection(float left, float right, float top,
            float bottom, float near, float far);

        void SetPerspectiveProjection(float fovy, float aspect, float near,
            float far);

        void SetViewDirection(glm::vec3 position, glm::vec3 direction,
            glm::vec3 up = glm::vec3{ 0.f, -1.f, 0.f });
        void SetViewTarget(glm::vec3 position, glm::vec3 target,
            glm::vec3 up = glm::vec3{ 0.f, -1.f, 0.f });
        void SetViewYXZ(glm::vec3 position, glm::vec3 rotation);

        const glm::mat4& projection_matrix() const { return projection_matrix_; }
        const glm::mat4& view_matrix() const { return view_matrix_; }
        const glm::mat4& inverse_view_matrix() const { return inverse_view_matrix_; }
        const glm::vec3 position() const { return glm::vec3(inverse_view_matrix_[3]); }

    private:
        glm::mat4 projection_matrix_{ 1.f };
        glm::mat4 view_matrix_{ 1.f };
        glm::mat4 inverse_view_matrix_{ 1.f };
    };
}  // namespace vulkeng
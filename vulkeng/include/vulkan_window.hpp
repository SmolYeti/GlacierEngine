#pragma once

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include <string>
#include <functional>
#include <mutex>

namespace vulkeng {
    class VulkanWindow {
    public:
        using KeyCallback = std::function<void(GLFWwindow* window, int key, int scancode, int action, int mods)>;
        using CursorPositionCallback = std::function<void(GLFWwindow* window, double xpos, double ypos)>;
        using MouseButtonCallback = std::function<void(GLFWwindow* window, int button, int action, int mods)>;
    private:
        /* Move to callback manager class*/
        class CallbackManager {
        public:
            void AddCursorPositionCallback(const CursorPositionCallback& callback) {
                const std::lock_guard<std::mutex> lock(callback_lock_);
                cursor_position_callback_ = callback;
            }

            void AddKeyCallback(const KeyCallback& callback)
            {
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

            void OnKeyPress(GLFWwindow* window, int key, int scancode, int action, int mods)
            {
                const std::lock_guard<std::mutex> lock(callback_lock_);
                for (KeyCallback& callback : key_callbacks_)
                {
                    callback(window, key, scancode, action, mods);
                }
            }

            void OnMouseButtonPress(GLFWwindow* window, int button, int action, int mods) {
                const std::lock_guard<std::mutex> lock(callback_lock_);
                for (MouseButtonCallback& callback : mouse_button_callbacks_)
                {
                    callback(window, button, action, mods);
                }
            }

        private:
            std::mutex callback_lock_;
            std::vector<KeyCallback> key_callbacks_;
            std::vector<MouseButtonCallback> mouse_button_callbacks_;
            CursorPositionCallback cursor_position_callback_ = [](GLFWwindow* window, double xpos, double ypos) {};
        };
        CallbackManager callback_manager_;

    public:
        VulkanWindow(int width, int height, std::string name);
        ~VulkanWindow();

        bool ShouldClose() { return glfwWindowShouldClose(window_); }
        int width() { return width_; }
        int height() { return height_; }
        VkExtent2D extent() { return { static_cast<uint32_t>(width_), static_cast<uint32_t>(height_) }; }

        virtual void init() {};
        virtual void frame() {};
        virtual void cleanup() {};

        GLFWwindow* window() { return window_; }

        // Frame Buffer Resized info
        bool WasFrameBufferResized() {
            return framebuffer_resized_;
        }
        void ResetFrameBufferResized() {
            framebuffer_resized_ = false;
        }

        void CreateWindowSurface(VkInstance instance, VkSurfaceKHR *surface);

        // Callback
        void AddCursorPositionCallback(const CursorPositionCallback& callback) {
            callback_manager_.AddCursorPositionCallback(callback);
        }

        void AddKeyCallback(const KeyCallback& callback) {
            callback_manager_.AddKeyCallback(callback);
        }

        void AddMouseButtonCallback(const MouseButtonCallback& callback) {
            callback_manager_.AddMouseButtonCallback(callback);
        }

    private:
        static void framebufferResizeCallback(GLFWwindow* window, int width, int height);
        void InitWindow();

        int width_ = 0;
        int height_ = 0;
        bool framebuffer_resized_ = false;

        std::string name_;
        GLFWwindow* window_ = nullptr;
    };
}
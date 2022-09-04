#include "vulkeng/include/vulkan_application.hpp"

#include "imgui.h"
#//include "imgui_impl_glfw.h"
//#include "imgui_impl_vulkan.h"

// STD
#include <cstdlib>
#include <iostream>
#include <stdexcept>

static constexpr int WINDOW_WIDTH = 2000;
static constexpr int WINDOW_HEIGHT = 1600;

int main(int argc, char* argv[]) {
    vulkeng::VulkanApplication app(WINDOW_WIDTH, WINDOW_HEIGHT, "Vulkan Grow Project");

    try {
        app.Run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
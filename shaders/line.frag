#version 450
//#extension GL_ARB_separate_shader_objects : enable

layout(location = 0) in vec3 fragColor;

layout(location = 0) out vec4 outColor;

struct PointLight {
  vec4 position; // ignore w
  vec4 color; // w is intensity
};

layout(set = 0, binding = 0) uniform GlobalUbo {
    mat4 projection;
    mat4 view;
    mat4 inv_view;
    vec4 ambient_light_color;
    PointLight lights[10];
    float offset;
    int active_lights;
} ubo;

layout(push_constant) uniform Push {
    mat4 model_matrix;
    mat4 normal_matrix;
} push;

void main() {
    outColor = vec4(fragColor.xyz, 1.0);
}
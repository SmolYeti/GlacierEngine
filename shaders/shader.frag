#version 450
//#extension GL_ARB_separate_shader_objects : enable

layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec3 fragPosWorld;
layout(location = 2) in vec3 fragNormWorld;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform GlobalUbo {
    mat4 projection_view_matrix;
    vec4 ambient_light_color;
    vec3 light_position;
    vec4 light_color;
} ubo;

layout(push_constant) uniform Push {
    mat4 model_matrix;
    mat4 normal_matrix;
} push;

void main() {
    vec3 direction_to_light = ubo.light_position - fragPosWorld;
    float attenuation = 1.f / dot(direction_to_light, direction_to_light);

    vec3 light_color = ubo.light_color.xyz * ubo.light_color.w * attenuation;
    vec3 ambient_color = ubo.ambient_light_color.xyz * ubo.ambient_light_color.w;
    vec3 diffuse_light = light_color *  max(dot(normalize(fragNormWorld), normalize(direction_to_light)), 0.0);
    outColor = vec4((ambient_color + diffuse_light) * fragColor, 1.0);
}
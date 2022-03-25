#version 450
//#extension GL_ARB_separate_shader_objects : enable

layout(location = 0) in vec3 fragColor;
layout(location = 1) in vec3 fragPosWorld;
layout(location = 2) in vec3 fragNormWorld;

layout(location = 0) out vec4 outColor;

struct PointLight {
  vec4 position; // ignore w
  vec4 color; // w is intensity
};

layout(set = 0, binding = 0) uniform GlobalUbo {
    mat4 projection;
    mat4 view;
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
    vec3 diffuse_light = ubo.ambient_light_color.xyz * ubo.ambient_light_color.w;
    vec3 surface_normal = normalize(fragNormWorld);

    for (int i = 0; i < ubo.active_lights; ++i) {
        PointLight light = ubo.lights[i];
        vec3 direction_to_light = light.position.xyz - fragPosWorld;
        float attenuation = 1.f / dot(direction_to_light, direction_to_light);
        float cos_angle_incidence = max(dot(surface_normal, normalize(direction_to_light)), 0.0);
        vec3 intensity = light.color.xyz * light.color.w * attenuation;
        diffuse_light += intensity *  cos_angle_incidence;
    }

    outColor = vec4(diffuse_light * fragColor, 1.0);
}
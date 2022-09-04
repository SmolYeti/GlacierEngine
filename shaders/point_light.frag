#version 450

layout (location = 0) in vec2 frag_offset;
layout (location = 0) out vec4 outColor;

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
    vec4 position;
    vec4 color;
    float radius;
} push;

const float M_PI = 3.1415;

void main() {
  float dis = (dot(frag_offset, frag_offset));
  if (dis >= 1.0) {
    discard;
  }
  outColor = vec4(push.color.xyz, 1.0 - abs(cos(dis * M_PI) * sin(dis * M_PI)));
}
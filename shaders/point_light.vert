#version 450

const vec2 OFFSETS[6] = vec2[](
  vec2(-1.0, -1.0),
  vec2(-1.0, 1.0),
  vec2(1.0, -1.0),
  vec2(1.0, -1.0),
  vec2(-1.0, 1.0),
  vec2(1.0, 1.0)
);

layout (location = 0) out vec2 frag_offset;

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
    vec4 position;
    vec4 color;
    float radius;
} push;

void main() {
  frag_offset = OFFSETS[gl_VertexIndex];

  vec4 light_camera_space = ubo.view * vec4(push.position.xyz, 1.0);
  vec4 position_camera_space = light_camera_space + push.radius * vec4(frag_offset, 0.0, 0.0);

  gl_Position = ubo.projection * position_camera_space;
}
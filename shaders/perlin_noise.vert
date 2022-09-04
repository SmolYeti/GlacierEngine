#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;
layout(location = 2) in vec3 normal;
layout(location = 3) in vec2 uv;

layout(location = 0) out vec3 fragColor;
layout(location = 1) out vec3 fragPosWorld;
layout(location = 2) out vec3 fragNormWorld;

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

int p[512] = {151, 160, 137, 91,  90,  15,  131, 13,  201, 95,  96,  53,  194, 233,
       7,   225, 140, 36,  103, 30,  69,  142, 8,   99,  37,  240, 21,  10,
       23,  190, 6,   148, 247, 120, 234, 75,  0,   26,  197, 62,  94,  252,
       219, 203, 117, 35,  11,  32,  57,  177, 33,  88,  237, 149, 56,  87,
       174, 20,  125, 136, 171, 168, 68,  175, 74,  165, 71,  134, 139, 48,
       27,  166, 77,  146, 158, 231, 83,  111, 229, 122, 60,  211, 133, 230,
       220, 105, 92,  41,  55,  46,  245, 40,  244, 102, 143, 54,  65,  25,
       63,  161, 1,   216, 80,  73,  209, 76,  132, 187, 208, 89,  18,  169,
       200, 196, 135, 130, 116, 188, 159, 86,  164, 100, 109, 198, 173, 186,
       3,   64,  52,  217, 226, 250, 124, 123, 5,   202, 38,  147, 118, 126,
       255, 82,  85,  212, 207, 206, 59,  227, 47,  16,  58,  17,  182, 189,
       28,  42,  223, 183, 170, 213, 119, 248, 152, 2,   44,  154, 163, 70,
       221, 153, 101, 155, 167, 43,  172, 9,   129, 22,  39,  253, 19,  98,
       108, 110, 79,  113, 224, 232, 178, 185, 112, 104, 218, 246, 97,  228,
       251, 34,  242, 193, 238, 210, 144, 12,  191, 179, 162, 241, 81,  51,
       145, 235, 249, 14,  239, 107, 49,  192, 214, 31,  181, 199, 106, 157,
       184, 84,  204, 176, 115, 121, 50,  45,  127, 4,   150, 254, 138, 236,
       205, 93,  222, 114, 67,  29,  24,  72,  243, 141, 128, 195, 78,  66,
       215, 61,  156, 180, 151, 160, 137, 91,  90,  15,  131, 13,  201, 95, 
       96,  53,  194, 233, 7,   225, 140, 36,  103, 30,  69,  142, 8,   99, 
       37,  240, 21,  10,  23,  190, 6,   148, 247, 120, 234, 75,  0,   26,
       197, 62,  94,  252, 219, 203, 117, 35,  11,  32,  57,  177, 33,  88,
       237, 149, 56,  87,  174, 20,  125, 136, 171, 168, 68,  175, 74,  165,
       71,  134, 139, 48,  27,  166, 77,  146, 158, 231, 83,  111, 229, 122,
       60,  211, 133, 230, 220, 105, 92,  41,  55,  46,  245, 40,  244, 102,
       143, 54,  65,  25,  63,  161, 1,   216, 80,  73,  209, 76,  132, 187,
       208, 89,  18,  169, 200, 196, 135, 130, 116, 188, 159, 86,  164, 100,
       109, 198, 173, 186, 3,   64,  52,  217, 226, 250, 124, 123, 5,   202,
       38,  147, 118, 126, 255, 82,  85,  212, 207, 206, 59,  227, 47,  16,
       58,  17,  182, 189, 28,  42,  223, 183, 170, 213, 119, 248, 152, 2,
       44,  154, 163, 70,  221, 153, 101, 155, 167, 43,  172, 9,   129, 22,  39,  253, 19,  98,
       108, 110, 79,  113, 224, 232, 178, 185, 112, 104, 218, 246, 97,  228,
       251, 34,  242, 193, 238, 210, 144, 12,  191, 179, 162, 241, 81,  51,
       145, 235, 249, 14,  239, 107, 49,  192, 214, 31,  181, 199, 106, 157,
       184, 84,  204, 176, 115, 121, 50,  45,  127, 4,   150, 254, 138, 236,
       205, 93,  222, 114, 67,  29,  24,  72,  243, 141,
       128, 195, 78,  66,  215, 61,  156, 180};
       

float fade(float t) {
  return t * t * t * (t * (t * 6 - 15) + 10);
}

float lerp(float t, float a, float b) {
  return a + t * (b - a);
}

float grad(int hash, float x, float y, float z) {
  int h = hash & 15;
  // Convert lower 4 bits of hash into 12 gradient directions
  float u = h < 8 ? x : y, v = h < 4 ? y : h == 12 || h == 14 ? x : z;
  return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

float noise(float x, float y, float z) {
  // Find the unit cube that contains the point
  int X = int(mod(floor(x), 256));
  int Y = int(mod(floor(y), 256));
  int Z = int(mod(floor(z), 256));

  // Find relative x, y, z of point in cube
  x -= floor(x);
  y -= floor(y);
  z -= floor(z);

  // Compute fade curves for each of x, y, z
  float u = fade(x);
  float v = fade(y);
  float w = fade(z);

  // Hash coordinates of the 8 cube corners
  int A = p[X] + Y;
  int AA = p[A] + Z;
  int AB = p[A + 1] + Z;
  int B = p[X + 1] + Y;
  int BA = p[B] + Z;
  int BB = p[B + 1] + Z;

  // Add blended results from 8 corners of cube
  float res = lerp(
      w,
      lerp(v, lerp(u, grad(p[AA], x, y, z), grad(p[BA], x - 1, y, z)),
           lerp(u, grad(p[AB], x, y - 1, z), grad(p[BB], x - 1, y - 1, z))),
      lerp(v,
           lerp(u, grad(p[AA + 1], x, y, z - 1),
                grad(p[BA + 1], x - 1, y, z - 1)),
           lerp(u, grad(p[AB + 1], x, y - 1, z - 1),
                grad(p[BB + 1], x - 1, y - 1, z - 1))));
  return (res + 1.0) / 2.0;
}

float delta = 0.0001;

void main() {
    float x_pos = float(10 * (position.x + ubo.offset));
    float n = noise(x_pos, 10 * position.z, 0.8);
    vec3 model_position = vec3((position.x * 2.0) - 1.0, n, (position.z * 2.0) - 1.0);
    vec4 position_world =  push.model_matrix * vec4(model_position, 1.0);
	gl_Position = ubo.projection * ubo.view * position_world;
    
    float n_x = noise(x_pos + delta, 10 * position.z, 0.8);
    float n_z = noise(x_pos, 10 * position.z + delta, 0.8);
    vec3 model_x_offset = vec3(model_position.x + delta, n, model_position.z);
    vec3 model_y_offset = vec3(model_position.x, n, model_position.z + delta);
    vec3 model_x_grad = model_x_offset - model_position;
    vec3 model_y_grad = model_y_offset - model_position;

    vec3 model_normal = normalize(cross(model_x_grad, model_y_grad));

    fragNormWorld = normalize(mat3(push.normal_matrix) * model_normal);
    fragPosWorld = position_world.xyz;
	fragColor = color;
}
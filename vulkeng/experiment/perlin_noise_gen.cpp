#include "vulkeng/experiment/perlin_noise_gen.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>

#include <chrono>
#include <iostream>

namespace vulkeng {
PerlinNoiseGen::PerlinNoiseGen(VulkanDevice* device) : device_(device) {
  // Initialize the permutation vector with the reference values
  p = {151, 160, 137, 91,  90,  15,  131, 13,  201, 95,  96,  53,  194, 233,
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
       215, 61,  156, 180};
  // Duplicate the permutation vector
  p.insert(p.end(), p.begin(), p.end());
}

// Generate a new permutation vector based on the value of seed
PerlinNoiseGen::PerlinNoiseGen(VulkanDevice* device, unsigned int seed)
    : device_(device) {
  p.resize(256);

  // Fill p with values from 0 to 255
  std::iota(p.begin(), p.end(), 0);

  // Initialize a random engine with seed
  std::default_random_engine engine(seed);

  // Suffle  using the above random engine
  std::shuffle(p.begin(), p.end(), engine);

  // Duplicate the permutation vector
  p.insert(p.end(), p.begin(), p.end());
}

double PerlinNoiseGen::noise(double x, double y, double z) {
  // Find the unit cube that contains the point
  int X = (int)floor(x) & 255;
  int Y = (int)floor(y) & 255;
  int Z = (int)floor(z) & 255;

  // Find relative x, y,z of point in cube
  x -= floor(x);
  y -= floor(y);
  z -= floor(z);

  // Compute fade curves for each of x, y, z
  double u = fade(x);
  double v = fade(y);
  double w = fade(z);

  // Hash coordinates of the 8 cube corners
  int A = p[X] + Y;
  int AA = p[A] + Z;
  int AB = p[A + 1] + Z;
  int B = p[X + 1] + Y;
  int BA = p[B] + Z;
  int BB = p[B + 1] + Z;

  // Add blended results from 8 corners of cube
  double res = lerp(
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

double PerlinNoiseGen::noise2(double x, double y) {
  // Find the unit square that contains the point
  int X = (int)floor(x) & 255;
  int Y = (int)floor(y) & 255;

  // Find relative x, y of point in square
  x -= floor(x);
  y -= floor(y);

  // Compute fade curves for each of x and y
  double u = fade(x);
  double v = fade(y);

  // Hash coordinates of the 4 square corners
  int A = p[X] + Y;
  int AA = p[A];
  int AB = p[A + 1];
  int B = p[X + 1] + Y;
  int BA = p[B];
  int BB = p[B + 1];

  // Add blended results from 8 corners of cube
  double res =
      lerp(v, lerp(u, grad2(p[AA], x, y), grad2(p[BA], x - 1, y)),
           lerp(u, grad2(p[AB], x, y - 1), grad2(p[BB], x - 1, y - 1)));
  return (res + 1.0) / 2.0;
}

double PerlinNoiseGen::fade(double t) {
  return t * t * t * (t * (t * 6 - 15) + 10);
}

double PerlinNoiseGen::lerp(double t, double a, double b) {
  return a + t * (b - a);
}

double PerlinNoiseGen::grad(int hash, double x, double y, double z) {
  int h = hash & 15;
  // Convert lower 4 bits of hash into 12 gradient directions
  double u = h < 8 ? x : y, v = h < 4 ? y : h == 12 || h == 14 ? x : z;
  return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

double PerlinNoiseGen::grad2(int hash, double x, double y) {
  int h = hash & 15;
  // Convert lower 4 bits of hash into 12 gradient directions
  double u = h < 8 ? x : y, v = h < 4 ? y : x;
  return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

std::shared_ptr<TriangleModel> PerlinNoiseGen::GetModelFlat(
    std::array<uint32_t, 2> size) {
  auto start_time = std::chrono::steady_clock::now();
  TriangleModel::Builder builder;

  // Vertices
  for (uint32_t i = 0; i < size[1]; ++i) {    // y
    for (uint32_t j = 0; j < size[0]; ++j) {  // x
      double x = (double)j / ((double)size[1]);
      double y = (double)i / ((double)size[0]);

      TriangleModel::Vertex v;
      v.pos = {x, 0, y};
      v.color = {1, 1, 1};

      builder.vertices.push_back(v);
    }
  }
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start_time;
  std::cout << "Flat model elapsed time: " << elapsed_seconds.count() << "s\n";

  // Indicies
  for (uint32_t i = 0; i < size[1] - 1; ++i) {    // y
    for (uint32_t j = 0; j < size[0] - 1; ++j) {  // x
      uint32_t index = i * size[0] + j;
      builder.indices.push_back(index);
      builder.indices.push_back(index + size[0]);
      builder.indices.push_back(index + 1);

      builder.indices.push_back(index + 1);
      builder.indices.push_back(index + size[0]);
      builder.indices.push_back(index + size[0] + 1);
    }
  }

  std::shared_ptr<TriangleModel> model =
      std::make_shared<TriangleModel>(device_, builder);

  return model;
}

std::shared_ptr<TriangleModel> PerlinNoiseGen::GetModel(
    std::array<double, 2> offset, std::array<uint32_t, 2> size) {
  auto start_time = std::chrono::steady_clock::now();
  TriangleModel::Builder builder;

  // Vertices
  for (uint32_t i = 0; i < size[1]; ++i) {    // y
    for (uint32_t j = 0; j < size[0]; ++j) {  // x
      double x = (double)j / ((double)size[1]);
      double y = (double)i / ((double)size[0]);

      // Typical Perlin noise
      // double n = noise(10 * x, 10 * y, 0.8);
      double n = noise2(10 * x + offset[0], 10 * y + offset[1]);

      // Wood like structure
      // double c = 20 * noise(x, y, 0.8);
      // c = c - floor(c);

      TriangleModel::Vertex v;
      v.pos = {(x * 10.0) - 5.0, (n * 5.0) + 5.0, (y * 10.0) - 5.0};
      v.color = {1, 1, 1};
      // v.color = {1, c, c};
      builder.vertices.push_back(v);
    }
  }
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start_time;
  std::cout << "Noisy model elapsed time: " << elapsed_seconds.count() << "s\n";

  // Indicies
  for (uint32_t i = 0; i < size[1] - 1; ++i) {    // y
    for (uint32_t j = 0; j < size[0] - 1; ++j) {  // x
      uint32_t index = i * size[0] + j;
      builder.indices.push_back(index);
      builder.indices.push_back(index + size[0]);
      builder.indices.push_back(index + 1);

      builder.indices.push_back(index + 1);
      builder.indices.push_back(index + size[0]);
      builder.indices.push_back(index + size[0] + 1);
    }
  }

  std::shared_ptr<TriangleModel> model =
      std::make_shared<TriangleModel>(device_, builder);

  return model;
}
}  // namespace vulkeng
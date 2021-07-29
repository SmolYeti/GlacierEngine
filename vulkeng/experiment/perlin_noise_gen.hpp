#include "vulkeng/include/triangle_model.hpp"
#include "vulkeng/include/vulkan_device.hpp"

#include <array>
#include <memory>
#include <vector>

// Most code stolen from https://github.com/sol-prog/Perlin_Noise
// https://cs.nyu.edu/~perlin/noise/
// Only for private use!!!
namespace vulkeng {
class PerlinNoiseGen {
 public:
  PerlinNoiseGen(VulkanDevice* device);
  PerlinNoiseGen(VulkanDevice* device, unsigned int seed);

  std::shared_ptr<TriangleModel> GetModel(std::array<double, 2> offfet,
                                        std::array<uint32_t, 2> size);
  std::shared_ptr<TriangleModel> GetModelFlat(std::array<uint32_t, 2> size);

 private:
  // 3D
  double noise(double x, double y, double z);
  double grad(int hash, double x, double y, double z);

  // 2D
  double noise2(double x, double y);
  double grad2(int hash, double x, double y);

  // General
  double fade(double t);
  double lerp(double t, double a, double b);

  VulkanDevice* device_;
  // The permutation vector
  std::vector<int> p;

  std::shared_ptr<TriangleModel> model_;
};
}  // namespace vulkeng
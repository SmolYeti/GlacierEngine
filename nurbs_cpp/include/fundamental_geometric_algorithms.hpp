#pragma once

// GLM
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>

#include <vector>

namespace nurbs {
namespace knots {
void CurveKnotInsertion(uint32_t p, const std::vector<double> &UP,
                        const std::vector<glm::dvec3> &Pw, double u, uint32_t k,
                        uint32_t s, uint32_t r, std::vector<double> &UQ,
                        std::vector<glm::dvec3> &Qw);

} // namespace knots
} // namespace nurbs
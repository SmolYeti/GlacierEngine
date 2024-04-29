#pragma once
// STD
#include <vector>

namespace nurbs {
namespace der_knots {
std::vector<double> BasisFuns(uint32_t i, double u, uint32_t degree,
                              std::vector<double> knots);
std::vector<std::vector<double>> DersBasisFuns(uint32_t i, double u,
                                               uint32_t degree, uint32_t n,
                                               std::vector<double> knots);
double OneBasisFun(uint32_t degree, std::vector<double> knots, uint32_t i,
                   double u);
std::vector<double> DersOneBasisFun(uint32_t degree, std::vector<double> knots,
                                    uint32_t i, double u, uint32_t n);
} // namespace der_knots
} // namespace nurbs
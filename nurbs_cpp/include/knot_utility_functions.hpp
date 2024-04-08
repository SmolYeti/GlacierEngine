#pragma once

// STD
#include <vector>

namespace nurbs {
namespace knots {
// Returns the max index of u
uint32_t FindSpanKnot(uint32_t degree, const std::vector<uint32_t> &knots,
                       uint32_t knot);
uint32_t FindSpanParam(uint32_t degree, const std::vector<uint32_t> &knots,
                       double u, double tolerance);

// Returns the start index of the provided knot
uint32_t FindStartKnot(uint32_t degree, const std::vector<uint32_t> &knots,
                       uint32_t knot);
uint32_t FindStartParam(uint32_t degree, const std::vector<uint32_t> &knots,
                   double param, double tolerance);

std::vector<double> BasisFuns(uint32_t i, double u, uint32_t degree,
                              const std::vector<uint32_t> &knots, double tolerance);

std::vector<std::vector<double>>
DersBasisFuns(uint32_t i, double u, uint32_t degree, uint32_t n,
              const std::vector<uint32_t> &knots);

double OneBasisFun(uint32_t degree, const std::vector<uint32_t> &knots,
                   uint32_t i, double u);

std::vector<double> DersOneBasisFun(uint32_t degree,
                                    const std::vector<uint32_t> &knots,
                                    uint32_t i, double u, uint32_t n);

std::vector<std::vector<double>>
AllBasisFuns(uint32_t span, double u, uint32_t degree,
             const std::vector<uint32_t> &knots);

std::vector<std::vector<double>> BinomialCoefficients(uint32_t n, uint32_t k);

uint32_t MultiplicityKnotI(int32_t degree, const std::vector<uint32_t> &knots,
                      int knot);
uint32_t MultiplicityKnotU(int32_t degree, const std::vector<uint32_t> &knots,
                      uint32_t knot);
uint32_t MultiplicityParam(int32_t degree, const std::vector<uint32_t> &knots,
                      double param, double tolerance);
} // namespace knots
} // namespace nurbs
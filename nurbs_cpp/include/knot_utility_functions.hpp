#pragma once

// STD
#include <vector>

namespace nurbs {
    namespace knots {
        uint32_t FindSpan(uint32_t degree, double u, std::vector<uint32_t> knots);
        
        std::vector<double> BasisFuns(uint32_t i, double u, uint32_t degree, std::vector<uint32_t> knots);
        
        std::vector<std::vector<double>> DersBasisFuns(uint32_t i, double u, uint32_t degree, uint32_t n, std::vector<uint32_t> knots);
        
        double OneBasisFun(uint32_t degree, std::vector<uint32_t> knots, uint32_t i, double u);

        std::vector<double> DersOneBasisFun(uint32_t degree, std::vector<uint32_t> knots, uint32_t i, double u, uint32_t n);

        std::vector<std::vector<double>> AllBasisFuns(uint32_t span, double u, uint32_t degree, std::vector<uint32_t> knots);
    }
}
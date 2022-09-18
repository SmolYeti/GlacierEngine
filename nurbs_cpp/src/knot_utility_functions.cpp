#include "include/knot_utility_functions.hpp"

// STD
#include <array>
#include <iostream>
namespace nurbs {
    namespace knots {
        // ALGORITHM A.2.1 int FindSpan(n,p,u,U)
        // n - set as Max index of U
        // p - degree
        // u - input value along the curve/knot vector
        // U - Knot vector
        // Returns - span index of u
        uint32_t FindSpan(uint32_t degree, double u, std::vector<uint32_t> knots) {
            uint32_t n = static_cast<uint32_t>(knots.size()) - degree - 2;
            if (u >= static_cast<double>(knots[n + 1])) {
                return n;
            }
            if (u <= static_cast<double>(knots[degree])) {
                return degree;
            }
            uint32_t low = degree;
            uint32_t high = n + 1;
            uint32_t mid = (low + high) / 2;
            while (u < static_cast<double>(knots[mid]) ||
                u >= static_cast<double>(knots[mid + 1])) {
                if (u < static_cast<double>(knots[mid])) {
                    high = mid;
                }
                else {
                    low = mid;
                }
                mid = (low + high) / 2;
            }
            return mid;
        }

        // ALGORITHM A2.2  BasisFuns(i,u,p,U,N)
        // i - Span index (From find span)
        // u - input value along the curve/knot vector
        // p - degree
        // U - Knot vector
        // N - Returned basis vector
        std::vector<double> BasisFuns(uint32_t i, double u, uint32_t degree, std::vector<uint32_t> knots) {
            u = std::min(u, static_cast<double>(knots[knots.size() - 1]));
            std::vector<double> bases(degree + 1, 0);
            bases[0] = 1.0;
            std::vector<double> left(degree + 1), right(degree + 1);
            for (uint32_t j = 1; j <= degree; ++j) {
                left[j] = u - static_cast<double>(knots[i + 1 - j]);
                right[j] = static_cast<double>(knots[i + j]) - u;
                double saved = 0.0;
                for (uint8_t r = 0; r < j; ++r) {
                    double temp = bases[r] / (right[r + 1] + left[j - r]);
                    bases[r] = saved + (right[r + 1] * temp);
                    saved = left[j - r] * temp;
                }
                bases[j] = saved;
            }
            return bases;
        };

        // ALGORITHM A2.3  DersBasisFuns(i,u,p,n,U,ders)
        // Same as above
        // n - nth derivative calculated (max)
        // ders - Returned basis derivative vector
        std::vector<std::vector<double>> DersBasisFuns(uint32_t i, double u, uint32_t degree,
            uint32_t n, std::vector<uint32_t> knots) {
            if (knots.empty()) {
                return {};
            }

            u = std::min(u, static_cast<double>(knots[knots.size() - 1]));

            if (i + degree == static_cast<uint32_t>(knots.size() - 1)) {
                std::vector<std::vector<double>> derivatives(n + 1);
                for (auto& vect : derivatives) {
                    vect.resize(degree + 1);
                    for (double& val : vect) {
                        val = 0.0;
                    }
                }
                return derivatives;
            }

            std::vector<std::vector<double>> ndu(degree + 1);
            for (auto& vect : ndu) {
                vect.resize(degree + 1, 0);
            }
            ndu[0][0] = 1.0;
            std::vector<double> left(degree + 1), right(degree + 1);
            for (uint32_t j = 1; j <= degree; ++j) {
                left[j] = u - static_cast<double>(knots[i + 1 - j]);
                right[j] = static_cast<double>(knots[i + j]) - u;

                double saved = 0.0;
                for (uint32_t r = 0; r < j; ++r) {
                    ndu[j][r] = right[r + 1] + left[j - r];
                    double temp = ndu[r][j - 1] / ndu[j][r];

                    ndu[r][j] = saved + (right[r + 1] * temp);
                    saved = left[j - r] * temp;
                }
                ndu[j][j] = saved;
            }

            std::vector<std::vector<double>> derivatives(n + 1);
            for (auto& vect : derivatives) {
                vect.resize(degree + 1);
            }

            for (uint32_t j = 0; j <= degree; ++j) {
                derivatives[0][j] = ndu[j][degree];
            }

            /* This section computes the derivatives */
            std::array<std::vector<double>, 2> a;
            a[0].resize(static_cast<size_t>(degree) + 1);
            a[1].resize(static_cast<size_t>(degree) + 1);

            for (int r = 0; r <= static_cast<int>(degree); ++r) {
                uint32_t s1 = 0, s2 = 1;
                a[0][0] = 1.0;
                for (int k = 1; k <= static_cast<int>(n); ++k) {
                    double d = 0.0;
                    int rk = r - k;
                    int pk = static_cast<int>(degree) - k;
                    if (r >= k) {
                        a[s2][0] = a[s1][0] / ndu[pk + 1][rk];
                        d = a[s2][0] * ndu[rk][pk];
                    }
                    int j1, j2;
                    if (rk >= -1) {
                        j1 = 1;
                    }
                    else {
                        j1 = -rk;
                    }
                    if (r - 1 <= pk) {
                        j2 = k - 1;
                    }
                    else {
                        j2 = static_cast<int>(degree) - r;
                    }
                    for (int j = j1; j <= j2; ++j) {
                        a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][rk + j];
                        d += a[s2][j] * ndu[rk + j][pk];
                    }
                    if (r <= pk) {
                        a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                        d += a[s2][k] * ndu[r][pk];
                    }
                    derivatives[k][r] = d;
                    std::swap(s1, s2);
                }
            }

            // Multiply through the correct factors
            double r = static_cast<double>(degree);
            for (int k = 1; k <= static_cast<int>(n); ++k) {

                for (uint8_t j = 0; j <= degree; ++j) {
                    derivatives[k][j] *= r;
                }
                r *= static_cast<double>(static_cast<int>(degree) - k);
            }
            return derivatives;
        }

        // ALGORITHM A2.4  OneBasisFun(p,m,U,i,u,Nip)
        // p - degree
        // m - max index of U
        // U - Knot vector
        // i - ith basis value
        // u - input value along the curve/knot vector
        double OneBasisFun(uint32_t degree, std::vector<uint32_t> knots, uint32_t i, double u) {
            u = std::min(u, static_cast<double>(knots[knots.size() - 1]));
            if ((i == 0 && u <= static_cast<double>(knots[0]) + std::numeric_limits<double>::epsilon()) ||
                (i == (static_cast<uint32_t>(knots.size()) - degree - 2) &&
                    u >= static_cast<double>(knots[knots.size() - 1]) - std::numeric_limits<double>::epsilon())) {
                return 1.0;
            }
            if (u < static_cast<double>(knots[i]) || u >= static_cast<double>(knots[i + degree + 1])) {
                return 0.0;
            }
            std::vector<double> N(degree + 1);
            for (uint32_t j = 0; j <= degree; ++j) {
                if (u >= static_cast<double>(knots[i + j]) - std::numeric_limits<double>::epsilon() &&
                    u < static_cast<double>(knots[i + j + 1]) - std::numeric_limits<double>::epsilon()) {
                    N[j] = 1.0;
                }
                else {
                    N[j] = 0.0;
                }
            }
            for (uint32_t k = 1; k <= degree; ++k) {
                double saved = 0.0;
                if (std::abs(N[0]) > std::numeric_limits<double>::epsilon()) {
                    saved = ((u - static_cast<double>(knots[i])) * N[0]) / (static_cast<double>(knots[i + k] - knots[i]));
                }
                for (uint32_t j = 0; j < degree + 1 - k; ++j) {
                    double u_left = static_cast<double>(knots[i + j + 1]);
                    double u_right = static_cast<double>(knots[i + j + k + 1]);
                    if (std::abs(N[j + 1]) <= std::numeric_limits<double>::epsilon()) {
                        N[j] = saved;
                        saved = 0.0;
                    }
                    else {
                        double temp = N[j + 1] / (u_right - u_left);
                        N[j] = saved + ((u_right - u) * temp);
                        saved = (u - u_left) * temp;
                    }
                }
            }
            return N[0];
        }

        // ALGORITHM A2.5 - DersOneBasisFun(p,m,U,i,u,n,ders) 
        // p - degree
        // m - max index of U
        // U - Knot vector
        // i - ith basis value
        // u - input value along the curve / knot vector
        // n - nth derivative calculated(max)
        //ders - Returned basis derivative vector for only the ith basis
        std::vector<double> DersOneBasisFun(uint32_t degree, std::vector<uint32_t> knots, uint32_t i, double u, uint32_t n) {
            u = std::min(u, static_cast<double>(knots[knots.size() - 1]));
            // Local property
            if (u < static_cast<double>(knots[i]) - std::numeric_limits<double>::epsilon() ||
                static_cast<size_t>(i + degree + 1) >= knots.size() /*||
                u >= static_cast<double>(knots[i + degree + 1]) - std::numeric_limits<double>::epsilon()*/) {
                return std::vector<double>(n + 1, 0.0);
            }
            std::vector<std::vector<double>> N(degree + 1);
            for (auto& vect : N) {
                vect.resize(degree + 1);
            }

            // Initialize zero-degree functs
            for (uint32_t j = 0; j <= degree; ++j) {
                if (u >= static_cast<double>(knots[i + j]) - std::numeric_limits<double>::epsilon() &&
                    u < static_cast<double>(knots[i + j + 1]) - std::numeric_limits<double>::epsilon()) {
                    N[j][0] = 1.0;
                }
                else {
                    N[j][0] = 0.0;
                }
            }

            // Compute full trianglar table
            for (uint32_t k = 1; k <= degree; ++k) {
                double saved = 0.0;
                if (std::abs(N[0][k - 1]) > std::numeric_limits<double>::epsilon()) {
                    saved = ((u - static_cast<double>(knots[i])) * N[0][k - 1]) / (static_cast<double>(knots[i + k] - knots[i]));
                }
                for (uint32_t j = 0; j < degree + 1 - k; ++j) {
                    double left = static_cast<double>(knots[i + j + 1]);
                    double right = static_cast<double>(knots[i + j + k + 1]);
                    if (std::abs(N[j + 1][k - 1]) <= std::numeric_limits<double>::epsilon()) {
                        N[j][k] = saved;
                        saved = 0.0;
                    }
                    else {
                        double temp = N[j + 1][k - 1] / (right - left);
                        N[j][k] = saved + ((right - u) * temp);
                        saved = (u - left) * temp;
                    }
                }
            }
            std::vector<double> derivatives(n + 1);
            // The function value
            derivatives[0] = N[0][degree];
            // Compute the derivatives
            for (uint32_t k = 1; k <= n; ++k) {
                // Load the appropriate column
                std::vector<double> ND(k + 1);
                for (uint32_t j = 0; j <= k; ++j) {
                    ND[j] = N[j][degree - k];
                }
                // Compute the table of width k
                for (uint32_t jj = 1; jj <= k; ++jj) {
                    double saved = 0.0;
                    if (std::abs(ND[0]) > std::numeric_limits<double>::epsilon()) {
                        saved = ND[0] / (static_cast<double>(knots[i + degree + jj - k] - knots[i]));
                    }
                    for (uint32_t j = 0; j < k + 1 - jj; ++j) {
                        double left = static_cast<double>(knots[i + j + 1]);
                        double right = static_cast<double>(knots[knots.size() - 1]);
                        if (static_cast<size_t>(i + j + degree + jj + 1) < knots.size()) {
                            right = static_cast<double>(knots[i + j + degree + jj + 1]);
                        }
                        if (std::abs(ND[j + 1]) <= std::numeric_limits<double>::epsilon()) {
                            ND[j] = static_cast<double>(degree + jj - k) * saved;
                            saved = 0.0;
                        }
                        else {
                            double temp = ND[j + 1] / (right - left);
                            ND[j] = static_cast<double>(degree + jj - k) * (saved - temp);
                            saved = temp;
                        }
                    }
                }
                derivatives[k] = ND[0];
            }
            return derivatives;
        }

        // ALGORITHM  ALLBasisFuns(i,u,p,U,N) p99
        // i - Span index (From find span)
        // u - input value along the curve/knot vector
        // p - degree
        // U - Knot vector
        // N - Returned basis vector
        // Blurb:
        /* We assume a routine, AllBasisFuns, which is a simple
           modification of Basis Funs (Algorithm A2. 2), to return all nonzero basis functions of all degrees from 0 up top.
           In particular, N[j] [i] is the value of the
           ith-degree basis function, Nspan-i+j,i(u), where 0<=i<=p and 0<=j<=i. */
        // This has not been tested or used, Though it likely doesn't work
        std::vector<std::vector<double>> AllBasisFuns(uint32_t span, double u, uint32_t degree, std::vector<uint32_t> knots) {
            u = std::min(u, static_cast<double>(knots[knots.size() - 1]));
            std::vector<std::vector<double>> bases(degree + 1);
            for (auto& vect : bases) {
                vect.resize(degree + 1, 0);
            }
            bases[0][0] = 1.0;
            std::vector<double> left(degree + 1), right(degree + 1);
            for (uint8_t j = 1; j <= degree; ++j) {
                left[j] = u - static_cast<double>(knots[span + 1 - j]);
                right[j] = static_cast<double>(knots[span + j]) - u;
                double saved = 0.0;
                for (uint8_t r = 0; r < j; ++r) {
                    double temp = bases[j - 1][r] / (right[r + 1] + left[j - r]);
                    bases[j][r] = saved + (right[r + 1] * temp);
                    saved = left[j - r] * temp;
                }
                bases[j][j] = saved;
            }
            return bases;
        }
    }
}
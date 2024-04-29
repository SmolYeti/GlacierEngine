#include "include/fundamental_geometric_algorithms.hpp"

#include "include/knot_utility_functions.hpp"

namespace nurbs {
namespace knots {
// 5.2 Knot Insertion
// ALGORITHM A5.2 CurvePntByCornerCut(n, p, U, Pw, u, C) p.153
// n - number of knots
// p - degree of the curve
// U - knot vector
// Pw - control points
// u - parameter point to extract
// C - point on curve returned
void CurvePntByCornerCut(uint32_t n, uint32_t p, std::vector<double> U,
                         std::vector<double> Pw, uint32_t u, double &C) {
  // Compute point on rational B-spline curve
  // Input: n,p,U,Pw,u
  // Output: c
  // Endpoints are special cases

  // TODO - Change w to the point's weight
  double w = 1.0;
  if (u == U[0]) {
    C = Pw[0] / w;
    return;
  }
  if (u == U[n + p + 1]) {
    C = Pw[n] / w;
    return;
  }
  uint32_t s = 0, k = 0;
  // FindSpanMult(n, p, u, U, &k, &s);
  // I *General case *I
  uint32_t r = p - s;
  std::vector<uint32_t> Rw;
  for (uint32_t i = 0; i <= r; i++)
    Rw[i] = Pw[k - p + i];
  for (uint32_t j = 1; j <= r; j++)
    for (uint32_t i = 0; i <= r - j; i++) {
      uint32_t alfa =
          (u - U[k - p + j + i]) / (U[i + k + 1] - U[k - p + j + i]);
      Rw[i] = (alfa * Rw[i + 1]) + ((1.0 - alfa) * Rw[i]);
    }
  C = Rw[0] / w;
}

// Algorithm A5.3 SurfaceKnotIns(np, p, UP, mp, q, VP, Pw, dir, uv, k, s, r, nq, UQ, mq, VQ, Qw)
// np - U knot vector size
// p - U degree
// UP - U knot vector
// mp - V knot vector size
// q - V degree
// VP - V knot vector
// Pw - control polygon
// dir - Inserting a knot in the u or v direction
// uv - u or v parameter to insert the knot
// k -
// s -
// r - 
// nq - Output U knot vector size
// UQ - Output U knot vector
// mq - Output V knot vector size
// VQ - Output V knot vector
// Qw - Output control polygon

} // namespace knots
} // namespace nurbs
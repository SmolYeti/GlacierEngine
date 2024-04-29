#include "include/nurbs_surface.hpp"

#include "include/b_spline_surface.hpp"
#include "include/knot_utility_functions.hpp"

namespace nurbs {
namespace {
glm::dvec2 CorrectParameter(glm::dvec2 param, glm::dvec2 u_internal,
                            glm::dvec2 v_internal, glm::dvec2 u_interval,
                            glm::dvec2 v_interval) {
  double u_interval_scale = u_interval.y - u_interval.x;
  double u_internal_scale = u_internal.y - u_internal.x;
  double v_interval_scale = v_interval.y - v_interval.x;
  double v_internal_scale = v_internal.y - v_internal.x;
  // convert to 0 to 1
  double u_param = (param.x - u_interval.x) / u_interval_scale;
  double v_param = (param.y - v_interval.x) / v_interval_scale;
  // convert to internal interval
  u_param = (u_param * u_internal_scale) + u_internal.x;
  v_param = (v_param * v_internal_scale) + v_internal.x;
  return {u_param, v_param};
}
} // namespace
NURBSSurface::NURBSSurface(uint32_t u_degree, uint32_t v_degree,
                           std::vector<double> u_knots,
                           std::vector<double> v_knots,
                           std::vector<std::vector<glm::dvec4>> control_polygon,
                           glm::dvec2 u_interval, glm::dvec2 v_interval)
    : Surface(u_interval, v_interval), u_degree_(u_degree), v_degree_(v_degree),
      u_knots_(u_knots), v_knots_(v_knots), control_polygon_(control_polygon),
      u_internal_interval_(u_interval), v_internal_interval_(v_interval) {
  if (!u_knots_.empty()) {
    u_internal_interval_ = {static_cast<double>(u_knots_[0]),
                            static_cast<double>(u_knots_[u_knots_.size() - 1])};
  }
  if (!v_knots_.empty()) {
    v_internal_interval_ = {static_cast<double>(v_knots_[0]),
                            static_cast<double>(v_knots_[v_knots_.size() - 1])};
  }
}

// ALGORITHM A4.3 SurfacePoint(n,p,U,m,q,V,Pw,u,v,S) p134
// Compute point on rational B-spline surface
glm::dvec3 NURBSSurface::EvaluatePoint(glm::dvec2 uv) const {
  if (uv.x < u_interval_.x) {
    uv.x = u_interval_.x;
  }
  if (uv.x > u_interval_.y) {
    uv.x = u_interval_.y;
  }
  if (uv.y < v_interval_.x) {
    uv.y = v_interval_.x;
  }
  if (uv.y > v_interval_.y) {
    uv.y = v_interval_.y;
  }
  glm::dvec2 in_param = uv;
  /*glm::dvec2 in_param = CorrectParameter(
      uv, u_internal_interval_, v_internal_interval_, u_interval_,
     v_interval_);*/
  uint32_t u_span =
      knots::FindSpanParam(u_degree_, u_knots_, in_param.x, kTolerance);
  std::vector<double> u_basis =
      knots::BasisFuns(u_span, in_param.x, u_degree_, u_knots_, kTolerance);
  uint32_t v_span =
      knots::FindSpanParam(v_degree_, v_knots_, in_param.y, kTolerance);
  std::vector<double> v_basis =
      knots::BasisFuns(v_span, in_param.y, v_degree_, v_knots_, kTolerance);
  glm::dvec4 point{0.0, 0.0, 0.0, 0.0};
  for (uint32_t i = 0; i <= v_degree_; ++i) {
    glm::dvec4 temp_point{0.0, 0.0, 0.0, 0.0};
    for (uint32_t j = 0; j <= u_degree_; ++j) {
      temp_point +=
          u_basis[j] *
          control_polygon_[u_span - u_degree_ + j][v_span - v_degree_ + i];
    }
    point += v_basis[i] * temp_point;
  }
  point /= point.w;
  return {point.x, point.y, point.z};
}

/*std::vector<glm::dvec3> NURBSSurface::EvaluatePoints(
    uint32_t u_sample_count, uint32_t v_sample_count) const {}*/

// ALGORITHM A4.4 RatSurfaceDerivs(Aders,wders,d,SKL) p.137
std::vector<std::vector<double>>
PolygonWeightDerivatives(glm::dvec2 uv, uint32_t u_degree, uint32_t v_degree,
                         const std::vector<double> &u_knots,
                         const std::vector<double> &v_knots,
                         const std::vector<std::vector<double>> &weights,
                         uint32_t max_derivative, double tolerance) {
  // Cap the derivative
  uint32_t max_deriv_u = std::min(u_degree, max_derivative);
  uint32_t max_deriv_v = std::min(v_degree, max_derivative);

  // Calculate the derivatives in the U and V directions
  std::vector<std::vector<double>> derivs(
      max_derivative + 1, std::vector<double>(max_derivative + 1, 0.0));
  uint32_t u_span = knots::FindSpanParam(u_degree, u_knots, uv.x, tolerance);
  std::vector<std::vector<double>> u_derivs =
      knots::DersBasisFuns(u_span, uv.x, u_degree, max_derivative, u_knots);
  uint32_t v_span = knots::FindSpanParam(v_degree, v_knots, uv.y, tolerance);
  std::vector<std::vector<double>> v_derivs =
      knots::DersBasisFuns(v_span, uv.y, v_degree, max_derivative, v_knots);

  // Calculate the derivative polygon
  for (uint32_t i = 0; i <= max_deriv_u; ++i) {
    std::vector<double> temp_derivs(v_degree + 1, 0.0);
    for (uint32_t j = 0; j <= v_degree; ++j) {
      for (uint32_t k = 0; k <= u_degree; ++k) {
        temp_derivs[j] += u_derivs[i][k] *
                          weights[u_span - u_degree + k][v_span - v_degree + j];
      }
    }
    uint32_t dd = std::min(max_derivative - i, max_deriv_v);
    for (uint32_t j = 0; j <= dd; ++j) {
      for (uint32_t k = 0; k <= v_degree; ++k) {
        derivs[i][j] += v_derivs[j][k] * temp_derivs[k];
      }
    }
  }

  return derivs;
}

std::vector<std::vector<glm::dvec3>>
NURBSSurface::Derivatives(glm::dvec2 uv, uint32_t max_derivative) const {
  // Split the control polygon into the b_spline and weights control polygons
  std::vector<std::vector<glm::dvec3>> bspl_cpts;
  std::vector<std::vector<double>> weights;
  for (auto &vec : control_polygon_) {
    bspl_cpts.push_back({});
    weights.push_back({});

    for (auto &cpt : vec) {
      bspl_cpts.back().push_back({cpt.x, cpt.y, cpt.z});
      weights.back().push_back(cpt.w);
    }
  }

  BSplineSurface bspl_surface(u_degree_, v_degree_, u_knots_, v_knots_,
                              bspl_cpts, u_interval_, v_interval_);
  // Calculate the point and weight derivatives
  std::vector<std::vector<glm::dvec3>> a_derivs =
      bspl_surface.Derivative(uv, max_derivative);
  std::vector<std::vector<double>> weight_derivs =
      PolygonWeightDerivatives(uv, u_degree_, v_degree_, u_knots_, v_knots_,
                               weights, max_derivative, kTolerance);
  std::vector<std::vector<double>> bin =
      knots::BinomialCoefficients(max_derivative, max_derivative);

  // Calculate the derivatives
  std::vector<std::vector<glm::dvec3>> derivs(
      max_derivative + 1,
      std::vector<glm::dvec3>(max_derivative + 1, {0.0, 0.0, 0.0}));
  for (uint32_t k = 0; k <= max_derivative; ++k) {
    for (uint32_t l = 0; l <= max_derivative - k; ++l) {
      glm::dvec3 v = a_derivs[k][l];
      for (uint32_t j = 1; j <= l; ++j) {
        v -= bin[l][j] * weight_derivs[0][j] * derivs[k][l - j];
      }
      for (uint32_t i = 1; i <= k; ++i) {
        v -= bin[k][i] * weight_derivs[i][0] * derivs[k - i][l];

        glm::dvec3 v2 = {0.0, 0.0, 0.0};
        for (uint32_t j = 1; j <= l; ++j) {
          v2 += bin[l][j] * weight_derivs[i][j] * derivs[k - i][l - j];
        }
        v -= bin[k][i] * v2;
      }
      derivs[k][l] = v / weight_derivs[0][0];
    }
  }
  return derivs;
}

// Algorithm 5.3 SurfaceKnotIns p.155
NURBSSurface NURBSSurface::KnotInsert(SurfaceDirection dir, double knot,
                                      uint32_t times) {
  // Input: np,p,UP,Pw,u,k,s,r
  // np - Number of columns in the control polygon
  // p - U Degree of the surface
  // UP - U Knot vector before insertion
  // mp - Number of rows in the control polygon
  // q - V degree of the surface
  // Pw - Control points before insertion
  // dir - Direction to insert the knot (U or V)
  // uv - value of knot to insert
  // k - location of knot in insert
  // s - inital knot multiplicity
  // r - times to insert knot
  auto np = control_polygon_.size();
  auto p = u_degree_;
  auto UP = u_knots_;
  auto mp = control_polygon_[0].size();
  auto q = v_degree_;
  auto VP = v_knots_;
  auto Pw = control_polygon_;
  auto uv = knot;
  auto r = times;

  // Output
  // nq - Number of columns in the control polygon
  // UQ - U knot vector
  // mq - Number of rows in the control polygon
  // VQ - V knot vector
  // Qw - Control Polygon
  std::vector<double> UQ;
  std::vector<double> VQ;
  std::vector<std::vector<glm::dvec4>> Qw;

  if (dir == SurfaceDirection::kUDir) {
    // Get the remaining input values
    auto k = knots::FindSpanParam(p, UP, knot, kTolerance);
    auto s = knots::MultiplicityParam(p, UP, knot, kTolerance);

    // Max value of r is (p + 1) - s
    r = std::min(r, (p + 1) - s);

    // Resize the output arrays
    UQ.resize(UP.size() + r);
    Qw.resize(Pw.size() + r);
    for (auto &column : Qw) {
      column.resize(mp);
    }

    // Load u vector as in A5.1
    // Load new knot vector
    for (uint32_t i = 0; i <= k; i++) {
      UQ[i] = UP[i];
    }
    for (uint32_t i = 1; i <= r; i++) {
      UQ[k + i] = uv;
    }
    for (uint32_t i = k + 1; i < UP.size(); i++) {
      UQ[i + r] = UP[i];
    }

    // Convert the knot to a double
    double u = static_cast<double>(uv);

    // Copy v vector into VQ (return vector)
    VQ = v_knots_;

    // Save the Alphas
    std::vector<std::vector<double>> alphas(p + 1);
    for (auto &alpha : alphas) {
      alpha.resize(r + 1);
    }

    uint32_t L;
    for (uint32_t j = 1; j <= r; ++j) {
      L = k - p + j;
      for (uint32_t i = 0; i <= p - j - s + 1; i++) {
        alphas[i][j] = (u - static_cast<double>(UP[L + i])) /
                       (static_cast<double>(UP[i + k + 1]) -
                        static_cast<double>(UP[L + i]));
      }
    }
    std::vector<glm::dvec4> Rw(p + 1);

    for (size_t row = 0; row < mp; row++) {
      /* Save unaltered control points */
      for (uint32_t i = 0; i <= k - p; ++i) {
        Qw[i][row] = Pw[i][row];
      }
      for (size_t i = k; i < np; i++) {
        Qw[i + r][row] = Pw[i][row];
      }
      // Load auxiliary control points
      for (uint32_t i = 0; i <= p - s; ++i) {
        Rw[i] = Pw[k - p + i][row];
      }

      for (uint32_t j = 1; j <= r; ++j) {
        L = k - p + j;
        for (uint32_t i = 0; i <= p - j - s + 1; ++i) {
          Rw[i] = (alphas[i][j] * Rw[i + 1]) + ((1.0 - alphas[i][j]) * Rw[i]);
        }
        Qw[L][row] = Rw[0];
        Qw[k + r - j][row] = Rw[p - j];
      }

      // Load the remaining control points
      for (uint32_t i = L + 1; i < k; ++i) {
        Qw[i][row] = Rw[i - L];
      }
    }

  } else {
    /* Similar code as above with u and v directional parameters switched */

    // Get the remaining input values
    auto k = knots::FindSpanParam(q, VP, knot, kTolerance);
    auto s = knots::MultiplicityParam(q, VP, knot, kTolerance);

    // Max value of r is (p + 1) - s
    r = std::min(r, (q + 1) - s);

    // Resize the output arrays
    VQ.resize(VP.size() + r);
    Qw.resize(np);
    for (auto &column : Qw) {
      column.resize(mp + r);
    }

    // Load v vector as in A5.1
    // Load new knot vector
    for (uint32_t i = 0; i <= k; i++) {
      VQ[i] = VP[i];
    }
    for (uint32_t i = 1; i <= r; i++) {
      VQ[k + i] = uv;
    }
    for (uint32_t i = k + 1; i < VP.size(); i++) {
      VQ[i + r] = VP[i];
    }

    // Convert the knot to a double
    double v = static_cast<double>(uv);

    // Copy v vector into VQ (return vector)
    UQ = UP;

    // Save the Alphas
    std::vector<std::vector<double>> alphas(q + 1);
    for (auto &alpha : alphas) {
      alpha.resize(r + 1);
    }

    uint32_t L;
    for (uint32_t j = 1; j <= r; ++j) {
      L = k - q + j;
      for (uint32_t i = 0; i <= q - j - s + 1; i++) {
        alphas[i][j] = (v - static_cast<double>(VP[L + i])) /
                       (static_cast<double>(VP[i + k + 1]) -
                        static_cast<double>(VP[L + i]));
      }
    }
    std::vector<glm::dvec4> Rw(q + 1);

    for (size_t col = 0; col < np; col++) {
      /* Save unaltered control points */
      for (uint32_t i = 0; i <= k - q; ++i) {
        Qw[col][i] = Pw[col][i];
      }
      for (size_t i = k; i < mp; i++) {
        Qw[col][i + r] = Pw[col][i];
      }
      // Load auxiliary control points
      for (uint32_t i = 0; i <= q - s; ++i) {
        Rw[i] = Pw[col][k - q + i];
      }

      for (uint32_t j = 1; j <= r; ++j) {
        L = k - q + j;
        for (uint32_t i = 0; i <= q - j - s + 1; ++i) {
          Rw[i] = (alphas[i][j] * Rw[i + 1]) + ((1.0 - alphas[i][j]) * Rw[i]);
        }
        Qw[col][L] = Rw[0];
        Qw[col][k + r - j] = Rw[q - j];
      }

      // Load the remaining control points
      for (uint32_t i = L + 1; i < k; ++i) {
        Qw[col][i] = Rw[i - L];
      }
    }
  }
  return NURBSSurface(u_degree_, v_degree_, UQ, VQ, Qw, u_interval_,
                      v_interval_);
}
} // namespace nurbs
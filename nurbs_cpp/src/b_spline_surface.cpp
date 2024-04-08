#include "include/b_spline_surface.hpp"

#include "include/b_spline_curve.hpp"
#include "include/knot_utility_functions.hpp"

namespace nurbs {

BSplineSurface::BSplineSurface(
    uint32_t u_degree, uint32_t v_degree, const std::vector<uint32_t> &u_knots,
    const std::vector<uint32_t> &v_knots,
    const std::vector<std::vector<glm::dvec3>> &control_polygon,
    glm::dvec2 u_interval, glm::dvec2 v_interval)
    : Surface(u_interval, v_interval), u_degree_(u_degree), v_degree_(v_degree),
      u_knots_(u_knots), v_knots_(v_knots), control_polygon_(control_polygon) {}

// Chaper 3, ALGORITHM A3.5: SSurfacePoint(n,p,U,m,q,V,P,u,v,S) p103
glm::dvec3 BSplineSurface::EvaluatePoint(glm::dvec2 uv) const {
  uint32_t u_span = knots::FindSpanParam(u_degree_, u_knots_, uv.x, kTolerance);
  std::vector<double> u_bases =
      knots::BasisFuns(u_span, uv.x, u_degree_, u_knots_, kTolerance);
  uint32_t u_ind = u_span - u_degree_;
  uint32_t v_span = knots::FindSpanParam(v_degree_, v_knots_, uv.y, kTolerance);
  std::vector<double> v_bases =
      knots::BasisFuns(v_span, uv.y, v_degree_, v_knots_, kTolerance);
  glm::dvec3 point{0, 0, 0};
  for (uint32_t i = 0; i <= v_degree_; ++i) {
    glm::dvec3 temp = {0.0, 0.0, 0.0};
    uint32_t v_ind = v_span - v_degree_ + i;
    for (uint32_t j = 0; j <= u_degree_; ++j) {
      temp += u_bases[j] * control_polygon_[u_ind + j][v_ind];
    }
    point += v_bases[i] * temp;
  }
  return point;
}

std::vector<glm::dvec3>
BSplineSurface::EvaluatePoints(uint32_t u_sample_count,
                               uint32_t v_sample_count) const {
  std::vector<glm::dvec3> points(u_sample_count * v_sample_count, {0, 0, 0});
  double u_div =
      (u_interval_.y - u_interval_.x) / static_cast<double>(u_sample_count - 1);
  double v_div =
      (v_interval_.y - v_interval_.x) / static_cast<double>(v_sample_count - 1);
  for (uint32_t u_i = 0; u_i < u_sample_count; ++u_i) {
    glm::dvec2 uv = {u_interval_.x + static_cast<double>(u_i) * u_div, 0};
    uint32_t u_span =
        knots::FindSpanParam(u_degree_, u_knots_, uv.x, kTolerance);
    std::vector<double> u_bases =
        knots::BasisFuns(u_span, uv.x, u_degree_, u_knots_, kTolerance);
    uint32_t u_ind = u_span - u_degree_;
    for (uint32_t v_i = 0; v_i < v_sample_count; ++v_i) {
      uv.y = v_interval_.x + static_cast<double>(v_i) * v_div;
      uint32_t v_span =
          knots::FindSpanParam(v_degree_, v_knots_, uv.y, kTolerance);
      std::vector<double> v_bases =
          knots::BasisFuns(v_span, uv.y, v_degree_, v_knots_, kTolerance);
      for (uint32_t i = 0; i <= v_degree_; ++i) {
        glm::dvec3 temp = {0.0, 0.0, 0.0};
        uint32_t v_ind = v_span - v_degree_ + i;
        for (uint32_t j = 0; j <= u_degree_; ++j) {
          temp += u_bases[j] * control_polygon_[u_ind + j][v_ind];
        }
        points[(u_i * v_sample_count) + v_i] += v_bases[i] * temp;
      }
    }
  }
  return points;
}

// Chaper 3, ALGORITHM A3.6: SurfaceDerivsA1g1(n, p, U, m, q, V, P, u, v, d,
// SKL) p111
std::vector<std::vector<glm::dvec3>>
BSplineSurface::Derivative(glm::dvec2 uv, uint32_t max_derivative) const {
  uint32_t max_deriv_u = std::min(u_degree_, max_derivative);
  uint32_t max_deriv_v = std::min(v_degree_, max_derivative);
  std::vector<std::vector<glm::dvec3>> derivs(max_derivative + 1);
  for (auto &vec : derivs) {
    vec = std::vector<glm::dvec3>(max_derivative + 1, {0, 0, 0});
  }
  uint32_t u_span = knots::FindSpanParam(u_degree_, u_knots_, uv.x, kTolerance);
  std::vector<std::vector<double>> u_derivs =
      knots::DersBasisFuns(u_span, uv.x, u_degree_, max_deriv_u, u_knots_);
  uint32_t v_span = knots::FindSpanParam(v_degree_, v_knots_, uv.y, kTolerance);
  std::vector<std::vector<double>> v_derivs =
      knots::DersBasisFuns(v_span, uv.y, v_degree_, max_deriv_v, v_knots_);
  for (uint32_t i = 0; i <= max_deriv_u; ++i) {
    std::vector<glm::dvec3> temp_derivs(v_degree_ + 1, {0, 0, 0});
    for (uint32_t j = 0; j <= v_degree_; ++j) {
      for (uint32_t k = 0; k <= u_degree_; ++k) {
        temp_derivs[j] +=
            u_derivs[i][k] *
            control_polygon_[u_span - u_degree_ + k][v_span - v_degree_ + j];
      }
    }
    uint32_t dd = std::min(max_derivative - i, max_deriv_v);
    for (uint32_t j = 0; j <= dd; ++j) {
      for (uint32_t k = 0; k <= v_degree_; ++k) {
        derivs[i][j] += v_derivs[j][k] * temp_derivs[k];
      }
    }
  }
  return derivs;
}

// TODOS:
// Chapter 3: ALGORITHM A3.7: SurfaceDerivCpts(n, p, U, m, q, V, P, d, r1, r2,
// s1, s2, PKL) p 114
std::vector<std::vector<std::vector<std::vector<glm::dvec3>>>>
BSplineSurface::SurfaceDerivCpts(uint32_t d, uint32_t r_start, uint32_t r_end,
                                 uint32_t s_start, uint32_t s_end) const {
  if (control_polygon_.empty()) {
    return {};
  }
  // Not sure which one is r and s, but one should span the u direction and the
  // other the v direction... I think R is u and s if v, but I will update after
  // I get this working
  r_end = std::min(static_cast<size_t>(r_end), control_polygon_.size() - 1);
  s_end = std::min(static_cast<size_t>(s_end), control_polygon_[0].size() - 1);

  uint32_t du = std::min(d, u_degree_);
  uint32_t dv = std::min(d, v_degree_);

  uint32_t r_total = r_end - r_start;
  uint32_t s_total = s_end - s_start;
  std::vector<std::vector<std::vector<std::vector<glm::dvec3>>>> PKL;
  PKL.resize(du + 1);
  for (auto &vec_0 : PKL) {
    vec_0.resize(dv + 1);
    for (auto &vec_1 : vec_0) {
      vec_1.resize(r_total + 1);
      for (auto &vec_2 : vec_1) {
        vec_2 = std::vector<glm::dvec3>(s_total + 1, {0.0, 0.0, 0.0});
      }
    }
  }

  for (uint32_t j = s_start; j <= s_end; ++j) {
    // Not really sure wtf &P[][j] is... but I assume it is a vector from
    // P[0][j] to P[n][j].
    std::vector<glm::dvec3> control_points;
    for (size_t u = 0; u < control_polygon_.size(); ++u) {
      control_points.push_back(control_polygon_[u][j]);
    }
    BSplineCurve3D curve(u_degree_, control_points, u_knots_);
    std::vector<std::vector<glm::dvec3>> temp =
        curve.DerivativeControlPoints(du, r_start, r_end);
    for (uint32_t k = 0; k <= du; ++k) {
      for (uint32_t i = 0; i < r_total - k; ++i) {
        PKL[k][0][i][j - s_start] = temp[k][i];
      }
    }
  }
  for (uint32_t k = 0; k <= du; ++k) {
    for (uint32_t i = 0; i < r_total - k; ++i) {
      uint32_t dd = std::min(d - k, dv);
      std::vector<uint32_t> knots;
      for (size_t knot_idx = s_start; knot_idx < v_knots_.size(); ++knot_idx) {
        knots.push_back(v_knots_[knot_idx]);
      }
      BSplineCurve3D curve(v_degree_, PKL[k][0][i], knots);
      std::vector<std::vector<glm::dvec3>> temp =
          curve.DerivativeControlPoints(dd, 0, s_total);
      for (uint32_t l = 1; l <= dd; ++l) {
        for (uint32_t j = 0; j <= s_total - l; ++j) {
          PKL[k][l][i][j] = temp[l][j];
        }
      }
    }
  }
  return PKL;
}

// Chapter 3: ALGORITHM A3.8: SurfaceDerivsA1g2(n,p,U,m,q,V,P,u,v,d,SKL) p115
std::vector<std::vector<glm::dvec3>>
BSplineSurface::Derivatives2(glm::dvec2 uv, uint32_t max_derivative) const {
  std::vector<std::vector<glm::dvec3>> derivatives(max_derivative + 1);
  for (std::vector<glm::dvec3> &vec_d : derivatives) {
    vec_d.resize(max_derivative + 1);
  }
  uint32_t du = std::min(max_derivative, u_degree_);
  for (uint32_t k = u_degree_ + 1; k <= max_derivative; ++k) {
    for (uint32_t l = 0; l <= max_derivative - k; ++l) {
      derivatives[k][l] = {0.0, 0.0, 0.0};
    }
  }
  uint32_t dv = std::min(max_derivative, v_degree_);
  for (uint32_t l = u_degree_ + 1; l <= max_derivative; ++l) {
    for (uint32_t k = 0; k <= max_derivative - l; ++k) {
      derivatives[k][l] = {0.0, 0.0, 0.0};
    }
  }

  uint32_t u_span = knots::FindSpanParam(u_degree_, u_knots_, uv.x, kTolerance);
  std::vector<std::vector<double>> u_basis =
      knots::AllBasisFuns(u_span, uv.x, u_degree_, u_knots_);
  uint32_t v_span = knots::FindSpanParam(v_degree_, v_knots_, uv.y, kTolerance);
  std::vector<std::vector<double>> v_basis =
      knots::AllBasisFuns(v_span, uv.y, v_degree_, v_knots_);

  auto surf_ctps = SurfaceDerivCpts(max_derivative, u_span - u_degree_, u_span,
                                    v_span - v_degree_, v_span);

  for (uint32_t k = 0; k <= du; ++k) {
    uint32_t dd = std::min(max_derivative - k, dv);
    for (uint32_t l = 0; l <= dd; ++l) {
      derivatives[k][l] = {0.0, 0.0, 0.0};
      for (uint32_t i = 0; i <= v_degree_ - l; ++i) {
        glm::dvec3 temp = {0.0, 0.0, 0.0};
        for (uint32_t j = 0; j <= u_degree_ - k; ++j) {
          temp += u_basis[j][u_degree_ - k] * surf_ctps[k][l][j][i];
        }
        derivatives[k][l] += v_basis[i][v_degree_ - l] * temp;
      }
    }
  }

  return derivatives;
}
} // namespace nurbs
#include "include/nurbs_surface.hpp"

#include "include/b_spline_surface.hpp"
#include "include/knot_utility_functions.hpp"

namespace nurbs {
NURBSSurface::NURBSSurface(uint32_t u_degree, uint32_t v_degree,
                           std::vector<uint32_t> u_knots,
                           std::vector<uint32_t> v_knots,
                           std::vector<std::vector<glm::dvec4>> control_polygon,
                           glm::dvec2 u_interval, glm::dvec2 v_interval)
    : Surface(u_interval, v_interval),
      u_degree_(u_degree),
      v_degree_(v_degree),
      u_knots_(u_knots),
      v_knots_(v_knots),
      control_polygon_(control_polygon) {}

// ALGORITHM A4.3 SurfacePoint(n,p,U,m,q,V,Pw,u,v,S) p134
// Compute point on rational B-spline surface
glm::dvec3 NURBSSurface::EvaluatePoint(glm::dvec2 uv) const {
  uint32_t u_span = knots::FindSpan(u_degree_, uv.x, u_knots_);
  std::vector<double> u_basis =
      knots::BasisFuns(u_span, uv.x, u_degree_, u_knots_);
  uint32_t v_span = knots::FindSpan(v_degree_, uv.y, v_knots_);
  std::vector<double> v_basis =
      knots::BasisFuns(v_span, uv.y, v_degree_, v_knots_);
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
std::vector<std::vector<double>> PolygonWeightDerivatives(
    glm::dvec2 uv, uint32_t u_degree, uint32_t v_degree,
    const std::vector<uint32_t>& u_knots, const std::vector<uint32_t>& v_knots,
    const std::vector<std::vector<double>>& weights, uint32_t max_derivative) {
  // Cap the derivative
  uint32_t max_deriv_u = std::min(u_degree, max_derivative);
  uint32_t max_deriv_v = std::min(v_degree, max_derivative);

  // Calculate the derivatives in the U and V directions
  std::vector<std::vector<double>> derivs(
      max_derivative + 1, std::vector<double>(max_derivative + 1, 0.0));
  uint32_t u_span =
      static_cast<uint32_t>(knots::FindSpan(u_degree, uv.x, u_knots));
  std::vector<std::vector<double>> u_derivs =
      knots::DersBasisFuns(u_span, uv.x, u_degree, max_derivative, u_knots);
  uint32_t v_span =
      static_cast<uint32_t>(knots::FindSpan(v_degree, uv.y, v_knots));
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

std::vector<std::vector<glm::dvec3>> NURBSSurface::Derivatives(
    glm::dvec2 uv, uint32_t max_derivative) const {
  // Split the control polygon into the b_spline and weights control polygons
  std::vector<std::vector<glm::dvec3>> bspl_cpts;
  std::vector<std::vector<double>> weights;
  for (auto& vec : control_polygon_) {
    bspl_cpts.push_back({});
    weights.push_back({});

    for (auto& cpt : vec) {
      bspl_cpts.back().push_back({cpt.x, cpt.y, cpt.z});
      weights.back().push_back(cpt.w);
    }
  }

  BSplineSurface bspl_surface(u_degree_, v_degree_, u_knots_, v_knots_,
                              bspl_cpts, u_interval_, v_interval_);
  // Calculate the point and weight derivatives
  std::vector<std::vector<glm::dvec3>> a_derivs =
      bspl_surface.Derivative(uv, max_derivative);
  std::vector<std::vector<double>> weight_derivs = PolygonWeightDerivatives(
      uv, u_degree_, v_degree_, u_knots_, v_knots_, weights, max_derivative);
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
}  // namespace nurbs
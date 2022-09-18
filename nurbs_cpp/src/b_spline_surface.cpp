#include "include/b_spline_surface.hpp"

#include "include/knot_utility_functions.hpp"

namespace nurbs {
    BSplineSurface::BSplineSurface(uint32_t u_degree, uint32_t v_degree,
        std::vector<uint32_t> u_knots, std::vector<uint32_t> v_knots,
        std::vector< std::vector<glm::dvec3>> control_polygon, glm::dvec2 u_interval,
        glm::dvec2 v_interval) :
        Surface(u_interval, v_interval),
        u_degree_(u_degree), v_degree_(v_degree),
        u_knots_(u_knots), v_knots_(v_knots),
        control_polygon_(control_polygon) {}

    // Chaper 3, ALGORITHM A3.5: SSurfacePoint(n,p,U,m,q,V,P,u,v,S) p103
    glm::dvec3 BSplineSurface::EvaluatePoint(glm::dvec2 uv) const {
        uint32_t u_span = knots::FindSpan(u_degree_, uv.x, u_knots_);
        std::vector<double> u_bases = knots::BasisFuns(u_span, uv.x, u_degree_, u_knots_);
        uint32_t u_ind = u_span - u_degree_;
        uint32_t v_span = knots::FindSpan(v_degree_, uv.y, v_knots_);
        std::vector<double> v_bases = knots::BasisFuns(v_span, uv.y, v_degree_, v_knots_);
        glm::dvec3 point{ 0,0,0 };
        for (uint32_t i = 0; i <= v_degree_; ++i) {
            glm::dvec3 temp = { 0.0, 0.0, 0.0 };
            uint32_t v_ind = v_span - v_degree_ + i;
            for (uint32_t j = 0; j <= u_degree_; ++j) {
                temp += u_bases[j] * control_polygon_[u_ind + j][v_ind];
            }
            point += v_bases[i] * temp;
        }
        return point;
    }

    std::vector<glm::dvec3> BSplineSurface::EvaluatePoints(
        uint32_t u_sample_count, uint32_t v_sample_count) const {
        std::vector<glm::dvec3> points(u_sample_count * v_sample_count, { 0,0,0 });
        double u_div = (u_interval_.y - u_interval_.x) / static_cast<double>(u_sample_count - 1);
        double v_div = (v_interval_.y - v_interval_.x) / static_cast<double>(v_sample_count - 1);
        for (uint32_t u_i = 0; u_i < u_sample_count; ++u_i) {
            glm::dvec2 uv = { u_interval_.x + static_cast<double>(u_i) * u_div, 0 };
            uint32_t u_span = knots::FindSpan(u_degree_, uv.x, u_knots_);
            std::vector<double> u_bases = knots::BasisFuns(u_span, uv.x, u_degree_, u_knots_);
            uint32_t u_ind = u_span - u_degree_;
            for (uint32_t v_i = 0; v_i < v_sample_count; ++v_i) {
                uv.y = v_interval_.x + static_cast<double>(v_i) * v_div;
                uint32_t v_span = knots::FindSpan(v_degree_, uv.y, v_knots_);
                std::vector<double> v_bases = knots::BasisFuns(v_span, uv.y, v_degree_, v_knots_);
                for (uint32_t i = 0; i <= v_degree_; ++i) {
                    glm::dvec3 temp = { 0.0, 0.0, 0.0 };
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

    // Chaper 3, ALGORITHM A3.6: SurfaceDerivsA1g1(n, p, U, m, q, V, P, u, v, d, SKL) p111
    std::vector<std::vector<glm::dvec3>> BSplineSurface::Derivative(glm::dvec2 uv, uint32_t max_derivative) const {
        uint32_t max_deriv_u = std::min(u_degree_, max_derivative);
        uint32_t max_deriv_v = std::min(v_degree_, max_derivative);
        std::vector<std::vector<glm::dvec3>> derivs(max_derivative + 1);
        for (auto& vec : derivs) {
            vec = std::vector<glm::dvec3>(max_derivative + 1, { 0, 0, 0 });
        }
        uint32_t u_span = knots::FindSpan(u_degree_, uv.x, u_knots_);
        std::vector<std::vector<double>> u_derivs = knots::DersBasisFuns(u_span, uv.x, u_degree_, max_deriv_u, u_knots_);
        uint32_t v_span = knots::FindSpan(v_degree_, uv.y, v_knots_);
        std::vector<std::vector<double>> v_derivs = knots::DersBasisFuns(v_span, uv.y, v_degree_, max_deriv_v, v_knots_);
        for (uint32_t i = 0; i <= max_deriv_u; ++i) {
            std::vector<glm::dvec3> temp_derivs(v_degree_ + 1, { 0,0,0 });
            for (uint32_t j = 0; j <= v_degree_; ++j) {
                for (uint32_t k = 0; k <= u_degree_; ++k) {
                    temp_derivs[j] += u_derivs[i][k] * control_polygon_[u_span - u_degree_ + j][v_span - v_degree_ + j];
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
    // Chapter 3: ALGORITHM A3.7: SurfaceDerivCpts(n, p, U, m, q, V, P, d, r1, r2, s1, s2, PKL) p 114
    // Chapter 3: ALGORITHM A3.8: SurfaceDerivsA1g2(n,p,U,m,q,V,P,u,v,d,SKL) p115
}
#include "include/nurbs_surface.hpp"

#include "include/b_spline_surface.hpp"
#include "include/knot_utility_functions.hpp"

namespace nurbs {
namespace {}  // namespace
NURBSSurface::NURBSSurface(uint32_t u_degree, uint32_t v_degree,
                           std::vector<double> u_knots,
                           std::vector<double> v_knots,
                           std::vector<std::vector<Point4D>> control_polygon,
                           Point2D u_interval, Point2D v_interval)
    : Surface(u_interval, v_interval),
      u_degree_(u_degree),
      v_degree_(v_degree),
      u_knots_(u_knots),
      v_knots_(v_knots),
      control_polygon_(control_polygon),
      u_internal_interval_(u_interval),
      v_internal_interval_(v_interval) {
  if (u_knots_.size() != control_polygon_.size() + u_degree_ + 1) {
    throw std::exception("Invalid U Parameters for a NURBS Surface");
  }
  if (!control_polygon_.empty() &&
      v_knots_.size() != control_polygon_[0].size() + v_degree_ + 1) {
    throw std::exception("Invalid V Parameters for a NURBS Surface");
  }
}

// ALGORITHM A4.3 SurfacePoint(n,p,U,m,q,V,Pw,u,v,S) p134
// Compute point on rational B-spline surface
Point3D NURBSSurface::EvaluatePoint(Point2D uv) const {
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
  Point2D in_param = uv;
  /*Point2D in_param = CorrectParameter(
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
  Point4D point{0.0, 0.0, 0.0, 0.0};
  for (uint32_t i = 0; i <= v_degree_; ++i) {
    Point4D temp_point{0.0, 0.0, 0.0, 0.0};
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

/*std::vector<Point3D> NURBSSurface::EvaluatePoints(
    uint32_t u_sample_count, uint32_t v_sample_count) const {}*/

// ALGORITHM A4.4 RatSurfaceDerivs(Aders,wders,d,SKL) p.137
std::vector<std::vector<double>> PolygonWeightDerivatives(
    Point2D uv, uint32_t u_degree, uint32_t v_degree,
    const std::vector<double>& u_knots, const std::vector<double>& v_knots,
    const std::vector<std::vector<double>>& weights, uint32_t max_derivative,
    double tolerance) {
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

std::vector<std::vector<Point3D>> NURBSSurface::Derivatives(
    Point2D uv, uint32_t max_derivative) const {
  // Split the control polygon into the b_spline and weights control polygons
  std::vector<std::vector<Point3D>> bspl_cpts;
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
  std::vector<std::vector<Point3D>> a_derivs =
      bspl_surface.Derivative(uv, max_derivative);
  std::vector<std::vector<double>> weight_derivs =
      PolygonWeightDerivatives(uv, u_degree_, v_degree_, u_knots_, v_knots_,
                               weights, max_derivative, kTolerance);
  std::vector<std::vector<double>> bin =
      knots::BinomialCoefficients(max_derivative, max_derivative);

  // Calculate the derivatives
  std::vector<std::vector<Point3D>> derivs(
      max_derivative + 1,
      std::vector<Point3D>(max_derivative + 1, {0.0, 0.0, 0.0}));
  for (uint32_t k = 0; k <= max_derivative; ++k) {
    for (uint32_t l = 0; l <= max_derivative - k; ++l) {
      Point3D v = a_derivs[k][l];
      for (uint32_t j = 1; j <= l; ++j) {
        v -= bin[l][j] * weight_derivs[0][j] * derivs[k][l - j];
      }
      for (uint32_t i = 1; i <= k; ++i) {
        v -= bin[k][i] * weight_derivs[i][0] * derivs[k - i][l];

        Point3D v2 = {0.0, 0.0, 0.0};
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
                                      int times) const {
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
  int p = u_degree_;
  auto UP = u_knots_;
  int q = v_degree_;
  auto VP = v_knots_;
  auto Pw = control_polygon_;
  auto uv = knot;
  int r = times;

  // Output
  // nq - Number of columns in the control polygon
  // UQ - U knot vector
  // mq - Number of rows in the control polygon
  // VQ - V knot vector
  // Qw - Control Polygon
  std::vector<double> UQ;
  std::vector<double> VQ;
  std::vector<std::vector<Point4D>> Qw;

  if (dir == SurfaceDirection::kUDir) {
    // Get the remaining input values
    int k = knots::FindSpanParam(p, UP, knot, kTolerance);
    int s = knots::MultiplicityParam(p, UP, knot, kTolerance);

    // Max value of r is p - s
    r = std::min(r, p - s);

    // Resize the output arrays
    UQ.resize(UP.size() + r);
    Qw.resize(Pw.size() + r);
    for (auto& column : Qw) {
      column.resize(Pw[0].size());
    }

    // Load u vector as in A5.1
    // Load new knot vector
    for (int i = 0; i <= k; i++) {
      UQ[i] = UP[i];
    }
    for (int i = 1; i <= r; i++) {
      UQ[k + i] = uv;
    }
    for (int i = k + 1; i < UP.size(); i++) {
      UQ[i + r] = UP[i];
    }

    // Copy v vector into VQ (return vector)
    VQ = v_knots_;

    // Save the Alphas
    std::vector<std::vector<double>> alphas(p + 1);
    for (auto& alpha : alphas) {
      alpha.resize(r + 1);
    }

    int L = k - p;
    for (int j = 1; j <= r; ++j) {
      L = k - p + j;
      for (int i = 0; i <= p - j - s; i++) {
        alphas[i][j] = (uv - UP[L + i]) / (UP[i + k + 1] - UP[L + i]);
      }
    }

    std::vector<Point4D> Rw(p + 1);

    for (size_t row = 0; row < Pw[0].size(); row++) {
      /* Save unaltered control points */
      for (int i = 0; i <= k - p; ++i) {
        Qw[i][row] = Pw[i][row];
      }
      for (int i = k - s; i < Pw.size(); i++) {
        Qw[i + r][row] = Pw[i][row];
      }
      // Load auxiliary control points
      for (int i = 0; i <= p - s; ++i) {
        Rw[i] = Pw[k - p + i][row];
      }

      for (int j = 1; j <= r; ++j) {
        L = k - p + j;
        for (int i = 0; i <= p - j - s; ++i) {
          Rw[i] = (alphas[i][j] * Rw[i + 1]) + ((1.0 - alphas[i][j]) * Rw[i]);
        }
        Qw[L][row] = Rw[0];
        Qw[k + r - j - s][row] = Rw[p - j - s];
      }

      // Load the remaining control points
      for (uint32_t i = L + 1; i < k - s; ++i) {
        Qw[i][row] = Rw[i - L];
      }
    }
  } else {
    /* Similar code as above with u and v directional parameters switched */

    // Get the remaining input values
    int k = knots::FindSpanParam(q, VP, knot, kTolerance);
    int s = knots::MultiplicityParam(q, VP, knot, kTolerance);

    // Max value of r is q - s
    r = std::min(r, q - s);

    // Resize the output arrays
    VQ.resize(VP.size() + r);
    Qw.resize(Pw.size());
    for (auto& column : Qw) {
      column.resize(Pw[0].size() + r);
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

    // Copy u vector into UQ (return vector)
    UQ = UP;

    // Save the Alphas
    std::vector<std::vector<double>> alphas(q + 1);
    for (auto& alpha : alphas) {
      alpha.resize(r + 1);
    }

    int L = k - q;
    for (int j = 1; j <= r; ++j) {
      L = k - q + j;
      for (int i = 0; i <= q - j - s; i++) {
        alphas[i][j] = (uv - VP[L + i]) / (VP[i + k + 1] - VP[L + i]);
      }
    }
    std::vector<Point4D> Rw(q + 1);

    for (int col = 0; col < Pw.size(); col++) {
      /* Save unaltered control points */
      for (int i = 0; i <= k - q; ++i) {
        Qw[col][i] = Pw[col][i];
      }
      for (int i = k - s; i < Pw[0].size(); i++) {
        Qw[col][i + r] = Pw[col][i];
      }
      // Load auxiliary control points
      for (int i = 0; i <= q - s; ++i) {
        Rw[i] = Pw[col][k - q + i];
      }

      for (int j = 1; j <= r; ++j) {
        L = k - q + j;
        for (int i = 0; i <= q - j - s; ++i) {
          Rw[i] = (alphas[i][j] * Rw[i + 1]) + ((1.0 - alphas[i][j]) * Rw[i]);
        }
        Qw[col][L] = Rw[0];
        Qw[col][k + r - j - s] = Rw[q - j - s];
      }

      // Load the remaining control points
      for (int i = L + 1; i < k - s; ++i) {
        Qw[col][i] = Rw[i - L];
      }
    }
  }
  return NURBSSurface(u_degree_, v_degree_, UQ, VQ, Qw, u_interval_,
                      v_interval_);
}

// ALGORITHM A5.5 RefineKnotVectSurface(n,p,U,m,q,V,Pw,X,r,dir,Ubar,Vbar,Qw)
NURBSSurface NURBSSurface::RefineKnotVect(std::vector<double> knots,
                                          SurfaceDirection dir) const {
  if (control_polygon_.empty() || knots.empty()) {
    return NURBSSurface(u_degree_, v_degree_, u_knots_, v_knots_,
                        control_polygon_, u_interval_, v_interval_);
  }

  // Refine surface knot vector
  // Input: n,p,U,m,q,V,Pw,X,r,dir
  // n - number of points in the U direction
  // p - U degree
  // U - U knot vector
  // m - number of points int the V direction
  // q - V degree
  // V - V knot vector
  // Pw - control polygon
  // X - knot vector to merge in
  // r - size of X
  // dir - direction to merge
  int n = static_cast<int>(control_polygon_.size()) - 1;
  int p = u_degree_;
  auto U = u_knots_;
  int m = static_cast<int>(control_polygon_[0].size()) - 1;
  int q = v_degree_;
  auto V = v_knots_;
  auto Pw = control_polygon_;
  auto X = knots;
  int r = static_cast<int>(knots.size()) - 1;

  // Output: Ubar, Vbar, Qw
  // Ubar - U knot vector
  // Vbar - V out vector
  // Qw - control polygon
  std::vector<double> Ubar;
  std::vector<double> Vbar;
  std::vector<std::vector<Point4D>> Qw;

  if (dir == SurfaceDirection::kUDir) {
    // find indexes a and b;
    int a = knots::FindSpanParam(p, U, X[0], kTolerance);
    int b = knots::FindSpanParam(p, U, X[r], kTolerance);
    b += 1;

    // initialize Ubar;
    Ubar.resize(U.size() + r + 1);
    for (int i = 0; i <= a; ++i) {
      Ubar[i] = U[i];
    }
    for (int i = b + p; i < U.size(); ++i) {
      Ubar[i + r + 1] = U[i];
    }

    // copy V into Vbar;
    Vbar = V;

    // Save unaltered ctrl pts
    Qw.resize(Pw.size() + r + 1);
    for (int i = 0; i <= a - p; ++i) {
      Qw[i] = Pw[i];
    }
    for (int i = b - 1; i <= n; ++i) {
      Qw[i + r + 1] = Pw[i];
    }
    // Init middle points of Qw
    for (int i = a - p + 1; i < b + r; ++i) {
      Qw[i].resize(Pw[0].size());
    }

    int i = b + p - 1;
    int k = b + p + r;

    for (int j = r; j >= 0; j--) {
      while (X[j] <= U[i] && i > a) {
        Ubar[k] = U[i];
        for (int row = 0; row <= m; row++) {
          Qw[k - p - 1][row] = Pw[i - p - 1][row];
        }
        k = k - 1;
        i = i - 1;
      }

      for (int row = 0; row <= m; row++) {
        Qw[k - p - 1][row] = Qw[k - p][row];
      }

      for (int l = 1; l <= p; l++) {
        int ind = k - p + l;
        double alfa = Ubar[k + l] - X[j];
        if (abs(alfa) == 0.0) {
          for (int row = 0; row <= m; row++) {
            Qw[ind - 1][row] = Qw[ind][row];
          }
        } else {
          alfa = alfa / (Ubar[k + l] - U[i - p + l]);
          for (int row = 0; row <= m; row++) {
            Qw[ind - 1][row] =
                (alfa * Qw[ind - 1][row]) + ((1.0 - alfa) * Qw[ind][row]);
          }
        }
      }
      Ubar[k] = X[j];
      k = k - 1;
    }
  } else if (dir == SurfaceDirection::kVDir) {
    // Similar code as above with u and v directional parameters switched

    // find indexes a and b;
    int a = knots::FindSpanParam(q, V, X[0], kTolerance);
    int b = knots::FindSpanParam(q, V, X[r], kTolerance);
    b += 1;

    // initialize Vbar;
    Vbar.resize(V.size() + r + 1);
    for (int i = 0; i <= a; ++i) {
      Vbar[i] = V[i];
    }
    for (int i = b + q; i < V.size(); ++i) {
      Vbar[i + r + 1] = V[i];
    }

    // copy U into Ubar;
    Ubar = U;

    // Save unaltered ctrl pts
    Qw.resize(Pw.size());
    for (int i = 0; i <= n; ++i) {
      Qw[i].resize(Pw[0].size() + r + 1);
      for (int j = 0; j <= a - q; ++j) {
        Qw[i][j] = Pw[i][j];
      }
      for (int j = b - 1; j <= m; ++j) {
        Qw[i][j + r + 1] = Pw[i][j];
      }
    }

    int i = b + q - 1;
    int k = b + q + r;

    for (int j = r; j >= 0; j--) {
      while (X[j] <= V[i] && i > a) {
        Vbar[k] = V[i];
        for (int col = 0; col <= n; col++) {
          Qw[col][k - q - 1] = Pw[col][i - q - 1];
        }
        k = k - 1;
        i = i - 1;
      }

      for (int col = 0; col <= n; col++) {
        Qw[col][k - q - 1] = Qw[col][k - q];
      }

      for (int l = 1; l <= q; l++) {
        int ind = k - q + l;
        double alfa = Vbar[k + l] - X[j];
        if (abs(alfa) == 0.0) {
          for (int col = 0; col <= n; col++) {
            Qw[col][ind - 1] = Qw[col][ind];
          }
        } else {
          alfa = alfa / (Vbar[k + l] - V[i - q + l]);
          for (int col = 0; col <= n; col++) {
            Qw[col][ind - 1] =
                (alfa * Qw[col][ind - 1]) + ((1.0 - alfa) * Qw[col][ind]);
          }
        }
      }
      Vbar[k] = X[j];
      k = k - 1;
    }
  }
  return NURBSSurface(u_degree_, v_degree_, Ubar, Vbar, Qw, u_interval_,
                      v_interval_);
}

std::vector<NURBSSurface> NURBSSurface::DecomposeU() const {
  // Decompose surface into Bezier patches
  // Input: n, p, U, m, q, V, Pw, dir
  // n - Max index of U knot vector
  // p - U degree
  // U - U knot vector
  // m - # of points in the V direction
  // q - V degree
  // V - V knot vector
  // Pw - control polygon
  // dir - Direction to split
  std::vector<std::vector<Point4D>> Pw = control_polygon_;
  int p = u_degree_;
  int n = static_cast<int>(Pw.size()) + p;
  std::vector<double> U = u_knots_;
  int m = Pw.empty() ? 0 : static_cast<int>(Pw[0].size());
  // auto dir = SurfaceDirection::kUDir;

  // Output: nb, Qw
  // nb - # of bezier strips returned
  // Qw - Bezier strips
  std::vector<std::vector<std::vector<Point4D>>> Qw = {};
  std::vector<NURBSSurface> strips = {};

  // (dir == SurfaceDirection::kUDir)

  std::vector<double> new_knots(p + p + 2, 1.0);
  for (int index = 0; index <= p; ++index) {
    new_knots[index] = 0.0;
  }
  Point2D new_interval = {0.0, 1.0};
  std::vector<double> alphas(p + 1, 0.0);

  int a = p;
  int b = p + 1;

  Qw.resize(2);
  for (std::vector<std::vector<Point4D>>& strip : Qw) {
    strip.resize(p + 1);
    for (std::vector<Point4D>& points : strip) {
      points.resize(m);
    }
  }

  for (int i = 0; i <= p; ++i) {
    for (int row = 0; row < m; ++row) {
      Qw[0][i][row] = Pw[i][row];
    }
  }

  while (b < n) {
    // Get Mult
    int i = b;
    while (b < n && U[b + 1] == U[b]) {
      ++b;
    }
    int mult = b - i + 1;

    if (mult < p) {
      // Get the numerator and the alfas;
      double numer = U[b] - U[a];  // Numerator of alpha
      // Compute and store alphas
      for (int j = p; j > mult; --j) {
        alphas[j - mult - 1] = numer / (U[a + j] - U[a]);
      }
      int r = p - mult;  // Knot insert r times

      for (int j = 1; j <= r; ++j) {
        int save = r - j;
        int s = mult + j;  // This many new points
        for (int k = p; k >= s; --k) {
          double alpha = alphas[k - s];
          for (int row = 0; row < m; ++row) {
            Qw[0][k][row] =
                (alpha * Qw[0][k][row]) + ((1.0 - alpha) * Qw[0][k - 1][row]);
          }
        }
        if (b < n) {
          for (int row = 0; row < m; ++row) {
            Qw[1][save][row] = Qw[0][p][row];
          }
        }
      }
    }

    strips.emplace_back(p, v_degree_, new_knots, v_knots_, Qw[0], new_interval,
                        v_interval_);

    if (b < n) {
      Qw[0] = Qw[1];
      for (i = std::max(p - mult, 0); i <= p; ++i) {
        for (int row = 0; row < m; ++row) {
          Qw[0][i][row] = Pw[b - p + i][row];
        }
      }
      a = b;
      b = b + 1;
    }
  }
  return strips;
}

std::vector<BezierSurface> NURBSSurface::DecomposeV() const {
  // Fail quick if the U direction is not already decomposed
  if (control_polygon_.size() != u_degree_ + 1) {
    return {};
  }
  // Decompose surface into Bezier patches
  // Input: n, p, U, m, q, V, Pw, dir
  // n - # of points in the U dir
  // p - U degree
  // U - U knot vector
  // m - Max index of V knot vector
  // q - V degree
  // V - V knot vector
  // Pw - control polygon
  // dir - Direction to split
  std::vector<std::vector<Point4D>> Pw = control_polygon_;
  int p = u_degree_;
  int q = v_degree_;
  int m = Pw.empty() ? -1 : static_cast<int>(Pw[0].size()) + q;
  std::vector<double> V = v_knots_;
  // auto dir = SurfaceDirection::kUDir;

  // Output: nb, Qw
  // nb - # of bezier strips returned
  // Qw - Bezier strips
  std::vector<std::vector<std::vector<Point4D>>> Qw = {};
  std::vector<BezierSurface> patches = {};
  std::vector<BezierCurve3D> curves = {};
  std::vector<Point3D> curve_points = {};
  curves.reserve(q + 1);
  curve_points.reserve(p + 1);

  // (dir == SurfaceDirection::kVDir)

  std::vector<double> alphas(q + 1, 0.0);

  int a = q;
  int b = q + 1;

  Qw.resize(2);
  for (std::vector<std::vector<Point4D>>& patch : Qw) {
    patch.resize(p + 1);
    for (std::vector<Point4D>& points : patch) {
      points.resize(q + 1);
    }
  }

  for (int i = 0; i <= p; ++i) {
    for (int j = 0; j <= q; ++j) {
      Qw[0][i][j] = Pw[i][j];
    }
  }

  while (b < m) {
    // Get Mult
    int i = b;
    while (b < m && V[b + 1] == V[b]) {
      ++b;
    }
    int mult = b - i + 1;

    if (mult < q) {
      // Get the numerator and the alfas;
      double numer = V[b] - V[a];  // Numerator of alpha
      // Compute and store alphas
      for (int j = q; j > mult; --j) {
        alphas[j - mult - 1] = numer / (V[a + j] - V[a]);
      }
      int r = q - mult;  // Knot insert r times

      for (int j = 1; j <= r; ++j) {
        int save = r - j;
        int s = mult + j;  // This many new points
        for (int k = q; k >= s; --k) {
          double alpha = alphas[k - s];
          for (int col = 0; col <= p; ++col) {
            Qw[0][col][k] =
                (alpha * Qw[0][col][k]) + ((1.0 - alpha) * Qw[0][col][k - 1]);
          }
        }
        if (b < m) {
          for (int col = 0; col <= p; ++col) {
            Qw[1][col][save] = Qw[0][col][q];
          }
        }
      }
    }

    for (size_t row = 0; row < Qw[0][0].size(); ++row) {
      for (size_t col = 0; col < Qw[0].size(); ++col) {
        const Point4D& point = Qw[0][col][row];
        curve_points.push_back(Point3D(point.x, point.y, point.z) / point.w);
      }
      curves.emplace_back(curve_points);
      curve_points.clear();
    }

    patches.emplace_back(curves);
    curves.clear();

    if (b < m) {
      Qw[0] = Qw[1];
      for (int col = 0; col <= p; ++col) {
        for (i = std::max(q - mult, 0); i <= q; ++i) {
          Qw[0][col][i] = Pw[col][b - q + i];
        }
      }
      a = b;
      b = b + 1;
    }
  }
  return patches;
}

// ALGORITHM A5.7 DecomposeSurface(n, p, U, m, q, V, Pw, dir, nb, Qw)
std::vector<std::vector<BezierSurface>> NURBSSurface::Decompose() const {
  std::vector<NURBSSurface> nurbs_surfaces = DecomposeU();
  std::vector<std::vector<BezierSurface>> bezier_surfaces;
  bezier_surfaces.reserve(nurbs_surfaces.size());
  for (const NURBSSurface& surface : nurbs_surfaces) {
    bezier_surfaces.push_back(surface.DecomposeV());
  }
  return bezier_surfaces;
}
}  // namespace nurbs
#pragma once
// Minimal dependency-free dense linear algebra for the smoother and the
// I_theta certificate. Sizes are small (certificate: 3x3, smoother: ~7+2K).

#include <cassert>
#include <cmath>
#include <vector>

namespace gfs {

struct DenseMat {
    int rows = 0, cols = 0;
    std::vector<double> a;

    DenseMat() = default;
    DenseMat(int r, int c) : rows(r), cols(c), a(static_cast<size_t>(r) * c, 0.0) {}

    double& operator()(int i, int j) { return a[static_cast<size_t>(i) * cols + j]; }
    double operator()(int i, int j) const { return a[static_cast<size_t>(i) * cols + j]; }

    static DenseMat identity(int n, double s = 1.0) {
        DenseMat m(n, n);
        for (int i = 0; i < n; ++i) m(i, i) = s;
        return m;
    }
};

// In-place Cholesky solve A x = b for symmetric positive definite A.
// Returns false if A is not (numerically) positive definite.
inline bool choleskySolve(DenseMat A, std::vector<double> b,
                          std::vector<double>* x) {
    const int n = A.rows;
    assert(A.cols == n && static_cast<int>(b.size()) == n);
    // Factor A = L L^T (lower triangle stored in A).
    for (int j = 0; j < n; ++j) {
        double d = A(j, j);
        for (int k = 0; k < j; ++k) d -= A(j, k) * A(j, k);
        if (d <= 0.0) return false;
        const double ljj = std::sqrt(d);
        A(j, j) = ljj;
        for (int i = j + 1; i < n; ++i) {
            double s = A(i, j);
            for (int k = 0; k < j; ++k) s -= A(i, k) * A(j, k);
            A(i, j) = s / ljj;
        }
    }
    // Forward substitution L y = b.
    for (int i = 0; i < n; ++i) {
        double s = b[i];
        for (int k = 0; k < i; ++k) s -= A(i, k) * b[k];
        b[i] = s / A(i, i);
    }
    // Back substitution L^T x = y.
    for (int i = n - 1; i >= 0; --i) {
        double s = b[i];
        for (int k = i + 1; k < n; ++k) s -= A(k, i) * b[k];
        b[i] = s / A(i, i);
    }
    *x = std::move(b);
    return true;
}

// Inverse of a symmetric positive definite matrix via Cholesky solves against
// unit vectors. Used only for small marginal blocks.
inline bool spdInverse(const DenseMat& A, DenseMat* inv) {
    const int n = A.rows;
    *inv = DenseMat(n, n);
    std::vector<double> e(n, 0.0), col;
    for (int j = 0; j < n; ++j) {
        e.assign(n, 0.0);
        e[j] = 1.0;
        if (!choleskySolve(A, e, &col)) return false;
        for (int i = 0; i < n; ++i) (*inv)(i, j) = col[i];
    }
    // Symmetrize.
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j) {
            const double s = 0.5 * ((*inv)(i, j) + (*inv)(j, i));
            (*inv)(i, j) = (*inv)(j, i) = s;
        }
    return true;
}

// Check symmetric matrix is positive semidefinite (via eigen-free test:
// Cholesky of A + eps I succeeds for small eps).
inline bool isPsd(const DenseMat& A, double eps = 1e-9) {
    DenseMat B = A;
    for (int i = 0; i < B.rows; ++i) B(i, i) += eps;
    std::vector<double> x;
    std::vector<double> b(static_cast<size_t>(B.rows), 0.0);
    return choleskySolve(B, b, &x);
}

}  // namespace gfs

#pragma once
// Minimal dependency-free 2D math utilities for the GPS-free seeking project.

#include <array>
#include <cmath>
#include <cstddef>
#include <random>

namespace gfs {

constexpr double kPi = 3.14159265358979323846;

struct Vec2 {
    double x{0.0};
    double y{0.0};

    Vec2() = default;
    Vec2(double x_, double y_) : x(x_), y(y_) {}

    Vec2 operator+(const Vec2& o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(const Vec2& o) const { return {x - o.x, y - o.y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    Vec2& operator+=(const Vec2& o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(const Vec2& o) { x -= o.x; y -= o.y; return *this; }

    double norm() const { return std::sqrt(x * x + y * y); }
    double dot(const Vec2& o) const { return x * o.x + y * o.y; }
    // z-component of the 2D cross product.
    double cross(const Vec2& o) const { return x * o.y - y * o.x; }
};

inline Vec2 operator*(double s, const Vec2& v) { return v * s; }

// Counter-clockwise rotation by angle a applied to v: R(a) v.
inline Vec2 rotate(double a, const Vec2& v) {
    const double c = std::cos(a);
    const double s = std::sin(a);
    return {c * v.x - s * v.y, s * v.x + c * v.y};
}

// R(a)^T v = R(-a) v.
inline Vec2 rotateT(double a, const Vec2& v) { return rotate(-a, v); }

inline double wrapAngle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

inline Vec2 saturate(const Vec2& v, double vmax) {
    const double n = v.norm();
    if (n <= vmax || n == 0.0) return v;
    return v * (vmax / n);
}

// Small dense square-matrix helper used only by the EKF baseline (N = 7).
template <std::size_t N>
struct MatN {
    std::array<double, N * N> a{};

    double& operator()(std::size_t i, std::size_t j) { return a[i * N + j]; }
    double operator()(std::size_t i, std::size_t j) const { return a[i * N + j]; }

    static MatN identity(double s = 1.0) {
        MatN m;
        for (std::size_t i = 0; i < N; ++i) m(i, i) = s;
        return m;
    }
};

struct Rng {
    std::mt19937 gen;
    std::normal_distribution<double> normal{0.0, 1.0};
    std::uniform_real_distribution<double> uniform{0.0, 1.0};

    explicit Rng(unsigned seed) : gen(seed) {}

    double gauss(double sigma) { return sigma > 0.0 ? sigma * normal(gen) : 0.0; }
    double uni(double lo, double hi) { return lo + (hi - lo) * uniform(gen); }
};

}  // namespace gfs

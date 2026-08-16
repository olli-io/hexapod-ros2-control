// Hand-rolled single-precision 3-vector, in place of Eigen: the RP2350 has a
// single-precision FPU only, so the whole port runs in float.
#pragma once

#include <array>
#include <cmath>

namespace hexa {

struct Vec3 {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  constexpr Vec3() = default;
  constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

  static constexpr Vec3 Zero() { return Vec3(0.0f, 0.0f, 0.0f); }

  constexpr float& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
  constexpr const float& operator[](int i) const {
    return i == 0 ? x : (i == 1 ? y : z);
  }

  constexpr Vec3 operator+(const Vec3& o) const {
    return Vec3(x + o.x, y + o.y, z + o.z);
  }
  constexpr Vec3 operator-(const Vec3& o) const {
    return Vec3(x - o.x, y - o.y, z - o.z);
  }
  constexpr Vec3 operator-() const { return Vec3(-x, -y, -z); }
  constexpr Vec3 operator*(float s) const { return Vec3(x * s, y * s, z * s); }
  constexpr Vec3 operator/(float s) const { return Vec3(x / s, y / s, z / s); }

  constexpr Vec3& operator+=(const Vec3& o) {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }
  constexpr Vec3& operator-=(const Vec3& o) {
    x -= o.x;
    y -= o.y;
    z -= o.z;
    return *this;
  }
  constexpr Vec3& operator*=(float s) {
    x *= s;
    y *= s;
    z *= s;
    return *this;
  }
  constexpr Vec3& operator/=(float s) {
    x /= s;
    y /= s;
    z /= s;
    return *this;
  }

  constexpr float dot(const Vec3& o) const {
    return x * o.x + y * o.y + z * o.z;
  }
  constexpr Vec3 cross(const Vec3& o) const {
    return Vec3(y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x);
  }
  constexpr float squaredNorm() const { return x * x + y * y + z * z; }

  float norm() const { return std::hypot(x, y, z); }
  Vec3 normalized() const {
    const float n = norm();
    return n > 0.0f ? Vec3(x / n, y / n, z / n) : Zero();
  }
};

constexpr Vec3 operator*(float s, const Vec3& v) { return v * s; }

constexpr bool operator==(const Vec3& a, const Vec3& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}
constexpr bool operator!=(const Vec3& a, const Vec3& b) { return !(a == b); }

constexpr float dot(const Vec3& a, const Vec3& b) { return a.dot(b); }
constexpr Vec3 cross(const Vec3& a, const Vec3& b) { return a.cross(b); }
inline float norm(const Vec3& v) { return v.norm(); }

// IK-convention joint-angle triple (theta_coxa, theta_femur, theta_tibia), rad.
using JointAngles = std::array<float, 3>;

}  // namespace hexa

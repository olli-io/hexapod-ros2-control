// Hand-rolled single-precision 3-vector (plan part 04).
//
// Drop-in replacement for Eigen::Vector3d in the forked pure libs (kinematics,
// gait, posture). The RP2350 has a single-precision FPU only, so the whole port
// runs in float; Eigen is dropped to avoid its M33 alignment / SIMD tuning and
// to keep the math trivially auditable. The API mirrors the slice of Eigen the
// ported code actually uses:
//
//   - Vec3(x, y, z) construction and Vec3::Zero(),
//   - element access via operator[] (leg.mount_xyz[0], target[2], ...) and the
//     named .x / .y / .z members,
//   - operator + - (binary and unary), scalar * (either side) and /,
//     compound += -= *= /=,
//   - dot / cross / squaredNorm / norm (hypotf-based) / normalized.
//
// JointAngles mirrors hexa_kinematics::JointAngles / hexa_gait::JointAngles
// (std::array<double,3> upstream) as a float triple.
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

  // Eigen-style indexed access (0=x, 1=y, 2=z). No bounds check, matching the
  // forked code's fixed 0/1/2 usage.
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

  // 3-argument hypotf: matches the double engine's std::hypot(x, y, z) for
  // magnitude-stable norms (used by stride clamping, reachability checks).
  float norm() const { return std::hypot(x, y, z); }
  Vec3 normalized() const {
    const float n = norm();
    return n > 0.0f ? Vec3(x / n, y / n, z / n) : Zero();
  }
};

// scalar * vec (the vec * scalar order is the member operator above).
constexpr Vec3 operator*(float s, const Vec3& v) { return v * s; }

constexpr bool operator==(const Vec3& a, const Vec3& b) {
  return a.x == b.x && a.y == b.y && a.z == b.z;
}
constexpr bool operator!=(const Vec3& a, const Vec3& b) { return !(a == b); }

// Free-function forms mirroring the Eigen calls in the ported code.
constexpr float dot(const Vec3& a, const Vec3& b) { return a.dot(b); }
constexpr Vec3 cross(const Vec3& a, const Vec3& b) { return a.cross(b); }
inline float norm(const Vec3& v) { return v.norm(); }

// IK-convention joint-angle triple (theta_coxa, theta_femur, theta_tibia), rad.
using JointAngles = std::array<float, 3>;

}  // namespace hexa

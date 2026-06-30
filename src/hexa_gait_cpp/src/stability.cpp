#include "hexa_gait_cpp/stability.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace hexa_gait {

namespace {
using Point = std::pair<double, double>;

double cross(const Point& o, const Point& a, const Point& b) {
  return (a.first - o.first) * (b.second - o.second) -
         (a.second - o.second) * (b.first - o.first);
}

// sorted(set(points)) — lexicographic sort + dedup.
std::vector<Point> sorted_unique(const std::vector<Point>& points) {
  std::vector<Point> pts = points;
  std::sort(pts.begin(), pts.end());
  pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
  return pts;
}

// Andrew monotone chain; returns the hull in CCW order. Collinear boundary
// points are dropped. Degenerate inputs return fewer than 3 vertices.
std::vector<Point> convex_hull(const std::vector<Point>& points) {
  std::vector<Point> pts = sorted_unique(points);
  if (pts.size() <= 2) {
    return pts;
  }
  std::vector<Point> lower;
  for (const auto& p : pts) {
    while (lower.size() >= 2 &&
           cross(lower[lower.size() - 2], lower.back(), p) <= 0.0) {
      lower.pop_back();
    }
    lower.push_back(p);
  }
  std::vector<Point> upper;
  for (auto it = pts.rbegin(); it != pts.rend(); ++it) {
    while (upper.size() >= 2 &&
           cross(upper[upper.size() - 2], upper.back(), *it) <= 0.0) {
      upper.pop_back();
    }
    upper.push_back(*it);
  }
  std::vector<Point> hull(lower.begin(), lower.end() - 1);
  hull.insert(hull.end(), upper.begin(), upper.end() - 1);
  if (hull.size() >= 3) {
    return hull;
  }
  std::vector<Point> fallback = sorted_unique(points);
  if (fallback.size() > 2) {
    fallback.resize(2);
  }
  return fallback;
}

double point_segment_distance(const Point& p, const Point& a, const Point& b) {
  const double ab_x = b.first - a.first;
  const double ab_y = b.second - a.second;
  const double ap_x = p.first - a.first;
  const double ap_y = p.second - a.second;
  const double denom = ab_x * ab_x + ab_y * ab_y;
  if (denom <= 0.0) {
    return std::hypot(ap_x, ap_y);
  }
  double t = (ap_x * ab_x + ap_y * ab_y) / denom;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(ap_x - t * ab_x, ap_y - t * ab_y);
}
}  // namespace

double support_polygon_margin(
    const std::vector<std::pair<double, double>>& stance_feet_xy,
    std::pair<double, double> com_xy) {
  if (stance_feet_xy.empty()) {
    return -std::numeric_limits<double>::infinity();
  }
  std::vector<Point> feet = stance_feet_xy;
  std::vector<Point> hull = convex_hull(feet);
  if (hull.size() < 3) {
    if (hull.size() == 1) {
      return -std::hypot(com_xy.first - hull[0].first,
                         com_xy.second - hull[0].second);
    }
    return -point_segment_distance(com_xy, hull[0], hull[1]);
  }

  double margin = std::numeric_limits<double>::infinity();
  bool inside = true;
  for (std::size_t i = 0; i < hull.size(); ++i) {
    const Point& a = hull[i];
    const Point& b = hull[(i + 1) % hull.size()];
    const double edge_len = std::hypot(b.first - a.first, b.second - a.second);
    // Signed perpendicular distance; positive on the interior side of a CCW
    // edge.
    const double signed_dist = cross(a, b, com_xy) / edge_len;
    if (signed_dist < 0.0) {
      inside = false;
    }
    margin = std::min(margin, signed_dist);
  }
  if (inside) {
    return margin;
  }
  // Outside: report the true distance to the hull boundary, negated.
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < hull.size(); ++i) {
    best = std::min(best, point_segment_distance(
                              com_xy, hull[i], hull[(i + 1) % hull.size()]));
  }
  return -best;
}

}  // namespace hexa_gait

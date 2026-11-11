#include "image.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gfx {

namespace {

inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Rows/columns whose pixel centres can fall inside [lo, hi].
inline void span(double lo, double hi, int limit, int& a, int& b) {
    a = clampi(int(std::floor(lo - 0.5)), 0, limit);
    b = clampi(int(std::ceil(hi - 0.5)), -1, limit - 1);
}

}  // namespace

void fill_rect(Image& im, double x0, double y0, double x1, double y1, uint8_t v) {
    x1 += 1;
    y1 += 1;
    int ia, ib, ja, jb;
    span(x0, x1, im.w, ia, ib);
    span(y0, y1, im.h, ja, jb);
    // columns whose centres are inside [x0, x1]
    while (ia <= ib && ia + 0.5 < x0) ++ia;
    while (ib >= ia && ib + 0.5 > x1) --ib;
    if (ia > ib) return;
    for (int j = ja; j <= jb; ++j) {
        double cy = j + 0.5;
        if (cy < y0 || cy > y1) continue;
        std::memset(&im.at(ia, j), v, size_t(ib - ia + 1));
    }
}

namespace {

// Pixel columns [a, b] whose centres lie in the rounded rectangle on the row
// with centre height cy, or false if the row misses it.
bool rrect_row(double cy, double cx, double cy0, double hw, double hh, double r, int w,
               int& a, int& b) {
    double dy = std::fabs(cy - cy0);
    if (dy > hh) return false;
    double qy = dy - (hh - r);
    double half = hw;
    if (qy > 0) {
        if (qy > r) return false;
        half = (hw - r) + std::sqrt(r * r - qy * qy);
    }
    a = clampi(int(std::ceil(cx - half - 0.5)), 0, w);
    b = clampi(int(std::floor(cx + half - 0.5)), -1, w - 1);
    return a <= b;
}

}  // namespace

void fill_rounded_rect(Image& im, double x0, double y0, double x1, double y1, double r,
                       uint8_t v) {
    x1 += 1;
    y1 += 1;
    double cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    double hw = (x1 - x0) / 2, hh = (y1 - y0) / 2;
    r = std::max(0.0, std::min({r, hw, hh}));
    int ja, jb, a, b;
    span(y0, y1, im.h, ja, jb);
    for (int j = ja; j <= jb; ++j)
        if (rrect_row(j + 0.5, cx, cy, hw, hh, r, im.w, a, b))
            std::memset(&im.at(a, j), v, size_t(b - a + 1));
}

void outline_rounded_rect(Image& im, double x0, double y0, double x1, double y1,
                          double r, double width, uint8_t v) {
    x1 += 1;
    y1 += 1;
    double cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    double hw = (x1 - x0) / 2, hh = (y1 - y0) / 2;
    double ro = std::max(0.0, std::min({r, hw, hh}));
    double ihw = hw - width, ihh = hh - width;
    double ri = std::max(0.0, std::min({r - width, ihw, ihh}));
    int ja, jb, oa, ob, ia, ib;
    span(y0, y1, im.h, ja, jb);
    for (int j = ja; j <= jb; ++j) {
        if (!rrect_row(j + 0.5, cx, cy, hw, hh, ro, im.w, oa, ob)) continue;
        if (ihw > 0 && ihh > 0 && rrect_row(j + 0.5, cx, cy, ihw, ihh, ri, im.w, ia, ib)) {
            if (ia > oa) std::memset(&im.at(oa, j), v, size_t(ia - oa));
            if (ob > ib) std::memset(&im.at(ib + 1, j), v, size_t(ob - ib));
        } else {
            std::memset(&im.at(oa, j), v, size_t(ob - oa + 1));
        }
    }
}

void fill_ellipse(Image& im, double x0, double y0, double x1, double y1, uint8_t v) {
    x1 += 1;
    y1 += 1;
    double cx = (x0 + x1) / 2, cy = (y0 + y1) / 2;
    double a = (x1 - x0) / 2, b = (y1 - y0) / 2;
    if (a <= 0 || b <= 0) return;
    int ja, jb;
    span(y0, y1, im.h, ja, jb);
    for (int j = ja; j <= jb; ++j) {
        double dy = (j + 0.5 - cy) / b;
        if (dy * dy > 1.0) continue;
        double half = a * std::sqrt(1.0 - dy * dy);
        int ia = clampi(int(std::ceil(cx - half - 0.5)), 0, im.w);
        int ib = clampi(int(std::floor(cx + half - 0.5)), -1, im.w - 1);
        if (ia <= ib) std::memset(&im.at(ia, j), v, size_t(ib - ia + 1));
    }
}

void fill_polygon(Image& im, const std::vector<Pt>& pts, uint8_t v) {
    size_t n = pts.size();
    if (n < 3) return;
    double ymin = pts[0].y, ymax = pts[0].y;
    for (const Pt& p : pts) {
        ymin = std::min(ymin, p.y);
        ymax = std::max(ymax, p.y);
    }
    int ja, jb;
    span(ymin, ymax, im.h, ja, jb);
    std::vector<double> xs;
    for (int j = ja; j <= jb; ++j) {
        double cy = j + 0.5;
        xs.clear();
        for (size_t k = 0; k < n; ++k) {
            const Pt& p = pts[k];
            const Pt& q = pts[(k + 1) % n];
            if ((p.y <= cy && q.y > cy) || (q.y <= cy && p.y > cy))
                xs.push_back(p.x + (cy - p.y) / (q.y - p.y) * (q.x - p.x));
        }
        std::sort(xs.begin(), xs.end());
        for (size_t k = 0; k + 1 < xs.size(); k += 2) {
            int a = clampi(int(std::ceil(xs[k] - 0.5)), 0, im.w);
            int b = clampi(int(std::floor(xs[k + 1] - 0.5)), -1, im.w - 1);
            for (int i = a; i <= b; ++i) im.at(i, j) = v;
        }
    }
}

void polyline(Image& im, const std::vector<Pt>& pts, double width, uint8_t v,
              bool round_joints) {
    double half = width / 2;
    for (size_t k = 0; k + 1 < pts.size(); ++k) {
        Pt p = pts[k], q = pts[k + 1];
        double dx = q.x - p.x, dy = q.y - p.y;
        double len = std::hypot(dx, dy);
        if (len == 0) continue;
        double nx = -dy / len * half, ny = dx / len * half;
        fill_polygon(im, {{p.x + nx, p.y + ny}, {q.x + nx, q.y + ny},
                          {q.x - nx, q.y - ny}, {p.x - nx, p.y - ny}}, v);
    }
    if (round_joints)
        for (size_t k = 1; k + 1 < pts.size(); ++k)
            fill_ellipse(im, pts[k].x - half, pts[k].y - half, pts[k].x + half - 1,
                         pts[k].y + half - 1, v);
}

namespace {

double lanczos3(double x) {
    if (x == 0) return 1.0;
    if (x <= -3 || x >= 3) return 0.0;
    double px = M_PI * x;
    return 3.0 * std::sin(px) * std::sin(px / 3.0) / (px * px);
}

// Filter taps as fixed-point weights that sum to exactly 1 << kShift.
constexpr int kShift = 14;

struct Taps {
    int first;
    std::vector<int> w;
};

std::vector<Taps> make_taps(int in, int out, int factor) {
    std::vector<Taps> t(out);
    double support = 3.0 * factor;
    for (int i = 0; i < out; ++i) {
        double center = (i + 0.5) * factor;
        int lo = std::max(0, int(center - support + 0.5));
        int hi = std::min(in, int(center + support + 0.5));
        t[i].first = lo;
        std::vector<double> w;
        double sum = 0;
        for (int x = lo; x < hi; ++x) {
            w.push_back(lanczos3((x + 0.5 - center) / factor));
            sum += w.back();
        }
        int total = 0, biggest = 0;
        for (size_t k = 0; k < w.size(); ++k) {
            t[i].w.push_back(int(std::lround(w[k] / sum * (1 << kShift))));
            total += t[i].w.back();
            if (w[k] > w[biggest]) biggest = int(k);
        }
        t[i].w[biggest] += (1 << kShift) - total;
    }
    return t;
}

inline uint8_t to_u8(int acc) {
    return uint8_t(clampi((acc + (1 << (kShift - 1))) >> kShift, 0, 255));
}

}  // namespace

Image downscale_lanczos(const Image& src, int factor) {
    int ow = src.w / factor, oh = src.h / factor;
    std::vector<Taps> tx = make_taps(src.w, ow, factor);
    std::vector<Taps> ty = make_taps(src.h, oh, factor);
    // horizontal pass; rows that are all black stay black
    Image mid(ow, src.h);
    std::vector<uint8_t> blank(size_t(src.w), 0);
    for (int y = 0; y < src.h; ++y) {
        const uint8_t* row = &src.px[size_t(y) * src.w];
        if (std::memcmp(row, blank.data(), blank.size()) == 0) continue;
        uint8_t* out = &mid.px[size_t(y) * ow];
        for (int x = 0; x < ow; ++x) {
            const int* w = tx[x].w.data();
            const uint8_t* p = row + tx[x].first;
            int n = int(tx[x].w.size()), acc = 0;
            for (int k = 0; k < n; ++k) acc += w[k] * p[k];
            out[x] = to_u8(acc);
        }
    }
    // vertical pass, row by row so the inner loop runs along a row
    Image out(ow, oh);
    std::vector<int> acc(static_cast<size_t>(ow), 0);
    for (int y = 0; y < oh; ++y) {
        std::fill(acc.begin(), acc.end(), 0);
        const int* w = ty[y].w.data();
        int n = int(ty[y].w.size());
        for (int k = 0; k < n; ++k) {
            const uint8_t* row = &mid.px[size_t(ty[y].first + k) * ow];
            int wk = w[k];
            for (int x = 0; x < ow; ++x) acc[x] += wk * row[x];
        }
        uint8_t* o = &out.px[size_t(y) * ow];
        for (int x = 0; x < ow; ++x) o[x] = to_u8(acc[x]);
    }
    return out;
}

Image rotate_bilinear(const Image& src, double deg, double cx, double cy,
                      const Rect* region) {
    Image out(src.w, src.h);
    Rect all{0, 0, src.w, src.h};
    const Rect& rg = region ? *region : all;
    double th = deg * M_PI / 180.0;
    double c = std::cos(th), s = std::sin(th);
    for (int j = rg.y0; j < rg.y1; ++j)
        for (int i = rg.x0; i < rg.x1; ++i) {
            double rx = i + 0.5 - cx, ry = j + 0.5 - cy;
            // inverse of a counter-clockwise turn (y points down)
            double sx = c * rx - s * ry + cx - 0.5;
            double sy = s * rx + c * ry + cy - 0.5;
            int x0 = int(std::floor(sx)), y0 = int(std::floor(sy));
            double fx = sx - x0, fy = sy - y0;
            auto px = [&](int x, int y) -> double {
                return (x < 0 || y < 0 || x >= src.w || y >= src.h) ? 0.0 : src.at(x, y);
            };
            double v = px(x0, y0) * (1 - fx) * (1 - fy) + px(x0 + 1, y0) * fx * (1 - fy) +
                       px(x0, y0 + 1) * (1 - fx) * fy + px(x0 + 1, y0 + 1) * fx * fy;
            out.at(i, j) = uint8_t(clampi(int(std::lround(v)), 0, 255));
        }
    return out;
}

void rotate_bilinear_pair(const Image& a, const Image& b, double deg, double cx, double cy,
                          const Rect& rg, Image& out_a, Image& out_b) {
    out_a = Image(a.w, a.h);
    out_b = Image(a.w, a.h);
    double th = deg * M_PI / 180.0;
    double c = std::cos(th), s = std::sin(th);
    for (int j = rg.y0; j < rg.y1; ++j) {
        double ry = j + 0.5 - cy;
        for (int i = rg.x0; i < rg.x1; ++i) {
            double rx = i + 0.5 - cx;
            double sx = c * rx - s * ry + cx - 0.5;
            double sy = s * rx + c * ry + cy - 0.5;
            int x0 = int(std::floor(sx)), y0 = int(std::floor(sy));
            if (x0 < -1 || y0 < -1 || x0 >= a.w || y0 >= a.h) continue;
            double fx = sx - x0, fy = sy - y0;
            double w00 = (1 - fx) * (1 - fy), w10 = fx * (1 - fy), w01 = (1 - fx) * fy,
                   w11 = fx * fy;
            bool inside = x0 >= 0 && y0 >= 0 && x0 + 1 < a.w && y0 + 1 < a.h;
            double va = 0, vb = 0;
            if (inside) {
                size_t k = size_t(y0) * a.w + x0;
                va = a.px[k] * w00 + a.px[k + 1] * w10 + a.px[k + a.w] * w01 +
                     a.px[k + a.w + 1] * w11;
                vb = b.px[k] * w00 + b.px[k + 1] * w10 + b.px[k + b.w] * w01 +
                     b.px[k + b.w + 1] * w11;
            } else {
                auto px = [&](const Image& im, int x, int y) -> double {
                    return (x < 0 || y < 0 || x >= im.w || y >= im.h) ? 0.0 : im.at(x, y);
                };
                va = px(a, x0, y0) * w00 + px(a, x0 + 1, y0) * w10 + px(a, x0, y0 + 1) * w01 +
                     px(a, x0 + 1, y0 + 1) * w11;
                vb = px(b, x0, y0) * w00 + px(b, x0 + 1, y0) * w10 + px(b, x0, y0 + 1) * w01 +
                     px(b, x0 + 1, y0 + 1) * w11;
            }
            if (vb == 0 && va == 0) continue;
            out_a.at(i, j) = uint8_t(clampi(int(std::lround(va)), 0, 255));
            out_b.at(i, j) = uint8_t(clampi(int(std::lround(vb)), 0, 255));
        }
    }
}

void paste_masked(Image& dst, const Image& src, const Image& mask, int ox, int oy,
                  const Rect* region) {
    Rect all{0, 0, src.w, src.h};
    const Rect& rg = region ? *region : all;
    for (int j = rg.y0; j < rg.y1; ++j) {
        int dj = j + oy;
        if (dj < 0 || dj >= dst.h) continue;
        for (int i = rg.x0; i < rg.x1; ++i) {
            int di = i + ox;
            if (di < 0 || di >= dst.w) continue;
            int m = mask.at(i, j);
            if (m == 0) continue;
            int d = dst.at(di, dj);
            dst.at(di, dj) = uint8_t((src.at(i, j) * m + d * (255 - m) + 127) / 255);
        }
    }
}

}  // namespace gfx

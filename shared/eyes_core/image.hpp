// 8-bit greyscale image and the few drawing primitives the eyes need.
// Shapes are filled without anti-aliasing, like Pillow's ImageDraw; the
// eyes are drawn at 4x size and shrunk with downscale_lanczos().
#pragma once

#include <cstdint>
#include <vector>

struct Pt {
    double x, y;
};

// Pixel rectangle: x0, y0 inclusive, x1, y1 exclusive.
struct Rect {
    int x0, y0, x1, y1;
};

struct Image {
    int w, h;
    std::vector<uint8_t> px;

    Image(int w_, int h_, uint8_t v = 0) : w(w_), h(h_), px(size_t(w_) * h_, v) {}
    uint8_t& at(int x, int y) { return px[size_t(y) * w + x]; }
    uint8_t at(int x, int y) const { return px[size_t(y) * w + x]; }
};

namespace gfx {

// Box coordinates are inclusive, as in Pillow: (x0, y0)..(x1, y1) covers
// the pixels from x0 up to and including x1.
void fill_rect(Image& im, double x0, double y0, double x1, double y1, uint8_t v);
void fill_rounded_rect(Image& im, double x0, double y0, double x1, double y1,
                       double r, uint8_t v);
void outline_rounded_rect(Image& im, double x0, double y0, double x1, double y1,
                          double r, double width, uint8_t v);
void fill_ellipse(Image& im, double x0, double y0, double x1, double y1, uint8_t v);
void fill_polygon(Image& im, const std::vector<Pt>& pts, uint8_t v);

// Thick line through the points. Ends are flat; interior joints are rounded
// when round_joints is set (Pillow's joint="curve").
void polyline(Image& im, const std::vector<Pt>& pts, double width, uint8_t v,
              bool round_joints);

// Shrink by an integer factor with a Lanczos-3 filter (same weights as
// Pillow's Image.resize(..., LANCZOS), two passes with 8-bit rounding).
Image downscale_lanczos(const Image& src, int factor);

// Rotate counter-clockwise by deg about (cx, cy), bilinear, 0 outside.
// Only the pixels inside `region` (default: all) are computed; the rest stay 0.
Image rotate_bilinear(const Image& src, double deg, double cx, double cy,
                      const Rect* region = nullptr);

// rotate_bilinear() of two same-sized images with one set of coordinates.
void rotate_bilinear_pair(const Image& a, const Image& b, double deg, double cx, double cy,
                          const Rect& region, Image& out_a, Image& out_b);

// dst = src where mask is 255, dst where it is 0, blended between.
// (ox, oy) is where src's top-left lands in dst.
// Only the part of src inside `region` (default: all of it) is pasted.
void paste_masked(Image& dst, const Image& src, const Image& mask, int ox, int oy,
                  const Rect* region = nullptr);

}  // namespace gfx

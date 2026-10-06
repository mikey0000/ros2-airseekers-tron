#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "det_ros_cpp/yolo.hpp"

using namespace det_ros_cpp;

TEST(Letterbox, Oa960x540To480x640) {
  auto lb = compute_letterbox(960, 540, 480, 640);
  EXPECT_DOUBLE_EQ(lb.scale, 0.5);
  EXPECT_EQ(lb.new_w, 480);
  EXPECT_EQ(lb.new_h, 270);
  EXPECT_EQ(lb.pad_x, 0);
  EXPECT_EQ(lb.pad_y, 185);
}

TEST(Letterbox, Vio640x480And1080p) {
  auto lb = compute_letterbox(640, 480, 480, 640);
  EXPECT_DOUBLE_EQ(lb.scale, 0.75);
  EXPECT_EQ(lb.new_w, 480);
  EXPECT_EQ(lb.new_h, 360);
  EXPECT_EQ(lb.pad_y, 140);
  auto hd = compute_letterbox(1920, 1080, 480, 640);
  EXPECT_EQ(hd.new_w, 480);
  EXPECT_EQ(hd.new_h, 270);
}

TEST(Letterbox, RoundHalfEvenLikePython) {
  // 5 * 0.5 = 2.5 -> Python round() = 2 ; 7 * 0.5 = 3.5 -> 4
  auto a = compute_letterbox(10, 5, 5, 100);
  EXPECT_EQ(a.new_h, 2);
  auto b = compute_letterbox(10, 7, 5, 100);
  EXPECT_EQ(b.new_h, 4);
}

TEST(Letterbox, ToSourceRoundTrip) {
  auto lb = compute_letterbox(960, 540, 480, 640);
  Box b{10.f, 185.f + 20.f, 110.f, 185.f + 70.f, 3, 0.9f};
  auto s = to_source(b, lb, 960, 540);
  EXPECT_NEAR(s.x1, 20.0, 1e-4);
  EXPECT_NEAR(s.y1, 40.0, 1e-4);
  EXPECT_NEAR(s.x2, 220.0, 1e-4);
  EXPECT_NEAR(s.y2, 140.0, 1e-4);
  Box out{-50.f, 0.f, 9999.f, 9999.f, 0, 0.5f};  // clamps
  auto c = to_source(out, lb, 960, 540);
  EXPECT_EQ(c.x1, 0.0);
  EXPECT_EQ(c.y1, 0.0);
  EXPECT_EQ(c.x2, 960.0);
  EXPECT_EQ(c.y2, 540.0);
}

// Synthetic int8 9-output head: 3 scales x (box [1,64,h,w], cls [1,22,h,w], sum [1,1,h,w])
struct Synth {
  std::vector<std::vector<int8_t>> bufs;
  std::vector<Tensor> t;
  static constexpr float kScale = 1.f / 64.f;  // cls/box dequant: (q - zp) * scale
  static constexpr int kZp = -128;
  Synth() {
    const int grids[3][2] = {{80, 60}, {40, 30}, {20, 15}};
    bufs.reserve(9);
    for (auto &g : grids) {
      const int h = g[0], w = g[1];
      for (int c : {64, 22, 1}) {
        bufs.emplace_back(static_cast<size_t>(c) * h * w, static_cast<int8_t>(kZp));
        Tensor x;
        x.data = bufs.back().data();
        x.is_int8 = true;
        x.zp = kZp;
        x.scale = kScale;
        x.c = c; x.h = h; x.w = w;
        t.push_back(x);
      }
    }
  }
  static int8_t q(float v) { return static_cast<int8_t>(std::lround(v / kScale) + kZp); }
  // put an object of class cls/score at cell (y,x) of branch b, with DFL peak bins d[4]
  void put(int b, int y, int x, int cls, float score, const int d[4]) {
    Tensor &box = t[3 * b], &cl = t[3 * b + 1];
    const int hw = cl.h * cl.w, idx = y * cl.w + x;
    auto *bb = bufs[3 * b].data();
    for (int k = 0; k < 4; ++k) bb[static_cast<size_t>(k * 16 + d[k]) * hw + idx] = q(1.98f);  // 127
    bufs[3 * b + 1][static_cast<size_t>(cls) * hw + idx] = q(score);
    (void)box;
  }
};

static float dfl_expect(int peak) {
  // bins: peak dequantises to (-1 - zp) * scale = 127/64, others to 0 -> softmax expectation
  const float hi = 127 / 64.f;
  float sum = 0.f, acc = 0.f;
  for (int j = 0; j < 16; ++j) {
    float e = std::exp((j == peak ? hi : 0.f) - hi);
    sum += e;
    acc += e * j;
  }
  return acc / sum;
}

TEST(Decode, SyntheticInt8Tensor) {
  Synth s;
  const int d[4] = {2, 3, 4, 5};
  s.put(0, 40, 30, 3, 0.9f, d);  // stride 8
  s.put(2, 5, 7, 21, 0.6f, d);   // stride 32
  auto boxes = decode(s.t, 480, 640, 0.25f);
  ASSERT_EQ(boxes.size(), 2u);
  const Box &a = boxes[0];
  EXPECT_EQ(a.cls, 3);
  EXPECT_NEAR(a.score, 0.90625f, 1e-6);  // q(0.9) dequantised
  EXPECT_NEAR(a.x1, (30.5f - dfl_expect(2)) * 8.f, 1e-3);
  EXPECT_NEAR(a.y1, (40.5f - dfl_expect(3)) * 8.f, 1e-3);
  EXPECT_NEAR(a.x2, (30.5f + dfl_expect(4)) * 8.f, 1e-3);
  EXPECT_NEAR(a.y2, (40.5f + dfl_expect(5)) * 8.f, 1e-3);
  const Box &b = boxes[1];
  EXPECT_EQ(b.cls, 21);
  EXPECT_NEAR(b.x1, (7.5f - dfl_expect(2)) * 32.f, 1e-3);
  EXPECT_NEAR(b.y2, (5.5f + dfl_expect(5)) * 32.f, 1e-3);
  // below-threshold cells are dropped
  EXPECT_TRUE(decode(s.t, 480, 640, 0.95f).empty());
}

TEST(Decode, FloatTensorSameAsInt8) {
  Synth s;
  const int d[4] = {1, 1, 6, 6};
  s.put(1, 10, 10, 0, 0.7f, d);
  std::vector<std::vector<float>> fb;
  std::vector<Tensor> ft;
  fb.reserve(9);
  for (auto &t : s.t) {
    fb.emplace_back(static_cast<size_t>(t.c) * t.h * t.w);
    for (size_t i = 0; i < fb.back().size(); ++i)
      fb.back()[i] = t.at(static_cast<int>(i / (t.h * t.w)), static_cast<int>(i % (t.h * t.w)));
    Tensor f = t;
    f.is_int8 = false;
    f.data = fb.back().data();
    ft.push_back(f);
  }
  auto a = decode(s.t, 480, 640, 0.25f), b = decode(ft, 480, 640, 0.25f);
  ASSERT_EQ(a.size(), 1u);
  ASSERT_EQ(b.size(), 1u);
  EXPECT_FLOAT_EQ(a[0].x1, b[0].x1);
  EXPECT_FLOAT_EQ(a[0].score, b[0].score);
}

TEST(Decode, NativeNc1hwc2SameAsNchw) {
  Synth s;
  const int d[4] = {3, 2, 7, 1};
  s.put(0, 12, 50, 17, 0.8f, d);
  s.put(1, 39, 0, 5, 0.5f, d);
  // repack every tensor into NC1HWC2 with c2 = 16 and a padded row stride (w + 4)
  std::vector<std::vector<int8_t>> nb;
  std::vector<Tensor> nt;
  nb.reserve(9);
  for (auto &t : s.t) {
    Tensor n = t;
    n.c2 = 16;
    n.ws = t.w + 4;
    const int c1 = (t.c + 15) / 16;
    nb.emplace_back(static_cast<size_t>(c1) * t.h * n.ws * 16, static_cast<int8_t>(0x55));
    auto *src = static_cast<const int8_t *>(t.data);
    for (int c = 0; c < t.c; ++c)
      for (int i = 0; i < t.h * t.w; ++i) nb.back()[n.offset(c, i)] = src[t.offset(c, i)];
    n.data = nb.back().data();
    nt.push_back(n);
  }
  auto a = decode(s.t, 480, 640, 0.25f), b = decode(nt, 480, 640, 0.25f);
  ASSERT_EQ(a.size(), 2u);
  ASSERT_EQ(b.size(), 2u);
  for (size_t i = 0; i < a.size(); ++i) {
    EXPECT_EQ(a[i].cls, b[i].cls);
    EXPECT_FLOAT_EQ(a[i].x1, b[i].x1);
    EXPECT_FLOAT_EQ(a[i].y2, b[i].y2);
    EXPECT_FLOAT_EQ(a[i].score, b[i].score);
  }
}

TEST(Nms, ClassAwareSuppression) {
  std::vector<Box> in = {
      {0, 0, 100, 100, 1, 0.8f},
      {5, 5, 105, 105, 1, 0.9f},    // overlaps the first (same class) -> keeps this one
      {2, 2, 102, 102, 0, 0.5f},    // same place, other class -> kept
      {200, 200, 250, 250, 1, 0.3f} // far away -> kept
  };
  auto out = nms_per_class(in, 0.45f);
  ASSERT_EQ(out.size(), 3u);
  // classes ascending, descending score within a class
  EXPECT_EQ(out[0].cls, 0);
  EXPECT_EQ(out[1].cls, 1);
  EXPECT_FLOAT_EQ(out[1].score, 0.9f);
  EXPECT_FLOAT_EQ(out[2].score, 0.3f);
}

TEST(Nms, ThresholdBoundary) {
  // IoU of two 10x10 boxes offset by 5 in x = 50/150 = 0.333
  std::vector<Box> in = {{0, 0, 10, 10, 2, 0.9f}, {5, 0, 15, 10, 2, 0.8f}};
  EXPECT_EQ(nms_per_class(in, 0.45f).size(), 2u);
  EXPECT_EQ(nms_per_class(in, 0.30f).size(), 1u);
}

TEST(Topics, Annotated) {
  EXPECT_EQ(annotated_topic_for("/left_oa_camera/image_raw"), "/left_oa_camera/image_annotated");
  EXPECT_EQ(annotated_topic_for("/vio/right/image_raw"), "/vio/right/image_annotated");
  EXPECT_EQ(annotated_topic_for("cam"), "/cam_annotated");
  EXPECT_EQ(label_for(21, kBestLargeClasses), "dock");
  EXPECT_EQ(label_for(22, kBestLargeClasses), "22");
}

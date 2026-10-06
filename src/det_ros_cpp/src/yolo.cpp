#include "det_ros_cpp/yolo.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <numeric>

namespace det_ros_cpp {

const std::vector<std::string> kBestLargeClasses = {
    "person", "dog", "cat", "sports ball", "hedgehog", "rabbit", "stone", "hoe",
    "shovel", "manhole", "brick", "trashbin", "toycars", "potted plant", "cans",
    "bottle", "book", "backpack", "wood", "chair", "trunk", "dock"};

std::string label_for(int cls, const std::vector<std::string> &classes) {
  if (cls >= 0 && cls < static_cast<int>(classes.size())) return classes[cls];
  return std::to_string(cls);
}

std::string annotated_topic_for(const std::string &image_topic) {
  std::string t = image_topic;
  while (!t.empty() && t.front() == '/') t.erase(t.begin());
  while (!t.empty() && t.back() == '/') t.pop_back();
  t = "/" + t;
  auto pos = t.rfind('/');
  if (pos == 0) return t + "_annotated";
  return t.substr(0, pos) + "/image_annotated";
}

Letterbox compute_letterbox(int src_w, int src_h, int dst_w, int dst_h) {
  Letterbox lb;
  lb.scale = std::min(static_cast<double>(dst_w) / src_w, static_cast<double>(dst_h) / src_h);
  // std::nearbyint uses the current rounding mode (round-half-to-even) like Python round()
  lb.new_w = static_cast<int>(std::nearbyint(src_w * lb.scale));
  lb.new_h = static_cast<int>(std::nearbyint(src_h * lb.scale));
  // Python floor division; the difference is >= 0 here
  lb.pad_x = (dst_w - lb.new_w) / 2;
  lb.pad_y = (dst_h - lb.new_h) / 2;
  return lb;
}

namespace {

// argmax over channels at cell idx; works in the int8 domain when possible (monotonic).
inline void best_class(const Tensor &t, int idx, int &best, float &best_score) {
  if (t.is_int8 && t.c2 > 0) {  // native NC1HWC2: c2 contiguous channels per cell
    auto *d = static_cast<int8_t const *>(t.data);
    const size_t plane = t.offset(t.c2, idx) - t.offset(0, idx);  // one C1 block
    const int8_t *p = d + t.offset(0, idx);
    int8_t bq = p[0];
    best = 0;
    for (int c = 1; c < t.c; ++c) {
      const int8_t q = p[(c / t.c2) * plane + c % t.c2];
      if (q > bq) { bq = q; best = c; }
    }
    best_score = (bq - t.zp) * t.scale;
  } else if (t.is_int8) {
    auto *d = static_cast<int8_t const *>(t.data);
    int8_t bq = d[t.offset(0, idx)];
    best = 0;
    for (int c = 1; c < t.c; ++c) {
      int8_t q = d[t.offset(c, idx)];
      if (q > bq) { bq = q; best = c; }
    }
    best_score = (bq - t.zp) * t.scale;
  } else {
    auto *d = static_cast<float const *>(t.data);
    float bs = d[t.offset(0, idx)];
    best = 0;
    for (int c = 1; c < t.c; ++c) {
      float v = d[t.offset(c, idx)];
      if (v > bs) { bs = v; best = c; }
    }
    best_score = bs;
  }
}

}  // namespace

std::vector<Box> decode(const std::vector<Tensor> &outputs, int img_w, int img_h,
                        float obj_thresh) {
  std::vector<Box> out;
  const int branches = 3;
  const int per = static_cast<int>(outputs.size()) / branches;
  if (per < 2) return out;
  std::vector<float> bins;
  for (int b = 0; b < branches; ++b) {
    const Tensor &pos = outputs[per * b];
    const Tensor &cls = outputs[per * b + 1];
    const int gh = cls.h, gw = cls.w, hw = gh * gw;
    const int mc = pos.c / 4;
    bins.resize(mc);
    const float sx = static_cast<float>(static_cast<double>(img_w) / gw);
    const float sy = static_cast<float>(static_cast<double>(img_h) / gh);
    for (int idx = 0; idx < hw; ++idx) {
      int best;
      float score;
      best_class(cls, idx, best, score);
      if (!(score >= obj_thresh)) continue;
      float d[4];
      for (int k = 0; k < 4; ++k) {  // DFL soft-argmax over mc bins
        float mx = -INFINITY;
        for (int j = 0; j < mc; ++j) {
          bins[j] = pos.at(k * mc + j, idx);
          mx = std::max(mx, bins[j]);
        }
        float sum = 0.f, acc = 0.f;
        for (int j = 0; j < mc; ++j) {
          float e = std::exp(bins[j] - mx);
          sum += e;
          acc += e * static_cast<float>(j);
        }
        d[k] = acc / sum;
      }
      const float cx = static_cast<float>(idx % gw) + 0.5f;
      const float cy = static_cast<float>(idx / gw) + 0.5f;
      out.push_back({(cx - d[0]) * sx, (cy - d[1]) * sy, (cx + d[2]) * sx, (cy + d[3]) * sy,
                     best, score});
    }
  }
  return out;
}

std::vector<Box> nms_per_class(const std::vector<Box> &boxes, float nms_thresh) {
  std::map<int, std::vector<int>> by_cls;  // ascending class id like np.unique
  for (int i = 0; i < static_cast<int>(boxes.size()); ++i) by_cls[boxes[i].cls].push_back(i);
  std::vector<Box> keep;
  for (auto &kv : by_cls) {
    auto &order = kv.second;
    std::stable_sort(order.begin(), order.end(),
                     [&](int a, int b) { return boxes[a].score > boxes[b].score; });
    std::vector<char> dead(order.size(), 0);
    for (size_t i = 0; i < order.size(); ++i) {
      if (dead[i]) continue;
      const Box &bi = boxes[order[i]];
      keep.push_back(bi);
      const float wi = bi.x2 - bi.x1, hi = bi.y2 - bi.y1, ai = wi * hi;
      for (size_t j = i + 1; j < order.size(); ++j) {
        if (dead[j]) continue;
        const Box &bj = boxes[order[j]];
        const float wj = bj.x2 - bj.x1, hj = bj.y2 - bj.y1;
        const float xx1 = std::max(bi.x1, bj.x1), yy1 = std::max(bi.y1, bj.y1);
        const float xx2 = std::min(bi.x1 + wi, bj.x1 + wj), yy2 = std::min(bi.y1 + hi, bj.y1 + hj);
        const float w1 = std::max(0.f, xx2 - xx1 + 0.00001f);
        const float h1 = std::max(0.f, yy2 - yy1 + 0.00001f);
        const float inter = w1 * h1;
        const float ovr = inter / (ai + wj * hj - inter);
        if (!(ovr <= nms_thresh)) dead[j] = 1;
      }
    }
  }
  return keep;
}

std::vector<Box> post_process(const std::vector<Tensor> &outputs, int img_w, int img_h,
                              float obj_thresh, float nms_thresh) {
  return nms_per_class(decode(outputs, img_w, img_h, obj_thresh), nms_thresh);
}

SrcBox to_source(const Box &b, const Letterbox &lb, int src_w, int src_h) {
  const double inv = lb.scale != 0.0 ? 1.0 / lb.scale : 1.0;
  auto cl = [](double v, double hi) { return std::min(std::max(v, 0.0), hi); };
  return {cl((b.x1 - lb.pad_x) * inv, src_w), cl((b.y1 - lb.pad_y) * inv, src_h),
          cl((b.x2 - lb.pad_x) * inv, src_w), cl((b.y2 - lb.pad_y) * inv, src_h), b.cls,
          b.score};
}

}  // namespace det_ros_cpp

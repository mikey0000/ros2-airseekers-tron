// Pure (ROS-free, NPU-free) YOLOv8 helpers for det_ros_cpp: letterbox geometry,
// decode of the Rockchip 9-output head (int8 or float tensors) and class-aware NMS.
// Numerically mirrors det_ros/yolo_postprocess.py::post_process and
// mower_rknn/preprocess.py::letterbox so both backends publish the same detections.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace det_ros_cpp {

struct Letterbox {
  double scale{1.0};
  int new_w{0}, new_h{0};
  int pad_x{0}, pad_y{0};
};

// Same math as mower_rknn.letterbox: scale = min(W/iw, H/ih), round-half-even of the
// resized size (Python round()), centred integer padding.
Letterbox compute_letterbox(int src_w, int src_h, int dst_w, int dst_h);

struct Box {
  float x1, y1, x2, y2;
  int cls;
  float score;
};

// One output tensor, logical shape [1, C, H, W]; either int8 (affine quantised) or
// float32. Memory layout is NCHW (c2 == 0) or the NPU-native NC1HWC2 (c2 > 0:
// [1, ceil(C/c2), H, W, c2], read without the runtime's CPU re-layout).
struct Tensor {
  const void *data{nullptr};
  bool is_int8{true};
  int32_t zp{0};
  float scale{1.f};
  int c{0}, h{0}, w{0};
  int c2{0};
  int ws{0};  // NC1HWC2 row stride in elements (0 = w)
  size_t offset(int ch, int idx) const {
    if (c2 > 0) {
      if (ws > w) idx = (idx / w) * ws + idx % w;
      const int s = ws > w ? ws : w;
      return (static_cast<size_t>(ch / c2) * h * s + idx) * c2 + ch % c2;
    }
    return static_cast<size_t>(ch) * h * w + idx;
  }
  float at(int ch, int idx) const {  // idx = y*w + x
    size_t i = offset(ch, idx);
    if (is_int8) return (static_cast<int8_t const *>(data)[i] - zp) * scale;
    return static_cast<float const *>(data)[i];
  }
};

// Decode the 3 branches (outputs [box, cls, score_sum] x 3). Boxes are xyxy in
// model-input pixels (letterboxed space). Order of the returned vector is unspecified.
std::vector<Box> decode(const std::vector<Tensor> &outputs, int img_w, int img_h,
                        float obj_thresh);

// Class-aware NMS, identical to yolo_postprocess._nms_boxes run per class (classes
// ascending, kept boxes in descending score order within a class).
std::vector<Box> nms_per_class(const std::vector<Box> &boxes, float nms_thresh);

// decode + NMS
std::vector<Box> post_process(const std::vector<Tensor> &outputs, int img_w, int img_h,
                              float obj_thresh, float nms_thresh);

// Model-input box -> source-image box (undo letterbox, clamp to the image).
struct SrcBox {
  double x1, y1, x2, y2;
  int cls;
  float score;
};
SrcBox to_source(const Box &b, const Letterbox &lb, int src_w, int src_h);

// "/left_oa_camera/image_raw" -> "/left_oa_camera/image_annotated"; "/foo" -> "/foo_annotated"
std::string annotated_topic_for(const std::string &image_topic);

extern const std::vector<std::string> kBestLargeClasses;
std::string label_for(int cls, const std::vector<std::string> &classes);

}  // namespace det_ros_cpp

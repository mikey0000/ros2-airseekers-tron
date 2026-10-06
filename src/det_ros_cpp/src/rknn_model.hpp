// Thin RAII wrapper over the RKNN C API (librknnrt 2.3.0) for one YOLOv8 context.
#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "det_ros_cpp/rknn/rknn_api.h"
#include "det_ros_cpp/yolo.hpp"

namespace det_ros_cpp {

class RknnModel {
 public:
  // Load from file (first context) ...
  RknnModel(const std::string &path, const std::string &core_mask, bool zero_copy);
  // ... or duplicate an existing one (shares the weights, own core mask / io buffers).
  RknnModel(RknnModel &src, const std::string &core_mask, bool zero_copy);
  ~RknnModel();
  RknnModel(const RknnModel &) = delete;
  RknnModel &operator=(const RknnModel &) = delete;

  int in_w() const { return in_w_; }
  int in_h() const { return in_h_; }
  // Input buffer to write the letterboxed RGB image into (H x W x 3, row stride
  // in_stride()). In pass-through mode the bytes are int8 (= uint8 ^ 0x80), see
  // input_xor().
  uint8_t *input() { return in_ptr_; }
  int in_stride() const { return in_stride_; }
  uint8_t input_xor() const { return passthrough_ ? 0x80 : 0x00; }
  bool passthrough() const { return passthrough_; }

  // Run; fills tensors (valid until the next run()).
  void run(std::vector<Tensor> &outs);

  std::string sdk_version() const;
  std::string describe() const;

 private:
  void setup(const std::string &core_mask, bool zero_copy);
  rknn_context ctx_{0};
  rknn_input_output_num io_{};
  rknn_tensor_attr in_attr_{};
  std::vector<rknn_tensor_attr> out_attrs_;
  std::vector<rknn_output> outs_;
  bool outs_held_{false};
  rknn_tensor_mem *in_mem_{nullptr};
  std::vector<rknn_tensor_mem *> out_mems_;      // native NC1HWC2 outputs (zero-copy)
  std::vector<rknn_tensor_attr> out_native_;
  std::vector<uint8_t> in_host_;
  uint8_t *in_ptr_{nullptr};
  int in_w_{0}, in_h_{0}, in_stride_{0};
  bool passthrough_{false};
  std::string core_;
};

rknn_core_mask parse_core_mask(const std::string &s);

}  // namespace det_ros_cpp

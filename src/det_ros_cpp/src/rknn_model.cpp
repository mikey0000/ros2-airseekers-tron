#include "rknn_model.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>

namespace det_ros_cpp {

namespace {
void check(int ret, const char *what) {
  if (ret < 0) throw std::runtime_error(std::string(what) + " failed, ret=" + std::to_string(ret));
}
std::vector<char> read_file(const std::string &p) {
  std::ifstream f(p, std::ios::binary | std::ios::ate);
  if (!f) throw std::runtime_error("cannot open " + p);
  std::vector<char> buf(static_cast<size_t>(f.tellg()));
  f.seekg(0);
  f.read(buf.data(), static_cast<std::streamsize>(buf.size()));
  return buf;
}
}  // namespace

rknn_core_mask parse_core_mask(const std::string &s) {
  if (s == "0") return RKNN_NPU_CORE_0;
  if (s == "1") return RKNN_NPU_CORE_1;
  if (s == "2") return RKNN_NPU_CORE_2;
  if (s == "0_1") return RKNN_NPU_CORE_0_1;
  if (s == "0_1_2") return RKNN_NPU_CORE_0_1_2;
  if (s == "all") return RKNN_NPU_CORE_ALL;
  return RKNN_NPU_CORE_AUTO;
}

RknnModel::RknnModel(const std::string &path, const std::string &core_mask, bool zero_copy) {
  auto buf = read_file(path);
  check(rknn_init(&ctx_, buf.data(), static_cast<uint32_t>(buf.size()), 0, nullptr), "rknn_init");
  setup(core_mask, zero_copy);
}

RknnModel::RknnModel(RknnModel &src, const std::string &core_mask, bool zero_copy) {
  check(rknn_dup_context(&src.ctx_, &ctx_), "rknn_dup_context");
  setup(core_mask, zero_copy);
}

void RknnModel::setup(const std::string &core_mask, bool zero_copy) {
  core_ = core_mask;
  check(rknn_set_core_mask(ctx_, parse_core_mask(core_mask)), "rknn_set_core_mask");
  check(rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_, sizeof(io_)), "query io num");
  if (io_.n_input != 1) throw std::runtime_error("expected 1 model input");
  in_attr_.index = 0;
  check(rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &in_attr_, sizeof(in_attr_)), "query input");
  if (in_attr_.fmt == RKNN_TENSOR_NHWC) {
    in_h_ = in_attr_.dims[1];
    in_w_ = in_attr_.dims[2];
  } else {
    in_h_ = in_attr_.dims[2];
    in_w_ = in_attr_.dims[3];
  }
  out_attrs_.resize(io_.n_output);
  for (uint32_t i = 0; i < io_.n_output; ++i) {
    out_attrs_[i].index = i;
    check(rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &out_attrs_[i], sizeof(rknn_tensor_attr)),
          "query output");
  }
  outs_.assign(io_.n_output, rknn_output{});

  // Pass-through zero-copy input: the model was calibrated with mean 0 / std 255, so its
  // int8 input quantisation is q = pixel - 128 (scale 1/255, zp -128) == pixel ^ 0x80.
  // Write that directly into NPU memory instead of letting the runtime convert a copy.
  rknn_tensor_attr native{};
  native.index = 0;
  bool native_ok =
      rknn_query(ctx_, RKNN_QUERY_NATIVE_INPUT_ATTR, &native, sizeof(native)) == RKNN_SUCC;
  if (zero_copy && native_ok && in_attr_.type == RKNN_TENSOR_INT8 &&
      native.fmt == RKNN_TENSOR_NHWC && in_attr_.zp == -128 &&
      std::fabs(in_attr_.scale * 255.f - 1.f) < 1e-3f) {
    in_mem_ = rknn_create_mem(ctx_, native.size_with_stride);
    if (in_mem_) {
      native.pass_through = 1;
      if (rknn_set_io_mem(ctx_, in_mem_, &native) == RKNN_SUCC) {
        passthrough_ = true;
        in_ptr_ = static_cast<uint8_t *>(in_mem_->virt_addr);
        int ws = native.w_stride ? static_cast<int>(native.w_stride) : in_w_;
        in_stride_ = ws * 3;
      } else {
        rknn_destroy_mem(ctx_, in_mem_);
        in_mem_ = nullptr;
      }
    }
  }
  // Zero-copy native outputs: the NPU writes int8 NC1HWC2 into our buffers and the
  // decoder indexes that layout directly (skips rknn_outputs_get's CPU re-layout).
  if (passthrough_) {
    bool ok = true;
    out_native_.assign(io_.n_output, rknn_tensor_attr{});
    for (uint32_t i = 0; i < io_.n_output && ok; ++i) {
      auto &a = out_native_[i];
      a.index = i;
      ok = rknn_query(ctx_, RKNN_QUERY_NATIVE_OUTPUT_ATTR, &a, sizeof(a)) == RKNN_SUCC &&
           a.type == RKNN_TENSOR_INT8 && a.fmt == RKNN_TENSOR_NC1HWC2 && a.n_dims == 5;
      if (!ok) break;
      auto *m = rknn_create_mem(ctx_, a.size_with_stride);
      if (!m) { ok = false; break; }
      out_mems_.push_back(m);
      ok = rknn_set_io_mem(ctx_, m, &a) == RKNN_SUCC;
    }
    if (!ok) {  // fall back to rknn_outputs_get (needs a fresh context state)
      for (auto *m : out_mems_) rknn_destroy_mem(ctx_, m);
      out_mems_.clear();
      out_native_.clear();
    }
  }
  if (!passthrough_) {
    in_host_.assign(static_cast<size_t>(in_w_) * in_h_ * 3, 0);
    in_ptr_ = in_host_.data();
    in_stride_ = in_w_ * 3;
  }
}

RknnModel::~RknnModel() {
  if (outs_held_) rknn_outputs_release(ctx_, io_.n_output, outs_.data());
  for (auto *m : out_mems_) rknn_destroy_mem(ctx_, m);
  if (in_mem_) rknn_destroy_mem(ctx_, in_mem_);
  if (ctx_) rknn_destroy(ctx_);
}

void RknnModel::run(std::vector<Tensor> &outs) {
  if (outs_held_) {
    rknn_outputs_release(ctx_, io_.n_output, outs_.data());
    outs_held_ = false;
  }
  if (!passthrough_) {
    rknn_input in{};
    in.index = 0;
    in.buf = in_host_.data();
    in.size = static_cast<uint32_t>(in_host_.size());
    in.type = RKNN_TENSOR_UINT8;
    in.fmt = RKNN_TENSOR_NHWC;
    in.pass_through = 0;
    check(rknn_inputs_set(ctx_, 1, &in), "rknn_inputs_set");
  } else {
    rknn_mem_sync(ctx_, in_mem_, RKNN_MEMORY_SYNC_TO_DEVICE);
  }
  check(rknn_run(ctx_, nullptr), "rknn_run");
  if (!out_mems_.empty()) {
    outs.resize(io_.n_output);
    for (uint32_t i = 0; i < io_.n_output; ++i) {
      rknn_mem_sync(ctx_, out_mems_[i], RKNN_MEMORY_SYNC_FROM_DEVICE);
      const auto &a = out_native_[i];
      Tensor &t = outs[i];
      t.data = out_mems_[i]->virt_addr;
      t.is_int8 = true;
      t.zp = a.zp;
      t.scale = a.scale;
      t.c = static_cast<int>(out_attrs_[i].dims[1]);  // logical C (NCHW attr)
      t.h = static_cast<int>(a.dims[2]);
      t.w = static_cast<int>(a.dims[3]);
      t.c2 = static_cast<int>(a.dims[4]);
      t.ws = static_cast<int>(a.w_stride);
    }
    return;
  }
  for (uint32_t i = 0; i < io_.n_output; ++i) {
    outs_[i] = rknn_output{};
    outs_[i].index = i;
    // int8 outputs stay quantised (decoded in C++); anything else as float
    outs_[i].want_float = out_attrs_[i].type == RKNN_TENSOR_INT8 ? 0 : 1;
  }
  check(rknn_outputs_get(ctx_, io_.n_output, outs_.data(), nullptr), "rknn_outputs_get");
  outs_held_ = true;
  outs.resize(io_.n_output);
  for (uint32_t i = 0; i < io_.n_output; ++i) {
    const auto &a = out_attrs_[i];
    Tensor &t = outs[i];
    t.data = outs_[i].buf;
    t.is_int8 = !outs_[i].want_float;
    t.zp = a.zp;
    t.scale = a.scale;
    // outputs are returned in the queried (NCHW) layout; tolerate 3-D [1,C,S]
    if (a.n_dims == 4) {
      t.c = a.dims[1]; t.h = a.dims[2]; t.w = a.dims[3];
    } else {
      t.c = a.dims[1]; t.h = 1; t.w = a.dims[2];
    }
  }
}

std::string RknnModel::sdk_version() const {
  rknn_sdk_version v{};
  if (rknn_query(ctx_, RKNN_QUERY_SDK_VERSION, &v, sizeof(v)) != RKNN_SUCC) return "?";
  return std::string(v.api_version) + " / driver " + v.drv_version;
}

std::string RknnModel::describe() const {
  std::ostringstream s;
  s << "core " << core_ << ", input " << in_w_ << "x" << in_h_
    << (in_attr_.type == RKNN_TENSOR_INT8 ? " int8" : " ?") << " zp=" << in_attr_.zp
    << " scale=" << in_attr_.scale << (passthrough_ ? ", zero-copy pass-through" : ", inputs_set")
    << (out_mems_.empty() ? ", outputs_get" : ", native NC1HWC2 outputs") << ", outputs:";
  for (auto &a : out_attrs_) {
    s << " [";
    for (uint32_t d = 0; d < a.n_dims; ++d) s << (d ? "," : "") << a.dims[d];
    s << "]" << (a.type == RKNN_TENSOR_INT8 ? "i8" : "f");
  }
  return s.str();
}

}  // namespace det_ros_cpp

// det_ros_cpp — C++ drop-in for det_ros (YOLOv8 on the RK3588 NPU).
//
// Same node name, parameters (det_ros/config/det.yaml), topics and message contents as
// the Python node:
//   sub  <left_topic>, <right_topic>, <extra_topics...>   sensor_msgs/Image (sensor_data QoS)
//   pub  /ai/det/detections          vision_msgs/Detection2DArray (class_id = class name)
//        /<camera_ns>/image_annotated, /ai/det/image_annotated   bgr8, only while subscribed
//
// Per camera: the subscription takes the SERIALIZED message and only stores it if the
// per-camera rate gate (max_rate_hz) lets it through, so skipped 10 Hz frames are never
// deserialised. A worker thread per camera owns its own RKNN context (rknn_dup_context of
// the first, core from core_masks[i % n]) and does letterbox -> NPU -> int8 decode ->
// NMS -> publish.
#include <time.h>

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include "det_ros_cpp/yolo.hpp"
#include "rknn_model.hpp"

using namespace std::chrono_literals;
using sensor_msgs::msg::Image;
using vision_msgs::msg::Detection2DArray;

namespace det_ros_cpp {

static const char *kDefaultModel = "/userdata/ros2/models/best_large_0208.rknn";

static bool file_exists(const std::string &p) {
  std::ifstream f(p);
  return static_cast<bool>(f);
}
static std::string basename_of(const std::string &p) {
  auto s = p.rfind('/');
  return s == std::string::npos ? p : p.substr(s + 1);
}

class DetNode;

class CameraWorker {
 public:
  CameraWorker(DetNode *node, std::string topic, std::unique_ptr<RknnModel> model,
               rclcpp::Publisher<Image>::SharedPtr ann_pub)
      : node_(node), topic_(std::move(topic)), model_(std::move(model)), ann_pub_(ann_pub) {}
  ~CameraWorker() { stop(); }

  void start() { th_ = std::thread([this] { loop(); }); }
  void stop() {
    {
      std::lock_guard<std::mutex> lk(m_);
      stopped_ = true;
    }
    cv_.notify_all();
    if (th_.joinable()) th_.join();
  }
  void offer(std::shared_ptr<rclcpp::SerializedMessage> msg, double max_rate);
  const std::string &topic() const { return topic_; }

 private:
  void loop();
  void process(const Image &msg);

  DetNode *node_;
  std::string topic_;
  std::unique_ptr<RknnModel> model_;
  rclcpp::Publisher<Image>::SharedPtr ann_pub_;
  std::thread th_;
  std::mutex m_;
  std::condition_variable cv_;
  std::shared_ptr<rclcpp::SerializedMessage> pending_;
  bool stopped_{false};
  std::chrono::steady_clock::time_point next_due_{};
  Letterbox lb_{};
  int lb_src_w_{-1}, lb_src_h_{-1};
  std::vector<Tensor> outs_;
  rclcpp::Serialization<Image> ser_;
  cv::Mat resized_;
  // per-stage thread CPU time (ms) and wall time of the NPU call, logged every 100 frames
  double t_deser_{0}, t_pre_{0}, t_npu_cpu_{0}, t_npu_wall_{0}, t_post_{0};
  int n_stats_{0};
};

static double thread_ms() {
  timespec ts;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return ts.tv_sec * 1e3 + ts.tv_nsec * 1e-6;
}
static double wall_ms() {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now().time_since_epoch()).count();
}

class DetNode : public rclcpp::Node {
 public:
  DetNode() : Node("det_ros") {
    declare_parameter("model_path", std::string(kDefaultModel));
    declare_parameter("models_dir", std::string(""));
    declare_parameter("dry_run", false);
    declare_parameter("classes", std::vector<std::string>{""});
    declare_parameter("img_size", std::vector<int64_t>{480, 640});
    declare_parameter("obj_thresh", 0.25);
    declare_parameter("nms_thresh", 0.45);
    declare_parameter("core_mask", std::string("0"));
    declare_parameter("core_masks", std::vector<std::string>{""});
    declare_parameter("left_topic", std::string("/left_oa_camera/image_raw"));
    declare_parameter("right_topic", std::string("/right_oa_camera/image_raw"));
    declare_parameter("extra_topics", std::vector<std::string>{""});
    declare_parameter("publish_annotated", true);
    declare_parameter("max_rate_hz", 5.0);
    // C++ only: write the int8 input straight into NPU memory (pass-through)
    declare_parameter("zero_copy", true);

    model_path_ = get_parameter("model_path").as_string();
    obj_thresh_ = static_cast<float>(get_parameter("obj_thresh").as_double());
    nms_thresh_ = static_cast<float>(get_parameter("nms_thresh").as_double());
    max_rate_ = get_parameter("max_rate_hz").as_double();
    auto sz = get_parameter("img_size").as_integer_array();
    if (sz.size() == 2) { img_w_ = static_cast<int>(sz[0]); img_h_ = static_cast<int>(sz[1]); }
    load_classes();

    std::string core_mask = get_parameter("core_mask").as_string();
    std::vector<std::string> core_masks;
    for (auto &c : str_array("core_masks"))
      if (!c.empty()) core_masks.push_back(c);
    const bool zero_copy = get_parameter("zero_copy").as_bool();
    const bool dry_run = get_parameter("dry_run").as_bool();

    std::vector<std::string> topics = {get_parameter("left_topic").as_string(),
                                       get_parameter("right_topic").as_string()};
    for (auto &t : str_array("extra_topics")) topics.push_back(t);
    topics.erase(std::remove(topics.begin(), topics.end(), std::string()), topics.end());

    // ---- model ----
    std::string resolved, problem;
    std::vector<std::string> cands = {model_path_};
    const std::string base = basename_of(model_path_);
    const std::string mdir = get_parameter("models_dir").as_string();
    if (!mdir.empty() && !base.empty()) cands.push_back(mdir + "/" + base);
    if (!base.empty()) cands.push_back(std::string("/userdata/ros2/models/") + base);
    for (auto &c : cands)
      if (file_exists(c)) { resolved = c; break; }
    std::vector<std::unique_ptr<RknnModel>> models;
    if (resolved.empty()) {
      problem = "model file not found (tried " + model_path_ + ", models_dir, /userdata/ros2/models)";
    } else {
      try {
        for (size_t i = 0; i < topics.size(); ++i) {
          std::string core = core_masks.empty() ? core_mask : core_masks[i % core_masks.size()];
          if (models.empty())
            models.push_back(std::make_unique<RknnModel>(resolved, core, zero_copy));
          else
            models.push_back(std::make_unique<RknnModel>(*models.front(), core, zero_copy));
          RCLCPP_INFO(get_logger(), "%s -> %s", topics[i].c_str(), models.back()->describe().c_str());
        }
      } catch (const std::exception &e) {
        problem = std::string("NPU init failed for ") + resolved + ": " + e.what();
        models.clear();
      }
    }
    if (!problem.empty()) {
      if (dry_run) {
        RCLCPP_WARN(get_logger(), "dry_run: no inference, publishing nothing (%s)", problem.c_str());
        return;
      }
      throw std::runtime_error(problem);
    }
    const std::string sdk = models.front()->sdk_version();
    RCLCPP_INFO(get_logger(), "loaded %s, librknnrt %s", resolved.c_str(), sdk.c_str());
    if (sdk.rfind("2.3.0", 0) != 0)
      RCLCPP_WARN(get_logger(), "librknnrt is not 2.3.0 (vendored rknn_api.h is 2.3.0)");
    if (models.front()->in_w() != img_w_ || models.front()->in_h() != img_h_) {
      RCLCPP_WARN(get_logger(), "img_size %dx%d != model input %dx%d; using the model's",
                  img_w_, img_h_, models.front()->in_w(), models.front()->in_h());
      img_w_ = models.front()->in_w();
      img_h_ = models.front()->in_h();
    }

    // ---- io ----
    auto sensor_qos = rclcpp::SensorDataQoS();
    det_pub_ = create_publisher<Detection2DArray>("/ai/det/detections", 10);
    ann_pub_ = create_publisher<Image>("/ai/det/image_annotated", 10);
    for (size_t i = 0; i < topics.size(); ++i) {
      auto cam_pub = create_publisher<Image>(annotated_topic_for(topics[i]), sensor_qos);
      auto w = std::make_shared<CameraWorker>(this, topics[i], std::move(models[i]), cam_pub);
      workers_.push_back(w);
      std::weak_ptr<CameraWorker> ww = w;
      subs_.push_back(create_subscription<Image>(
          topics[i], sensor_qos, [ww, this](std::shared_ptr<rclcpp::SerializedMessage> m) {
            if (auto s = ww.lock()) s->offer(m, max_rate_);
          }));
      w->start();
    }
    RCLCPP_INFO(get_logger(),
                "det_ros_cpp up: %zu classes, %zu cameras, input %dx%d, max %.1f Hz/camera",
                classes_.size(), topics.size(), img_w_, img_h_, max_rate_);
  }

  ~DetNode() override { workers_.clear(); }

  // shared state used by workers
  std::mutex pub_mutex_;
  rclcpp::Publisher<Detection2DArray>::SharedPtr det_pub_;
  rclcpp::Publisher<Image>::SharedPtr ann_pub_;
  std::vector<std::string> classes_;
  float obj_thresh_{0.25f}, nms_thresh_{0.45f};
  int img_w_{480}, img_h_{640};
  double max_rate_{5.0};
  bool publish_annotated() { return get_parameter("publish_annotated").as_bool(); }

 private:
  // by value: range-for over get_parameter(..).as_string_array() would dangle
  std::vector<std::string> str_array(const std::string &name) {
    return get_parameter(name).as_string_array();
  }
  void load_classes() {
    for (auto &c : str_array("classes"))
      if (!c.empty()) classes_.push_back(c);
    if (!classes_.empty()) return;
    std::ifstream f(model_path_ + ".list");
    if (f) {
      std::string ln;
      while (std::getline(f, ln)) {
        auto b = ln.find_first_not_of(" \t\r\n"), e = ln.find_last_not_of(" \t\r\n");
        if (b != std::string::npos) classes_.push_back(ln.substr(b, e - b + 1));
      }
      return;
    }
    if (basename_of(model_path_).rfind("best_large", 0) == 0) classes_ = kBestLargeClasses;
    else RCLCPP_WARN(get_logger(), "no class list; publishing class indices only");
  }

  std::string model_path_;
  std::vector<std::shared_ptr<CameraWorker>> workers_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
};

void CameraWorker::offer(std::shared_ptr<rclcpp::SerializedMessage> msg, double max_rate) {
  {
    std::lock_guard<std::mutex> lk(m_);
    if (max_rate > 0.0) {
      // Deadline schedule (not "now + period"): with a 10 Hz camera and a 5 Hz cap,
      // "now - last >= 200 ms" misses every other slot on jitter and averages ~4 Hz.
      const auto now = std::chrono::steady_clock::now();
      const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(1.0 / max_rate));
      if (now < next_due_ - period / 4) return;  // dropped before deserialisation
      next_due_ = std::max(next_due_ + period, now + period / 2);
    }
    pending_ = std::move(msg);
  }
  cv_.notify_one();
}

void CameraWorker::loop() {
  Image img;
  for (;;) {
    std::shared_ptr<rclcpp::SerializedMessage> msg;
    {
      std::unique_lock<std::mutex> lk(m_);
      cv_.wait(lk, [this] { return pending_ || stopped_; });
      if (stopped_) return;
      msg.swap(pending_);
    }
    try {
      const double t0 = thread_ms();
      ser_.deserialize_message(msg.get(), &img);
      msg.reset();
      t_deser_ += thread_ms() - t0;
      process(img);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 10000,
                            "%s: detection failed: %s", topic_.c_str(), e.what());
    }
  }
}

void CameraWorker::process(const Image &msg) {
  const double c_pre0 = thread_ms();
  std::string frame = msg.header.frame_id;
  if (frame.empty()) {
    std::string t = topic_;
    while (!t.empty() && t.front() == '/') t.erase(t.begin());
    frame = t.substr(0, t.find('/'));
  }
  // Source as BGR (no copy for bgr8/rgb8)
  cv::Mat src;
  bool is_rgb = false;
  cv_bridge::CvImageConstPtr holder;
  if (msg.encoding == "bgr8" || msg.encoding == "rgb8") {
    src = cv::Mat(static_cast<int>(msg.height), static_cast<int>(msg.width), CV_8UC3,
                  const_cast<uint8_t *>(msg.data.data()), msg.step);
    is_rgb = msg.encoding == "rgb8";
  } else {
    holder = cv_bridge::toCvCopy(msg, "bgr8");
    src = holder->image;
  }
  const int W = model_->in_w(), H = model_->in_h();
  const uint8_t x = model_->input_xor();
  cv::Mat in(H, W, CV_8UC3, model_->input(), static_cast<size_t>(model_->in_stride()));
  if (src.cols != lb_src_w_ || src.rows != lb_src_h_) {
    lb_ = compute_letterbox(src.cols, src.rows, W, H);
    lb_src_w_ = src.cols;
    lb_src_h_ = src.rows;
    in.setTo(cv::Scalar::all(x));  // black padding (0, or -128 in int8 pass-through)
  }
  cv::Mat roi = in(cv::Rect(lb_.pad_x, lb_.pad_y, lb_.new_w, lb_.new_h));
  if (src.cols == lb_.new_w && src.rows == lb_.new_h) resized_ = src;
  else cv::resize(src, resized_, cv::Size(lb_.new_w, lb_.new_h), 0, 0, cv::INTER_LINEAR);
  if (is_rgb) resized_.copyTo(roi);
  else cv::cvtColor(resized_, roi, cv::COLOR_BGR2RGB);  // writes into the NPU buffer
  if (x) cv::bitwise_xor(roi, cv::Scalar::all(x), roi);

  const double c1 = thread_ms(), w1 = wall_ms();
  model_->run(outs_);
  const double c2 = thread_ms();
  t_npu_wall_ += wall_ms() - w1;
  auto boxes = post_process(outs_, W, H, node_->obj_thresh_, node_->nms_thresh_);
  t_post_ += thread_ms() - c2;
  t_npu_cpu_ += c2 - c1;
  t_pre_ += c1 - c_pre0;
  if (++n_stats_ == 100) {
    RCLCPP_DEBUG(node_->get_logger(),
                "%s: per frame cpu ms: deserialize %.2f, letterbox %.2f, npu call %.2f "
                "(wall %.1f), decode+nms %.2f",
                topic_.c_str(), t_deser_ / 100, t_pre_ / 100, t_npu_cpu_ / 100,
                t_npu_wall_ / 100, t_post_ / 100);
    n_stats_ = 0;
    t_deser_ = t_pre_ = t_npu_cpu_ = t_npu_wall_ = t_post_ = 0;
  }

  auto det = std::make_unique<Detection2DArray>();
  det->header = msg.header;
  det->header.frame_id = frame;
  std::vector<SrcBox> src_boxes;
  src_boxes.reserve(boxes.size());
  for (auto &b : boxes) {
    SrcBox s = to_source(b, lb_, src.cols, src.rows);
    src_boxes.push_back(s);
    vision_msgs::msg::Detection2D d;
    d.header = det->header;
    d.bbox.center.position.x = (s.x1 + s.x2) / 2.0;
    d.bbox.center.position.y = (s.y1 + s.y2) / 2.0;
    d.bbox.size_x = s.x2 - s.x1;
    d.bbox.size_y = s.y2 - s.y1;
    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = label_for(s.cls, node_->classes_);
    hyp.hypothesis.score = static_cast<double>(s.score);
    d.results.push_back(hyp);
    det->detections.push_back(std::move(d));
  }
  const auto header = det->header;
  {
    std::lock_guard<std::mutex> lk(node_->pub_mutex_);
    node_->det_pub_->publish(std::move(det));
  }

  if (!node_->publish_annotated()) return;
  const bool cam = ann_pub_->get_subscription_count() > 0;
  const bool merged = node_->ann_pub_->get_subscription_count() > 0;
  if (!cam && !merged) return;
  auto out = std::make_unique<Image>();
  out->header = header;
  out->height = static_cast<uint32_t>(src.rows);
  out->width = static_cast<uint32_t>(src.cols);
  out->encoding = "bgr8";
  out->is_bigendian = 0;
  out->step = static_cast<uint32_t>(src.cols * 3);
  out->data.resize(static_cast<size_t>(out->step) * src.rows);
  cv::Mat vis(src.rows, src.cols, CV_8UC3, out->data.data(), out->step);
  if (is_rgb) cv::cvtColor(src, vis, cv::COLOR_RGB2BGR);
  else src.copyTo(vis);
  char txt[96];
  for (auto &s : src_boxes) {
    cv::rectangle(vis, cv::Point(static_cast<int>(s.x1), static_cast<int>(s.y1)),
                  cv::Point(static_cast<int>(s.x2), static_cast<int>(s.y2)), cv::Scalar(0, 255, 0), 2);
    std::snprintf(txt, sizeof(txt), "%s %.2f", label_for(s.cls, node_->classes_).c_str(), s.score);
    cv::putText(vis, txt, cv::Point(static_cast<int>(s.x1), std::max(12, static_cast<int>(s.y1) - 6)),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
  }
  std::lock_guard<std::mutex> lk(node_->pub_mutex_);
  if (cam && merged) {
    node_->ann_pub_->publish(*out);
    ann_pub_->publish(std::move(out));
  } else if (cam) {
    ann_pub_->publish(std::move(out));
  } else {
    node_->ann_pub_->publish(std::move(out));
  }
}

}  // namespace det_ros_cpp

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // OpenCV's worker pool spin-waits (7 threads x ~1% each for a 1 ms resize); the per-camera
  // threads already give the parallelism.
  cv::setNumThreads(0);
  std::shared_ptr<det_ros_cpp::DetNode> node;
  try {
    node = std::make_shared<det_ros_cpp::DetNode>();
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("det_ros"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);  // callbacks only park serialized messages; work is on the workers
  node.reset();
  rclcpp::shutdown();
  return 0;
}

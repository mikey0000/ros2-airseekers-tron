// DecayingObstacleLayer (2026-10-21): nav2_costmap_2d::ObstacleLayer whose LETHAL marks
// expire after `decay_time` seconds unless re-marked by an observation.
//
// Why: the stock ObstacleLayer clears only by raytracing. A stereo / det_range mark that no
// later ray crosses (just outside the mowing-area line, behind the robot, out of the camera
// fov during a pivot) stays lethal forever and blocked coverage pivots. The stock firmware
// used STVL with a 5 s camera decay, but spatio_temporal_voxel_layer is not in the Jazzy
// image, and the clear_* costmap services clear every clearable layer at once (no per-cell
// ageing). This subclass keeps all ObstacleLayer semantics (sources, raytrace clearing,
// observation_persistence, expected_update_rate / isCurrent) and only adds the decay.
//
// Mechanism (no copy of the marking code): before the base updateBounds() every LETHAL cell
// is set to a sentinel (INSCRIBED, never written by ObstacleLayer). The base then raytraces
// (sentinel -> FREE) and marks (-> LETHAL). Afterwards a LETHAL cell was (re-)marked this
// cycle: stamp it now. A remaining sentinel was neither cleared nor re-marked: restore it to
// LETHAL while younger than decay_time, else free it and add it to the update bounds.
// Stamps are keyed by world cell, so a rolling window's origin shift does not misalign them.
// An observation kept in its buffer (observation_persistence) re-marks every update, so a
// continuously seen obstacle never decays; one that left the camera view expires.
#include <cmath>
#include <cstdint>
#include <mutex>
#include <unordered_map>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/obstacle_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace mower_costmap_layers
{

class DecayingObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void onInitialize() override
  {
    ObstacleLayer::onInitialize();
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error("DecayingObstacleLayer: node expired");
    }
    declareParameter("decay_time", rclcpp::ParameterValue(5.0));
    node->get_parameter(name_ + ".decay_time", decay_time_);
    RCLCPP_INFO(node->get_logger(), "%s: marks decay after %.1f s (<= 0: never)",
      name_.c_str(), decay_time_);
  }

  void reset() override
  {
    ObstacleLayer::reset();
    stamps_.clear();
  }

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    if (!enabled_ || decay_time_ <= 0.0) {
      ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
      return;
    }
    std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
    constexpr unsigned char LETHAL = nav2_costmap_2d::LETHAL_OBSTACLE;
    constexpr unsigned char SENTINEL = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    const size_t n0 = static_cast<size_t>(size_x_) * size_y_;
    for (size_t i = 0; i < n0; ++i) {
      if (costmap_[i] == LETHAL) {
        costmap_[i] = SENTINEL;
      }
    }
    ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);

    const double now = clock_->now().seconds();
    std::unordered_map<int64_t, double> fresh;
    fresh.reserve(stamps_.size() + 16);
    for (unsigned int my = 0; my < size_y_; ++my) {
      for (unsigned int mx = 0; mx < size_x_; ++mx) {
        unsigned char & c = costmap_[getIndex(mx, my)];
        if (c != LETHAL && c != SENTINEL) {
          continue;
        }
        const int64_t key = cellKey(mx, my);
        double t = now;
        if (c == SENTINEL) {
          auto it = stamps_.find(key);
          t = it == stamps_.end() ? now : it->second;
          if (now - t > decay_time_) {
            c = nav2_costmap_2d::FREE_SPACE;
            double wx, wy;
            mapToWorld(mx, my, wx, wy);
            touch(wx, wy, min_x, min_y, max_x, max_y);
            continue;
          }
          c = LETHAL;
        }
        fresh.emplace(key, t);
      }
    }
    stamps_.swap(fresh);
  }

private:
  int64_t cellKey(unsigned int mx, unsigned int my) const
  {
    const int64_t gx = static_cast<int64_t>(std::floor(origin_x_ / resolution_ + 0.5)) + mx;
    const int64_t gy = static_cast<int64_t>(std::floor(origin_y_ / resolution_ + 0.5)) + my;
    return (gx << 32) ^ (gy & 0xffffffffLL);
  }

  double decay_time_{5.0};
  std::unordered_map<int64_t, double> stamps_;
};

}  // namespace mower_costmap_layers

PLUGINLIB_EXPORT_CLASS(mower_costmap_layers::DecayingObstacleLayer, nav2_costmap_2d::Layer)

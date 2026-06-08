#pragma once

#include <nav_math/nav_math.hpp>

#include <array>
#include <cstdint>
#include <vector>

namespace spatial_awareness {

using nav_math::Vec2;

struct Pose2D {
  Vec2 position;
  Vec2 forward{0.0f, 1.0f};
};

struct OccupiedCell {
  Vec2 position;
  int grid_x{0};
  int grid_z{0};
};

struct DirectionalRisk {
  bool active{false};
  float intensity{0.0f};
  float distance_m{0.0f};
  float closing_speed_mps{0.0f};
  float time_to_collision_s{0.0f};
  Vec2 obstacle_position;
};

struct RiskResult {
  DirectionalRisk left;
  DirectionalRisk right;
  DirectionalRisk rear;
};

struct ActiveDirections {
  bool left{false};
  bool right{false};
  bool rear{false};
};

class RiskEvaluator {
 public:
  struct Config {
    int min_cluster_cells{3};
    float min_distance_m{0.20f};
    float max_distance_m{1.00f};
    float body_radius_m{0.20f};
    float cell_size_m{0.10f};
    float min_motion_speed_mps{0.10f};
    float min_closing_speed_mps{0.10f};
    float closing_speed_hysteresis_mps{0.03f};
    float ttc_min_s{0.25f};
    float ttc_max_s{2.00f};
    float fov_margin_deg{3.0f};
    float rear_half_angle_deg{45.0f};
  };

  RiskEvaluator();
  explicit RiskEvaluator(const Config& config);

  RiskResult evaluate(const Pose2D& pose, const Vec2& velocity,
                      float aperture_half_angle_deg,
                      const std::vector<OccupiedCell>& occupied_cells,
                      const ActiveDirections& previously_active = {}) const;

 private:
  enum class Region : std::uint8_t { kLeft, kRight, kRear };

  struct Candidate {
    Region region;
    DirectionalRisk risk;
  };

  std::vector<std::vector<std::size_t>> buildClusters(
      const std::vector<OccupiedCell>& cells) const;
  bool makeCandidate(const Pose2D& pose, const Vec2& velocity,
                     float aperture_half_angle_deg, const OccupiedCell& cell,
                     const ActiveDirections& previously_active,
                     Candidate& candidate) const;
  static void keepMostUrgent(DirectionalRisk& current,
                             const DirectionalRisk& candidate);

  Config config_;
};

}  // namespace spatial_awareness

#include "spatial_awareness/risk_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace spatial_awareness {
namespace {

float clamp(float value, float low, float high) {
  return std::max(low, std::min(value, high));
}

std::int64_t cellKey(int x, int z) {
  const auto upper = static_cast<std::uint64_t>(static_cast<std::uint32_t>(x));
  const auto lower = static_cast<std::uint32_t>(z);
  return static_cast<std::int64_t>((upper << 32) | lower);
}

}  // namespace

RiskEvaluator::RiskEvaluator() : RiskEvaluator(Config{}) {}

RiskEvaluator::RiskEvaluator(const Config& config) : config_(config) {}

std::vector<std::vector<std::size_t>> RiskEvaluator::buildClusters(
    const std::vector<OccupiedCell>& cells) const {
  std::unordered_map<std::int64_t, std::size_t> by_grid_position;
  by_grid_position.reserve(cells.size());
  for (std::size_t i = 0; i < cells.size(); ++i) {
    by_grid_position[cellKey(cells[i].grid_x, cells[i].grid_z)] = i;
  }

  std::vector<bool> visited(cells.size(), false);
  std::vector<std::vector<std::size_t>> clusters;
  for (std::size_t seed = 0; seed < cells.size(); ++seed) {
    if (visited[seed]) {
      continue;
    }

    std::vector<std::size_t> cluster;
    std::queue<std::size_t> pending;
    pending.push(seed);
    visited[seed] = true;

    while (!pending.empty()) {
      const std::size_t index = pending.front();
      pending.pop();
      cluster.push_back(index);

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dz == 0) {
            continue;
          }
          const auto neighbor = by_grid_position.find(
              cellKey(cells[index].grid_x + dx, cells[index].grid_z + dz));
          if (neighbor == by_grid_position.end() || visited[neighbor->second]) {
            continue;
          }
          visited[neighbor->second] = true;
          pending.push(neighbor->second);
        }
      }
    }

    if (static_cast<int>(cluster.size()) >= config_.min_cluster_cells) {
      clusters.push_back(std::move(cluster));
    }
  }
  return clusters;
}

bool RiskEvaluator::makeCandidate(const Pose2D& pose, const Vec2& velocity,
                                  float aperture_half_angle_deg,
                                  const OccupiedCell& cell,
                                  const ActiveDirections& previously_active,
                                  Candidate& candidate) const {
  const float speed = velocity.norm();
  if (speed < config_.min_motion_speed_mps) {
    return false;
  }

  const Vec2 relative = cell.position - pose.position;
  const float center_distance = relative.norm();
  // Report and evaluate clearance to the user's body, rather than distance to
  // the user's centre. The occupied cell radius is already accounted for when
  // deciding whether the cell intersects the collision corridor below.
  const float distance =
      std::max(0.0f, center_distance - config_.body_radius_m);
  if (distance < config_.min_distance_m || distance > config_.max_distance_m) {
    return false;
  }

  const Vec2 forward = pose.forward.normalized();
  if (forward.norm() < 0.5f) {
    return false;
  }
  const Vec2 right{forward.z, -forward.x};
  const float angle = std::atan2(right.dot(relative), forward.dot(relative));
  const float aperture =
      nav_math::deg2rad(std::max(0.0f, aperture_half_angle_deg) +
                        config_.fov_margin_deg);
  if (std::abs(angle) <= aperture) {
    return false;
  }

  const Vec2 velocity_direction = velocity * (1.0f / speed);
  const float longitudinal = relative.dot(velocity_direction);
  const Vec2 lateral_vector = relative - velocity_direction * longitudinal;
  const float cell_radius = config_.cell_size_m * nav_math::kHalfSqrt2;
  if (longitudinal <= 0.0f ||
      lateral_vector.norm() > config_.body_radius_m + cell_radius) {
    return false;
  }

  const float rear_start =
      nav_math::kPi - nav_math::deg2rad(config_.rear_half_angle_deg);
  bool region_was_active = false;
  if (std::abs(angle) >= rear_start) {
    candidate.region = Region::kRear;
    region_was_active = previously_active.rear;
  } else if (angle > 0.0f) {
    candidate.region = Region::kRight;
    region_was_active = previously_active.right;
  } else {
    candidate.region = Region::kLeft;
    region_was_active = previously_active.left;
  }

  if (center_distance <= 1e-6f) {
    return false;
  }
  const Vec2 obstacle_direction = relative * (1.0f / center_distance);
  const float closing_speed = std::max(0.0f, velocity.dot(obstacle_direction));
  const float closing_threshold =
      region_was_active
          ? std::max(0.0f, config_.min_closing_speed_mps -
                               config_.closing_speed_hysteresis_mps)
          : config_.min_closing_speed_mps;
  if (closing_speed < closing_threshold) {
    return false;
  }

  const float ttc = distance / closing_speed;
  if (ttc >= config_.ttc_max_s) {
    return false;
  }

  DirectionalRisk risk;
  risk.active = true;
  risk.distance_m = distance;
  risk.closing_speed_mps = closing_speed;
  risk.time_to_collision_s = ttc;
  risk.obstacle_position = cell.position;
  const float denominator =
      std::max(1e-3f, config_.ttc_max_s - config_.ttc_min_s);
  risk.intensity =
      100.0f * clamp((config_.ttc_max_s - ttc) / denominator, 0.0f, 1.0f);

  candidate.risk = risk;
  return true;
}

void RiskEvaluator::keepMostUrgent(DirectionalRisk& current,
                                   const DirectionalRisk& candidate) {
  if (!current.active || candidate.intensity > current.intensity + 1e-4f ||
      (std::abs(candidate.intensity - current.intensity) <= 1e-4f &&
       candidate.time_to_collision_s < current.time_to_collision_s)) {
    current = candidate;
  }
}

RiskResult RiskEvaluator::evaluate(
    const Pose2D& pose, const Vec2& velocity, float aperture_half_angle_deg,
    const std::vector<OccupiedCell>& occupied_cells,
    const ActiveDirections& previously_active) const {
  RiskResult result;
  const auto clusters = buildClusters(occupied_cells);

  for (const auto& cluster : clusters) {
    for (const std::size_t index : cluster) {
      Candidate candidate;
      if (!makeCandidate(pose, velocity, aperture_half_angle_deg,
                         occupied_cells[index], previously_active, candidate)) {
        continue;
      }

      switch (candidate.region) {
        case Region::kLeft:
          keepMostUrgent(result.left, candidate.risk);
          break;
        case Region::kRight:
          keepMostUrgent(result.right, candidate.risk);
          break;
        case Region::kRear:
          keepMostUrgent(result.rear, candidate.risk);
          break;
      }
    }
  }
  return result;
}

}  // namespace spatial_awareness

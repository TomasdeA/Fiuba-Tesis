#include <gtest/gtest.h>

#include <vector>

#include "spatial_awareness/risk_evaluator.hpp"

namespace {

using spatial_awareness::OccupiedCell;
using spatial_awareness::Pose2D;
using spatial_awareness::RiskEvaluator;
using nav_math::Vec2;

std::vector<OccupiedCell> verticalCluster(float x, float z) {
  return {
      {{x, z - 0.1f}, 10, 10},
      {{x, z}, 10, 11},
      {{x, z + 0.1f}, 10, 12},
  };
}

std::vector<OccupiedCell> horizontalCluster(float x, float z) {
  return {
      {{x - 0.1f, z}, 10, 10},
      {{x, z}, 11, 10},
      {{x + 0.1f, z}, 12, 10},
  };
}

Pose2D forwardPose() { return {{0.0f, 0.0f}, {0.0f, 1.0f}}; }

TEST(RiskEvaluatorTest, SignalsRightWhenApproachingRightObstacle) {
  RiskEvaluator evaluator;
  const auto result = evaluator.evaluate(forwardPose(), Vec2{0.5f, 0.0f}, 45.0f,
                                         verticalCluster(0.5f, 0.0f));

  EXPECT_TRUE(result.right.active);
  EXPECT_GT(result.right.intensity, 0.0f);
  EXPECT_FALSE(result.left.active);
  EXPECT_FALSE(result.rear.active);
}

TEST(RiskEvaluatorTest, DoesNotSignalWhenStationaryOrMovingAway) {
  RiskEvaluator evaluator;
  const auto cells = verticalCluster(0.5f, 0.0f);

  const auto stationary =
      evaluator.evaluate(forwardPose(), Vec2{}, 45.0f, cells);
  const auto moving_away =
      evaluator.evaluate(forwardPose(), Vec2{-0.5f, 0.0f}, 45.0f, cells);

  EXPECT_FALSE(stationary.right.active);
  EXPECT_FALSE(moving_away.right.active);
}

TEST(RiskEvaluatorTest, ExcludesObstacleInsideCommunicatedFov) {
  RiskEvaluator evaluator;
  const auto result = evaluator.evaluate(forwardPose(), Vec2{0.0f, 0.5f}, 45.0f,
                                         horizontalCluster(0.0f, 0.5f));

  EXPECT_FALSE(result.left.active);
  EXPECT_FALSE(result.right.active);
  EXPECT_FALSE(result.rear.active);
}

TEST(RiskEvaluatorTest, SignalsRearWhenMovingBackward) {
  RiskEvaluator evaluator;
  const auto result = evaluator.evaluate(forwardPose(), Vec2{0.0f, -0.5f},
                                         45.0f, horizontalCluster(0.0f, -0.5f));

  EXPECT_TRUE(result.rear.active);
  EXPECT_FALSE(result.left.active);
  EXPECT_FALSE(result.right.active);
}

TEST(RiskEvaluatorTest, SignalsLeftWhenApproachingLeftObstacle) {
  RiskEvaluator evaluator;
  const auto result = evaluator.evaluate(forwardPose(), Vec2{-0.5f, 0.0f},
                                         45.0f, verticalCluster(-0.5f, 0.0f));

  EXPECT_TRUE(result.left.active);
  EXPECT_FALSE(result.right.active);
  EXPECT_FALSE(result.rear.active);
}

TEST(RiskEvaluatorTest, RejectsIsolatedOccupiedCell) {
  RiskEvaluator evaluator;
  const std::vector<OccupiedCell> cells{
      {{0.5f, 0.0f}, 10, 10},
  };
  const auto result =
      evaluator.evaluate(forwardPose(), Vec2{0.5f, 0.0f}, 45.0f, cells);

  EXPECT_FALSE(result.right.active);
}

TEST(RiskEvaluatorTest, EnforcesDistanceRange) {
  RiskEvaluator evaluator;
  const auto too_close = evaluator.evaluate(forwardPose(), Vec2{0.5f, 0.0f},
                                            45.0f, verticalCluster(0.1f, 0.0f));
  const auto too_far = evaluator.evaluate(forwardPose(), Vec2{1.0f, 0.0f},
                                          45.0f, verticalCluster(1.3f, 0.0f));

  EXPECT_FALSE(too_close.right.active);
  EXPECT_FALSE(too_far.right.active);
}

TEST(RiskEvaluatorTest, HigherClosingSpeedProducesHigherIntensity) {
  RiskEvaluator evaluator;
  const auto cells = verticalCluster(0.5f, 0.0f);
  const auto slow =
      evaluator.evaluate(forwardPose(), Vec2{0.3f, 0.0f}, 45.0f, cells);
  const auto fast =
      evaluator.evaluate(forwardPose(), Vec2{0.8f, 0.0f}, 45.0f, cells);

  ASSERT_TRUE(slow.right.active);
  ASSERT_TRUE(fast.right.active);
  EXPECT_GT(fast.right.intensity, slow.right.intensity);
}

TEST(RiskEvaluatorTest, DistanceAndTtcEndAtBodyCircumference) {
  RiskEvaluator::Config config;
  config.min_cluster_cells = 1;
  RiskEvaluator evaluator(config);

  const std::vector<OccupiedCell> cells{{{0.5f, 0.0f}, 10, 10}};
  const auto result =
      evaluator.evaluate(forwardPose(), Vec2{0.5f, 0.0f}, 45.0f, cells);

  ASSERT_TRUE(result.right.active);
  EXPECT_NEAR(result.right.distance_m, 0.30f, 1e-5f);
  EXPECT_NEAR(result.right.closing_speed_mps, 0.50f, 1e-5f);
  EXPECT_NEAR(result.right.time_to_collision_s, 0.60f, 1e-5f);
}

TEST(RiskEvaluatorTest, DistanceRangeUsesClearanceToBody) {
  RiskEvaluator::Config config;
  config.min_cluster_cells = 1;
  config.min_distance_m = 0.20f;
  RiskEvaluator evaluator(config);

  // Its centre is 0.30 m away, but its clearance to a 0.20 m body is 0.10 m.
  const std::vector<OccupiedCell> cells{{{0.30f, 0.0f}, 10, 10}};
  const auto result =
      evaluator.evaluate(forwardPose(), Vec2{0.5f, 0.0f}, 45.0f, cells);

  EXPECT_FALSE(result.right.active);
}

TEST(RiskEvaluatorTest, CorridorRejectsTangentialObstacle) {
  RiskEvaluator evaluator;
  const auto result = evaluator.evaluate(forwardPose(), Vec2{0.0f, -0.5f},
                                         45.0f, verticalCluster(0.8f, -0.5f));

  EXPECT_FALSE(result.rear.active);
  EXPECT_FALSE(result.right.active);
}

TEST(RiskEvaluatorTest, ClosingSpeedHysteresisAvoidsThresholdChatter) {
  RiskEvaluator::Config config;
  config.min_closing_speed_mps = 0.50f;
  config.closing_speed_hysteresis_mps = 0.10f;
  RiskEvaluator evaluator(config);
  const auto cells = verticalCluster(0.5f, 0.0f);

  const auto inactive =
      evaluator.evaluate(forwardPose(), Vec2{0.45f, 0.0f}, 45.0f, cells);
  const auto retained = evaluator.evaluate(
      forwardPose(), Vec2{0.45f, 0.0f}, 45.0f, cells,
      spatial_awareness::ActiveDirections{false, true, false});

  EXPECT_FALSE(inactive.right.active);
  EXPECT_TRUE(retained.right.active);
}

}  // namespace

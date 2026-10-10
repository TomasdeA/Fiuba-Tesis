#include <gtest/gtest.h>
#include "local_mapper/timed_pose_buffer.hpp"
using Buffer = local_mapper::TimedPoseBuffer;
using Status = Buffer::Status;

TEST(TimedPoseBuffer, EmptyAndFutureWaitInsteadOfExtrapolating) {
  Buffer b; Buffer::Pose p{};
  EXPECT_EQ(b.lookup(100,200,p),Status::Pending);
  b.insert({100,1,2,.1});
  EXPECT_EQ(b.lookup(101,200,p),Status::Pending);
}
TEST(TimedPoseBuffer, ExactStampPreservesPose) {
  Buffer b; Buffer::Pose p{}; b.insert({100,1,2,.3});
  ASSERT_EQ(b.lookup(100,1,p),Status::Ready);
  EXPECT_EQ(p.stamp,100); EXPECT_DOUBLE_EQ(p.x,1); EXPECT_DOUBLE_EQ(p.z,2); EXPECT_DOUBLE_EQ(p.yaw,.3);
}
TEST(TimedPoseBuffer, InterpolatesPositionAndYaw) {
  Buffer b; Buffer::Pose p{}; b.insert({100,0,2,0});b.insert({200,4,6,1});
  ASSERT_EQ(b.lookup(125,100,p),Status::Ready);
  EXPECT_EQ(p.stamp,125); EXPECT_DOUBLE_EQ(p.x,1); EXPECT_DOUBLE_EQ(p.z,3); EXPECT_DOUBLE_EQ(p.yaw,.25);
}
TEST(TimedPoseBuffer, YawUsesShortestArcAcrossPi) {
  Buffer b; Buffer::Pose p{}; const double pi=std::acos(-1.);
  b.insert({100,0,0,179*pi/180});b.insert({200,0,0,-179*pi/180});
  ASSERT_EQ(b.lookup(150,100,p),Status::Ready);
  EXPECT_NEAR(std::abs(p.yaw),pi,1e-12);
}
TEST(TimedPoseBuffer, RejectsLargeGapButAcceptsExactEndpoints) {
  Buffer b; Buffer::Pose p{};b.insert({100,0,0,0});b.insert({500,1,1,1});
  EXPECT_EQ(b.lookup(200,200,p),Status::Gap);
  EXPECT_EQ(b.lookup(500,200,p),Status::Ready);
}
TEST(TimedPoseBuffer, DoesNotUseFuturePoseForStartup) {
  Buffer b; Buffer::Pose p{};b.insert({200,0,0,0});
  EXPECT_EQ(b.lookup(100,200,p),Status::TooOld);
}
TEST(TimedPoseBuffer, SortsAndReplacesDuplicates) {
  Buffer b; Buffer::Pose p{};b.insert({200,2,0,0});b.insert({100,1,0,0});b.insert({200,4,0,0});
  ASSERT_EQ(b.lookup(150,100,p),Status::Ready);EXPECT_DOUBLE_EQ(p.x,2.5);
}
TEST(TimedPoseBuffer, BoundsHistory) {
  Buffer b(2);Buffer::Pose p{};b.insert({100,1,0,0});b.insert({200,2,0,0});b.insert({300,3,0,0});
  EXPECT_EQ(b.lookup(100,200,p),Status::TooOld);
  EXPECT_EQ(b.lookup(250,200,p),Status::Ready);
}

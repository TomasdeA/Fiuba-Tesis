#include <gtest/gtest.h>
#include "depth_obstacle_filter/depth_projector.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <cmath>

/// Helper: crea un CameraInfo con intrínsecos típicos (fx=fy=600, cx=320, cy=240).
static sensor_msgs::msg::CameraInfo makeCameraInfo() {
  sensor_msgs::msg::CameraInfo info;
  info.k[0] = 600.0;  // fx
  info.k[2] = 320.0;  // cx
  info.k[4] = 600.0;  // fy
  info.k[5] = 240.0;  // cy
  return info;
}

TEST(DepthProjector, ProjectPixelAtCenter) {
  depth_obstacle_filter::DepthProjector proj(makeCameraInfo());

  // Píxel central a 1 m → (0, 0, 1)
  auto pt = proj.projectPixel(320, 240, 1000, 1e-3f);
  EXPECT_NEAR(pt.x, 0.0f, 0.01f);
  EXPECT_NEAR(pt.y, 0.0f, 0.01f);
  EXPECT_NEAR(pt.z, 1.0f, 0.01f);
}

TEST(DepthProjector, ProjectPixelAtCorner) {
  depth_obstacle_filter::DepthProjector proj(makeCameraInfo());

  // Píxel desplazado 600 px a la derecha del centro, a 1 m → x ≈ 1.0
  auto pt = proj.projectPixel(920, 240, 1000, 1e-3f);
  EXPECT_NEAR(pt.x, 1.0f, 0.01f);
  EXPECT_NEAR(pt.y, 0.0f, 0.01f);
  EXPECT_NEAR(pt.z, 1.0f, 0.01f);
}

TEST(DepthProjector, ProjectImageHandlesInvalidPixels) {
  depth_obstacle_filter::DepthProjector proj(makeCameraInfo());

  uint16_t depth_data[4 * 3] = {
      1000, 1000, 2000, 0,
      1000, 0,    1000, 1000,
      0,    2000, 1000, 1000
  };

  auto points = proj.projectDepthImage(depth_data, 4, 3, 1e-3f);
  EXPECT_EQ(points.size(), 12u);
  EXPECT_NEAR(points[0].z, 1.0f, 0.01f);
  EXPECT_NEAR(points[2].z, 2.0f, 0.01f);
  EXPECT_TRUE(std::isnan(points[3].z));  // píxel inválido (depth=0) -> NaN
}

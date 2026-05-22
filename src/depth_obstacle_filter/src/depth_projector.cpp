#include "depth_obstacle_filter/depth_projector.hpp"
#include <algorithm>
#include <limits>

namespace depth_obstacle_filter {

DepthProjector::DepthProjector(const sensor_msgs::msg::CameraInfo& camera_info)
    : fx_(camera_info.k[0]),
      fy_(camera_info.k[4]),
      cx_(camera_info.k[2]),
      cy_(camera_info.k[5]) {}

DepthProjector::Point3D DepthProjector::projectPixel(
    int u, int v, uint16_t depth_mm, float depth_scale) const {
  float z = static_cast<float>(depth_mm) * depth_scale;
  float x = (u - cx_) * z / fx_;
  float y = (v - cy_) * z / fy_;
  return {x, y, z};
}

std::vector<DepthProjector::Point3D> DepthProjector::projectDepthImage(
    const uint16_t* depth_data,
    int width,
    int height,
    float depth_scale,
    int pixel_stride) const {
  const int stride = std::max(1, pixel_stride);
  std::vector<Point3D> points;
  const int sampled_width = (width + stride - 1) / stride;
  const int sampled_height = (height + stride - 1) / stride;
  points.reserve(sampled_width * sampled_height);

  for (int v = 0; v < height; v += stride) {
    for (int u = 0; u < width; u += stride) {
      uint16_t depth_mm = depth_data[v * width + u];
      if (depth_mm == 0 || depth_mm == 65535) {
        constexpr float nan = std::numeric_limits<float>::quiet_NaN();
        points.push_back({nan, nan, nan});
        continue;
      }
      points.push_back(projectPixel(u, v, depth_mm, depth_scale));
    }
  }

  return points;
}

}  // namespace depth_obstacle_filter

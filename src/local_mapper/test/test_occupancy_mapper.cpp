#include <gtest/gtest.h>

#include "local_mapper/occupancy_mapper.hpp"

#include <cstddef>
#include <vector>

namespace {

using Mapper = local_mapper::OccupancyMapper;

std::size_t cellIndex(int ci, int cj, int grid_size) {
  return static_cast<std::size_t>(ci * grid_size + cj);
}

Mapper::Config testConfig() {
  Mapper::Config cfg;
  cfg.cell_size_m = 1.0f;
  cfg.grid_size = 10;
  cfg.l_occ = 1.0f;
  cfg.l_free = -0.25f;
  cfg.l_min = -5.0f;
  cfg.l_max = 5.0f;
  cfg.max_range_m = 10.0f;
  cfg.forget_radius_m = 0.0f;
  cfg.enable_raycasting = true;
  return cfg;
}

TEST(OccupancyMapper, UpdatesOccupiedCellOnlyOncePerFrame) {
  Mapper mapper(testConfig());

  // Los tres puntos caen en la celda (7, 5). La cámara está en (5, 5).
  const std::vector<Mapper::Point2D> obstacles{
      {2.10f, 0.10f}, {2.20f, 0.20f}, {2.90f, 0.90f}};
  mapper.update(obstacles, 0.0f, 0.0f);

  EXPECT_FLOAT_EQ(mapper.logOdds()[cellIndex(7, 5, 10)], 1.0f);
}

TEST(OccupancyMapper, SameOccupiedCellCanAccumulateAcrossFrames) {
  Mapper mapper(testConfig());
  const std::vector<Mapper::Point2D> obstacles{
      {2.10f, 0.10f}, {2.20f, 0.20f}};

  mapper.update(obstacles, 0.0f, 0.0f);
  mapper.update(obstacles, 0.0f, 0.0f);

  EXPECT_FLOAT_EQ(mapper.logOdds()[cellIndex(7, 5, 10)], 2.0f);
}

TEST(OccupancyMapper, RayStillMarksIntermediateCellsFree) {
  Mapper mapper(testConfig());
  const std::vector<Mapper::Point2D> obstacles{{2.10f, 0.10f}};

  mapper.update(obstacles, 0.0f, 0.0f);

  const auto& grid = mapper.logOdds();
  EXPECT_FLOAT_EQ(grid[cellIndex(5, 5, 10)], -0.25f);
  EXPECT_FLOAT_EQ(grid[cellIndex(6, 5, 10)], -0.25f);
  EXPECT_FLOAT_EQ(grid[cellIndex(7, 5, 10)], 1.0f);
}

}  // namespace

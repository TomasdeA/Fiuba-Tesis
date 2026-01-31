#pragma once
#include <sensor_msgs/msg/image.hpp>
#include <cstddef>
#include <limits> 
#include <cassert>

#include "tesis_nav_interfaces/msg/depth_grid.hpp"
#include "tesis_nav_interfaces/msg/depth_cell_stats.hpp"

namespace tesis_nav {
    
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// Config for depth filter and output dimmensions
struct GridConfig {
    int rows{1};            // number of rows in output grid
    int cols{10};           // number of cols in output grid
    float z_min_m{0.2f};    // filter - min value
    float z_max_m{5.0f};    // filter - max value
    //std::string reference{"min"};
};

/****************  GRID WRAPPER (HELPERS) ***************************
 * `DepthGrid.cells` is stored as a flat 1D vector of cell statistics.
 *
 * ROS2-generated messages provide data only (no indexing helpers),
 * so these utilities provide:
 *
 *   - clean 2D-style access to (row, col)
 *   - safe bounds checking via `assert`
 *   - allocation helpers for rows × cols
 *
 * This keeps the API ergonomic without defining custom structs
 * or duplicating the message types.
 *******************************************************************/

// Safe access to a grid cell (mutable)
inline tesis_nav_interfaces::msg::DepthCellStats&
at(tesis_nav_interfaces::msg::DepthGrid & grid, size_t r, size_t c)
{
    assert(r < grid.rows && c < grid.cols);
    return grid.cells[r * grid.cols + c];
}

// Safe access to a grid cell (const)
inline const tesis_nav_interfaces::msg::DepthCellStats&
at(const tesis_nav_interfaces::msg::DepthGrid & grid, size_t r, size_t c)
{
    assert(r < grid.rows && c < grid.cols);
    return grid.cells[r * grid.cols + c];
}

// Allocate and size the DepthGrid message.
// Ensures that `cells` contains rows × cols entries.
inline void allocate(tesis_nav_interfaces::msg::DepthGrid & grid,
                     uint32_t rows, uint32_t cols)
{
    grid.rows = rows;
    grid.cols = cols;
    grid.cells.resize(rows * cols);
}

/* END GRID WRAPPER */

// Convert a depth image into a row × col grid of cell statistics.
bool compute_depth_stats(   const sensor_msgs::msg::Image& msg,
                            const GridConfig& cfg,
                            tesis_nav_interfaces::msg::DepthGrid& out);

} // namespace tesis_nav
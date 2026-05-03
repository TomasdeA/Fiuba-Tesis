#include "depth_grid_encoder/depth_utils.hpp"

namespace tesis_nav {

using custom_interfaces::msg::DepthGrid;
using custom_interfaces::msg::DepthCellStats;

// Vista simple sobre una imagen de profundidad 16UC1 (valores en metros)
struct DepthView16UC1 {

    int width{0};
    int height{0};
    size_t stride{0};            // elementos por fila
    const uint16_t* data{nullptr};
    size_t n_elems{0};

    // Leer píxel (x,y) como z en metros; filtra raw==0 (sin medición)
    bool sample(int x, int y, float& z_m) const {
        if (x < 0 || y < 0 || x >= width || y >= height) return false;
        const size_t idx = static_cast<size_t>(y) * stride + static_cast<size_t>(x);
        if (idx >= n_elems) return false;
        const uint16_t raw = data[idx];
        if (raw == 0) return false; // sin medición
        z_m = raw * 0.001f;         // mm -> m
        return true;
    }
};


static bool make_view_16UC1(const sensor_msgs::msg::Image& msg,
                            DepthView16UC1& view)
{
    if (msg.encoding != "16UC1") return false;
    if (msg.width == 0 || msg.height == 0) return false;
    if (msg.step % sizeof(uint16_t) != 0) return false;

    view.width   = static_cast<int>(msg.width);
    view.height  = static_cast<int>(msg.height);
    view.stride  = msg.step / sizeof(uint16_t);
    // Tratar los bytes de imagen raw como un array de píxeles de profundidad de 16 bits (16UC1).
    view.data = reinterpret_cast<const uint16_t*>(msg.data.data());
    view.n_elems = msg.data.size() / sizeof(uint16_t);

    if (view.stride * static_cast<size_t>(view.height) > view.n_elems) return false;
    return true;
}


// Acumula estadísticas para ROI [x0,x1) × [y0,y1)
static void accumulate_roi(const DepthView16UC1& view,
                           int x0, int x1, int y0, int y1,
                           float z_min_m, float z_max_m,
                           DepthCellStats& out)
{
    double sum = 0.0;
    int cnt = 0;
    float vmin = std::numeric_limits<float>::infinity();
    float vmax = -std::numeric_limits<float>::infinity();

    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            float z;
            if (!view.sample(x, y, z)) continue;
            if (z < z_min_m || z > z_max_m) continue;

            sum += z;
            ++cnt;
            if (z < vmin) vmin = z;
            if (z > vmax) vmax = z;
        }
    }

    out.count = cnt;
    if (cnt > 0) {
        out.mean_m = static_cast<float>(sum / cnt);
        out.min_m  = vmin;
        out.max_m  = vmax;
    } else {
        out.mean_m = kNaN;
        out.min_m  = kNaN;
        out.max_m  = kNaN;
    }
}

// Imagen de profundidad --> grilla rows × cols de estadísticas
bool compute_depth_stats(const sensor_msgs::msg::Image& msg,
                         const GridConfig& cfg,
                         DepthGrid& out)
{

    DepthView16UC1 view;

    if (!make_view_16UC1(msg, view)) {
        out = DepthGrid{};   // limpiar salida
        return false;
    }
    // view ahora contiene todos los datos de la imagen de una forma más fácil de acceder.

    const int rows = cfg.rows;
    const int cols = cfg.cols;

    if (rows <= 0 || cols <= 0) {
        out = DepthGrid{};
        return false;
    }
    // Calcular cuántos píxeles tendrá cada ROI en x e y.
    std::vector<int> x_edges(cols + 1); //[0,x1,x2, ...] => ROI_1_x = [0,x1) , ROI_2_x = [x1,x2) , ...
    std::vector<int> y_edges(rows + 1); //igual
    // Esto generará una diferencia de como máximo 1 píxel entre ROIs.
    for (int c = 0; c<=cols; ++c) {
        x_edges[c] = c * view.width / cols;
    }
    for (int r = 0; r<=rows; ++r) {
        y_edges[r] = r * view.height / rows;
    }

    out.rows = rows;
    out.cols = cols;
    out.cells.assign(static_cast<size_t>(rows) * cols , DepthCellStats{}); // NaN + count=0

    for (int r = 0; r < rows; ++r) {
        const int y0 = y_edges[r];
        const int y1 = y_edges[r + 1];

        for (int c = 0; c < cols; ++c) {
            const int x0 = x_edges[c];
            const int x1 = x_edges[c+1];

            DepthCellStats& cell = at(out,r,c);
            accumulate_roi(view, x0, x1, y0, y1,
                           cfg.z_min_m, cfg.z_max_m,
                           cell);
        }
    }

    return true;
}

} // namespace tesis_nav

#include "tesis_nav_mapper_cpp/depth_utils.hpp"
#include <sensor_msgs/msg/image.hpp>

namespace tesis_nav {

bool compute_16UC1( const sensor_msgs::msg::Image& msg,
                    const RoiConfig& cfg,
                    DepthStats& out)
{

    if (msg.encoding != "16UC1") return false;
    if (msg.width == 0 || msg.height == 0) return false;
    if (msg.step % sizeof(uint16_t) != 0) return false;

    const auto w = msg.width;
    const auto h = msg.height;

    const size_t stride = msg.step / sizeof(uint16_t); // elements per row
    const size_t n_elems = msg.data.size() / sizeof(uint16_t);  // elements in buffer

    if (stride * h > n_elems) return false;
    
    const auto* p = reinterpret_cast<const uint16_t*>(msg.data.data()); // pointer to raw image bytes
    
    const int cx = static_cast<int>(w) / 2; // floor(w/2)
    const int cy = static_cast<int>(h) / 2; // floor(h/2)
    
    out = DepthStats{}; //NaN en todos los floats, cnt=0
    
    const size_t idxc = static_cast<size_t>(cy) * stride + cx; // center idx

    if(idxc < n_elems){
        // idxc inside of the buffer, safe to read. stride * h is the total number of elements
        const uint16_t raw = p[idxc];
        if(raw > 0) out.center_m =static_cast<float>(raw / 1000.0f); 
    }
    //else -> center_m = NaN (soft-fail)

    const int halfL = cfg.roi_px / 2;
    const int halfR = cfg.roi_px - halfL;
    // In case roi_px is greater than w or h
    const int x0 = std::max(0,  static_cast<int>(cx) - halfL);
    const int x1 = std::min<int>(w, static_cast<int>(cx) + halfR);
    const int y0 = std::max(0,  static_cast<int>(cy) - halfL);
    const int y1 = std::min<int>(h, static_cast<int>(cy) + halfR);

    double sum = 0.0;
    int cnt = 0;
    float vmin = std::numeric_limits<float>::infinity();
    float vmax = -std::numeric_limits<float>::infinity();

    for( int y = y0; y < y1; ++y) {
        const size_t base = static_cast<size_t>(y) * stride;
        for(int x = x0; x < x1; ++x) {
            //idx = y * stride + x
            const size_t k = base + x;
            if(k >= n_elems) continue; //extra check
            const uint16_t raw = p[k];
            if(raw == 0) continue; //invalid (no measurement)
            const float m=raw* 0.001f; //mm->m

            if(m < cfg.z_min_m || m > cfg.z_max_m) continue;

            sum += m;
            ++cnt;
            if(m < vmin) vmin = m;
            if(m > vmax) vmax = m;
        }
    }

    out.roi_count = cnt;
    if (cnt > 0){
        out.roi_mean_m = static_cast<float>(sum / cnt);
        out.roi_min_m = vmin;
        out.roi_max_m = vmax;
    }
    else {
        out.roi_mean_m =kNaN;
        out.roi_min_m =kNaN;
        out.roi_max_m =kNaN;
    }
    return true;
}
/*
bool compute_32FC1( const sensor_msgs::msg::Image& msg,
                    const RoiConfig& cfg,
                    DepthStats& out){
    return false;
}
*/
bool compute_depth_stats(   const sensor_msgs::msg::Image& msg,
                            const GridConfig& cfg,
                            DepthGrid& out)
{
    if (msg.encoding == "16UC1") return compute_16UC1(msg, cfg, out);
    //if (msg.encoding == "32FC1") return compute_32FC1(msg, cfg, out);
    out = DepthGrid{};
    return false; // encoding no soportado
}

}
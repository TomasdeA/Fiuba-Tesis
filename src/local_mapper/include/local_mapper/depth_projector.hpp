#pragma once

#include <vector>
#include <cstdint>
#include <sensor_msgs/msg/camera_info.hpp>

namespace local_mapper {

/**
 * Proyector pinhole de profundidad a 3D.
 *
 * Convierte imágenes de profundidad de 16 bits (milímetros) a nubes de puntos
 * 3D usando los parámetros intrínsecos de la cámara (modelo pinhole).
 *
 * Sistema de coordenadas (camera_depth_optical_frame):
 *   X = derecha,  Y = abajo,  Z = adelante (hacia la escena)
 *
 * Fórmulas de retroproyección:
 *   z = depth_mm * depth_scale
 *   x = (u - cx) * z / fx
 *   y = (v - cy) * z / fy
 */
class DepthProjector {
 public:
  /// Punto 3D en coordenadas de cámara.
  struct Point3D {
    float x, y, z;
  };

  /**
   * Construye el proyector a partir de CameraInfo de ROS.
   * Extrae fx, fy, cx, cy de la matriz K (3×3).
   */
  explicit DepthProjector(const sensor_msgs::msg::CameraInfo& camera_info);

  /**
   * Proyecta una imagen de profundidad completa a puntos 3D.
   *
   * @param depth_data  Puntero a datos de profundidad 16-bit (mm)
   * @param width       Ancho de la imagen en píxeles
   * @param height      Alto de la imagen en píxeles
   * @param depth_scale Factor de conversión mm → metros (default: 1e-3)
   * @return Vector de puntos 3D. Píxeles inválidos (0 o 65535) se marcan
   *         como (0, 0, 0).
   */
  [[nodiscard]] std::vector<Point3D> projectDepthImage(
      const uint16_t* depth_data,
      int width,
      int height,
      float depth_scale = 1e-3f) const;

  /**
   * Proyecta un solo píxel a 3D.
   *
   * @param u, v       Coordenadas del píxel
   * @param depth_mm   Profundidad en milímetros
   * @param depth_scale Factor de conversión (default: 1e-3)
   * @return Punto 3D en coordenadas de cámara
   */
  [[nodiscard]] Point3D projectPixel(
      int u, int v,
      uint16_t depth_mm,
      float depth_scale = 1e-3f) const;

  // Getters de los intrínsecos
  [[nodiscard]] float fx() const { return fx_; }
  [[nodiscard]] float fy() const { return fy_; }
  [[nodiscard]] float cx() const { return cx_; }
  [[nodiscard]] float cy() const { return cy_; }

 private:
  float fx_, fy_, cx_, cy_;
};

}  // namespace local_mapper

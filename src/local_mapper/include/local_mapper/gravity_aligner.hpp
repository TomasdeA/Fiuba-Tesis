#pragma once

#include <array>
#include <cmath>

namespace local_mapper {

/**
 * Alinea una nube de puntos a la gravedad usando aceleración del IMU.
 *
 * Estima roll y pitch a partir de los datos del acelerómetro (asumiendo
 * condición cuasi-estática) y calcula el cuaternión de rotación mínima que
 * lleva el vector gravedad medido al eje "abajo" canónico del frame óptico.
 *
 * Sistema de coordenadas de entrada: camera_depth_optical_frame
 *   X = derecha, Y = abajo, Z = adelante
 * Cuando la cámara está perfectamente horizontal el acelerómetro mide
 * (0, -g, 0).  La rotación calculada lleva cualquier vector de gravedad
 * medido a coincidir con (0, -1, 0), alineando la nube con el plano del
 * mundo.
 *
 * No depende de ROS: se puede construir y testar sin runtime de ROS.
 * El filtrado del acelerómetro es responsabilidad de ImuFilter.
 */
class GravityAligner {
 public:
  struct Quaternion {
    float w, x, y, z;  ///< Parte escalar primero (convención Hamilton)

    /// Rota un punto usando este cuaternión (asumiendo unitario).
     std::array<float, 3> rotatePoint(
        float px, float py, float pz) const;
  };

  GravityAligner() = default;

  /**
   * Calcula el cuaternión de rotación que alinea el vector de gravedad medido
   * con el eje Y negativo del frame óptico: (0, -1, 0).
   *
   * @param ax, ay, az  Aceleración filtrada en m/s² (frame óptico del sensor)
   * @return Cuaternión de rotación unitario. Retorna identidad si la norma
   *         de la entrada es demasiado pequeña para normalizar.
   */
   Quaternion estimateOrientation(
      float ax, float ay, float az) const;

  /**
   * Retroproyecta un punto 3D y aplica la corrección gravitacional en un
   * solo paso.
   *
   * @param px, py, pz  Punto en el frame del sensor (sin alinear)
   * @param ax, ay, az  Lectura del acelerómetro (m/s²)
   * @return Punto en el frame alineado a gravedad
   */
   std::array<float, 3> alignToGravity(
      float px, float py, float pz,
      float ax, float ay, float az) const;

};

}  // namespace local_mapper

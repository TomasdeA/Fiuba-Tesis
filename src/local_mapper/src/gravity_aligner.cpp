#include "local_mapper/gravity_aligner.hpp"
#include <cmath>

namespace local_mapper {

// ── Quaternion::rotatePoint ───────────────────────────────────────────────────
// Rotación de un vector por un cuaternión unitario: v' = q * v * q*
// donde q* es el conjugado de q.

std::array<float, 3> GravityAligner::Quaternion::rotatePoint(
    float px, float py, float pz) const
{
  const float qw = w, qx = x, qy = y, qz = z;

  // Producto q * p (p tratado como cuaternión puro: parte escalar = 0)
  const float qpw = -qx*px - qy*py - qz*pz;
  const float qpx =  qw*px + qy*pz - qz*py;
  const float qpy =  qw*py + qz*px - qx*pz;
  const float qpz =  qw*pz + qx*py - qy*px;

  // Producto (q*p) * q*  (q* = {w, -x, -y, -z})
  // v' = q * v * q*  →  result = (q*v) * q*
  // Usando la fórmula de multiplicación de cuaterniones con q2 = (qw,-qx,-qy,-qz):
  //   result.x = q1.w*(-qx) + q1.x*qw  + q1.y*(-qz) - q1.z*(-qy)
  const float rpx = -qpw*qx + qpx*qw - qpy*qz + qpz*qy;
  const float rpy = -qpw*qy + qpy*qw + qpx*qz - qpz*qx;
  const float rpz = -qpw*qz + qpz*qw - qpx*qy + qpy*qx;

  return {rpx, rpy, rpz};
}

// ── GravityAligner::estimateOrientation ──────────────────────────────────────
// Calcula el cuaternión de rotación mínima que lleva el vector aceleración
// normalizado al vector "abajo" canónico (0, -1, 0) del frame óptico.
//
// Fórmula: dada la rotación de un vector 'a' a un vector 'b',
//   q = (1 + a·b,  a×b)   luego normalizado.
//

GravityAligner::Quaternion GravityAligner::estimateOrientation(
    float ax, float ay, float az) const
{
  const float norm = std::sqrt(ax*ax + ay*ay + az*az);
  if (norm < 1e-6f) {
    return {1.0f, 0.0f, 0.0f, 0.0f};  // identidad
  }
  ax /= norm;
  ay /= norm;
  az /= norm;

  // Vector "abajo" target en camera_depth_optical_frame: b = (0, -1, 0)
  // (el acelerómetro de la D435i mide -g en Y cuando está horizontal)
  //
  // a·b  = ax·0 + ay·(-1) + az·0 = -ay
  //
  // a×b  = (ay·bz - az·by,  az·bx - ax·bz,  ax·by - ay·bx)
  //      = (ay·0 - az·(-1), az·0 - ax·0,    ax·(-1) - ay·0)
  //      = (az,              0,              -ax)
  const float dot = -ay;
  const float cx  =  az;
  const float cy  =  0.0f;
  const float cz  = -ax;

  // Caso degenerado: vectores antiparalelos (ay ≈ +g, cámara completamente invertida).
  if (dot < -0.999f) {
    return {0.0f, 1.0f, 0.0f, 0.0f};
  }

  Quaternion q;
  q.w = 1.0f + dot;
  q.x = cx;
  q.y = cy;
  q.z = cz;

  const float qn = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (qn < 1e-8f) {
    return {1.0f, 0.0f, 0.0f, 0.0f};
  }
  q.w /= qn;
  q.x /= qn;
  q.y /= qn;
  q.z /= qn;

  return q;
}

// ── GravityAligner::alignToGravity ───────────────────────────────────────────

std::array<float, 3> GravityAligner::alignToGravity(
    float px, float py, float pz,
    float ax, float ay, float az) const
{
  const Quaternion q = estimateOrientation(ax, ay, az);
  return q.rotatePoint(px, py, pz);
}

}  // namespace local_mapper

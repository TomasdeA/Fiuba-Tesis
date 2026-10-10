"""Conversiones entre las convenciones de odometría soportadas."""

import math


def rtabmap_position(ros_x, ros_y):
    """Convierte una posición del plano ROS XY al plano interno XZ."""
    return -ros_y, ros_x


def _multiply_quaternions(lhs, rhs):
    """Multiplica quaternions expresados como tuplas (w, x, y, z)."""
    lw, lx, ly, lz = lhs
    rw, rx, ry, rz = rhs
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def rtabmap_yaw_quaternion(orientation):
    """Convierte la orientación ROS de RTAB-Map al yaw del plano XZ interno."""
    q_msg = (
        orientation.w,
        orientation.x,
        orientation.y,
        orientation.z,
    )
    norm = math.sqrt(sum(component * component for component in q_msg))
    if norm <= 1e-12:
        q_msg = (1.0, 0.0, 0.0, 0.0)
    else:
        q_msg = tuple(component / norm for component in q_msg)

    # Debe coincidir con q_ros_to_internal de occupancy_mapper_node.cpp.
    q_internal = _multiply_quaternions(
        (0.5, 0.5, -0.5, 0.5),
        q_msg,
    )

    # Rotar el vector frontal (0, 0, 1) y conservar solamente su yaw.
    w, x, y, z = q_internal
    forward_x = 2.0 * (x * z + w * y)
    forward_z = 1.0 - 2.0 * (x * x + y * y)
    horizontal_norm = math.hypot(forward_x, forward_z)
    if horizontal_norm <= 1e-4:
        return (1.0, 0.0, 0.0, 0.0)

    yaw = math.atan2(forward_x, forward_z)
    return (math.cos(yaw * 0.5), 0.0, math.sin(yaw * 0.5), 0.0)

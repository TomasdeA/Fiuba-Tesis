# sensor_topic_remapper

Normalizador simple de topics de sensores a espacio de nombres estandarizado.

## Descripción

Remapea topics crudos de RealSense D435i (y otros sensores) a un espacio de nombres limpio `/sensors/*`. Utiliza `SensorDataQoS` para transporte confiable de mejor esfuerzo optimizado para datos de sensores.

## Mapeos

| Origen | Destino | Tipo |
|--------|---------|------|
| `/camera/camera/depth/image_rect_raw` | `/sensors/depth/image` | sensor_msgs/Image |
| `/camera/camera/depth/camera_info` | `/sensors/depth/camera_info` | sensor_msgs/CameraInfo |
| `/camera/camera/accel/sample` | `/sensors/imu/accel` | sensor_msgs/Imu |
| `/camera/camera/gyro/sample` | `/sensors/imu/gyro` | sensor_msgs/Imu |

Todos los mapeos son configurables a través de parámetros de lanzamiento.

## Política de QoS

Utiliza `rclcpp::SensorDataQoS()`:
- Historial: Mantener Último (tamaño=5)
- Durabilidad: Volátil
- Confiabilidad: Mejor Esfuerzo
- Optimizado para datos de sensores de alta frecuencia

## Lanzamiento

```bash
ros2 launch sensor_topic_remapper sensor_topic_remapper.launch.py
```

### Topics de Origen Personalizados

```bash
ros2 launch sensor_topic_remapper sensor_topic_remapper.launch.py \
  depth_image_src:=/mi_camara/profundidad \
  depth_camera_info_src:=/mi_camara/info \
  accel_src:=/mi_imu/accel \
  gyro_src:=/mi_imu/gyro
```
## Testing

```bash
# Verificar topics remapeados
ros2 topic list | grep /sensors/
ros2 topic echo /sensors/depth/image
```

# local_mapper

Mapeador local de grilla de ocupación basado en imágenes de profundidad, IMU y odometría visual.

## Arquitectura

### Componentes actuales

1. **DepthProjector** — Convierte imágenes de profundidad 16-bit (mm) a nubes
   de puntos 3D usando el modelo pinhole de la cámara.

### Nodos

- **depth_projection_node** — Suscribe imagen de profundidad + CameraInfo + IMU,
  retroproyecta a 3D, publica la nube cruda y la nube alineada a gravedad,
  y emite la TF `gravity_aligned_frame → camera_depth_optical_frame`.

### Topics

| Dirección | Topic | Tipo | Descripción |
|-----------|-------|------|-------------|
| Sub | `depth/image` | `sensor_msgs/Image` | Imagen de profundidad 16-bit |
| Sub | `depth/camera_info` | `sensor_msgs/CameraInfo` | Intrínsecos de la cámara |
| Sub | `imu` | `sensor_msgs/Imu` | Acelerómetro + giroscopio fusionados |
| Pub | `/local_mapper/debug/depth_cloud` | `sensor_msgs/PointCloud2` | Nube cruda (frame `gravity_aligned_frame`, puntos sin rotar) |
| Pub | `/local_mapper/debug/aligned_cloud` | `sensor_msgs/PointCloud2` | Nube alineada a gravedad (frame `camera_depth_optical_frame`, corregida vía TF) |

### TF publicadas

| Parent | Child | Descripción |
|--------|-------|-------------|
| `gravity_aligned_frame` | `camera_depth_optical_frame` | Cuaternión `q` que corrige roll/pitch de la cámara. Se actualiza a la frecuencia del IMU. |

### Consumir la nube alineada en otro nodo

La nube `aligned_cloud` se publica en `camera_depth_optical_frame` con puntos
crudos. La corrección gravitacional está codificada en la TF. Para obtener
puntos ya corregidos en un nodo consumidor, usar `tf2`:

```cpp
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// En el constructor:
tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

// En el callback de aligned_cloud:
sensor_msgs::msg::PointCloud2 corrected;
tf_buffer_->transform(input_cloud, corrected, "gravity_aligned_frame");
// 'corrected' tiene los puntos con la gravedad alineada a (0,-1,0).
```

> Los topics de suscripción son nombres genéricos. El launch file los remapea
> a los topics reales del hardware configurados en `config/params.yaml`.

## Compilación

```bash
cd ~/Tesis/develop/nav_mapper
colcon build --packages-select local_mapper
```

## Ejecución

```bash
# Terminal 1: Driver RealSense
ros2 launch realsense2_camera rs_launch.py \
  depth_module.depth_profile:=640x480x30

# Terminal 2: Nodo de proyección
ros2 launch local_mapper depth_projection.launch.py

# Terminal 3: RViz
rviz2 -d src/local_mapper/rviz/depth_projection.rviz
```

## Testing

### Tests unitarios (gtest)

```bash
colcon test --packages-select local_mapper
colcon test-result --verbose
```

### Test visual (sin cámara real)

Publica una escena sintética de profundidad para verificar visualmente
en RViz que la retroproyección es geométricamente correcta. No requiere
hardware.

La escena contiene:
- **Pared de fondo** a 2.0 m (todo el frame)
- **Bloque centrado** de 100×100 px a 1.0 m
- **Franja izquierda** con depth=0 (zona inválida, sin puntos)

```bash
# Terminal 1: nodo de proyección
ros2 run local_mapper depth_projection_node

# Terminal 2: publicador de escena sintética
ros2 run local_mapper test_depth_projection_visual.py

# Terminal 3: RViz
rviz2 -d src/local_mapper/rviz/depth_projection.rviz
```

En RViz se debe observar el plano a 2 m y el bloque más cercano a 1 m
con colores distintos (eje Z), y un hueco a la izquierda donde no hay
puntos.

### Test de GravityAligner con TFs (demo de alineación gravitacional)

Publica la TF `gravity_aligned_frame → camera_depth_optical_frame` y dos
nubes de puntos para demostrar visualmente que `GravityAligner` corrige
la inclinación de la cámara:

- **`gravity_aligned_frame`** — frame corregido por el acelerómetro: el eje Y
  apunta siempre hacia abajo, sin importar cómo esté inclinada la cámara.
  Es el Fixed Frame de RViz.
- **`camera_depth_optical_frame`** — frame real de la cámara, hijo de
  `gravity_aligned_frame` con rotación `q`.
- **`/local_mapper/debug/depth_cloud`** — puntos crudos en
  `gravity_aligned_frame` (sin TF). Se inclinan al mover la cámara.
- **`/local_mapper/debug/aligned_cloud`** — puntos en
  `camera_depth_optical_frame`. RViz les aplica la TF `q` y se ven
  siempre estables.

```bash
# Terminal 1: camera con IMU
ros2 launch realsense2_camera rs_launch.py \
  enable_accel:=true enable_gyro:=true unite_imu_method:=2

# Terminal 2: nodo de proyección
source install/setup.bash
ros2 launch local_mapper depth_projection.launch.py

# Terminal 3: RViz
rviz2 -d src/local_mapper/rviz/depth_projection.rviz
```

**Qué observar en RViz:**
- Al inclinar la cámara, `DepthCloud` y los ejes de `CameraFrame` se
  inclinan junto a ella.
- `AlignedCloud` y los ejes de `GravityAlignedFrame` permanecen estables,
  con la grilla del suelo horizontal.
- La diferencia angular entre ambos frames es exactamente la corrección
  calculada por `GravityAligner::estimateOrientation()`.

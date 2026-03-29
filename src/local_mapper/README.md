# local_mapper

Mapeador local de grilla de ocupación basado en imágenes de profundidad, IMU y odometría visual.

## Arquitectura

### Componentes actuales

1. **DepthProjector** — Convierte imágenes de profundidad 16-bit (mm) a nubes
   de puntos 3D usando el modelo pinhole de la cámara.

### Nodos

- **depth_projection_node** — Suscribe imagen de profundidad + CameraInfo,
  retroproyecta a 3D y publica `PointCloud2` para visualización en RViz.

### Topics

| Dirección | Topic (genérico del nodo) | Tipo | Descripción |
|-----------|--------------------------|------|-------------|
| Sub | `depth/image` | `sensor_msgs/Image` | Imagen de profundidad 16-bit |
| Sub | `depth/camera_info` | `sensor_msgs/CameraInfo` | Intrínsecos de la cámara |
| Pub | `/local_mapper/debug/depth_cloud` | `sensor_msgs/PointCloud2` | Nube de puntos 3D |

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

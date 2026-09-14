# nav_mapper

Workspace ROS 2 para la tesis **"Navegación asistida para ciegos mediante mapeo del entorno"** de Ingeniería Electrónica, Facultad de Ingeniería de la Universidad de Buenos Aires (FIUBA).

El repositorio implementa una pipeline de percepción y mapeo local para asistir la navegación con una cámara Intel RealSense D435i. A partir de datos de profundidad e IMU, el sistema genera una representación compacta del entorno cercano, estima obstáculos, mantiene un mapa local de ocupación y produce una grilla háptica/visual pensada para alimentar actuadores o herramientas de análisis.

## Qué incluye

- Entorno de desarrollo reproducible con Docker y ROS 2 Humble.
- Captura de cámara RealSense D435i.
- Encodificación de profundidad en grillas compactas.
- Filtrado de obstáculos con alineación gravitacional y remoción de suelo.
- Estimación de odometría visual-inercial.
- Mapeo local incremental en una grilla de ocupación 2D.
- Estimación de riesgo espacial fuera del campo de visión comunicado.
- Generación de salida háptica y visualización de diagnóstico.
- Control auxiliar para grabación de rosbags.

## Arquitectura general

La pipeline principal puede ejecutarse en dos modos:

- `raw`: usa directamente la imagen de profundidad y la convierte a una grilla compacta.
- `filtered`: filtra la nube de profundidad, remueve suelo, genera obstáculos y habilita mapeo local/odometría.

Flujo simplificado:

```text
RealSense D435i
  -> depth_grid_encoder / depth_obstacle_filter
  -> local_mapper + spatial_awareness
  -> haptic_grid_generator
  -> output_viewer / hardware_manager
```

Los mensajes propios se definen en `custom_interfaces`, incluyendo `DepthGrid`, `HapticGrid` y `SpatialAwareness`.

## Requisitos

- Linux con Docker y Docker Compose.
- Cámara Intel RealSense D435i para ejecución con hardware real.
- Acceso a dispositivos `/dev` desde Docker.
- Servidor X11 disponible si se usan visualizadores gráficos o RViz2.

El contenedor del proyecto instala el entorno ROS 2 esperado, por lo que la forma recomendada de trabajar es mediante Docker.

## Puesta en marcha

Desde la raíz del repositorio:

```bash
make build
tesis-run
```

Dentro del contenedor:

```bash
tesis-run
colcon build
source install/setup.bash
```

Para abrir otra terminal dentro del contenedor:

```bash
tesis-terminal
```

## Ejecución

Las tres versiones de la pipeline se pueden iniciar dentro del contenedor con:

```bash
nav-start-v0  # profundidad raw
nav-start-v1  # profundidad filtrada
nav-start-v2  # filtrada + mapa local + spatial awareness
```

Los comandos usan la cámara RealSense, la salida háptica en `/dev/ttyACM0`
y dejan las visualizaciones desactivadas. `nav-start` se mantiene como alias
compatible de `nav-start-v1`.

Launch principal con cámara, pipeline básica y salida por hardware:

```bash
ros2 launch nav_bringup nav.launch.py use_perception:=true
```

Ejecución sin hardware de actuadores y con visualización:

```bash
ros2 launch nav_bringup nav.launch.py \
  use_perception:=true \
  use_hw:=false \
  use_viz:=true
```

Pipeline filtrada con mapeo local, RViz2 y odometría interna:

```bash
ros2 launch nav_bringup nav.launch.py \
  use_perception:=true \
  pipeline_mode:=filtered \
  use_local_mapper:=true \
  odom_source:=nav_odom \
  use_hw:=false \
  use_rviz:=true
```

Pipeline filtrada con RTAB-Map como fuente de odometría y estimación de riesgo espacial:

```bash
ros2 launch nav_bringup nav.launch.py \
  use_perception:=true \
  pipeline_mode:=filtered \
  use_local_mapper:=true \
  odom_source:=rtabmap_odom \
  use_spatial_awareness:=true \
  use_hw:=false \
  use_rviz:=true
```

Reproducción desde rosbag en lugar de cámara:

```bash
ros2 launch nav_bringup nav.launch.py \
  use_realsense:=false \
  use_bag:=true \
  bag_path:=/ruta/al/bag \
  use_perception:=true
```

Si `bag_path` se deja vacío, el launch intenta usar el bag más reciente en `~/bags`.

## Paquetes principales

- `nav_bringup`: launch principal y configuración compartida de la pipeline.
- `custom_interfaces`: mensajes ROS 2 propios del sistema.
- `nav_math`: utilidades compartidas de vectores y cuaterniones.
- `depth_grid_encoder`: conversión de profundidad u obstáculos a grillas compactas.
- `depth_obstacle_filter`: proyección de profundidad, alineación con gravedad, remoción de suelo y publicación de obstáculos.
- `nav_odometry`: odometría visual-inercial para RealSense D435i.
- `local_mapper`: mapeo local incremental en `nav_msgs/OccupancyGrid`.
- `spatial_awareness`: estimación de riesgos laterales/traseros a partir del mapa local.
- `haptic_grid_generator`: fusión de grilla de profundidad y riesgos espaciales en una salida háptica.
- `hardware_manager`: envío de la grilla háptica a actuadores por puerto serie y utilidades GPIO.
- `output_viewer`: visualizadores y monitores de diagnóstico.
- `rosbag_controller`: servicio para iniciar/detener grabación de rosbags.

## Estructura del repositorio

```text
.
├── docker/             # Dockerfile y recursos del contenedor
├── .devcontainer/      # Configuración para VS Code Dev Containers
├── src/                # Paquetes ROS 2 del workspace
├── tools/              # Utilidades auxiliares
├── Images/             # Imágenes usadas por documentación
├── docker-compose.yml  # Orquestación del entorno de desarrollo
└── Makefile            # Atajos para build, shell, run, clean y formato
```

## Comandos útiles

```bash
make build   # Construye la imagen Docker
make run     # Levanta el contenedor y abre una shell
make shell   # Entra a un contenedor ya levantado
make clean   # Baja el entorno y elimina volúmenes asociados
make fmt     # Formatea código C/C++ con clang-format
tesis-run
tesis-terminal
```

Dentro del contenedor:

```bash
colcon build
colcon test
source install/setup.bash
```

## Configuración

El rango válido del sensor de profundidad se centraliza en:

```text
src/nav_bringup/config/sensor_range.yaml
```

El launch principal lee ese archivo y propaga los valores a los nodos que usan límites de profundidad. Las configuraciones específicas de cada paquete viven en sus respectivos directorios `config/`.

## Estado del proyecto

Este repositorio forma parte de un trabajo de tesis y está orientado a investigación, prototipado y validación experimental. Algunas piezas están diseñadas para correr con hardware específico, por lo que ciertos nodos requieren cámara RealSense, acceso a dispositivos serie, GPIO o visualización gráfica.

## Repositorios relacionados

- `docs`: informe, presentación y material escrito de la tesis.

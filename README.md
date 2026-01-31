# Tesis – ROS 2 Humble (Baseline CPU)

## Requisitos
- Ubuntu 22.04
- Docker + Docker Compose
- (Opcional) VS Code + Dev Containers

## Levantar docker
```bash
cd ~/Tesis/develop/nav_mapper/
make build # Construye las imágenes de Docker definidas en docker-compose.yml
```
## Buildear
Dentro
```bash
make run   # Levanta todos los servicios definidos en docker-compose.yml y entra al contenedor
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```
## Correr SW
### Correr con el vizualisador de salida
**Terminal 1**
```bash
make shell
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py enable_gyro:=false enable_accel:=false
```
**Terminal 2**
```bash
make shell
source install/setup.bash
ros2 run tesis_nav_mapper_cpp depth_to_matrix 
```
**Terminal 3**
```bash
make shell
source install/setup.bash
ros2 run output_viewer depth_grid_heatmap 
```
## Estructura del proyecto

### `src/`
Contiene los paquetes ROS 2 del workspace.

- **`tesis_nav_mapper_cpp`**  
  Genera el tópico `/depth_grid` a partir de la imagen de profundidad de la cámara RealSense.

- **`tesis_nav_interfaces`**  
  Define los tipos de mensajes personalizados utilizados para la comunicación entre nodos (por ejemplo `DepthGrid`, `DepthCellStats`, etc.).

- **`hardware_manager`**  
  Consume el tópico `/depth_grid` y traduce la información de distancia en señales de control enviadas al hardware (motores / actuadores).

- **`output_viewer`**  
  Visualiza el contenido del tópico `/depth_grid` como un gráfico de intensidad (heatmap), representando las señales enviadas a cada motor.

---

### `docker/`
Dockerfiles utilizados para construir las imágenes del entorno de desarrollo y ejecución.

### `.devcontainer/`
Configuración del Dev Container para trabajar con VS Code dentro de Docker.

### `docker-compose.yml`
Orquesta el entorno completo de desarrollo, incluyendo contenedores, volúmenes y dispositivos.


<p align="center">
  <img src="Images/fiuba_logo.png" alt="FIUBA logo" width="280"/>
</p>

<h1 align="center">Tesis – Captación del entorno y mapeo local</h1>

<p align="center">
Facultad de Ingeniería – Universidad de Buenos Aires (FIUBA)
</p>

## Descripción
Este repositorio contiene el desarrollo correspondiente a la tesis *“Captación del entorno y mapeo local”* de la carrera de Ingeniería Electrónica (FIUBA).

El trabajo se enfoca en la adquisición y procesamiento de información de profundidad en tiempo real a partir de una cámara RGB-D, con el objetivo de generar una representación compacta del entorno inmediato frente al usuario.  
Dicha representación se materializa en una matriz de baja resolución publicada como el tópico `/depth_grid`, que resume la información espacial relevante del frame actual.

Sobre esta salida se proyecta el desarrollo de una **capa de mapeo local de corto alcance**, concebida como una memoria temporal de obstáculos recientemente detectados (No desarrollado todavia).
Este mapa local permitirá aumentar la robustez del sistema frente a oclusiones, cambios de orientación del sensor y limitaciones del campo visual, habilitando en etapas posteriores estrategias de alerta anticipada ante colisiones potenciales fuera del campo de visión de la cámara.

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
ros2 run depth_grid_encoder depth_to_matrix 
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

- **`depth_grid_encoder`**  
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


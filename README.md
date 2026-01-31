# Tesis – ROS 2 Humble (Baseline CPU)

## Requisitos
- Ubuntu 22.04
- Docker + Docker Compose
- (Opcional) VS Code + Dev Containers

## Levantar docker
```bash
cd ~/tesis_nav_assistant_ws
make build # Construye las imágenes de Docker definidas en docker-compose.yml
make run   # Levanta todos los servicios definidos en docker-compose.yml y entra al contenedor
# Dentro:
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
ros2 run ...
```
## Arbol

* `src/` paquetes ROS 2 (propios y third_party)

* `docker/` Dockerfile(s)

* `.devcontainer/` Devcontainer de VS Code

* `docker-compose.yml` orquesta el entorno

* `ros2.repos` dependencias externas (vcs)
Se necesitan de la camara los topicos de las camara monocromatica infrarojas izquierda y derecha. Tambien los datos de la imu.

Por lo que para pruebas se debe correr en un nodo la camara como:
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_gyro:=true \
  enable_accel:=true \
  enable_depth:=true \
  unite_imu_method:=1 \
```

Se necesitan de la camara los topicos de las camara monocromatica infrarojas izquierda y derecha. Tambien los datos de la imu.

Por lo que para pruebas se debe correr en un nodo la camara como:
```bash
ros2 launch realsense2_camera rs_launch.py \
  enable_gyro:=true \
  enable_accel:=true \
  enable_depth:=true \
  unite_imu_method:=1 \
```

## Salida REP-103

El nodo conserva por defecto la convención óptica histórica usada por
`local_mapper`. Para integrarlo con nodos ROS estándar se puede activar:

```bash
ros2 launch nav_odometry odometry.launch.py \
  output_topic:=vio_odom \
  output_rep103:=true \
  publish_tf:=false
```

El modo `odom_source:=vio+icp` de `nav_bringup` configura estas opciones
automáticamente y fusiona la VIO con `rtabmap_odom/icp_odometry` mediante
`robot_localization/ekf_node`.

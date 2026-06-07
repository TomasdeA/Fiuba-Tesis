# spatial_awareness

Detecta riesgo de colisión con obstáculos ocupados del mapa local que están
fuera del campo horizontal ya comunicado al usuario.

## Entradas

- `/local_mapper/occupancy_grid` (`nav_msgs/OccupancyGrid`)
- `nav_odom` (`nav_msgs/Odometry`) de RTAB-Map
- `/perception/depth_grid/aperture_state_deg` (`std_msgs/Float32`)

La apertura es un semiángulo. Un valor de `45` representa el intervalo
horizontal `[-45 deg, +45 deg]`.

## Salida

- `/spatial_awareness/collision_risk`
  (`custom_interfaces/SpatialAwareness`)
- `/spatial_awareness/debug/markers`
  (`visualization_msgs/MarkerArray`, diagnóstico opcional)

El mensaje contiene canales independientes `left`, `right` y `rear`. Cada
canal informa intensidad `[0, 100]`, distancia, velocidad de cierre y tiempo
estimado hasta colisión.

## Criterio

Solo se consideran componentes de al menos `min_cluster_cells` celdas con
ocupación mayor o igual a `occupancy_threshold`. Una celda genera riesgo si:

- está fuera del FOV comunicado;
- su distancia a la cámara está entre `min_distance_m` y `max_distance_m`;
- intersecta el corredor definido por `body_radius_m`;
- el usuario se mueve hacia ella por encima de
  `min_closing_speed_mps`; y
- su TTC es menor que `ttc_max_s`.

La desactivación usa `closing_speed_hysteresis_mps` para evitar conmutaciones
por ruido alrededor del umbral de velocidad de cierre.

RTAB-Map publica la pose en REP-103. El nodo la convierte a la convención
interna X-derecha, Y-abajo, Z-adelante. La velocidad usa `twist` cuando es
consistente con la diferencia de pose; en caso contrario usa `dpose/dt`.

Si el mapa o la odometría están vencidos, el mensaje se publica con
`valid=false` y los tres canales apagados.

## Ejecución

```bash
ros2 launch nav_bringup nav.launch.py \
  pipeline_mode:=filtered \
  use_perception:=true \
  orientation_source:=rtabmap_odom \
  use_local_mapper:=true \
  use_spatial_awareness:=true
```

El mapa se publica en cada fotograma. Con la cámara configurada a 6 Hz, la
tasa efectiva de actualización geométrica es aproximadamente 6 Hz.

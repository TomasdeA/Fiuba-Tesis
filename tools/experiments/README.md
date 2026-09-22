# Diagnóstico reproducible de odometría RGB-D

Rama: `feature/odometry-robustness`. Referencia de código:
`746f0557e1db160cd10732014343666f91aae740`.
Entorno probado: ROS 2 Humble, RTAB-Map 0.22.1 y Madgwick 2.1.5.

## Problema y cambio

La configuración anterior perdía el seguimiento al comenzar la traslación de
`R03_prueba_giro_360_1280x720_2`, aun reproduciendo el bag a mitad de velocidad.
La auditoría comprueba que RGB y profundidad alineada tienen la misma resolución,
calibración compatible y sellos idénticos en más del 95 % de los pares.

El fallo observado es una escasez de correspondencias **geométricamente válidas**
en el emparejamiento visual. No basta con aumentar la resolución de entrada.
Reducir la escala de detección y aumentar/distribuir los puntos mejora de forma
marcada el seguimiento con las mismas imágenes. Esto identifica una limitación
de esta configuración; no demuestra que el ruido de imagen sea su única causa
física, ni permite generalizar a todos los escenarios.

La configuración pasa a `src/nav_bringup/config/rtabmap_odometry.yaml` y puede
reemplazarse mediante `rtabmap_odom_config:=/ruta/perfil.yaml` sin editar el launch.

| Parámetro | Referencia | Perfil final |
|---|---:|---:|
| `Odom/ImageDecimation` | 1 | 2 |
| `Vis/MaxFeatures` | 600 | 1500 |
| `Vis/GridRows`, `Vis/GridCols` | 1 × 1 | 3 × 4 |
| `OdomF2M/MaxSize` | 3000 | 6000 |
| `approx_sync_max_interval` | 0,08 s | 0,01 s |

Se conservan frame-to-map, GFTT/ORB, rango visual de 4 m, mínimo de 15 inliers,
filtro Kalman y tolerancia de reproyección de 2 píxeles. Esta última se explicita
en el YAML. No se habilitan reinicios automáticos ni se rebaja el mínimo de
correspondencias para disimular pérdidas. Se conserva la entrada Madgwick y
`wait_imu_to_init: true`.

La resolución de grabación no cambia. La reducción ocurre dentro de la odometría:
1280×720 pasa a 640×360; 640×480 pasa a 320×240. Los demás nodos siguen recibiendo
las imágenes originales.

## Reproducción

Ejecutar en el contenedor `tesis_nav_dev`, con ROS y el workspace cargados:

```bash
source /opt/ros/humble/setup.bash
source /home/tomasdea/Tesis/develop/Fiuba-Tesis/install/setup.bash
python3 tools/experiments/odometry_benchmark.py \
  /home/tomasdea/Tesis/bags/experimentos_tesis/R03_prueba_giro_360_1280x720_2 \
  /home/tomasdea/Tesis/bags/experimentos_tesis/derivados/odom_benchmark/nueva_prueba \
  --variant dense_decimation2 --full-stack \
  --override src/nav_bringup/config/rtabmap_odometry.yaml
```

El script utiliza un dominio ROS aislado, reinicia los estimadores para cada
ensayo y detiene únicamente sus propios procesos. La carpeta de salida debe ser
nueva. Guarda parámetros efectivos, logs, poses y `/odom_info` en JSON. El bag
original se abre solamente para reproducción. Sin `--full-stack` ejecuta únicamente
Madgwick y `rgbd_odometry`; con esa opción ejecuta percepción, mapa y awareness.
No ejecutar benchmarks simultáneamente si se pretende comparar tiempo real.

Para repetir la referencia usar `--variant baseline` y omitir `--override`.
`--rate 0.5` aumenta el tiempo de procesamiento disponible, sin modificar los
sellos de los sensores. El porcentaje inválido se refiere a **poses publicadas**,
no a la totalidad de imágenes grabadas ni directamente al porcentaje de tiempo.
Examinar también intervalos sin seguimiento, frecuencia y trayectoria.

Los resultados de esta sesión están en
`bags/experimentos_tesis/derivados/odom_benchmark/`, fuera del repositorio de código.
Las carpetas `movement_*_v2` contienen la exploración; `*_final` y
`movement_full_*` validan el perfil final. `rotation*_selected` corresponden a una
candidata intermedia sin Kalman y con tolerancia de 4 píxeles, no al YAML final.

## Resultados de validación

Reproducción a velocidad real; misma entrada para comparar referencia y perfil.
Los dos giros y la marcha larga se procesaron con odometría aislada. La traslación + giro se validó
también con percepción, mapa y awareness ejecutándose.

| Grabación / ejecución | Inválidas, anterior | Inválidas, final | Seguimiento final |
|---|---:|---:|---|
| Giro 640×480 | 118/411 (28,71 %) | 1/632 (0,16 %) | Continuo después de inicializar |
| Giro 1280×720 | 224/328 (68,29 %) | 1/539 (0,19 %) | Continuo después de inicializar |
| Traslación + giro, v2 completo | 300/363 (82,64 %) | 1/570 (0,18 %) | Continuo después de inicializar |
| R03 marcha y riesgo, 99 s | 277/945 (29,31 %) | 1/1412 (0,07 %) | Continuo después de inicializar |

En cada ejecución final de esta tabla la única pose inválida fue la inicial.
Las pérdidas sostenidas anteriores fueron de 12,31 s, 28,79 s, 35,15 s y 40,06 s,
respectivamente. Recuperar una pose al final no equivale a haber seguido el giro.

En la traslación + giro con v2 completo, el perfil final estimó un máximo de
avance de +1,389 m y un mínimo de −1,078 m respecto del inicio. Captura la secuencia
de ida, retroceso y regreso; no prueba que cada metro nominal se mida exactamente.
El yaw acumulado fue 357,76° y la distancia 3D entre primera y última pose válida
fue 0,305 m. La mediana de inliers fue 214 (antes, 39 entre poses válidas), con
11,33 actualizaciones publicadas por segundo y 61,8 ms de tiempo de estimación
mediano en esta máquina.

El mismo perfil aislado, con otro conjunto de fotogramas procesados, produjo
343,36° y 0,415 m de cierre. La continuidad mejora de manera consistente, pero
estos ensayos **no certifican exactitud centimétrica ni una cota de error angular**.
La selección de fotogramas bajo carga y el estimador visual afectan la trayectoria.

Gráficos regenerables con `plot_odometry_benchmark.py` en la carpeta de resultados:
`movement_final_comparison.png`, `rotation640_final_comparison.png` y
`rotation1280_final_comparison.png`. Los JSON homónimos incluyen las métricas y
los intervalos válidos usados para dibujarlos.

## Criterios y descartes

- Quitar IMU o Kalman, sincronizar exactamente, ampliar el rango, usar ORB/SIFT
  o ampliar solamente la tolerancia de reproyección no resolvió por sí solo la
  toma de traslación y giro.
- Más puntos distribuidos o reducción de imagen por separado mejoraron el
  resultado, pero mantuvieron pérdidas. Su combinación completó la toma.
- `Vis/CorType=1` no está soportado con `Odom/Strategy=0` en RTAB-Map 0.22.1:
  el programa advierte que vuelve a descriptores. El ensayo `movement_optical_flow_v2`
  queda excluido de la comparación; no fue realmente flujo óptico.
- El flujo óptico real con frame-to-frame produjo más del 97 % de poses válidas,
  pero desplazamientos falsos superiores a 10 m. Se descarta por geometría,
  aunque su porcentaje de poses válidas sea alto. Desactivar Kalman o reducir
  la imagen no corrigió ese comportamiento en los ensayos realizados.
- Las poses inválidas con posición cero se excluyen de las curvas. No deben
  interpretarse como un regreso físico al origen.

Las distancias indicadas por el operador (+1, −2, +1 m) y el giro de 360° son
referencias nominales: no hay una trayectoria externa calibrada. Un desplazamiento
circular durante un giro corporal puede ser real por la separación entre cámara
y eje de giro. La distancia entre pose inicial y final se informa como cierre
estimado, no como error absoluto certificado.

`integrate_bag_gyro.py` proporciona un contraste angular adicional, no ground truth.
Su variante con corrección de bias supone inmovilidad en los primeros cuatro
segundos: cualquier movimiento real en ese intervalo contamina esa corrección.
Los tiempos `delay` del log ROS incluyen la antigüedad del bag; no son una medida
de latencia de procesamiento. Usar `time_estimation` de `/odom_info` para esta última.

Referencia de parámetros: [RTAB-Map Parameters.h](https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h).
Los valores se verificaron además contra el header instalado de la versión 0.22.1.

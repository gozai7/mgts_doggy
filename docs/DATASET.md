# Настройка неизвестного датасета

Основной файл: `configs/competition/dataset.yaml`. Все пути bag/output в нём — пути **в контейнере**.
На хосте `data/` соответствует `/data`, `results/` — `/results`, проект — `/mgts`.
Не вставляйте `/home/имя/...` в dataset.yaml без соответствующего mount.

| Поле | Что требуется |
|---|---|
| `topics.lidar`, `imu` | Точное имя либо auto при единственном topic нужного типа |
| `topics.odometry` | null для отключения, иначе конкретный nav_msgs/Odometry |
| `source_frames` | Ожидаемые frame_id исходных сообщений; null разрешает один неизменный frame |
| `source_frames.odometry_child` | Обязательно для одометрии; должен физически соответствовать base |
| `frames` | Различные канонические имена map/odom/base/lidar/imu |
| `extrinsics.T_base_lidar`, `T_base_imu` | Измеренные `[x,y,z,qx,qy,qz,qw]`, метры, XYZW |
| `units.xyz` | m, cm или mm |
| `units.acceleration` | m/s2 либо g; стандартное g=9.80665 |
| `units.angular_velocity` | rad/s либо deg/s |
| `point_time.mode` | relative, absolute либо none |
| `point_time.field`, `unit` | Фактическое имя любого поля, единицы s/ms/us/ns |
| `point_time.offset_s` | Дополнительное смещение точки относительно header после перевода единиц |
| `time_offsets_s` | Исправленное время = исходное + offset; отдельно lidar/imu/odometry |
| `lidar.scan_period_s` | Фактический период, также sanity limit времени точек |
| `lidar.range_m` | Фильтр дальности `[min,max]` в SI |
| `lidar.body_box_base_m` | null либо `[[xmin,ymin,zmin],[xmax,ymax,zmax]]` в физическом base |
| `localization.initial_pose` | `null` включает автоматический поиск. Известную позу base в map можно передать как `[xyz, qxyzw]` |
| `occupancy.floor_z_m` | Высота пола в окончательной map; не обязательно ноль |

Bag Doctor ищет распространённые time/t/timestamp/time_stamp/offset_time, но не объявляет
найденное поле временем без подтверждения. Тип PointField и семантика определяются датасетом.
Absolute point time вычитается из header с повышенной промежуточной точностью. Исходный
float32 timestamp с потерянной точностью невозможно восстановить программно.
Если LiDAR публикует stamp конца скана, относительные времена могут быть отрицательными;
их величина должна укладываться в два scan_period. Не меняйте одновременно разные offsets,
пока не проверена их семантика. Время IMU и LiDAR должно иметь общую шкалу.

Для отсутствующего времени точек:

```yaml
point_time: {mode: none, field: null, unit: null, offset_s: 0.0}
mapping:
  profile: no_point_time
  mode: cpu
  timeout_s: 7200
  min_trajectory_coverage: 0.75
```

Это отключает deskew, не восстанавливает неизвестные времена по азимуту.
Поля intensity сохраняются при наличии; ring и другие поля не требуются выбранным адаптерам.
NaN/Inf точки удаляются; некорректная структура сообщения, меняющийся frame, нулевые/дублирующиеся
или убывающие timestamps приводят к ошибке. IMU NaN/Inf не интерполируются автоматически.
Нормализатор оставляет `.partial` для анализа сбоя и не портит исходник.

Если во втором bag отличаются топики/калибровки, используйте частичные переопределения:

```yaml
phase_overrides:
  localization:
    topics: {lidar: /second/points, imu: /second/imu}
    source_frames: {lidar: lidar_second, imu: imu_second}
```

Это пример синтаксиса, не калибровка реального робота. Оставшиеся значения наследуются.
Менять физическую калибровку между проходами следует только по измеренным данным.

## Записи Unitree Go2 из hackathon

В `configs/competition/go2.yaml` заданы калибровки LiDAR и IMU, исходные топики
и относительное время точек в секундах. Для записи B бортовая одометрия предсказывает
движение между сканами. Её первая поза добавляется к кандидатам автостарта и
проверяется по карте, в том числе на отдельном скане.

Распакуйте архивы A и B так, чтобы в `DATA_DIR` находились каталоги
`bag_for_mapping` и `bag_for_localization`. Из корня проекта выполните:

```bash
export DATA_DIR=/absolute/path/to/rosbags
make build
docker compose run --rm competition python3 tools/go2_reference_map.py /data/bag_for_mapping /results/go2-prior
make localization CONFIG=configs/competition/go2.yaml
```

Экспорт создаёт новый каталог `results/go2-prior` в стандартном `RESULTS_DIR`.
Карта строится из записанных облаков A `/utlidar/cloud_deskewed` в системе `odom`.
Траектория A берётся из `/utlidar/robot_odom`. Для RViz укажите
`/results/go2-calibrated` в [команде просмотра](VISUALIZATION.md#rviz-готовый-результат).
Сборка карты A из сырых данных Fast-LIO пока расходится. В профиле Go2 этап
`mapping` останавливается, если шаг между соседними позами больше 0,5 м.
Команда `localization` использует указанный в профиле PCD независимо от этапа `mapping`.

## Настройка алгоритма

Сначала изменяйте range, плотность/voxel и профиль. Сгенерированные нативные конфигурации
сохраняются в `mapping/config/`, однако изменения нужно вносить в dataset.yaml,
чтобы следующий запуск и manifest воспроизводили их.

GLIM: секции `preprocess`, `odometry_estimation`, `sub_mapping`, `global_mapping`.
Ключ должен существовать в выбранном CPU/GPU JSON закреплённой версии; опечатка останавливает запуск.

```yaml
mapping:
  # Сохраните остальные поля основной конфигурации.
  glim_parameters:
    preprocess: {downsample_resolution: 0.12, random_downsample_target: 10000}
    global_mapping: {max_implicit_loop_distance: 4.0, min_implicit_loop_overlap: 0.6}
```

RKO позволяет менять voxel_size и initialization_phase; graph_parameters — геометрические
параметры, уже перечисленные в `competition/backends.py`, без изменения путей/экспорта/extrinsics.

```yaml
mapping:
  rko_parameters: {voxel_size: 0.25}
  graph_parameters:
    voxel_leaf_size: 0.15
    submap_distance_threshold: 0.5
    loop_min_overlap_ratio: 0.7
    loop_max_translation_delta: 2.0
    loop_max_rotation_delta_deg: 20.0
```

Требуемая неподвижная фаза и качество IMU initialization проверяются по логам upstream.
Не отключайте initialization только для устранения сообщения об ошибке без понимания причины.
Перед финалом испытайте профиль с реальным датасетом. Предобработанные camera points
не объединяются автоматически с LiDAR: у них могут отличаться шум, плотность и время.

# Соревновательная обработка LiDAR + IMU

Ветка `competition-fast-lio` строит 3D-карту по записи A на CPU. По записи B общий локализатор
находит начальную позу на этой карте и восстанавливает маршрут. Выбранный алгоритм указан в
`BACKEND`. Команды запуска и основные форматы совпадают с ветками GLIM и RKO-LIO.

| Версия | Первый проход | Глобальная коррекция | Второй проход |
|---|---|---|---|
| GLIM | GLIM CPU GICP или GPU | Глобальный граф GLIM | lidar_localization_ros2 |
| RKO-LIO | RKO-LIO из закреплённого форка стека lidar_slam | graph_based_slam, GICP, геометрические проверки петель | Тот же localizer |
| Fast-LIO | Fast-LIO2, офлайн-чтение LiDAR и IMU | Инкрементальная карта с вертикалью по гравитации | Тот же localizer |

Из демо Fast-LIO + Nav2 взята исходная реализация Fast-LIO из репозитория базового образа.
Ветка использует ROS 2 Jazzy и штатный пайплайн проекта для обработки rosbag, локализации
и экспорта 2D-карты. [Исходники и зависимости](THIRD_PARTY.md).

## Первый запуск

Хост: Linux x86_64, Docker Engine + Compose v2/v5. Ubuntu 22.04 на ноутбуке подходит как хост;
ROS 2 Jazzy и Ubuntu 24.04 находятся внутри контейнера. Системный Humble менять не нужно.
Практический ориентир для сборки — 16+ ГБ RAM, 30+ ГБ свободного диска; это запас, а не измеренный минимум.
Сборка требует интернета. Обработка по умолчанию идёт без сети. Собирать образы нужно до соревнования.

```bash
git switch competition-fast-lio
mkdir -p data results
make build
make smoke-test
```

`smoke-test` генерирует две пары записей асимметричного офиса, с временем отдельных точек и
без него. В каждой паре Fast-LIO строит карту A, локализатор автоматически запускается на B,
а пайплайн экспортирует 2D-карту. Эталон используется при независимой оценке результатов.
[Протокол и результаты проверки](TEST_REPORT.md#fast-lio-на-ros-2-jazzy).

Положите записи в `data/mapping/` и `data/localization/` (полные каталоги bag с metadata.yaml),
либо укажите другие пути внутри `/data`. Отдельный `.mcap` также читается адаптером.
Оригинальные данные примонтированы только для чтения.

```bash
make doctor
make doctor PHASE=localization
# Заполнить configs/competition/dataset.yaml по отчёту и измеренным калибровкам.
make prepare
make baseline             # необязательная независимая LiDAR-only одометрия KISS-ICP
make mapping
# Для неизвестного старта B оставить initial_pose: null, manual_initial_pose: false.
make localization
make occupancy
make evaluate
# При уже согласованной конфигурации все четыре этапа: make run-all
```

Не запускайте реальные данные с пустыми extrinsics/единицами/point_time: конфигурация намеренно
останавливается на неизвестных величинах. `doctor` работает до заполнения этих полей.
`prepare` только генерирует конфигурации и не сообщает об успешном SLAM.
Другой конфиг выбирается через `make mapping CONFIG=configs/competition/my_dataset.yaml`.
Каталоги хоста: `DATA_DIR=/absolute/bags RESULTS_DIR=/absolute/results make mapping`.
При изменении параметров выбирайте новый `output`, например `/results/run02`.

## Данные и системы координат

Описание всех полей и примеры: [docs/DATASET.md](docs/DATASET.md).
Трансформация `T_base_lidar=[x,y,z,qx,qy,qz,qw]` переводит точку LiDAR в base.
Единицы — метры; кватернион XYZW, единичной длины. Аналогично задаётся `T_base_imu`.
Нулевое смещение допустимо только как измеренная калибровка, автоматически оно не подставляется.
Входные сообщения записей имеют типы `sensor_msgs/msg/PointCloud2` и `sensor_msgs/msg/Imu`.
PointCloud2 нормализуется в XYZ SI и относительное поле `time` в секундах. При `mode: none`
поле времени не выдумывается, deskew отключается. Поправки времени применяются ровно один раз.
Fast-LIO записывает позу на время последней точки скана или на stamp облака при `mode: none`.
Для инициализации IMU запись A должна начинаться с неподвижной фазы.

При пустом `localization.initial_pose` начальную позу корпуса вычисляет стек. Он накапливает сканы B в локальной системе первого скана, ищет кандидатов по всей горизонтальной проекции карты A методом BBS и уточняет их по 3D-карте с ICP. Отдельный скан проверяет совпадение. При близких оценках разных поз поиск продолжается, а при исчерпании записи возвращает ошибку. Высоту для уточнения подсказывает ближайшая по XY поза маршрута A. Режим рассчитан на одноэтажное помещение.

С найденной позы NDT повторно обрабатывает B с первого скана. В конфиге можно выбрать предиктор движения,
в том числе кинематическую одометрию или IMU. Отклонённые регистрации не превращаются в «успешные» позы.

## Результаты

При `output: /results/run01` файлы находятся на хосте в `results/run01/`.

| Путь внутри результата | Содержание |
|---|---|
| `mapping/map.pcd` | Инкрементальная 3D-карта Fast-LIO |
| `mapping/trajectory_raw.tum` | Одометрическая траектория base в локальной системе алгоритма |
| `mapping/trajectory_optimized.tum` | Траектория base в map в общем формате |
| `mapping/trajectory.tum`, `trajectory.csv` | Та же траектория в форматах TUM и CSV |
| `mapping/trajectory_lidar.tum` | Позы физического LiDAR в map |
| `mapping/frames.json`, `config_resolved.yaml` | Frame-контракт и фактическая конфигурация |
| `localization/trajectory.tum`, `trajectory.csv` | Только принятые регистрации второго прохода |
| `localization/diagnostics.json`, `alignment_status.jsonl` | Принятие/отклонение регистраций и восстановление |
| `occupancy/map.yaml`, `map.pgm`, `occupancy_bag/` | 2D-карта и ROS nav_msgs/OccupancyGrid |
| `diagnostics/mapping/`, `diagnostics/localization/` | Bag Doctor JSON и Markdown |
| `diagnostics/metrics.json`, `system.json`, `report.md` | Метрики, версии и отчёт |
| `logs/`, `mapping/backend/` | Логи, IMU-траектория и облака для 2D-экспорта |

В Fast-LIO `trajectory_raw.tum` и `trajectory_optimized.tum` совпадают. Имя второго файла
сохранено для совместимости. Отдельная оптимизация петель в этой ветке отсутствует.

TUM/CSV: `t x y z qx qy qz qw`; секунды исходной временной шкалы после поправок, метры, XYZW.
Числа времени в TUM double имеют ограничение точности возле больших Unix timestamps;
внутри адаптера и ROS-сопоставления используются целые наносекунды.
`complete.json` появляется только после проверки обязательных выходов; `failed.json` означает
частичный результат. При сбое локализации готовая карта сохраняется. Не смешивайте результаты разных запусков.

## CPU, GPU и просмотр

Для Fast-LIO установите `mapping.mode: cpu`. Доступны профили `default_indoor`,
`sparse_lidar`, `cpu_only` и `no_point_time`. Профиль `no_point_time` требует `point_time.mode: none`.
RViz, PlotJuggler и ручная инициализация описаны в
[docs/VISUALIZATION.md](docs/VISUALIZATION.md); основной pipeline работает без GUI.

## Проверки и диагностика

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements-competition.txt
make test-unit
python tools/bag_doctor.py data/mapping --output results/doctor
python tools/perturb_bag.py data/mapping results/fault_imu_20ms \
  --topic /imu/data --header-shift-s 0.02 --seed 42
```

Сравнение траекторий требует явного `T_a_b`: системы отдельных SLAM-запусков обычно различаются.

```bash
python tools/compare_results.py results/a/mapping/trajectory_optimized.tum \
  results/b/mapping/trajectory_optimized.tum --T-a-b 0 0 0 0 0 0 1 \
  --output results/comparison.json
```

Identity в примере разрешён **только если системы уже совпадают**. Расхождение не равно абсолютной ошибке.
Дополнительные команды: [docs/DIAGNOSTICS.md](docs/DIAGNOSTICS.md).

## Исходная заготовка и дальнейшие шаги

Сохранены `solver/`, `sim/`, сценарии, Karto-патч, исходные тесты и исследования.
Старый 2D workflow вызывается через `make -f Makefile.legacy help/build/...`;
его контейнеры описаны в `Dockerfile.legacy` и `compose.legacy.yaml`.
Показатели в `benchmarks/` относятся к исходной 2D-системе.

Читайте [COMPETITION_RUNBOOK.md](COMPETITION_RUNBOOK.md), [ARCHITECTURE.md](ARCHITECTURE.md),
[KNOWN_ISSUES.md](KNOWN_ISSUES.md), [CHANGELOG.md](CHANGELOG.md),
[THIRD_PARTY.md](THIRD_PARTY.md) и `THIRD_PARTY.lock.json`.
Удалённые репозитории не изменялись; `.git` в архиве отсутствует.

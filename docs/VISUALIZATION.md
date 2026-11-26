# Просмотр и ручная начальная поза

## RViz: готовый результат

В основном профиле сеть и GUI отключены. Следующие команды — явное включение локального X11.
Задайте DISPLAY и XAUTHORITY своей desktop-сессии. Не используйте неограниченный `xhost +`.
Wayland/XWayland и headless-хосты требуют отдельной настройки; GUI здесь не проверен.

```bash
test -n "$DISPLAY"
test -f "$XAUTHORITY"
docker compose -f compose.yaml -f compose.gui.yaml run --rm --name mgts-view \
  competition python3 tools/view_results.py /results/run01
```

Во втором терминале:

```bash
docker exec mgts-view /mgts/docker/competition/entrypoint.sh \
  rviz2 -d /mgts/configs/competition/results.rviz
```

Из PCD публикуется ограниченная выборка точек для экрана; исходная карта не изменяется.
Path отображает финальную mapping/localization траекторию, Occupancy включается отдельным display.
Fixed Frame по умолчанию `map`; если изменён frames.map, установите соответствующее имя в RViz.
Сырые одометрические траектории нельзя накладывать на map без известного преобразования.
Для живых GLIM/RKO keyframes/constraints включены штатные upstream RViz-конфиги в vendor;
их топики зависят от режима работы, offline replay не обещает постоянную публикацию графа.

## Неизвестный старт второго прохода

В dataset.yaml: `initial_pose: null`, `manual_initial_pose: true`, увеличьте
`initialization_timeout_s`, если нужно время на осмотр. Затем:

```bash
docker compose -f compose.yaml -f compose.gui.yaml run --rm --name mgts-localize \
  competition python3 -m competition.runner localization --config /mgts/configs/competition/dataset.yaml
```

В другом терминале запустите RViz через `docker exec mgts-localize ...` как выше.
Включите display **Localizer map** (`/map`, PointCloud2), выберите **2D Pose Estimate**
и задайте положение/направление **base** на карте. Replay ждёт `/initialpose` до первого скана.
Если нужен точный z/roll/pitch, задайте полный quaternion через dataset.yaml вместо 2D-инструмента.
Ручная гипотеза не считается доказанной: проверьте fitness и первые последовательные сканы.
Автоматический поиск стартовой позы выполняется перед replay при пустом `initial_pose` и выключенном ручном режиме.

## PlotJuggler

Сначала сформируйте плоский CSV со стабильными именами колонок:

```bash
python tools/export_plot_csv.py results/run01/normalized/localization/bag \
  results/run01/diagnostics/plot.csv \
  --diagnostics results/run01/localization/alignment_status.jsonl
```

Для анализа только входных данных опустите `--diagnostics`. CSV включает gyro, accel,
интервалы LiDAR/IMU, кинематическую позицию, fitness, время alignment и accepted.
Пустое поле означает отсутствие значения. При нескольких событиях одного времени возможны строки
с одинаковым t; это данные разных сенсоров, а не дублирование их измерений.

Откройте CSV в PlotJuggler, выберите `t` как time column, без добавления префикса к именам
сигналов. Загрузите `configs/competition/diagnostics.plotjuggler.xml`, затем Auto Zoom.
Подготовлены вкладки Gyro, Acceleration, Intervals, Odometry, Registration.
Layout основан на сериализации upstream 3.9.2; если установленная версия переименовала поля,
перенесите соответствующие CSV-колонки на графики и сохраните свой layout.
ROS-версия запускается в GUI-контейнере через `ros2 run plotjuggler plotjuggler`.

## GLIM offline viewer

Только GLIM; опциональная сборка не влияет на headless pipeline:

```bash
WITH_VIEWER=ON make build
docker compose -f compose.yaml -f compose.gui.yaml run --rm competition \
  ros2 run glim_ros offline_viewer /results/run01/mapping/dump \
  --config_path /results/run01/mapping/config
```

Для GPU добавьте `-f compose.gpu.yaml` и предварительно соберите `WITH_VIEWER=ON make build-gpu`.
Iridescence submodules скачиваются при build по закреплённым SHA.
Upstream viewer позволяет осматривать submaps и вручную добавлять/проверять ограничения.
Он не проверен в этой среде. Не добавляйте петлю только ради замкнутой картинки и не смешивайте
карту после ручной правки с прежней траекторией/occupancy. Сохраните отдельный dump и экспортируйте
полный согласованный комплект. Автоматический импорт ручных правок в checkpoint пока отсутствует.

Для живого GLIM из уже подготовленного canonical bag можно использовать штатный
`ros2 run glim_ros glim_rosnode --ros-args -p config_path:=...` и `ros2 bag play ... --clock`.
Нормализация/TF-контракт должны быть теми же. Это диагностический режим, а не основной offline runner.

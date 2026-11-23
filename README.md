# mgts_doggy · Карта и маршрут инспекции

По записи обхода **A** SLAM Toolbox строит 2D-карту помещения. По записи инспекции **B** AMCL восстанавливает маршрут на выбранной карте. Результат — сетка занятости и траектория робота с временными метками. [Условия кейса](vyzhimka-dvukh-vebinarov.pdf)

## Запись rosbag в Gazebo (необязательно)

Если записи уже выданы, переходите к [построению карты](#построение-карты). Для съёмки своих bag выполните `make build`, затем:

```bash
make robot A=survey                   # обход для карты
make robot MODE=inspection B=patrol   # отдельный запуск для инспекции
```

В каждом запуске дождитесь `All requested topics are subscribed`, во втором терминале выполните `make teleop`. Клавиши `i` и `,` — вперёд и назад, `j/l` — поворот, `k` — остановка. Завершите запись через `make stop` перед следующим запуском. Bag сохраняются в `data/survey/sensors/` и `data/patrol/sensors/`, эталон — в `truth/`.

## Построение карты

Из корня проекта выполните `make build-solver`. Нужны GNU Make и Docker Compose. Для обработки готовых записей достаточно образа решателя.

Положите bag полного обхода в `data/survey/`, а bag инспекции в `data/patrol/`. Для стандартного rosbag2 внутри каждого каталога лежат `metadata.yaml` и файлы `.mcap` или `.db3`. Имена каталогов произвольные: дальше их передают как `A` и `B`.

```bash
make map A=survey MAP=office           # создаёт runs/office/map.yaml и map.pgm
make maps                             # показывает имена доступных карт
make localize B=patrol MAP=office      # строит маршрут на карте office
make eval B=patrol MAP=office          # сохраняет отчёт и наложение на эту карту
```

**`MAP` — имя каталога карты внутри `runs/`.** При построении это имя новой карты, при локализации и оценке — имя уже существующей. Для готовой карты Nav2 положите YAML и изображение в тот же каталог. Например:

```text
runs/
  office/                     # карта, построенная через make map
    map.yaml
    map.pgm
    trajectory.csv            # оптимизированные ключевые позы обхода A
  floorplan/                  # готовая карта, которую вы принесли
    map.yaml                  # поле image: floor.pgm
    floor.pgm
  routes/
    office/patrol/            # результат B на карте office
    floorplan/patrol/         # тот же B на карте floorplan
```

У готовой карты файл описания должен называться `map.yaml`. Оставьте в нём исходные `resolution`, `origin` и пороги занятости. Путь в поле `image` должен указывать на доступный контейнеру файл, проще всего — на изображение рядом с YAML.

## Один маршрут на нескольких картах

Для нового прогона `make test MAP=… B=…` объединяет шаги `localize` и `eval`. Например, один bag можно проверить на двух картах:

```bash
make test MAP=office   B=patrol
make test MAP=floorplan B=patrol
```

Сравните `report.json` и `overlay.png` в каталогах `runs/routes/office/patrol/` и `runs/routes/floorplan/patrol/`. Если маршрут уже построен, достаточно `make eval MAP=office B=patrol`. Чтобы повторить опыт **на той же карте** с другими параметрами, задайте новый `ROUTE`:

```bash
make test MAP=office B=patrol ROUTE=office-known-start ARGS="--initial 0.5 0 0"
```

`--initial X Y YAW` задаёт старт в координатах **выбранной карты** (метры, радианы). По умолчанию AMCL ищет старт глобально. Значение `ROUTE` отсчитывается от `runs/`; в этом примере результат окажется в `runs/office-known-start/`. Так же можно открыть прежний прогон, например `make eval MAP=final-A ROUTE=final-B`. Существующие карты и маршруты защищены от перезаписи.

## Данные хакатона и настройки

- Скопируйте [профиль входа](configs/go2.yaml) в `configs/final.yaml`. Для обеих записей передайте `ARGS="--profile /mgts/configs/final.yaml"` в `make map` и `make localize` или `make test MAP=…`. Если формат A и B различается, укажите отдельный профиль для каждого запуска.
- В профиле задаются топик `LaserScan` или `PointCloud2`, высотный срез облака, источник одометрии и разрешённые рёбра TF от базы к лидару. Старый `map → odom` во вход не включайте. Для одометрии из TF укажите `odometry: null` и разрешите локальное ребро одометрии; для отсутствующей IMU задайте `imu: null`. Имена TF-топиков меняются через `tf_topic` и `static_topic`; внешняя калибровка задаётся через `calibrations` (поля `frames`, `xyz`, `rpy`).
- Параметры SLAM и локализатора находятся в [mapping.yaml](configs/mapping.yaml) и [amcl.yaml](configs/amcl.yaml). Для ускоренной обработки добавьте `--rate 2` в `ARGS`.

## Что смотреть в результатах

| Файл | Содержание |
|---|---|
| `runs/<MAP>/map.yaml`, `map.pgm` | Описание карты для Nav2 и растр занятости. Рядом решатель сохраняет сериализованный граф SLAM. |
| `runs/routes/<MAP>/<B>/trajectory.csv` | Маршрут инспекции в системе карты, столбцы `t,x,y,yaw`. Между обновлениями AMCL поза переносится по локальной одометрии. |
| `report.json`, `overlay.png` рядом с маршрутом | Счётчики обработки, разрывы, скачки и сканы на карте. Синим показан маршрут B, голубым — попадания лидара. |

Для собственной карты чекер находит эталоны в `truth/` по **фактическим входным bag** обоих прогонов. Тогда доступны RMSE и P95 положения, ошибка курса, покрытие времени и качество карты. При заданном старте отчёт сравнивает AMCL с поправками по карте и ту же локальную одометрию без поправок. У Go2 эта одометрия уже прошла EKF. Для привязки истины к карте нужен прогон картографирования A. На внешней карте из одного YAML и растра чекер строит наложение и диагностику, поле `absolute_errors` получает `null`.

## Готовый пример и проверка сборки

[Релиз с данными](https://github.com/gozai7/mgts_doggy/releases/tag/simulation-data-v1) содержит записи, карты, траектории и эталоны. После `make build-solver` в свежем клоне:

```bash
gh release download simulation-data-v1 --pattern 'mgts-doggy-recordings.tar.gz' --pattern SHA256SUMS
sha256sum -c SHA256SUMS && tar -xzf mgts-doggy-recordings.tar.gz
make maps
make test MAP=final-A B=office-B ARGS="--initial 0.5 0 0"
```

`make test` без выбранной карты запускает отдельную проверку сборки на коротких аналитических bag. Все команды показывает `make help`. На машине подготовки Make автоматически выбирает Docker-контекст `mgts`; после перезагрузки служба запускается через `systemctl --user start mgts-docker`.

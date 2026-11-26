# Диагностика и эксперименты

Команды ниже выполняются в активном Python venv из корня проекта; для ROS-only действий
используйте Docker. В контейнере заменяйте `results/` на `/results/`, `data/` на `/data/`.

## Аудит

```bash
python tools/bag_doctor.py data/mapping --output results/doctor
python tools/bag_doctor.py data/localization --output results/doctor_second
```

JSON/Markdown включают metadata, число сообщений, частоты и gaps, header-record delta,
поля/точки/NaN/дальности, кандидаты point time, IMU нормы, путь одометрии, TF-компоненты,
циклы, несколько родителей и временное перекрытие. Нормы остаются в raw units до адаптера.
Семплирование ограничено; `complete_header_audit` сообщает, прочитан ли весь bag.
TF не проверяется на каждом времени скана. Плато IMU не доказывает насыщение.

## Временной сдвиг

```bash
python tools/estimate_time_offset.py results/run01/normalized/mapping/bag \
  results/run01/baseline/trajectory.tum \
  --config results/run01/normalized/mapping/config_resolved.yaml \
  --max-shift 0.1 --output results/offset.json
```

Коррелируются gyro IMU, повёрнутый в base, и угловая скорость независимой base-траектории.
Корреляция требует информативных вращений, общей временной шкалы и правильных единиц.
Возвращаемая поправка имеет смысл **corrected IMU time = recorded IMU time + offset**.
Она не применяется автоматически. Если bag уже нормализован с поправкой, результат — остаточная
поправка, которую нельзя вслепую повторно внести. На почти неподвижной записи результат —
`insufficient_excitation`. KISS-ICP должен быть независимо успешным; его отказ ничего не калибрует.

## Fault injection

Утилита всегда создаёт новый bag и manifest. Не допускается output внутри исходного bag.
`--point-shift` задан в **единицах исходного поля**, `--header-shift-s`/`--record-shift-s` — секунды.
Смена скорости rosbag play не заменяет рассинхронизацию сенсоров.

```bash
python tools/perturb_bag.py data/mapping results/imu20 \
  --topic /imu/data --header-shift-s .02 --seed 42
python tools/perturb_bag.py data/mapping results/sparse25 \
  --topic /lidar/points --keep-points .25 --drop-scans .05 --seed 42
python tools/perturb_bag.py data/mapping results/gap \
  --topic /lidar/points --gap-s 5 6.5 --seed 42
python tools/perturb_bag.py data/mapping results/no_time \
  --topic /lidar/points --time-field time --remove-time
python tools/perturb_bag.py data/mapping results/gyro_bias \
  --topic /imu/data --gyro-bias 0 0 .01 --gyro-noise .002 --gyro-clip 1.0 --seed 42
python tools/perturb_bag.py data/mapping results/extrinsic_fault \
  --topic /tf_static --tf-child lidar --tf-rpy-deg 0 0 3 --tf-translation .05 0 0
```

Gap задаётся относительно начала bag. Bias/noise/clip имеют исходные единицы IMU.
TF-изменение — изменение сообщения в тестовом bag, **не** автоматическое изменение
измеренной калибровки dataset.yaml: адаптер использует явные extrinsics и игнорирует исходный TF.
Для испытания именно неверной калибровки сделайте отдельную копию dataset.yaml и измените её
extrinsics, сохранив точное описание perturbation. Оригинальный файл оставьте нетронутым.

## Оценка и сравнение

`make evaluate` считает длину/длительность/coverage, максимальные шаги и gaps,
выводит диагностику принятых регистраций, время без принятой регистрации и серии отказов.
Расстояние между концами маршрута подписано как endpoint distance, не как loop error.
Согласие двух методов не доказывает правильность обоих. Отчёт может содержать частичные результаты;
для сдачи дополнительно проверяются complete.json каждого этапа.

Для настоящей эталонной base-траектории в известных системах:

```bash
python tools/evaluate_ground_truth.py reference_base.tum \
  results/run01/mapping/trajectory_optimized.tum \
  --T-reference-map 0 0 0 0 0 0 1 --delta-s 1 --max-gap-s .5 \
  --output results/run01/diagnostics/ground_truth.json
```

Считаются APE translation/rotation и RPE translation/rotation через 1 секунду с перекрывающимися
парами. SE(3)-преобразование передаётся явно; scale fitting отсутствует. Вне временного диапазона
и через большие gaps интерполяция не применяется. Кинематическая одометрия не является GT.

Для принятого организаторами протокола можно отдельно использовать [evo](https://github.com/MichaelGrupp/evo/wiki/Metrics):
`evo_ape tum reference_base.tum estimate.tum --align` и
`evo_rpe tum reference_base.tum estimate.tum --delta 1 --delta_unit s`.
Укажите способ alignment и association в отчёте; эти числа могут отличаться от нашей интерполяционной
метрики. evo не включён в обязательное окружение и здесь не запускался.

## Симптомы

| Наблюдение | Следующая проверка |
|---|---|
| Двойные стены на повороте | Point time, scan stamp, extrinsics, offset и deskew |
| Наклон карты нарастает | IMU единицы/гравитация, mount, initialization |
| Скачки траектории | Timestamp gaps, отвергнутые регистрации, TF |
| Дрейф вдоль коридора | Геометрическая вырожденность, плотность и перекрытие |
| Неверный одинаковый коридор | Начальная гипотеза, ложный loop, последовательность сканов |
| KISS работает, LIO нет | IMU, синхронизация и LiDAR–IMU calibration; это не доказательство причины |
| Пол — препятствие | floor_z_m и height_m в финальной map |
| Слишком большая free-область | Начала лучей, позы, floor filter, missing returns |

При смене параметров маппинга используйте новый output. Начальную позу второго прохода и floor
можно уточнить после mapping: они не инвалидируют уже готовую карту. Успешную occupancy-карту
с другими параметрами нужно убрать в отдельный архивный каталог перед повторным экспортом.

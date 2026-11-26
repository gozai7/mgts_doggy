# Отчёт проверок — 26 ноября 2026

## Выполнено

| Проверка | Результат |
|---|---|
| Python unit suite, архив GLIM | 50 passed |
| Python unit suite, архив RKO-LIO | 48 passed, 2 skipped (GLIM-specific config tests) |
| Те же тесты с закреплёнными NumPy/SciPy/PyYAML | 50 passed; 48 passed + 2 skipped |
| Генерация двух реальных сериализованных synthetic bag | По 201 PointCloud2 и 2001 Imu, 20 секунд |
| Bag Doctor + нормализация обоих synthetic bag | Пройдены, исходники не изменены (SHA256 сравнены) |
| Occupancy из synthetic scans + известных fixture poses | 101 скан; 1327 occupied, 24131 free, 1038 unknown; без пропусков поз |
| Внесение IMU header shift +20 ms | Doctor измерил ровно 0.02 s header-record delta |
| Корреляция synthetic gyro и независимой fixture trajectory | Кандидат около 0 ms, автоматически не применён |
| Экспорт CSV диагностики | 2202 строки |
| Генерация нативных CPU/GPU GLIM, RKO/graph и localizer configs | Выполнена без запуска алгоритмов |

Окружение тестов: Python 3.12.14, rosbags 0.11.0, pytest 8.4.2.
Первый прогон: NumPy 2.3.5, SciPy 1.17.0, PyYAML 6.0.3. Повторный прогон:
NumPy 1.26.4, SciPy 1.14.1, PyYAML 6.0.2 — версии requirements-competition.txt.

Тесты покрывают SQLite3/MCAP, PointCloud2 endian/row padding/поля времени,
SI-конверсию и ковариации, защиту оригиналов, ошибки timestamp/extrinsics,
Slerp без экстраполяции через gaps, lever arm, raycasting/unknown endpoints,
seeded fault injection, знак временного сдвига, low-excitation отказ,
APE/RPE на аналитических траекториях, PCD reader и повторное использование нормализованных
данных после изменения initial pose/пола. Сверены имена используемых localizer parameters.

**Fixture poses использованы только для проверки экспортера OccupancyGrid.
Это не результат SLAM.** Ни один алгоритм не получил эталонную траекторию как собственный результат.
Синтетическая сцена не является benchmark реального робота; точность, производительность
GLIM/RKO и качество локализации по этим проверкам не установлены.

## Не выполнено

- `docker build`, C++ compile/link и upstream C++ test suites.
- Полный `make smoke-test` с ROS, GLIM/RKO/graph/localizer/KISS.
- Реальный bag, APE/RPE реального SLAM, loop acceptance и устойчивость в офисе.
- CUDA/VRAM/CPU/RAM profiling, RViz, PlotJuggler и GLIM offline viewer.
- Повторный запуск исходного legacy 2D pipeline.

В среде подготовки не было docker, ros2, colcon, cmake и доступной ROS runtime.
Проверка исходников/API не заменяет компиляцию. Патч rosbag2 Writer сверялся с Jazzy:
в write передаётся объект сообщения, не shared_ptr.

## Повторение проверок

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements-competition.txt
make test-unit
python tools/utility_smoke.py /tmp/competition-utility-new
make build
make smoke-test
```

Каталог utility_smoke должен быть новым. Команда ROS-free; её отчёт явно содержит
`executed_slam: false`. Последние две команды требуют Docker и запускают настоящий стек.
После них обязательно нужен реальный bag и ручной просмотр карты перед использованием на финале.

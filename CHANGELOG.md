# Изменения относительно mgts_doggy-main

- Сохранены исходные solver/sim, Karto-патч, исследования, legacy-тесты и benchmarks.
- Прежние README, Makefile, Dockerfile и Compose доступны с суффиксом `.legacy`.
  Исправлены ссылки legacy Make/Compose на старый Dockerfile и рекурсивные make-вызовы.
- Добавлены два самостоятельных выбора `BACKEND` с общим Python/ROS интерфейсом.
- Включены закреплённые исходники upstream, лицензии, lock-файл и небольшие интеграционные патчи.
- GLIM: CPU/GPU-конфигурации, headless offline runner, экспорт финального PCD.
- RKO: прямой offline bag из frontend, настоящий graph backend и плотный исправленный экспорт.
- Общий localizer: lifecycle, пошаговый replay, известный/ручной старт, один motion predictor,
  приём только подтверждённых поз и диагностика восстановления.
- Добавлены Bag Doctor, адаптер SI, per-point time, корректировка timestamps, строгие extrinsics,
  дисковая сортировка и phase overrides для двух записей.
- Добавлены KISS-ICP baseline, временная корреляция, воспроизводимое внесение неисправностей,
  conservative OccupancyGrid, сравнение траекторий и APE/RPE при наличии настоящего GT.
- Добавлены RViz/PlotJuggler-конфигурации, просмотр PCD/TUM/OccupancyGrid, CSV диагностики,
  unit-тесты и генератор синтетических ROS bag для полного локального smoke-test.
- Исправлен исходный whitelist `.dockerignore`, который не пропускал новые исходники в image.

Ни один сохранённый legacy benchmark не переименован в результат GLIM/RKO.

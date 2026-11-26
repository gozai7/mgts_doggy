# Источники, версии и лицензии

Основные исходники включены в `vendor/`, выбранный набор — в `THIRD_PARTY.lock.json`.
Проверка содержимого архива — `SOURCE_MANIFEST.sha256`. Версии ниже соответствуют подготовке
26 ноября 2026; последующие изменения upstream автоматически не подтягиваются.

| Компонент | Версия/commit | Лицензия/где смотреть |
|---|---|---|
| [GLIM](https://github.com/koide3/glim) | `1a197c999e4865f5cab7c9cd25474128413ebbca` | MIT, `vendor/glim/LICENSE` |
| [GLIM ROS](https://github.com/koide3/glim_ros2) | `4d4ec524ccf1b02aa09b0af2af767ecc54343798` | MIT заявлена в package.xml; отдельного LICENSE в снимке нет |
| [gtsam_points](https://github.com/koide3/gtsam_points) | `8e94654558dfbe7ddb0341de300daaf17e25ee02` | MIT, собственный LICENSE |
| [GTSAM](https://github.com/borglab/gtsam) | 4.3a0, `3ad4b4c3cb28394c9597f48fa02dad361c8450e3` | BSD, LICENSE и LICENSE.BSD; сторонние части имеют свои условия |
| [Iridescence](https://github.com/koide3/iridescence) | `1139d6e358c1432c284a06686632113d24f7c16f` | MIT, LICENSE; viewer optional |
| [lidar_slam](https://github.com/rsasaki0109/lidar_slam) | `bfe6d280e06cf6433f714a70f8e0f725a274890c` | BSD-2-Clause, LICENSE и notices вложенных компонентов |
| [RKO-LIO интеграционный форк](https://github.com/rsasaki0109/rko_lio) | `74e6c27e9533e77b564638b18923ce573086acb4` | MIT, `vendor/rko_fork/LICENSE` |
| [lidar_localization_ros2](https://github.com/rsasaki0109/lidar_localization_ros2) | `a1d56ecbde0eb65ad6e238bca124aac95a226c30` | BSD-2-Clause, LICENSE; см. third_party отдельно |
| [ndt_omp_ros2](https://github.com/rsasaki0109/ndt_omp_ros2) | `8b77fa5a6cdcad45bf35918361c892b6d94a287e` | BSD-2-Clause, LICENSE |
| [KISS-ICP](https://github.com/PRBonn/kiss-icp) | `1ffa7d7512f10bfc8b1185095011fa31184019e3` | MIT, LICENSE |

Строки GLIM/GTSAM/Iridescence относятся к архиву A, lidar_slam/RKO — к B, остальные общие.
Лицензия всего ZIP не заменяет лицензии входящих компонентов. Исходный проект команды сохранён
со своими файлами и уведомлениями; ему не приписана новая общая лицензия.

## Почему именно этот RKO

Исследован официальный [PRBonn/rko_lio](https://github.com/PRBonn/rko_lio),
снимок `500d61627a72e6a40fd18574a6d45fe836afc623`.
Для сборки B взят конкретный fork commit, на который ссылается submodule выбранного
`rsasaki0109/lidar_slam` (старый URL `lidar_slam_ros2` перенаправляется в этот репозиторий).
Это RKO-LIO с ROS-интеграцией данного стека, а не точный неизменённый PRBonn master.
Дополнительный наш патч только записывает offline-пары сообщений, не меняет ICP/IMU-математику.
Отдельный graph backend обязателен и включён в workflow.

## Изменения upstream

- `patches/glim-optimized-pcd.patch`: binary PCD после финального global save/export_points.
- `patches/rko-direct-bag-header.patch`, `rko-direct-bag-source.patch`: rosbag2 Writer
  для deskewed cloud и LiDAR-rate Odometry. API сверено с [Jazzy writer.hpp](https://github.com/ros2/rosbag2/blob/jazzy/rosbag2_cpp/include/rosbag2_cpp/writer.hpp).
- Localizer, graph optimizer и KISS исходно не переписаны; конфигурация и orchestration внешние.

Полные SHA в lock относятся к исходному снимку до этих явно перечисленных патчей.
Демонстрационные медиа/датасеты upstream исключены; выбранные executable source и лицензии сохранены.

## Зависимости сборки

Python-версии: `requirements-competition.txt`. CMake/FetchContent транзитивные зависимости
закреплены версиями в vendored CMake: в частности KISS использует Eigen 3.4.0, Sophus 1.24.6,
TBB 2022.1.0; RKO — Eigen 5.0.1, Sophus 1.24.6, TBB 2023.0.0, robin-map 1.4.0, JSON 3.12.0.
Окончательные значения смотрите в соответствующих CMake-файлах, не заменяйте их latest.
Viewer submodules отдельно закреплены в `docker/competition/viewer-submodules.lock.json`.
Их источники нужны только при WITH_VIEWER=ON и скачиваются при сборке.

APT/ROS/PCL/Boost/Eigen/G2O/CUDA и другие системные пакеты имеют собственные лицензии в
пакетах и `/usr/share/doc`. Полный список реально установленных версий создаётся сборкой,
а не объявляется выполненным заранее. CUDA устанавливается только в optional GPU stage.
ZIP не содержит CUDA binaries, готовые apt-пакеты, Docker image или чужие датасеты.

## Методические первичные источники

- [Документация GLIM](https://koide3.github.io/glim/), [установка](https://koide3.github.io/glim/installation.html), [параметры](https://koide3.github.io/glim/parameters.html).
- [RKO-LIO](https://prbonn.github.io/rko_lio/), [статья](https://arxiv.org/abs/2509.06593).
- [rosbags](https://ternaris.gitlab.io/rosbags/), [rosbag2](https://github.com/ros2/rosbag2).
- [Метрики evo](https://github.com/MichaelGrupp/evo/wiki/Metrics).
- [PlotJuggler 3.9.2](https://github.com/facontidavide/PlotJuggler/tree/3.9.2): layout подготовлен по xmlSaveState/XML loader.

Приложенные заметки/вебинары команды сохранены в исходных reports/research_notes;
предположения о будущих bag не превратились в выдуманные калибровки или результаты измерений.

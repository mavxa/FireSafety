# FireSafety

Решение для отборочного этапа соревнования `Контроль и обеспечение пожарной безопасности в лесных зонах`.

Проект закрывает две части задания:

- автономный полёт в Gazebo по трассе с 8 цветными блоками;
- подготовка ML-пайплайна для YOLO-детекции классов `костер` и `автомобиль` через Ultralytics.

В регламенте упоминается NNTrack. В этом проекте вместо него используется Ultralytics YOLO11: это тот же тип задачи `Detection (YOLO)`, экспорт в `ONNX` поддерживается, классы и структура датасета соответствуют требованиям задания.

## Структура

```text
docs/fireforest.pdf             регламент
world/worlds/clover_aruco.world кастомный Gazebo-мир
world/maps/map_7x7.txt          карта ArUco 7x7, id 0..48
scripts/installWorld.sh         установка мира, моделей и ArUco-карты
scripts/restoreWorld.sh         откат мира и ArUco-карты
scripts/main.py                 автономный полёт по цветным блокам
scripts/collectDataset.py       сбор кадров с камеры Clover
scripts/trainYolo.py            обучение YOLO11 и экспорт ONNX
```

## Требования Полёта

По PDF требуется:

- поле ArUco `7 x 7`, метки `0..48`;
- взлёт с marker `42` на высоту `1 м`;
- белая LED-индикация при взлёте;
- пролёт по маршруту через цветные блоки;
- над каждым блоком LED должен соответствовать цвету блока;
- посадка на marker `42`;
- после посадки светодиодная лента выключается.

## Установка Мира

В VM Clover:

```bash
cd ~/scripts/FireSafety
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash 2>/dev/null || source ~/catkin_ws/install/setup.bash
scripts/installWorld.sh
```

Скрипт устанавливает:

- `clover_aruco.world` из `world/worlds/`;
- модели из `world/models/`;
- карту ArUco `7x7` в `~/catkin_ws/src/clover/aruco_pose/map/map.txt`;
- ту же карту в `~/catkin_ws/src/clover/aruco_pose/map/cmit.txt`.

Проверка карты:

```bash
grep -E '^8\s|^12\s|^17\s|^38\s|^42\s|^48\s' ~/catkin_ws/src/clover/aruco_pose/map/map.txt
```

Ключевые координаты:

```text
8   -> 1.0 5.0
12  -> 5.0 5.0
17  -> 3.0 4.0
38  -> 3.0 1.0
42  -> 0.0 0.0
48  -> 6.0 0.0
```

После установки полностью перезапустите Gazebo/Clover.

## Запуск Симуляции

В первом терминале:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash 2>/dev/null || source ~/catkin_ws/install/setup.bash
roslaunch clover_simulation simulator.launch
```

Проверка сервисов:

```bash
rosservice list | grep -E 'navigate|get_telemetry|land|led/set_effect'
```

## Запуск Полёта

Во втором терминале:

```bash
cd ~/scripts/FireSafety
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash 2>/dev/null || source ~/catkin_ws/install/setup.bash
python3 scripts/main.py
```

Маршрут в `scripts/main.py`:

```text
42 takeoff -> 44 -> 38 pink -> right side blocks -> 12 violet -> 11 -> 17 red -> 9 -> 8 yellow -> left side blocks -> 38 pink -> 42 land
```

## Что Снимать На Видео

- Окно Gazebo с полем ArUco `0..48` и цветными блоками.
- Терминал с запуском `python3 scripts/main.py`.
- Взлёт с marker `42`, белая LED.
- Пролёт над цветными блоками, LED меняется под цвет блока.
- Посадка на marker `42`.
- Выключение LED после завершения.

Видео по регламенту: не более 4 минут, без монтажа и разрывов.

## ML Часть Без NNTrack

Регламент просит датасет минимум `400` изображений, классы:

```text
костер
автомобиль
```

YOLO/Ultralytics обычно удобнее с ASCII class names. Чтобы не ломать кодировки в Linux/VM, рекомендуется в `data.yaml` использовать:

```yaml
names:
  0: fire
  1: car
```

Пример лежит в `ml/data.example.yaml`.

В README и отчёте это соответствует классам `костер` и `автомобиль`.

Сбор кадров из симулятора:

```bash
python3 scripts/collectDataset.py --duration 120 --interval 0.3
```

Обучение YOLO11n:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip ultralytics
python3 scripts/trainYolo.py --data dataset/data.yaml --model yolo11n.pt --epochs 100 --batch 8 --device cpu --export-onnx
```

Результат для сдачи ML-части:

- архив датасета с `images/`, `labels/`, `data.yaml`;
- `best.onnx` после экспорта;
- папка `runs/detect/firesafety_yolo11n` как проект обучения.

## Откат Мира

```bash
scripts/restoreWorld.sh
```

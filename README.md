# 🛰️ Drone Swarm Simulation with ROS 2, Gazebo & ArduPilot

Cимулятор автономного рою дронів з підтримкою моделювання дронів, лазерної гармати та алгоритмів координації. Він розроблений у межах дипломної роботи для демонстрації можливостей управління групою дронів у віртуальному середовищі.

## 📦 Зміст

- [Особливості](#✨-особливості)
- [Архітектура](#🧱-архітектура)
- [Технології](#🧰-технології)
- [Встановлення](#🛠️-встановлення)
- [Запуск](#🚀-запуск)
- [Приклади](#📸-приклади)

---

## ✨ Особливості

- **Моделювання дронів з автопілотом** на основі ArduCopter
- **Інтеграція ROS 2 Humble з Gazebo Harmonic та ArduPilot SITL**
- **Координація рою дронів** з автоматизованим злетом, формуванням строю, атакою і поверненням
- **Віртуальна лазерна гармата**, підключена до симулятора
- **Модульна архітектура** для розширення та відладки

---

## 🧱 Архітектура

```
+------------------+        +------------------+       +------------------+
|   ROS 2 Nodes    | <----> |   MAVROS Bridge  | <---> |  ArduPilot SITL  |
+------------------+        +------------------+       +------------------+
         |                                                       ^
         v                                                       |
+-------------------+                                 +--------------------+
|   Sensor Plugins  | <------------------------------ |   Gazebo Harmonic  |
+-------------------+                                 +--------------------+
```

---

## 🧰 Технології

- **ROS 2 Humble** — основа взаємодії між компонентами
- **Gazebo Harmonic** — 3D-симулятор
- **ArduPilot SITL** — програмна симуляція автопілота
- **MAVROS** — міст між ArduPilot та ROS 2
- **Python / C++** — реалізація логіки керування роєм

---

## 🛠️ Встановлення

> ❗ Рекомендована ОС: **Ubuntu 22.04**

1. **Клонування репозиторію**\
   git clone [https://github.com/your-username/drone-swarm-simulation.git](https://ardupilot.org/dev/docs/ros2-gazebo.html)\
   cd drone-swarm-simulation

2. **Інсталювання та налаштування середовища**\
   Слідуйте [офіційній інструкції ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) для встановлення.

   Додати до ~/.bashrc:
   > export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/home/<your_username>/    
   > autonomous-uavs-simulation-analysis/ros2_ws/src/drone_swarm/models source ~/ros2_ws/
   > install/setup.bash source ~/ardu_ws/install/setup.bash

3. **При виникненні проблем спробуйте довстановити наступні залежності**\
   sudo apt update\
   sudo apt install -y python3-colcon-common-extensions \\\
   ros-humble-gazebo-ros-pkgs \\\
   ros-humble-mavros ros-humble-mavros-extras \\\
   libgazebo11-dev \\\

4. **Збірка workspace**\
   cd \~/drone-swarm-simulation\
   source /opt/ros/humble/setup.bash\
   colcon build

---

## 🚀 Запуск

> **Термінал 1(дрони)**
>
> ros2 launch drone_swarm multi_drone_launch.py

> **Термінал 2(mavros)**
>
> ros2 launch drone_swarm mavros_launch.py

> **Термінал 3(контролер рою)**
>
> ros2 run drone_swarm drone_controller.py
---

# 📸 Приклади

## Робота з контролером
- **ARM_TAKEOFF** - Озброїти та запустити всі дрони
- **LAND** - Посадити всі дрони
- **RTL** - Повернутися до точки запуску
- **quit** - Вийти з програми

### Команди формації
- **FORM_FIGURE CIRCLE [radius]** - Сформувати кругову формацію
  - Приклад: `FORM_FIGURE CIRCLE 10`
- **FORM_FIGURE LINE [spacing]** - Сформувати лінійну формацію
  - Приклад: `FORM_FIGURE LINE 5`
- **FORM_FIGURE TRIANGLE [size]** - Сформувати трикутну формацію
  - Приклад: `FORM_FIGURE TRIANGLE 8`

### Бойові команди
- **ATTACK_TARGET x y z** - Атакувати ціль за координатами
  - Приклад: `ATTACK_TARGET 100 200 50`

---

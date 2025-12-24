# VINS-Mono-Preprocessor(ROS2)

---

# 简介 Introduction

这是一个基于 VINS-Mono 改良的 ROS 2 数据预处理系统。它能够在线估计相机与 IMU 之间的时间延迟，并输出时间对齐后的图像帧与对应的 IMU 积分序列。相比于ROS1版本，ROS2版本增添了一键启动功能，无需多个终端同时操作；同时将IMU数据与时间延迟融合，时间延迟同时会在一键启动的终端打印出来。

---

# 功能 Fuctions

- ROS 2 Foxy 兼容

- 在线时间标定：自动估算并补偿 Camera-IMU 时间偏移

- 数据对齐输出：提供严格对应两帧图像之间的 IMU 数据包，以及时间戳对齐后的图像的灰度信息

- 轻量化：移除了回环检测与非必要的可视化等模块

---

# 环境 Environment

- System
    - Ubuntu 20.04
    -  ROS2 foxy
- Libraries
    - OpenCV 4.2.0
    - Ceres Solver 1.14.0
    - Eigen 3.3.7

---

# 目录结构 Project Structure
.
├── camera_model
│   ├── cmake
│   ├── CMakeLists.txt
│   ├── include
│   ├── package.xml
│   ├── readme.md
│   └── src
├── config_pkg
│   ├── CMakeLists.txt
│   ├── config
│   ├── package.xml
│   └── support_files
├── feature_tracker
│   ├── cmake
│   ├── CMakeLists.txt
│   ├── launch
│   ├── package.xml
│   └── src
└── vins_estimator
    ├── cmake
    ├── CMakeLists.txt
    ├── launch
    ├── package.xml
    └── src

对于另一个文件夹vins_interfaces，其目录结构如下：
.
├── CMakeLists.txt
├── include
│   └── vins_interfaces
├── msg
│   ├── ImuSequence.msg
│   └── VinsSyncData.msg
├── package.xml
└── src


---

# 编译指南 Build
创建ROS2工作空间，并在该空间下新建文件夹src，并将本仓库ROS2目录下的文件vins_interfaces与VINS-Mono-Preprocessor(ROS2)放在src下
接下来在新终端执行如下指令：

```bash
cd $(PATH_TO_YOUR_ROS2_WS)
colcon build
```

---

# 运行步骤 Run
我们以EuRoC数据集MH_01_easy.bag为例。由于EuRoC数据集均为ROS1格式，故还需要将其转为ROS2格式。
我们使用rosbag工具来完成转换，打开一个终端，执行如下指令：

```bash
pip install rosbags
rosbags-convert MH_01_easy.bag --dst MH_01_ros2
```

完成转换后，我们执行如下操作：
打开一个新的终端，
```bash
cd $(PATH_TO_YOUR_ROS2_WS)
source install/setup.bash
ros2 launch vins_estimator run_preprocessing.launch.py
```
接下来打开另一个终端对数据集进行播放，

```bash
ros2 bag play $(PATH_TO_YOUR_DATASET)/MH_01_ros2 # 格式转换后名字可能变化，请根据自己新得到的bag名称进行修改
```
最后我们再打开一个终端对话题的输出进行验证，

```bash
cd $(PATH_TO_YOUR_ROS2_WS)
source install/setup.bash
ros2 topic echo /vins/sync_data # Ctrl+C中止，再将话题名换成/aligned_grayscale_image即可查看灰度信息是否输出
```


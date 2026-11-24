# Hunter130 实机部署

适用于已安装 **Ubuntu 24.04 和 ROS 2 Jazzy Desktop** 的 amd64 机器人电脑。仓库自带 AMP、Mimic 模型和 x86_64 版 ONNX Runtime 1.28.1。使用 ARM64 时，需要换用对应架构的 Encos 安装包和 ONNX Runtime。

首次部署按下面的顺序安装依赖、配置硬件并编译。之后日常使用可直接从[启动实机](#启动实机)开始。所有命令均在仓库根目录的 Bash 终端执行。

## 首次部署

### 1. 安装工作空间依赖

在 ROS 2 桌面版的基础上，补齐编译工具、ros2_control、Xacro 和遥控器串口驱动。

```bash
sudo apt update
sudo apt install -y \
  build-essential python3-colcon-common-extensions nlohmann-json3-dev \
  ros-jazzy-controller-manager ros-jazzy-joint-state-broadcaster ros-jazzy-ros2controlcli \
  ros-jazzy-xacro ros-jazzy-serial-driver ros-jazzy-asio-cmake-module
```

### 2. 安装 Encos 驱动、关节 SDK 和命令行工具

三个 amd64 安装包均依赖 `libc6` ≥ 2.28、`libstdc++6` ≥ 8 和 GCC 运行库（Ubuntu 24.04 使用 `libgcc-s1`）。

先安装系统依赖。

```bash
sudo apt install -y libc6 libstdc++6 libgcc-s1 libeigen3-dev libcap2-bin
```

| 安装包                         | 版本  | 额外依赖                                                |
| ------------------------------ | ----- | ------------------------------------------------------- |
| `libencosdriver` · 硬件驱动 | 4.3.1 | `libcap2-bin` |
| `jointsdk` · 关节 SDK       | 1.5.9 | `libeigen3-dev` ≥ 3.3.4、`libencosdriver` ≥ 4.0.0 |
| `emcli` · 命令行工具        | 2.1.4 | `libcap2-bin` |

当前新版安装包位于 `/home/milkyway/下载`。将三个本地包一起交给 APT，APT 会按依赖顺序安装驱动和关节 SDK。

硬件插件已经按这组 SDK 适配了关节的 `PVTControlMode`、抱闸生命周期、JSON 适配器扩展和反馈生命周期接口。

```bash
encos_dir="/home/milkyway/下载"
sudo apt install -y \
  "$encos_dir/libencosdriver_4.3.1_linux-glibc2.28_amd64.deb" \
  "$encos_dir/jointsdk_1.5.9_linux-glibc2.28_amd64.deb" \
  "$encos_dir/emcli_2.1.4_linux_amd64.deb"
```

检查安装结果。

```bash
dpkg-query -W -f='${Package} ${Version} ${db:Status-Status}\n' libencosdriver jointsdk emcli
```

### 3. 核对实机配置

| 项目          | 核对内容                                                                                                                                   | 文件                                                                                                                                                                          |
| ------------- | ------------------------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| EtherCAT 网卡 | 用 `ip -br link` 找到连接机器人的网卡。将关节的 `interface_name_0` 和 IMU 的 `interface_name` 设为实际接口名，默认均为 `enp86s0`。 | [ros2_control.xacro](src/ec_description/urdf/ros2_control.xacro)                                                                                                               |
| 关节映射      | 对照实机核对总线、电机 ID、转动方向和标定参数。                                                                                            | [control.json](src/ec_description/config/control.json)                                                                                                                         |
| 日志目录      | 确认运行账号可写入 `/home/milkyway/log/hunter130`。更换目录时，同步调整硬件的 `log_path` 和各策略控制器的 `log_dir`。                | [ros2_control.xacro](src/ec_description/urdf/ros2_control.xacro)、[amp.yaml](src/ec_controller/config/policy/amp.yaml)、[mimic.yaml](src/ec_controller/config/policy/mimic.yaml) |

### 4. 编译工作空间

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

编译完成后会生成 `install/setup.bash`。每个用于启动或查看状态的新终端都要加载它。

## 启动实机

### 终端 1 · 启动控制器

启动完成后，机器人进入 Default 阻尼模式。

```bash
source install/setup.bash
ros2 launch ec_controller real.launch.py
```

### 终端 2 · 启动遥控器

将 `/dev/ttyUSB0` 换成 iBUS 接收机的实际串口，默认波特率为 `115200`。

```bash
source install/setup.bash
ros2 run ec_radio radio_node --ros-args -p serial_port:=/dev/ttyUSB0 -p baud_rate:=115200
```

需要确认启动状态时，在另一个已加载工作空间的终端运行 `ros2 control list_controllers`。首次启动完成、尚未切换模式时，`joint_state_broadcaster` 和 `default_controller` 应显示 `active`，其余四个控制器应显示 `inactive`。

## 遥控器操作

每次只选一个模式通道。从 Default 依次切到 **CH4 站姿 → CH5 行走**。在 CH5 将摇杆回中、站稳后，再选 **CH6 跳舞** 或 **CH7 跟随**。

| 通道              | 模式                | 操作                                                                                                                          |
| ----------------- | ------------------- | ----------------------------------------------------------------------------------------------------------------------------- |
| CH4–CH7 全部松开 | Default 阻尼        | 从任一模式直接回到阻尼，平滑回站期间也可使用。                                                                                |
| CH4               | Stand 站姿          | 保持静态准备姿态，此时机体需要外部支撑。                                                                                      |
| CH5               | AMP Walk 平衡与行走 | CH2 控制前后，CH3 控制左右，CH0 控制转向。摇杆回中后保持平衡站立。                                                            |
| CH6               | Mimic 跳舞          | 默认播放一次 [Disco](src/ec_controller/config/motions/disco.csv)，结束后站立。要重播，先切回 CH5，待 AMP Walk 接管后再选 CH6。 |
| CH7               | Quest 实时跟随      | 跟随外部 Quest 桥接程序的动作。暂停跟踪后仍在 Quest 模式。                                                                    |

**退出跳舞或跟随**时，选 CH5 并保持，等 AMP Walk 接管后再切换模式。

**使用 Quest** 时，桥接电脑和机器人需同步时钟并使用相同的 `ROS_DOMAIN_ID`。桥接程序向 `/quest/reference` 发布参考动作。松开双侧握键，站直后按 A 校准，再握住双侧握键。用 `ros2 topic echo /quest/state` 确认状态为 `tracking` 后开始动作。

## 更换舞曲与调整策略

临时更换 CH6 舞曲时，在终端 1 使用下面的启动命令，并传入与 Mimic 关节顺序一致的 50 Hz 动作 CSV。要更改以后每次启动的默认舞曲，修改 [mimic.yaml](src/ec_controller/config/policy/mimic.yaml) 中的 `mimic_controller.ros__parameters.motion_path`。

```bash
ros2 launch ec_controller real.launch.py motion:="/绝对路径/动作.csv"
```

模型和动作的相对路径均以安装后的 `ec_controller` 共享目录为基准。

| 配置文件                                                     | 用途                                                                                                                                 |
| ------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------ |
| [controllers.yaml](src/ec_controller/config/controllers.yaml) | 控制器注册、更新频率、初始模式和模式切换表。                                                                                         |
| [default.yaml](src/ec_controller/config/policy/default.yaml)  | Default 的关节目标位置和阻尼增益。                                                                                                   |
| [stand.yaml](src/ec_controller/config/policy/stand.yaml)      | Stand 的准备姿态和 PD 增益。                                                                                                         |
| [amp.yaml](src/ec_controller/config/policy/amp.yaml)          | `walk_controller` 选择 `amp.onnx` 并设置 IMU 偏移。`amp_model` 保存策略频率、历史帧数和关节参数。                              |
| [mimic.yaml](src/ec_controller/config/policy/mimic.yaml)      | `mimic_controller` 和 `quest_controller` 分别设置动作播放与实时跟随的过渡时间。两者共用 `mimic.onnx` 和 `mimic_model` 参数。 |

使用随仓库提供的关节插件时，Mimic 和 Quest 的 `activation_duration` 均保持为 `0.0`。外部 Quest 桥接程序从 `mimic.yaml` 的 `mimic_model.ros__parameters` 读取模型参数。

## 硬件插件离线测试（可选）

关节和 IMU 插件提供离线测试，可在连接机器人前运行。

```bash
sudo apt install -y ros-jazzy-ament-cmake-gtest ros-jazzy-ament-cmake-pytest
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ec_joint_hardware ec_imu_hardware --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select ec_joint_hardware ec_imu_hardware --return-code-on-test-failure
colcon test-result --verbose
```

# SimEnv 启动说明

## 环境说明


| 项目          | 说明                                                  |
| ----------- | --------------------------------------------------- |
| 镜像          | `simenv-ros:latest`                                 |
| 项目挂载        | 宿主机 `~/SimEnv` → 容器 `/workspace`                    |
| libtorch 挂载 | 宿主机 `~/下载/libtorch` → 容器 `/libtorch`                |
| Docker      | **snap 安装**，必须用 `--runtime=nvidia`，不支持 `--gpus all` |


> ROS 命令只能在**容器内**执行。

---

## 1. 宿主机

```bash
xhost +local:docker
sudo docker start simenv
sudo docker exec -it simenv bash
```

容器不存在时重建：

```bash
sudo docker rm -f simenv 2>/dev/null

sudo docker run -it \
  --name simenv \
  --runtime=nvidia \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute,display \
  -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
  -e __NV_PRIME_RENDER_OFFLOAD=1 \
  -e LIBGL_DRI3_DISABLE=1 \
  -e QT_X11_NO_MITSHM=1 \
  -v /home/fengtianchao/_SimNav:/workspace \
  -v ~/下载/libtorch:/libtorch \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --network host \
  -w /workspace \
  simenv-ros:latest \
  bash
```

> `xhost +local:docker` 和 `DISPLAY` 挂载用于 **RViz** 显示。`GUI=false` 只关闭 Gazebo 3D 窗口，物理仿真（`gzserver`）和 ROS 话题照常运行。

---



## 2. 容器内 — 环境变量

每次进入容器后执行：

```bash
export LD_LIBRARY_PATH=/var/lib/snapd/hostfs/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __NV_PRIME_RENDER_OFFLOAD=1
export LIBGL_DRI3_DISABLE=1
export QT_X11_NO_MITSHM=1
export GAZEBO_PLUGIN_PATH=/opt/ros/noetic/lib

source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash
```

---



## 3. 新建容器时 — 安装依赖

```bash
apt update
apt install -y \
  ros-noetic-gazebo-ros-control \
  ros-noetic-controller-manager \
  ros-noetic-joint-state-controller \
  ros-noetic-effort-controllers \
  ros-noetic-catkin
```

---



## 4. Planner 接入与编译

planner 源码在 `planner/`，用**相对路径**链入 `src/`：

```bash
cd /workspace/src
for d in exploration_planner local_planner terrain_analysis terrain_analysis_ext \
         sensor_scan_generation semantic_mapping waypoint_rviz_plugin; do
  rm -f "$d"
  ln -sf "../planner/$d" "$d"
done
```

首次编译（需已放置 `planner/exploration_planner/or-tools/` 和 `include/nlohmann/json.hpp`）：

```bash
cd /workspace
catkin_make --force-cmake -j2 -DLIBTORCH_PATH=/libtorch
source /workspace/devel/setup.bash
```

代码改动后重新编译：

```bash
cd /workspace
catkin_make -j2 -DLIBTORCH_PATH=/libtorch
source /workspace/devel/setup.bash
```

---



## 5. 启动仿真 + Planner

默认采用 **Gazebo 无 GUI + RViz 观测** 方案：Gazebo 只跑后端仿真，可视化交给 TARE 的 RViz。以下默认值已写入对应文件，无需每次指定：


| 默认值来源                        | 修改内容                                                   |
| ---------------------------- | ------------------------------------------------------ |
| `auto.sh`                    | `GUI=false`、`ENABLE_SENSOR_DATA=0`、`ENABLE_LIVOX=true` |
| `system_indoor_base.launch`  | `rviz` 默认 `false`（避免双 RViz）                            |
| `tare_planner_indoor.launch` | `rviz` 默认 `true`（唯一 RViz 窗口）                           |



| 终端  | 作用                             |
| --- | ------------------------------ |
| 1   | 仿真 + 控制器（`auto.sh`）            |
| 2   | 局部规划，不开 RViz（`local_planner`）  |
| 3   | TARE 探索 + RViz（`tare_planner`） |




### 终端 1 — 仿真

```bash
./auto.sh
pkill -f gzserver; pkill -f gzclient
```

默认已关闭 Gazebo GUI，启用 Livox 雷达和点云转换。如需覆盖可在命令前添加对应环境变量（例如 `GUI=true ./auto.sh` 重新开启 GUI）。

出现 `[INFO] Gazebo joint state feedback is ready.` 后：**按** `2` **站立，按** `6` **切** `/cmd_vel` **模式**（须看到 `[INFO] Entered RL /cmd_vel mode.`）。

### 终端 2 — local_planner

```bash
roslaunch local_planner system_indoor_base.launch
```

不启动 RViz，避免与终端 3 重复。如需单独调试局部规划，可加 `rviz:=true`。

### 终端 3 — TARE 探索

```bash
roslaunch tare_planner tare_planner_indoor.launch
```

自动启动 RViz（`tare_planner_indoor.rviz`），显示探索子空间、frontier、全局路径、点云等。进门与控门由 TARE Phase1 自动处理。

---



## 6. Rosbag 录制（RealSense RGB + 深度 + 位姿）

录制前需开启 RealSense（例如 `ENABLE_SENSOR_DATA=0 ENABLE_REALSENSE=1 ./auto.sh`）。在容器内、已 `source` ROS 和 devel 后执行：

```bash
mkdir -p /workspace/bags

rosbag record -O /workspace/bags/robot_sensors_$(date +%Y%m%d_%H%M%S) \
  /real_sense/rgb/image_raw \
  /real_sense/rgb/camera_info \
  /real_sense/depth/image_raw \
  /real_sense/depth/camera_info \
  /Odometry_gazebo \
  /clock
```

用 **Ctrl+C** 结束录制，bag 保存在 `/workspace/bags/`（宿主机 `~/_SimNav/bags/`）。


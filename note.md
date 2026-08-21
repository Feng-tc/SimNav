# SimEnv 启动说明

## 环境说明


| 项目          | 说明                                                                               |
| ----------- | -------------------------------------------------------------------------------- |
| 镜像          | `simenv-ros:latest`                                                              |
| 项目挂载        | 宿主机 `~/_SimNav` → 容器 `/workspace`                                                |
| libtorch 挂载 | 宿主机 `/home/fengtianchao/下载/libtorch` → 容器 `/libtorch`（**须 CUDA 版 cu118**，见 §3.1） |
| GPU         | RTX 3060 Laptop，算力 **8.6**；宿主机只需 NVIDIA 驱动，无需装 CUDA Toolkit                      |
| Docker      | **snap 安装**，必须用 `--runtime=nvidia`，不支持 `--gpus all`                              |


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
  -v /home/fengtianchao/下载/libtorch:/libtorch \
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
export LD_LIBRARY_PATH=/libtorch/lib:/usr/local/cuda-11.8/lib64:/var/lib/snapd/hostfs/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export CUDA_HOME=/usr/local/cuda-11.8
export PATH=/usr/local/cuda-11.8/bin:$PATH
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
  ros-noetic-catkin \
  wget
```

### 3.1 junior_ctrl RL — LibTorch GPU 与 CUDA 11.8

`junior_ctrl` 的 RL 策略默认在检测到 CUDA 时使用 GPU（见 `State_RL_test.cpp`）。需满足：

1. **LibTorch 必须是 CUDA 版**（文件名含 `cu118`，非 `cpu`）
2. **容器内安装 CUDA Toolkit 11.8**（编译用；运行时主要用 libtorch 自带库）
3. 换 libtorch 后 **重启容器**（bind mount 绑的是 inode，运行中 `mv` 替换目录不会自动更新）

#### 宿主机 — 下载并放置 LibTorch

从 [pytorch.org/get-started/locally](https://pytorch.org/get-started/locally/) 选 LibTorch + CUDA 11.8，或使用：

```bash
cd ~/下载
wget https://download.pytorch.org/libtorch/cu118/libtorch-cxx11-abi-shared-with-deps-2.1.0%2Bcu118.zip
mv libtorch libtorch_cpu_backup   # 若有旧 CPU 版
unzip libtorch-cxx11-abi-shared-with-deps-2.1.0+cu118.zip
ls libtorch/lib/libtorch_cuda.so  # 必须存在
```

替换 libtorch 后重启容器：

```bash
sudo docker stop simenv && sudo docker start simenv
```

#### 容器内 — 安装 CUDA Toolkit 11.8（一次性）

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb
dpkg -i cuda-keyring_1.1-1_all.deb
apt update
apt install -y cuda-toolkit-11-8
ln -sf /usr/local/cuda-11.8 /usr/local/cuda
/usr/local/cuda-11.8/bin/nvcc --version   # 应显示 11.8
```

> 勿用 `apt install nvidia-cuda-toolkit`（Ubuntu 源为 CUDA 10.1，不满足 libtorch cu118 要求）。

#### 验证 libtorch 与链接

```bash
ls /libtorch/lib/libtorch_cuda.so
ldd /workspace/devel/lib/unitree_guide/junior_ctrl | grep -E 'torch_cuda|c10_cuda'
```

启动 `junior_ctrl` 后日志应出现 `cuda::is_available():1`。

---

## 4. Planner 接入与编译

planner 源码在 `planner/`，用**相对路径**链入 `src/`：

```bash
cd /workspace/src
for d in exploration_planner ease_planner local_planner terrain_analysis terrain_analysis_ext \
         sensor_scan_generation semantic_mapping waypoint_rviz_plugin; do
  rm -f "$d"
  ln -sf "../planner/$d" "$d"
done
```

编译

```bash
cd /workspace
catkin_make --force-cmake -j2 -DLIBTORCH_PATH=/libtorch
source /workspace/devel/setup.bash
```

---

## 5. 启动仿真 + Planner

默认采用 **Gazebo 无 GUI + RViz 观测** 方案：Gazebo 只跑后端仿真，可视化交给终端 3 的 RViz（TARE 或 ease）。以下默认值已写入对应文件，无需每次指定：


| 默认值来源                        | 修改内容                                                   |
| ---------------------------- | ------------------------------------------------------ |
| `auto.sh`                    | `GUI=false`、`ENABLE_SENSOR_DATA=0`、`ENABLE_LIVOX=true` |
| `system_indoor_base.launch`  | `rviz` 默认 `false`（避免双 RViz）                            |
| `tare_planner_indoor.launch` / `ease_planner_indoor.launch` | `rviz` 默认 `true`（唯一 RViz 窗口） |



| 终端  | 作用                             |
| --- | ------------------------------ |
| 1   | 仿真 + 控制器（`auto.sh`）            |
| 2   | 局部规划，不开 RViz（`local_planner`）  |
| 3   | 探索规划 + RViz（`tare_planner` 或 `ease_planner`，二选一） |


### 终端 1 — 仿真

```bash
pkill -f gzserver; pkill -f gzclient; pkill -f junior_ctrl
./auto.sh
```

默认已关闭 Gazebo GUI，启用 Livox 雷达和点云转换。如需覆盖可在命令前添加对应环境变量（例如 `GUI=true ./auto.sh` 重新开启 GUI）。

出现 `[INFO] Gazebo joint state feedback is ready.` 后：**按** `2` **站立，按** `6` **切** `/cmd_vel` **模式**（须看到 `[INFO] Entered RL /cmd_vel mode.`）。

### 终端 2 — local_planner

每次重新测试前先清理残留进程，避免旧地形图污染新一轮测试：

```bash
pkill -f "localPlanner\|pathFollower" 2>/dev/null || true
roslaunch local_planner system_indoor_base.launch
```

不启动 RViz，避免与终端 3 重复。如需单独调试局部规划，可加 `rviz:=true`。

### 终端 3 — 探索规划（二选一）

同一时间只启动其中一个。

TARE 智能探索：

```bash
pkill -f "tare_planner\|sensor_coverage_planner\|ease_planner" 2>/dev/null || true
roslaunch tare_planner tare_planner_indoor.launch
```

自动启动 RViz（`tare_planner_indoor.rviz`），显示探索子空间、frontier、全局路径、点云等。进门与控门由 TARE Phase1 自动处理。

手动航点巡游（`ease_planner`，目标点写在 `planner/ease_planner/config/indoor.yaml`）：

```bash
pkill -f "tare_planner\|sensor_coverage_planner\|ease_planner" 2>/dev/null || true
roslaunch ease_planner ease_planner_indoor.launch
```

自动启动 RViz（`ease_planner_indoor.rviz`）。Phase1 进门逻辑与 TARE 相同，随后按配置航点逐层巡游并乘电梯上 2F/3F。

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

### 手动开门（调试用）

```bash
# 开启主入口门（door_id 对应 indoor.yaml 中 kMainEntranceDoorId，默认 "main_entrance"）
rosservice call /set_door_state "{door_id: 'main_entrance', open: true}"
```


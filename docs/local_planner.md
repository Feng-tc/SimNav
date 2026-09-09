# Local Planner 原理说明

本文档描述 SimNav 中 `local_planner` 的核心算法环节，对应 `localPlanner`（路径选择）与 `pathFollower`（路径跟踪）两个节点。完整实现见 `src/local_planner/`。

---

## 3. 坐标系与状态估计

### 3.1 车辆位置补偿

设里程计位姿为 $(x_o, y_o, \psi)$，传感器相对车辆偏移为 $(o_x, o_y)$，则规划使用的车辆参考点为：

$$
\begin{aligned}
x_v &= x_o - \cos\psi \cdot o_x + \sin\psi \cdot o_y \\
y_v &= y_o - \sin\psi \cdot o_x - \cos\psi \cdot o_y
\end{aligned}
$$

### 3.2 世界坐标到车体系

对任意世界点 $\mathbf{p}_w = (p_x, p_y, p_z)$，变换到 `vehicle` 坐标系：

$$
\begin{bmatrix} x' \\ y' \end{bmatrix}
=
\begin{bmatrix}
\cos\psi & \sin\psi \\
-\sin\psi & \cos\psi
\end{bmatrix}
\begin{bmatrix}
p_x - x_v \\
p_y - y_v
\end{bmatrix}
$$

后续碰撞检测与路径评分均在车体系下进行。

---

## 4. 障碍物感知

### 4.1 输入源

| 模式 | 输入话题 | 说明 |
|---|---|---|
| 原始点云 | `/livox/Pointcloud2` | 直接体素滤波 |
| 地形分析（当前默认） | `/terrain_map` | 使用 `terrain_analysis` 输出的可通行/障碍高度图 |

当前 launch 配置：

$$
\text{useTerrainAnalysis} = \text{true}
$$

### 4.2 局部裁剪

仅保留车辆周围 $R_a = \texttt{adjacentRange}$ 内的点：

$$
\sqrt{(p_x - x_v)^2 + (p_y - y_v)^2} < R_a
$$

默认 $R_a = 4.25\,\text{m}$。

### 4.3 高度过滤

在车体系中，进一步保留：

$$
z_{\min} < z' < z_{\max}
$$

默认 $z_{\min}=-0.5\,\text{m}$，$z_{\max}=0.25\,\text{m}$。

### 4.4 附加约束

还可融合：

- `/navigation_boundary`：导航边界，离散采样后作为高代价障碍点（intensity=100）
- `/added_obstacles`：人工添加障碍（intensity=200）

---

## 5. 目标方向与规划范围

### 5.1 自主模式目标

在 `autonomyMode` 下，目标来自 `/way_point`，其在车体系中的位置为：

$$
\begin{aligned}
g_x' &= (g_x - x_v)\cos\psi + (g_y - y_v)\sin\psi \\
g_y' &= -(g_x - x_v)\sin\psi + (g_y - y_v)\cos\psi
\end{aligned}
$$

目标距离与方向：

$$
d_g = \sqrt{g_x'^2 + g_y'^2}, \quad
\psi_g = \mathrm{atan2}(g_y', g_x') \cdot \frac{180}{\pi}
$$

该方向作为期望行驶方向 $\psi_{\text{joy}} = \psi_g$。

### 5.2 跨楼层保护

若启用 `preventLowerFloorPlanning`，且目标高度明显低于当前楼层：

$$
g_z < z_v - \Delta z_{\max}
$$

则强制 $d_g = 0$，避免向低层“穿楼”规划。默认 $\Delta z_{\max} = 0.10\,\text{m}$。

### 5.3 动态规划范围

路径有效长度与缩放随速度变化：

$$
R_p =
\begin{cases}
R_a \cdot v_{\text{norm}}, & \text{pathRangeBySpeed=true} \\
R_a, & \text{otherwise}
\end{cases}
$$

$$
s =
\begin{cases}
s_0 \cdot v_{\text{norm}}, & \text{pathScaleBySpeed=true} \\
s_0, & \text{otherwise}
\end{cases}
$$

其中 $v_{\text{norm}} \in [0,1]$ 为归一化速度指令，$s_0=\texttt{pathScale}$。

若找不到可行路径，则逐步减小 $s$ 或 $R_p$ 重试。

---

## 6. 路径碰撞检测

### 6.1 旋转离散

对每个障碍点 $\mathbf{p}=(x,y,h)$，考虑 36 个候选旋转角：

$$
\psi_r = 10r - 180^\circ, \quad r = 0,1,\ldots,35
$$

先将点按路径缩放逆变换，再旋转：

$$
\begin{bmatrix} x_2 \\ y_2 \end{bmatrix}
=
\begin{bmatrix}
\cos\psi_r & \sin\psi_r \\
-\sin\psi_r & \cos\psi_r
\end{bmatrix}
\begin{bmatrix}
x/s \\
y/s
\end{bmatrix}
$$

### 6.2 扇形栅格索引

为适配路径库几何，使用非均匀栅格映射：

$$
\text{scaleY} = \frac{x_2}{L_x} + \frac{r_s}{L_y}\left(1 - \frac{x_2}{L_x}\right)
$$

$$
i_x = \left\lfloor \frac{L_x + \delta/2 - x_2}{\delta} \right\rfloor, \quad
i_y = \left\lfloor \frac{L_y + \delta/2 - y_2/\text{scaleY}}{\delta} \right\rfloor
$$

其中 $L_x=\texttt{gridVoxelOffsetX}=3.2$，$L_y=\texttt{gridVoxelOffsetY}=4.5$，$\delta=\texttt{gridVoxelSize}=0.02$。

通过预计算的 `correspondences[ind]`，一个栅格可对应多条候选 path。

### 6.3 路径阻塞计数

对组合索引 $k = 36 \cdot \text{pathID} + r$，维护阻塞计数 $N_k$：

- 若 $h > h_{\text{obs}}$（硬障碍）：

$$
N_k \leftarrow N_k + 1
$$

- 若使用地形代价模式，则记录高度惩罚 $P_k = \max(P_k, h)$

一条路径被认为 **可通行**，当且仅当：

$$
N_k < N_{\text{th}}
$$

默认 $N_{\text{th}} = \texttt{pointPerPathThre} = 2$。

### 6.4 方向约束

仅保留与目标方向接近的旋转：

$$
\Delta\psi = \min\big(|\psi_g - \psi_r|,\ 360^\circ - |\psi_g - \psi_r|\big) \le \psi_{\text{dirThre}}
$$

默认 $\psi_{\text{dirThre}} = 100^\circ$。

---

## 7. 路径评分与选择

### 7.1 单路径得分

对未阻塞路径，定义：

$$
\eta = 1 - \frac{P_k}{h_{\text{cost}}}, \quad
\eta \leftarrow \max(\eta, \eta_{\min})
$$

$$
\Delta\psi_{\text{end}} =
\min\big(
|\psi_g - \psi_{\text{end}} - \psi_r|,\ 
360^\circ - |\psi_g - \psi_{\text{end}} - \psi_r|
\big)
$$

旋转权重：

$$
w_r =
\begin{cases}
| |r-9| + 1 |, & r < 18 \\
| |r-27| + 1 |, & r \ge 18
\end{cases}
$$

最终得分：

$$
S_k =
\left(1 - \sqrt[4]{\alpha \cdot \Delta\psi_{\text{end}}}\right)
\cdot w_r^4 \cdot \eta
$$

其中 $\alpha = \texttt{dirWeight}$（默认 0.1）。

### 7.2 组级聚合

每条 path 属于某个 group $g$，组得分：

$$
G_{r,g} = \sum_{k \in \mathcal{P}_{r,g}} S_k
$$

其中 $\mathcal{P}_{r,g}$ 为旋转 $r$、组 $g$ 下所有可行 path。

### 7.3 最优选择

选择得分最大的 $(r^*, g^*)$：

$$
(r^*, g^*) = \arg\max_{r,g} G_{r,g}
$$

并附加约束：

1. 起始路径最低高度：$z_{\min,g} \ge -\Delta z_{\max}$
2. 可选旋转碰撞约束（`checkRotObstacle`）

### 7.4 输出路径

将选中 group 的 `startPaths[g*]` 按旋转 $\psi_{r^*}$ 和缩放 $s$ 变换：

$$
\begin{bmatrix} x_o \\ y_o \end{bmatrix}
=
s
\begin{bmatrix}
\cos\psi_{r^*} & -\sin\psi_{r^*} \\
\sin\psi_{r^*} & \cos\psi_{r^*}
\end{bmatrix}
\begin{bmatrix} x \\ y \end{bmatrix}
$$

并裁剪到：

$$
\|\mathbf{p}\| \le \min\left(\frac{R_p}{s}, \frac{d_g}{s}\right)
$$

发布为 `/path`，坐标系 `vehicle`。

若完全找不到路径，则发布单点零路径 $(0,0,0)$，等价于停车。

---

## 8. Path Follower 路径跟踪

`pathFollower` 订阅 `/path`，输出 `/cmd_vel`。

### 8.1 路径坐标系

收到新路径时，记录当前车辆位姿 $(x_r, y_r, \psi_r)$ 作为路径参考系。运行时车辆相对位置：

$$
\begin{aligned}
x_{\text{rel}} &= \cos\psi_r (x - x_r) + \sin\psi_r (y - y_r) \\
y_{\text{rel}} &= -\sin\psi_r (x - x_r) + \cos\psi_r (y - y_r)
\end{aligned}
$$

### 8.2 前视距离选择

沿路径寻找距当前位置大于前视距离 $L_d$ 的最近点：

$$
\|\mathbf{p}_i - \mathbf{p}_{\text{rel}}\| \ge L_d
$$

默认 $L_d = 0.8\,\text{m}$（launch 配置）。

### 8.3 航向误差

前视点方向：

$$
\psi_p = \mathrm{atan2}(y_i - y_{\text{rel}},\ x_i - x_{\text{rel}})
$$

航向误差：

$$
e_\psi = \mathrm{wrap}(\psi - \psi_r - \psi_p)
$$

其中 $\mathrm{wrap}(\cdot)$ 归一化到 $(-\pi, \pi]$。

### 8.4 双向驱动

若 `twoWayDrive=true`，当 $|e_\psi| > \pi/2$ 时可切换为倒车：

$$
e_\psi \leftarrow e_\psi + \pi, \quad v \leftarrow -v
$$

SimNav 当前默认 `twoWayDrive=false`（仅前进）。

### 8.5 角速度控制

低速时使用更大转向增益：

$$
\omega =
\begin{cases}
-K_{\text{stop}} e_\psi, & |v| < 2a_{\max}/100 \\
-K_{\text{yaw}} e_\psi, & \text{otherwise}
\end{cases}
$$

并限幅：

$$
|\omega| \le \omega_{\max}
$$

默认 $K_{\text{yaw}}=4.5$，$K_{\text{stop}}=4.5$，$\omega_{\max}=45^\circ/\text{s}$。

### 8.6 线速度控制

期望速度：

$$
v_{\text{cmd}} = v_{\max} \cdot v_{\text{norm}}
$$

接近终点时减速：

$$
v_{\text{cmd}} \leftarrow v_{\text{cmd}} \cdot \min\left(1,\ \frac{d_{\text{end}}}{D_{\text{slow}}}\right)
$$

默认 $D_{\text{slow}} = 1.2\,\text{m}$。

仅当 $|e_\psi| < e_{\psi,\text{th}}$ 且 $d > d_{\text{stop}}$ 时加速，否则减速：

$$
|v| \leftarrow |v| \pm a_{\max}/100
$$

默认 $e_{\psi,\text{th}}=0.35\,\text{rad}$，$d_{\text{stop}}=0.35\,\text{m}$，$a_{\max}=1.0\,\text{m/s}^2$。

### 8.7 安全机制

- `/stop`：`safetyStop>=1` 停线速度，`>=2` 停角速度
- 大倾角保护：`useInclToStop=true` 时，若 $|\text{roll}|,|\text{pitch}| > 45^\circ$ 强制停车
- 到达目标：`pathSize <= 1` 或 `noRotAtGoal=true` 且距离终点很近时，$v=0$，$\omega=0$

---

## 12. 小结

当前 `local_planner` 的核心思想可概括为：

$$
\boxed{
\text{最优局部路径}
=
\arg\max_{r,g}
\sum_{\text{path } k \in (r,g)}
\underbrace{\left(1 - \sqrt[4]{\alpha \Delta\psi_{\text{end}}}\right) w_r^4 \eta_k}_{\text{方向+旋转偏好}}
\cdot
\underbrace{\mathbb{1}[N_k < N_{\text{th}}]}_{\text{无碰撞}}
}
$$

随后由 `pathFollower` 通过前视点跟踪将其转化为：

$$
\boxed{
\omega = -K e_\psi, \quad
v = f(d_{\text{end}}, e_\psi, v_{\text{norm}})
}
$$

形成完整的 **“感知 → 选路 → 跟踪 → 控制”** 局部导航链路。

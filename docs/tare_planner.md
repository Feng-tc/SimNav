# TARE Planner 原理说明（节选）

本文档描述 SimNav 中 `tare_planner` 的核心算法环节，对应 `tare_planner_node`（`SensorCoveragePlanner3D`）与 `room_segmentation_node`。完整实现见 `planner/exploration_planner/`，配置见 `config/indoor.yaml`。

---

## 3. 感知与地图表示

### 3.1 输入

- 注册点云：`/livox/Pointcloud2`
- 里程计：`/Odometry_gazebo`
- 地形图：`/terrain_map`、`/terrain_map_ext`（碰撞/高度）
- 房间：`/room_nodes_list`、`/room_mask`（Image, CV_32S）
- 边界：`/navigation_boundary`、`/current_room_boundary`

### 3.2 Keypose 云栈

每次扫描更新 keypose 云，经体素滤波（叶大小 $\delta_k = 0.2\,\text{m}$）后送入 `PlanningEnv`：

$$
\mathcal{P}_{\text{key}} = \text{VoxelFilter}\left(\bigcup_{i=0}^{N-1} \mathcal{P}_i\right), \quad N = 5
$$

### 3.3 滚动占用栅格

分辨率 $(0.3, 0.3, 0.3)\,\text{m}$，随机器人移动滚动，用于 surface / frontier 提取。

### 3.4 Frontier

启用 `kUseFrontier: true` 时，从未知–已知边界聚类得到 frontier 点集 $\mathcal{F}$。每个视点 $v$ 可覆盖 frontier 子集，记为 $\mathcal{F}(v)$。

---

## 4. 视点管理（ViewPointManager）

### 4.1 局部规划视界

在机器人周围维护 $50 \times 50 \times 1$ 视点栅格，分辨率 $0.4\,\text{m}$，有效局部视界约：

$$
L_x = L_y \approx 50 \times 0.4 / 5 = 4\,\text{m}
$$

### 4.2 视点覆盖评分

对候选视点 $v$，定义：

$$
s(v) = \left|\mathcal{U}(v)\right|, \quad f(v) = \left|\mathcal{F}(v)\right|
$$

其中 $\mathcal{U}(v)$ 为 $v$ 可见且尚未覆盖的表面点集。

传感器模型参数：

- 量程：$R_s = 10\,\text{m}$
- 遮挡阈值：$\tau_{\text{occ}} = 0.3$
- 覆盖膨胀半径：$r_d = 0.5\,\text{m}$

### 4.3 房间约束

当 `current_room_id_ > 0` 时，视点按 `room_mask` 过滤，仅保留当前房间内的候选视点用于探索与 TSP。

---

## 5. 全局子空间（GridWorld）

### 5.1 空间划分

全局划分为 $121 \times 121 \times 121$ 个 cell，cell 高度 $h_c = 3.0\,\text{m}$。机器人附近 $N_{\text{near}} = 12$ 个邻域 cell 参与规划。

cell 索引：

$$
\text{cell\_ind} = f\left(\frac{x}{s_c}, \frac{y}{s_c}, \frac{z}{h_c}\right)
$$

### 5.2 Cell 状态机

每个 cell 状态 $S \in \{\text{UNSEEN}, \text{EXPLORING}, \text{COVERED}\}$。

对邻域内 cell，统计：

$$
\begin{aligned}
n_{\text{cand}} &= |\{v \in \text{cell}\}| \\
n_{\text{high}} &= |\{v: s(v) > \theta_{\text{big}}\}| \\
n_{\text{frontier}} &= |\{v: f(v) > \theta_f\}| \\
n_{\text{sel}} &= |\{v: \text{selected}\}|
\end{aligned}
$$

默认阈值：$\theta_{\text{small}}=60$，$\theta_{\text{big}}=80$，$\theta_f=30$。

**EXPLORING → COVERED**（当前房间且无未访问视点）：

$$
S = \text{EXPLORING} \land n_{\text{frontier}} < \tau_{e2c} \land n_{\text{high}} < \tau_{e2c} \land n_{\text{sel}} = 0
$$

默认 $\tau_{e2c} = 0$（`kCellExploringToCoveredThr`），即要求 frontier 高分视点归零才标记 COVERED。

**COVERED → EXPLORING**（重新发现未覆盖区域）：

$$
S = \text{COVERED} \land \left(n_{\text{high}} \ge \theta_{c2e} \lor n_{\text{frontier}} \ge \theta_{c2e}\right)
$$

默认 $\theta_{c2e} = 50$。

跨房间 transit 期间冻结 EXPLORING→COVERED 转换，避免中途误判房间完成。

---

## 6. 全局规划（Global TSP）

### 6.1 探索 cell 集合

收集所有 $S=\text{EXPLORING}$ 且在当前房间内的 cell 连接点：

$$
\mathcal{C}_{\text{exp}} = \{c_i : S(c_i)=\text{EXPLORING} \land \text{room}(c_i) = r_{\text{cur}}\}
$$

若 $\mathcal{C}_{\text{exp}} = \emptyset$，则：

$$
\text{room\_finished} = \text{true}
$$

触发房间完成逻辑（见第 8 节）。

### 6.2 TSP 求解

在 cell 连接点之间用 `KeyposeGraph` 最短路构造距离矩阵：

$$
D_{ij} = \lfloor 10 \cdot L_{ij} \rfloor
$$

其中 $L_{ij}$ 为 keypose 图上最短路径长度。用 OR-Tools TSP 求解器求访问顺序：

$$
\pi^* = \arg\min_{\pi} \sum_{k} D_{\pi(k),\pi(k+1)}
$$

输出全局探索路径 $\mathcal{P}_{\text{global}}$，节点类型包括 `GLOBAL_VIEWPOINT`、`GLOBAL_VIA_POINT`、`DOOR`、`HOME`。

---

## 7. 局部覆盖规划（LocalCoveragePlanner）

### 7.1 问题定义

给定全局路径 $\mathcal{P}_{\text{global}}$ 和未覆盖点数 $N_u$、未覆盖 frontier 数 $N_f$，在局部视界内选视点集合 $\mathcal{V}^*$，最小化路径长度并覆盖尽可能多的未观测区域。

### 7.2 贪心视点采样

对每个候选视点 $v$，计算边际覆盖：

$$
\Delta s(v) = \left|\mathcal{U}(v) \setminus \bigcup_{v' \in \mathcal{V}_{\text{sel}}} \mathcal{U}(v')\right|
$$

若 $\Delta s(v) \ge \theta_u$（默认 60），加入覆盖优先队列；否则若启用 frontier 且 $\Delta f(v) \ge \theta_f$（默认 30），加入 frontier 队列。

队列按 $\Delta s$ 降序排列，从前 $K_g=5$ 个（`kGreedyViewPointSampleRange`）中随机选一个加入 $\mathcal{V}_{\text{sel}}$，迭代更新队列直至无法再增。

### 7.3 局部 TSP

对选中的视点集 $\mathcal{V}_{\text{sel}}$，用视点间最短路构造距离矩阵并求解 TSP：

$$
\pi_{\text{local}}^* = \arg\min_{\pi} \sum_k \text{dist}_{\text{vp}}(\mathcal{V}_{\text{sel}}[\pi(k)], \mathcal{V}_{\text{sel}}[\pi(k+1)])
$$

路径节点含 `ROBOT`、`LOOKAHEAD_POINT`、`LOCAL_VIEWPOINT`、`LOCAL_PATH_START/END`。

### 7.4 局部完成判定

当 frontier 队列无法选出有效视点时：

$$
\text{local\_coverage\_complete} = \text{true}
$$

即当前局部视界内无值得再去的视点。

---

## 8. 房间探索与切换

### 8.1 当前房间识别

机器人位置投影到 room mask 栅格（分辨率 $\rho_r = 0.1\,\text{m}$）：

$$
r_{\text{cur}} = M\left(\left\lfloor \frac{x - x_s}{\rho_r} \right\rfloor, \left\lfloor \frac{y - y_s}{\rho_r} \right\rfloor\right)
$$

### 8.2 房间完成条件

在 Phase 3/6/9，当同时满足：

$$
\text{IsRoomFinished}() \land \text{IsLocalCoverageComplete}()
$$

且不在跨房间 transit 状态时，标记当前房间 `IsCovered=true`，调用 `GetAnswer()` 选择下一目标房间。

其中 `IsRoomFinished` 等价于当前房间无 EXPLORING cell：

$$
|\{c \in \mathcal{C}_{\text{exp}} : \text{room}(c) = r_{\text{cur}}\}| = 0
$$

### 8.3 下一房间选择

`GetAnswer()` → `SelectNearestUnexploredRoom()`，在未覆盖房间中选最近者，通过 `GoalPointCallback` 设置目标点并启动跨房间导航（`transit_across_room_ = true`）。

若当前楼层无未探索房间，累计 `no_unexplored_room_counter`；达到阈值（默认 3 周期）后：

- Phase 3 → 进入 Phase 4（上 2F）
- Phase 6 → 进入 Phase 7（上 3F）
- Phase 9 → `exploration_finished = true`

### 8.4 跨房间导航

跨房间时航点优先级：

1. 未到门口：发布门位置 $\mathbf{p}_{\text{door}}$
2. 接近门口：发布 lookahead 点
3. 到达新房间：重置 transit，继续 TARE

---

## 9. 航点发布与下游接口

### 9.1 Lookahead 点

从拼接路径 $\mathcal{P} = \mathcal{P}_{\text{global}} \oplus \mathcal{P}_{\text{local}}$ 提取 lookahead 点 $\mathbf{p}_{\text{la}}$，可选视线延伸：

$$
\mathbf{p}_{\text{wp}} = \mathbf{p}_{\text{robot}} + \min\left(\|\mathbf{p}_{\text{la}} - \mathbf{p}_{\text{robot}}\|, L_{\text{ext}}\right) \cdot \frac{\mathbf{p}_{\text{la}} - \mathbf{p}_{\text{robot}}}{\|\mathbf{p}_{\text{la}} - \mathbf{p}_{\text{robot}}\|}
$$

默认 $L_{\text{ext,big}}=8.0\,\text{m}$，$L_{\text{ext,small}}=3.5\,\text{m}$。

### 9.2 输出话题

| 话题 | 类型 | 作用 |
|---|---|---|
| `/way_point` | `geometry_msgs/PointStamped` | 给 `local_planner` 的目标 |
| `/exploring_phase` | `std_msgs/Int32` | 当前 Phase |
| `/speed` | `std_msgs/Float32` | 期望速度 |
| `exploration_finish` | `std_msgs/Bool` | 探索结束 |
| `/runtime` | 运行时统计 | 各模块耗时 |

---

## 12. 小结

当前 `tare_planner` 的核心可概括为 **“Phase 任务编排 + TARE 分层探索 + 房间语义约束”**：

**全局层**——在房间约束下对 EXPLORING cell 求 TSP：

$$
\boxed{
\pi^* = \arg\min_{\pi} \sum_{c_i \in \mathcal{C}_{\text{exp}}} \text{dist}_{\text{kg}}(c_i, c_{i+1})
}
$$

**局部层**——贪心选视点 + 局部 TSP 覆盖未观测区域：

$$
\boxed{
\mathcal{V}^* = \text{GreedyCover}(\mathcal{U}, \mathcal{F}, \theta_u, \theta_f), \quad
\mathcal{P}_{\text{local}} = \text{TSP}(\mathcal{V}^*)
}
$$

**任务层**——按 Phase 1–9 完成开门、走廊、按房间探索、电梯换层，最终输出 `/way_point` 驱动 `local_planner`：

$$
\boxed{
\mathbf{p}_{\text{wp}} = f(\text{Phase},\, r_{\text{cur}},\, \mathcal{P}_{\text{global}} \oplus \mathcal{P}_{\text{local}},\, \mathbf{p}_{\text{door}})
}
$$

形成 **“房间分割 → 全局调度 → 局部覆盖 → 航点输出 → 局部执行”** 的完整室内探索链路。

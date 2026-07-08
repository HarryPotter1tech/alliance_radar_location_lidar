# Alliance Radar Overall Architecture

## Overview

Alliance Radar 采用 **ROS2 component + 独立驱动进程 + YAML bringup** 的分层架构。
以 LiDAR 为主链路（`radar_lidar` → `radar_fusion` → `/localization/pose`），
视觉观测（`radar_camera`）作为补充观测源，`radar_bridge` 把最终位姿写入共享内存供 GUI 读取，
`radar_calibration` 负责离线相机-雷达标定。系统对外统一输出 `/localization/pose`。

- 主链路传感器：Odin 为主，Mid-70 通过 `sensor:=mid70` 切换。
- 进程拓扑：`radar_lidar` + `radar_fusion` 合并进单一 component 容器（intra-process 零拷贝）。

## Design Principles

- 雷达驱动（Odin / Livox / WS-30）保持独立进程，不并入算法容器。
- `radar_fusion` 独占 `/localization/pose` 出口。
- 离线模型转换由 `tools/model_to_map/` 负责，模型文件存放于 `model/`。
- `radar_bringup` 只负责 launch、参数、remap 和组件编排，不写业务 C++。
- `radar_camera` 作为补充观测源，不绑死 LiDAR 主链路。
- 命名统一收敛到 `radar::` 顶层命名空间（`radar::fusion::` / `radar::camera::` 等子命名空间）。
- **零自定义 ROS 消息**：只用标准消息（`geometry_msgs` / `diagnostic_msgs` / `vision_msgs`）；
  唯一的跨进程数据契约是 `radar_bridge` 的共享内存 struct。不设消息接口包。

## Package Architecture

```text
packages/
├── radar_lidar        ← LiDAR 预处理 + 配准定位
├── radar_fusion       ← 系统统一位姿出口 + 多目标跟踪
├── radar_camera       ← 视觉观测生成
├── radar_calibration  ← 离线相机-雷达标定 + 模型预处理
├── radar_bridge       ← ROS2 → 共享内存桥接
└── radar_bringup      ← launch / yaml / remap / compose
```

## Package Boundaries

### radar_lidar

订阅单雷达点云，完成有效点过滤、（可选）ROI 裁剪、球面网格下采样、多帧累积、
GICP 配准定位，并在配准结果基础上做动态点提取与欧氏聚类。只做 LiDAR 感知。

核心库 `radar_lidar_core`（纯算法、无 ROS 依赖）：

| 类 / 命名空间 | 文件 | 职责 |
|---|---|---|
| `radar::MapData` | `map_data.{hpp,cpp}` | PCD 加载，构建 small_gicp + PCL KdTree；静态工厂 `Load() -> std::expected<>` |
| `radar::LocalizationStage` | `localization.{hpp,cpp}` | GICP scan-to-map：ROI 裁剪 → 帧累积 → 球面网格 → 配准 |
| `radar::SphericalGrid` | `spherical_grid.{hpp,cpp}` | 方位角/俯仰角分箱下采样 |
| `radar::FrameAccumulator` | `frame_accumulator.{hpp,cpp}` | 滑动窗口多帧累积 |
| `radar::DynamicCloudStage` | `dynamic_cloud.{hpp,cpp}` | KdTree 最近邻动态障碍提取（OpenMP 并行） |
| `radar::ClusterStage` | `cluster.{hpp,cpp}` | 欧氏聚类 → 质心 + AABB |
| `radar::geom::` | `geometry_utils.hpp` | ROI 裁剪 / 位姿构造 / look-at / 有效点过滤 |
| `radar::config::` | `config.hpp` | `LocalizationConfig` / `RoiBounds` 参数结构 |
| `radar::types::` | `types.hpp` | `Point` / `Frame` / `PoseEstimate` 核心数据类型 |

ROS 组件 `radar::LidarPipeline`（`rclcpp_components`，进入 `radar_algorithm_container`）。
同包离线工具：`registration_tool`（CLI PCD-to-PCD 配准）、`offline_test_node`（Foxglove 可视化）。

可选位姿源：Odin1 内置重定位（`custom_map_mode=2`）。`use_odin_relocalization_tf`
参数启用后，`LidarPipeline` 每帧优先查 `map -> <scan frame_id>` TF（Odin1 重定位
成功后发布），查不到（重定位未收敛/未开启）则回退到上方的 GICP scan-to-map，
两条路径共用同一套锁定策略与下游感知链路。当前比赛运行时统一收敛到
`radar_lidar/config/runtime.yaml`，不同传感器与重定位模式由 bringup launch 覆盖少量差异参数。

### radar_fusion

系统唯一的 `/localization/pose` 出口。订阅 LiDAR 观测，做数据关联 + 多目标卡尔曼跟踪。
不做感知，不做 IO 桥接。

| 类 | 文件 | 职责 |
|---|---|---|
| `radar::fusion::FusionNode` | `fusion_node.{hpp,cpp}` | 订阅/发布 + 数据关联 + 轨迹生命周期管理 |
| `radar::fusion::KalmanTracker` | `kalman_tracker.{hpp,cpp}` | 单目标 2D 常速 KF：`[x,y,vx,vy]` 状态 |
| `radar::fusion::KalmanState` | `kalman_tracker.hpp` | KF 状态 + 轨迹元数据（color/number/lifecycle） |
| `radar::fusion::FusionConfig` | `fusion_node.hpp` | 门限/超时/确认命中数等参数 |

ROS 组件，与 `radar_lidar` 同容器零拷贝。

### radar_camera

补充观测源，独立于 LiDAR 主链路。订阅相机图像与内参，完成去畸变、目标检测 / 视觉定位，
输出视觉观测供 `radar_fusion` 融合。只产出观测，不做融合，不进入 LiDAR 主链路容器。

| 文件 | 职责 |
|---|---|
| `include/radar_camera/types.hpp` | `radar::camera::Detection` / `CameraFrame` 数据类型 |
| `include/radar_camera/config.hpp` | `radar::camera::CameraConfig`——内参/外参路径、检测阈值、topic 名 |
| `include/radar_camera/camera_model.hpp` | 去畸变 + 2D→3D 投影（纯函数，无 ROS） |
| `include/radar_camera/detector.hpp` | 目标检测接口 `detect(image) -> std::vector<Detection>` |
| `src/camera_node.{hpp,cpp}` | ROS 薄封装：订阅图像 → detector + camera_model → 发布观测 |
| `src/runtime.cpp` | `main()` → spin |

依赖 `radar_calibration` 输出的相机内参/外参 YAML。独立进程或独立 container。

### radar_calibration

手动触发的一次性离线标定工具。用已有场地地图点云 + 一张相机图像做相机-雷达
（camera-map registration）自动标定，生成 `radar_camera` 启动所需的外参 YAML。
非 spin 节点（`main()` 跑完退出），不常驻主运行图，不参与实时定位链路。

核心标定算法复用第三方库 `direct_visual_lidar_calibration`（NID 直接图像-点云
配准，target-less，见 `ros_ws/third-party/direct_visual_lidar_calibration/`），
不重新实现该算法；`radar_calibration` 只负责编排 CLI 流程与外参格式转换。

标定流程（`.script/calibrate-camera`，3 步，全程无人工介入）：

```text
1. preprocess_map        地图 PCD + 相机图像 + 内参 → calib.json（无外参）
2. inject-initial-guess  把粗略初始外参（雷达站相机安装几何估算值，
                          见 config/initial_guess.yaml）写入 calib.json
3. calibrate --background  NID 直接配准，从初值收敛出精确外参
   → calib.json 的 results.T_lidar_camera（第三方库字段名，本项目内部读作 t_map_camera）
```

初值来源说明：NID 直接配准是图像级优化，收敛域较窄，必须有一个大致准确的
初始外参才能收敛（不同于 GICP 点云配准可以从粗略初值稳定收敛）。
`direct_visual_lidar_calibration` 官方提供两种自动初值路径——人工选点
（`initial_guess_manual`）或 SuperGlue 特征匹配（`initial_guess_auto` +
`find_matches_superglue.py`，需要额外 torch 依赖，且模型仅限非商业用途）。
本项目选择第三条路径：直接注入已知的粗略安装几何估算值，零人工介入、零额外
依赖。

| 文件 | 职责 |
|---|---|
| `include/radar_calibration/camera_lidar_calibration.hpp` | `radar::calibration::` 标定后处理：解析/写入 calib.json、注入初值、导出外参 YAML |
| `src/running.cpp` | CLI 入口：`inject-initial-guess` / `extract-result` 两个子命令 |
| `config/initial_guess.yaml` | 相机相对地图系的粗略外参估算（平移 + RPY，`Rz(yaw)*Ry(pitch)*Rx(roll)`） |

### radar_bridge

订阅 `radar_fusion` 的输出，合并写入共享内存供非 ROS GUI（egui）零拷贝读取。
轻量 IO 桥接，不做任何算法。系统唯一的跨进程数据契约（shm struct）由本包定义。

| 文件 | 职责 |
|---|---|
| `include/radar_bridge/shm_layout.hpp` | 共享内存 struct（pose + state + fitness），egui 侧同定义 |
| `include/radar_bridge/shm_writer.hpp` | `radar::bridge::ShmWriter`：mmap + Seqlock 无锁写（无 ROS） |
| `src/bridge_node.{hpp,cpp}` | 订阅 pose + status → 转 struct → 调 writer |
| `src/runtime.cpp` | `main()` → spin |

独立轻量进程，不要求 component。

### radar_bringup

系统唯一编排入口。负责 launch、参数装配、topic remap、不同传感器机型配置、组件编排。
纯 YAML + launch，无业务 C++。

TF 职责（最终架构）：

- **只负责 static tf**，不负责任何运行时动态位姿。
- static tf 表达**已知且固定的刚体安装关系 / 外参**，如
  `radar_base -> lidar_link`、`radar_base -> camera_link`、
  `camera_link -> camera_optical_frame`。
- 具体发布方式优先用 `tf2_ros` 提供的 `static_transform_publisher`，由 launch 按
  YAML 参数起多个静态发布器；若未来需要从标定 YAML 动态读取，也应落在独立的
  配置节点/发布器中，而不是放进算法节点。
- **禁止在 bringup 层发布 dynamic tf**：bringup 不拥有运行时状态，不能决定
  “系统当前位姿是什么”。

```text
config/
├── common/          ← 公共参数 (topic, frame, QoS, 滤波, 地图路径)
├── lidar/
│   └── runtime.yaml ← 比赛运行时唯一算法参数入口（launch 覆盖 sensor 差异）
└── system/
    └── *.yaml       ← 组件组合配置

launch/
├── localization.launch.py       ← 单雷达定位入口 (sensor:=odin|mid70)
├── localization_gui.launch.py   ← + bridge + GUI 入口
└── full_system.launch.py        ← 完整系统入口
```

`ComposableNodeContainer` 承载 `radar_lidar` + `radar_fusion`（intra-process），
`IncludeLaunchDescription` 拉起独立驱动进程。

### TF Tree & Authorities

最终推荐的系统 frame tree：

```text
map
└── radar_base                (dynamic)
    ├── lidar_link            (static)
    └── camera_link           (static)
        └── camera_optical_frame   (static, optional)
```

职责划分：

- **dynamic tf 只表达系统自身刚体位姿**，不表达目标观测、聚类结果、轨迹或其他
  非刚体数据。
- **目标观测 / 聚类 / 轨迹继续走 topic**：`/lidar/dynamic`、`/lidar/cluster`、
  `/fusion/tracks`、未来的 `/camera/detection` 都是数据流，不应塞进 TF tree。
- **当前阶段**：`radar::LidarPipeline` 持有 `t_map_lidar`，因此可作为临时的
  dynamic tf authority，发布 `map -> radar_base`（若 `radar_base` 尚未显式建模，
  可先退化为 `map -> lidar_link`）。
- **最终阶段**：`radar::fusion::FusionNode` 作为系统唯一 `/localization/pose`
  出口，应接管 **唯一系统级 dynamic tf authority**，发布最终的
  `map -> radar_base`。一旦 `FusionNode` 真正融合多源位姿，`LidarPipeline` 不再
  发布系统最终 dynamic tf，只保留 `/lidar/pose` 作为原始 LiDAR 位姿观测。
- **算法核心库**（`radar_lidar_core`、未来相机几何核心等）继续只维护
  `Eigen::Isometry3d t_*_*` 变换，不直接依赖 `tf2_ros`。

TF 输出规范（当前建议）：

| TF | 类型 | 当前发布者 | 最终发布者 | 来源 | 说明 |
|---|---|---|---|---|---|
| `map -> lidar_link` | dynamic（过渡方案） | `radar::LidarPipeline` | 无（若显式建模 `radar_base` 后应废弃） | `t_map_lidar` | 当 `radar_base` 尚未在代码里显式建模时的最小正确方案；可直接满足 Foxglove 点云/聚类/姿态统一可视化 |
| `map -> radar_base` | dynamic（最终方案） | `radar::LidarPipeline`（当前阶段） | `radar::fusion::FusionNode`（最终阶段） | 当前阶段由 `t_map_lidar` 与固定安装关系换算；最终阶段由融合后的系统主位姿给出 | 系统自身刚体位姿的唯一权威关系；一旦 `FusionNode` 真正融合多源位姿，它应成为唯一 dynamic tf authority |
| `radar_base -> lidar_link` | static | `radar_bringup` | `radar_bringup` | 固定安装关系 / 配置 | 若 LiDAR 视作基准传感器、暂不显式建模 `radar_base`，则该条可临时省略；一旦引入 `radar_base`，它应由 launch/static publisher 提供 |
| `radar_base -> camera_link` | static | `radar_bringup` | `radar_bringup` | 相机与雷达的固定机械外参（标定结果） | 推荐由 `radar_calibration` 导出的 `radar_camera/config/extrinsic.yaml` 作为唯一事实源，launch 只引用该文件 |
| `camera_link -> camera_optical_frame` | static | `radar_bringup` | `radar_bringup` | 相机坐标约定 | 主要服务于相机视锥、投影、标准视觉工具链；若当前阶段不需要 optical frame，可暂缓 |

`frame_id` 约束（当前建议）：

- 原始 LiDAR 点云：保留传感器原始 `frame_id`（如 `lidar_link`）。
- 变换到地图后的动态点/聚类/可视化 marker：使用 `map` 作为 `frame_id`。
- `/lidar/pose`：继续表示 LiDAR 原始位姿观测（当前来自 `radar_lidar`）。
- `/localization/pose`：只表示系统最终稳定主位姿（长期由 `radar_fusion` 输出）。
- 目标检测/跟踪消息保留为 topic 数据，不通过 TF tree 表达。

### 配置归属：先验 / 外参 / 运行时位姿

为避免把“启动先验”“固定外参”“运行时动态位姿”混成一类，约定如下：

| 对象 | 固定/动态 | 存放位置 | 产生者 | 消费者 | 说明 |
|---|---|---|---|---|---|
| 相机标定前粗略初始位姿 | 固定（标定输入） | `radar_calibration/config/initial_guess.yaml` | 人工估算 / 测量 | `radar_calibration` | 只用于帮助离线标定收敛，不直接给主进程使用 |
| 相机正式外参 | 固定（运行时事实源） | `radar_bringup/config/common/extrinsics.yaml`（当前 static tf 发布源） / `radar_camera/config/extrinsic.yaml`（未来相机节点输入） | `radar_calibration` 导出后同步到 bringup/static tf 配置 | `radar_camera`、`radar_bringup` | 当前 launch 直接读取 bringup 的 static tf 配置；未来若相机节点单独消费外参，应保持两者来源一致 |
| LiDAR / Odin 启动先验 | 固定（启动参数） | `radar_bringup/config/lidar/*.yaml` | 部署配置 / 红蓝方场次参数 | `radar_lidar` / `odin_ros_driver` | 启动时的位姿猜测，不是固定外参 |
| LiDAR 离线配准调试参数 | 固定（工具配置） | `radar_lidar/config/offline_registration.yaml` | 工具调试参数 | `offline_test_node` 等离线工具 | 只供离线验证 / 调试使用 |
| LiDAR 对地图的最终位姿 (`t_map_lidar`) | 动态（运行时结果） | 不落 YAML；经 topic / dynamic tf 发布 | `radar_lidar`（当前）/ `radar_fusion`（最终） | Foxglove、`radar_fusion`、`radar_bridge` 等 | 若使用 GICP 或 Odin1 重定位，则它属于运行时定位结果，不是 calibration 风格外参文件 |
| 固定安装关系（如 `radar_base -> lidar_link`） | 固定（static tf） | `radar_bringup` launch / config | 部署配置 | 全系统 TF 消费者 | 属于系统装配关系，不归 `radar_calibration` |

- **相机标定前粗略初始位姿**：放在 `radar_calibration/config/initial_guess.yaml`。
  这是离线标定流程的输入，只用于帮助 `direct_visual_lidar_calibration` 收敛，
  不直接给主进程运行时使用。
- **相机正式外参**：当前 `radar_bringup` 的 static tf 直接读取
  `radar_bringup/config/common/extrinsics.yaml`。若 `radar_calibration` 导出
  `radar_camera/config/extrinsic.yaml` 作为视觉节点输入，则应同步生成或转换到 bringup 的
  static tf 配置，避免两份外参漂移。
- **LiDAR / Odin 启动先验**：放在 `radar_bringup/config/lidar/*.yaml`。
  例如 Odin 内置重定位使用 `custom_init_pos`；未来若自研 GICP 主链路也需要启动
  先验，同样归入 bringup 的运行时 YAML。它们的语义都是“启动时的位姿猜测”，
  不是固定外参。
- **LiDAR 离线配准调试参数**：放在 `radar_lidar/config/offline_registration.yaml`。
  这是工具级配置，仅供 `offline_test_node` / 离线配准调试使用，不是主进程启动参数。
- **LiDAR 对地图的最终位姿**（如 `t_map_lidar`、最终的 `map -> radar_base`）是
  **运行时动态结果**：若使用 GICP 或 Odin1 内置重定位，则由定位链路实时求出并
  通过 topic / dynamic tf 发布，**不落成 calibration 风格的外参 YAML**。
- **固定安装关系**（如 `radar_base -> lidar_link`、`radar_base -> camera_link`）是
  **static tf**：归 `radar_bringup` 管理，不归 `radar_calibration`。

### 最终 bringup 编排（推荐）

启动流程拆成三个阶段：**离线一次性准备**、**每场次启动前配置**、**主进程运行时**。

#### 阶段 1：离线一次性准备（非主进程）

1. 运行 `radar_calibration`，得到相机正式外参：
   `radar_camera/config/extrinsic.yaml`。
2. 如需 Odin 建图，单独运行 `odin_slam_mapping.launch.py`，保存 `.bin` 地图供
   Odin 内置重定位使用。

这一步的产物是**固定机械外参**和（可选的）**Odin SLAM 地图文件**，不启动主系统。

#### 阶段 2：每场次启动前配置（静态配置）

1. 选择本场次的 LiDAR / Odin 启动先验（红方 / 蓝方 / 指定高台部署位姿），
   写入 `radar_bringup/config/lidar/*.yaml`。
2. 准备 static tf 所需的固定安装关系：
   - `radar_base -> lidar_link`
   - `radar_base -> camera_link`
   - `camera_link -> camera_optical_frame`（如启用）

这一步仍然不依赖运行时计算结果；**不允许先运行 GICP 再把结果写回 YAML 作为主流程**。

#### 阶段 3：主进程运行时（推荐图）

```text
radar_bringup (launch / YAML / static tf)
├── static_transform_publisher(s)
│   ├── radar_base -> lidar_link
│   ├── radar_base -> camera_link
│   └── camera_link -> camera_optical_frame   (optional)
├── 驱动进程
│   ├── odin_ros_driver   或
│   └── livox_ros_driver2
├── radar_lidar_node
│   ├── 读取 map.pcd + 启动先验
│   ├── 运行 GICP / Odin TF 回退
│   ├── 发布 /lidar/pose
│   ├── 发布 /lidar/dynamic / /lidar/cluster / /diagnostics
│   └── 发布 dynamic tf（当前阶段 authority）
├── radar_camera_node
│   ├── 读取 radar_camera/config/extrinsic.yaml
│   └── 发布 /camera/detection 或 /camera/pose
├── radar_fusion_node
│   ├── 订阅 /lidar/pose + /lidar/cluster + camera observation
│   ├── 发布 /localization/pose
│   └── 最终接管 dynamic tf（长期 authority）
└── radar_bridge_node
    └── 订阅 /localization/pose + /localization/status → 共享内存
```

#### 启动期与运行期的边界

- **主进程读取的 YAML** 仅包括：固定机械外参、启动先验、节点参数。
- **主进程不依赖“先运行 GICP 再生成 pose YAML”** 才能启动。
- GICP / Odin 内置重定位求出的 `t_map_lidar` 是运行期动态状态，应直接作为
  `/lidar/pose` 和 dynamic tf 被下游消费。
- 若未来确有“重启后加速收敛”的需求，可另做 `last_pose_cache.yaml` 一类的
  **运行时缓存**，但它的语义必须是“启动先验缓存”，不能命名或归类为外参文件。

## Topic & Interface Contracts

```text
topics/
├── /odin1/cloud_raw  或  /livox/lidar
│   ├── type: sensor_msgs/msg/PointCloud2
│   ├── producer: 驱动进程 (odin_ros_driver / livox_ros_driver2)
│   └── consumer: radar_lidar
├── /lidar/pose
│   ├── type: geometry_msgs/msg/PoseWithCovarianceStamped
│   ├── producer: radar_lidar
│   ├── consumer: radar_fusion
│   └── role: LiDAR 主观测
├── /lidar/dynamic
│   ├── type: sensor_msgs/msg/PointCloud2
│   ├── producer: radar_lidar
│   └── role: 地图系动态点
├── /lidar/cluster
│   ├── type: sensor_msgs/msg/PointCloud2
│   ├── producer: radar_lidar
│   ├── consumer: radar_fusion
│   └── role: 聚类质心（数据关联输入）
├── /lidar/cluster_viz
│   ├── type: visualization_msgs/msg/MarkerArray
│   ├── producer: radar_lidar
│   └── role: AABB + 质心可视化
├── /diagnostics
│   ├── type: diagnostic_msgs/msg/DiagnosticStatus
│   ├── producer: radar_lidar
│   └── role: fitness / 耗时 / 帧号
├── /camera/image_raw, /camera/camera_info
│   ├── type: sensor_msgs/msg/Image, sensor_msgs/msg/CameraInfo
│   ├── producer: 相机驱动 (ros2-hikcamera)
│   └── consumer: radar_camera
├── /camera/pose  或  /camera/detection
│   ├── type: geometry_msgs/msg/PoseWithCovarianceStamped  或  vision_msgs/msg/Detection3DArray
│   ├── producer: radar_camera
│   └── consumer: radar_fusion
├── /localization/pose
│   ├── type: geometry_msgs/msg/PoseWithCovarianceStamped
│   ├── producer: radar_fusion
│   ├── consumers: radar_bridge, 其他 ROS 消费者
│   └── role: 系统对外稳定位姿契约
├── /localization/status
│   ├── type: diagnostic_msgs/msg/DiagnosticStatus
│   ├── producer: radar_fusion
│   ├── consumers: radar_bridge, 其他 ROS 消费者
│   └── role: 定位状态 (state / fitness / converged)
├── /fusion/tracks
│   ├── type: visualization_msgs/msg/MarkerArray
│   ├── producer: radar_fusion
│   └── role: radar-only confirmed 轨迹可视化（融合前基线输出）
└── /fusion/fused_tracks
    ├── type: visualization_msgs/msg/MarkerArray
    ├── producer: radar_fusion
    └── role: 多源融合后的最终轨迹输出（融合后契约）

shared_memory/
└── /dev/shm/lidar_pose
    ├── type: shm_layout.hpp 中的 C++ struct (pose + state + fitness), 非 ROS 消息
    ├── producer: radar_bridge  (订阅 /localization/pose + /localization/status 后合并写入)
    ├── consumer: egui
    └── role: 系统唯一跨进程数据契约, Seqlock 无锁读写
```

## Data Flow

```text
驱动进程 (odin_ros_driver / livox_ros_driver2)
  └─ /odin1/cloud_raw | /livox/lidar : PointCloud2
       └─> radar_lidar (LidarPipeline)
             ├─ /lidar/pose      : PoseWithCovarianceStamped ─┐
             ├─ /lidar/cluster   : PointCloud2 (质心) ────────┤
             ├─ /lidar/dynamic   : PointCloud2                │
             ├─ /lidar/cluster_viz : MarkerArray              │
             └─ /diagnostics     : DiagnosticStatus           │
                                                              v
        /camera/pose|detection ────────────────> radar_fusion (FusionNode)
        (radar_camera)                             ├─ /localization/pose   : PoseWithCovarianceStamped
                                                   ├─ /localization/status : DiagnosticStatus
                                                   ├─ /fusion/tracks       : MarkerArray
                                                   └─ /fusion/fused_tracks : MarkerArray
                                                        │ pose + status
                                                        v
                                                    radar_bridge
                                                     └─ /dev/shm/lidar_pose (shm struct)
                                                          └─> egui (只读共享内存)
```

## Process Topology

```text
runtime/
├── 雷达驱动 (odin_ros_driver / livox_ros_driver2)   独立进程, 发布点云 topic
├── radar_algorithm_container                          component container
│   ├── radar_lidar   (component)
│   └── radar_fusion  (component)                       intra-process 零拷贝
├── radar_bridge                                        独立进程, /localization/* -> 共享内存
├── egui                                                非 ROS 进程, 只读共享内存
└── radar_camera                                        独立视觉观测进程
```

## Assumptions & Defaults

- 系统以 LiDAR 时间戳为主时钟。
- 标定精度（相机外参）为准，点云动态目标检测是辅助手段。
- `/lidar/pose` 表示 LiDAR 原始位姿观测，`/localization/pose` 表示系统最终主位姿契约；长期统一为 `PoseWithCovarianceStamped`。
- 主链路默认 Odin；Mid-70 通过 `sensor:=mid70` 切换。

## Fusion Track Contract & TF Handoff

### 融合前 / 融合后轨迹契约

- `/fusion/tracks`：融合前基线输出，表示 radar-only 轨迹集合。当前来源是 `/lidar/cluster`
  驱动的 2D Kalman tracking，用于保留纯雷达模式兼容性，也是未来多源融合效果的对比基线。
- `/fusion/fused_tracks`：融合后最终输出，表示 camera + radar（以及未来其他观测源）
  融合后的轨迹集合。相机观测尚未接入时，该话题可以缺省，但契约应预留。

| 话题 | 当前状态 | 长期语义 | 发布条件 |
|---|---|---|---|
| `/fusion/tracks` | 已有 | radar-only confirmed tracks | `radar_fusion` 运行时持续发布 |
| `/fusion/fused_tracks` | 待实现 | final fused tracks | 至少存在两类观测源并完成融合后发布 |

### Dynamic TF Authority 接管条件

当前阶段：

- `radar_lidar` 发布系统运行时 dynamic tf，作为临时 authority。
- 当前实现关系为 `map -> radar_base`，与 `radar_bringup` 提供的 static tf 共同组成完整 frame tree。

最终阶段：

- `radar_fusion` 在不再只是 `/lidar/pose` relay、而是产出真正多源融合主位姿后，
  接管唯一系统级 dynamic tf authority，发布最终 `map -> radar_base`。

接管前提：

1. `radar_fusion` 已消费至少两类观测源（如 LiDAR + camera）
2. `/localization/pose` 不再是 LiDAR pose passthrough
3. 融合主位姿具备稳定性判断（如 covariance / converged / source-health）
4. `radar_lidar` 保留 `/lidar/pose` 作为原始 LiDAR 位姿观测，但不再发布系统最终 dynamic tf

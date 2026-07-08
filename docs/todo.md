# radar-lidar Todo

更新时间：2026-07-05

## TF Authority 设计定稿（2026-07-06）

面向 Foxglove / RViz 的统一 frame 可视化需求，系统内坐标关系职责拆分如下：

- [x] **`radar_bringup` 只负责 static tf**：已知且固定的刚体安装关系 / 外参，
      如 `radar_base -> lidar_link`、`radar_base -> camera_link`、
      `camera_link -> camera_optical_frame`。`radar_bringup` 维持“纯 YAML + launch”
      边界，不承担运行时状态。
- [x] **`LidarPipeline` 当前阶段负责临时 dynamic tf**：在系统尚未拥有真正融合后的
      ego pose 前，由 LiDAR 主位姿来源发布 `map -> radar_base`（或过渡期
      `map -> lidar_link`）。
- [x] **`FusionNode` 最终接管唯一系统级 dynamic tf authority**：当
      `/localization/pose` 不再是 LiDAR pose passthrough，而是多源融合后的系统
      最终位姿时，由 `FusionNode` 统一发布 `map -> radar_base`。
- [x] **目标观测/轨迹不进 tf**：`/lidar/dynamic`、`/lidar/cluster`、`/fusion/tracks`
      以及未来 `/camera/detection` 继续走 topic；tf 只表达 frame 之间的刚体关系。
- [x] **配置归属定稿**：
      - 相机标定前粗略初值 → `radar_calibration/config/initial_guess.yaml`
      - 相机正式外参 → `radar_camera/config/extrinsic.yaml`
      - LiDAR / Odin 启动先验 → `radar_bringup/config/lidar/*.yaml`
      - LiDAR 离线配准调试参数 → `radar_lidar/config/offline_registration.yaml`
      - GICP / Odin1 重定位得到的 `t_map_lidar` / `map -> radar_base` → 运行时动态结果，
        不落成 calibration 风格 YAML

后续实施待办：
- [x] 在 architecture 文档基础上补一版具体 frame 名字与 `frame_id` 约束清单
      （`map` / `radar_base` / `lidar_link` / `camera_link` / `camera_optical_frame`）
- [x] 在 architecture 文档中补最终 bringup 启动图与阶段划分（离线一次性准备 /
      每场次启动前配置 / 主进程运行时），并明确拒绝“先 GICP、写外参 YAML、再启动主进程”
      作为主流程
- [x] `radar_bringup` launch 中加入 static tf 发布器（已落 `static_tf.launch.py` + `extrinsics.yaml`）
- [x] `LidarPipeline` 增加 dynamic tf broadcaster（当前阶段 authority，当前发布 `map -> radar_base`）
- [ ] `FusionNode` 从 pose relay 演进为真正多源 pose fusion 后，满足以下门槛再接管 dynamic tf：
      1) 至少接入两类观测源；2) `/localization/pose` 不再是 LiDAR passthrough；
      3) 具备稳定性判断（covariance / converged / source-health）；
      4) 切换后 `radar_lidar` 停止发布系统最终 dynamic tf
- [x] 明确 `/fusion/tracks`（radar-only）与 `/fusion/fused_tracks`（final fused）双轨迹契约，并补充 dynamic tf authority handoff 条件

## Odin1 内置重定位集成（2026-07-05 完成）

架构决策：Odin1 内置重定位为主位姿源，`radar_lidar` 自研 GICP 保留作为重定位
未收敛时的回退。当前比赛运行时参数统一收敛到 `runtime.yaml`，重定位模式由 bringup launch 覆盖 `use_odin_relocalization_tf`。

- [x] `LidarPipeline` 新增 `use_odin_relocalization_tf` 参数（默认 `false`，
      `get_parameter_or` 读取，未声明时不报错）：启用后每帧优先查
      `map -> <scan frame_id>` TF（Odin1 重定位成功后发布），查不到则回退到
      现有 `LocalizationStage::process`（GICP scan-to-map），核心配准逻辑
      （`localization.cpp` / `config.hpp`）未改动
- [x] 重定位模式改为由 bringup launch 覆盖 `use_odin_relocalization_tf: true`，
      不再维护 `odin_relocalization.yaml` 参数副本
- [x] 新增 Odin 驱动 profile：`odin_driver_slam.yaml`（`custom_map_mode=1`，
      赛前离线走图）、`odin_driver_relocalization.yaml`（`custom_map_mode=2`
      + `sendodom`/`send_odom_baselink_tf` 开）
- [x] 新增 launch：`odin_slam_mapping.launch.py`（单独走图）、
      `odin_relocalization_localization.launch.py`（重定位 + GICP 回退联合定位）
- [x] 回归零精度损失：radar_lidar 18 + radar_fusion 6 + radar_calibration 7
      测试全通过，synth-scan 逐位一致（fitness=24383.548556,
      translation=[0.428396,-0.432899,4.128009]）
- [x] README / architecture 文档补充重定位用法与位姿源切换说明

待办（未来）：
- [ ] `odin_driver_relocalization.yaml` 的 `custom_init_pos`（红/蓝方安装
      位置估算值）当前占位全零，需实测/估算后按红蓝方场次填入
- [ ] 真实 Odin1 硬件重定位收敛验证（当前只验证了 TF 回退路径的代码逻辑，
      未接入真实设备）

## 当前重点：Fusion 模块 T3

目标：让 `radar_fusion` 支持纯雷达模式与雷达 + 相机模式，并输出统一的融合结果。

- [ ] T3.1 定义相机检测消息接口
  - 创建 `radar_interfaces/msg/CameraDetection.msg`
  - 字段：`header`, `target_id`, `position` (`Point`), `size` (`Vector3`), `type`（`robot` / `dart` / `aerial`）
- [ ] T3.2 添加相机订阅到 `FusionNode`
  - 订阅 `/camera/detection` 话题
  - 接收相机检测结果
- [ ] T3.3 实现相机→地图坐标投影
  - 使用相机内参 + 外参将 2D 检测投影到 3D 地图坐标
  - 如果上游已经输出 3D 检测，则直接订阅 3D 结果
- [ ] T3.4 扩展数据关联
  - 支持雷达轨迹 ↔ 相机测量关联
  - 门限匹配 + 距离度量
- [ ] T3.5 实现融合逻辑
  - 雷达 + 相机测量加权融合
  - 更新卡尔曼状态
- [ ] T3.6 添加融合输出接口
  - 发布 `/fusion/fused_tracks`
  - 保留 `/fusion/tracks` 兼容纯雷达模式
- [ ] T3.7 添加测试
  - 单元测试：纯雷达模式
  - 单元测试：雷达 + 相机融合模式

## 完成判定

- `radar_interfaces/msg/CameraDetection.msg` 已生成并可在 `FusionNode` 中引用
- `/fusion/tracks` 保持纯雷达兼容
- `/fusion/fused_tracks` 在相机数据存在时输出融合结果
- 对应单元测试通过

---

# 离线配准重构（2026-07-01 完成）

设计文档：`docs/superpowers/specs/2026-07-01-offline-registration-redesign-design.md`

## 根因（最终定位）

初值丢了 z（4m 高度）和 pitch（俯仰）：旧 `init_pose` 写死 `z=0` 且只有 yaw，
GICP 从贴地平视的歪起点出发，陷局部最优。不是迭代次数/阈值问题。

## 已完成

- [x] 地图规范化：改用 `tools/model_to_map --y-up` 从 FBX 直接生成 Z-up 地图，
      去掉每次运行转 `/tmp` 的 hack（`map_y_up` 降级为带弃用警告的兼容回退）。
      一次性补丁工具 `normalize_map` 已删除（与 `model_to_map --y-up` 功能重叠）
- [x] look-at 6DOF 初值：由雷达位置指向注视点自动算 yaw+pitch，补回 z/pitch
- [x] yaw 局部多起点搜索（围绕 look-at 的 yaw，非 360 盲搜）
- [x] KdTree 归一化评分（内点率 + 内点 RMSE，可跨候选比较）
- [x] 精配准 + pose.json 输出（含坐标系标注、内点率、RMSE）
- [x] 合成真值验证：`make_synth_scan.py` 按 Odin1 真实规格（FOV 120°×90°、
      量程 70m、z-buffer 遮挡、2cm 噪声）合成 scan → `model/generated/synth_scan_odin.pcd`。
      配准从偏离初值恢复真值：平移误差毫米级、inlier=1.000、rmse=0.045m

## 待办

- [ ] **jinan 场外点融合（方案 A）**：把 `pcd_rmuc2026_jinan.pcd` 中场地框外
      （|x|>14 或 |y|>7.5）的真实点变换到工作系后叠进合成 scan，模拟真实场外
      墙/看台干扰，并验证扇形 ROI 能否滤掉这些杂点。需先粗对齐 jinan 在其自身
      坐标系下的场地位姿。
- [ ] 真实部署 scan 到手后，用它验证 look-at 默认初值（合成验证只证明算法机制，
      真实传感器遮挡/噪声/密度需实测）
- [ ] 红蓝方朝向：在 Foxglove 比对官方小地图，确认 180° 朝向自由度

---

# 大任务（独立排期）

## 1. LiDAR 算法重构优化（现代 C++ + 设计模式）

以现代 C++（C++23）和设计模式角度重构 `radar_lidar` 算法链，目标最优化 + 简洁。

- 范围：`localization` / `spherical_grid` / `frame_accumulator` / `dynamic_cloud` / `cluster` / `pipeline`
- 关注点：消除重复、明确职责边界、RAII/所有权清晰、零成本抽象、可测试性
- 约束：不破坏现有配准精度（用 `make_synth_scan` 合成真值回归验证）

### 阶段一：消除重复 + 最小化清理（2026-07-04 完成，PR #32 / Issue #30）

- [x] 新增 `geometry_utils.hpp`：`clip_roi_aabb` / `pose_from_yaw_pitch` /
      `look_at_yaw_pitch` / `filter_valid_points`，消除约 8 处重复
- [x] `config.hpp` 新增 `RoiBounds`，`LocalizationConfig` / `DynamicCloudConfig` 共享嵌入
- [x] `dynamic_cloud`：OpenMP 线程缓冲区改为构造时预分配成员变量
- [x] 修复 `pipeline` / `runtime` 在 map_path 缺失/加载失败时的静默挂起（补 shutdown + 退出码）
- [x] 回归零精度损失：inlier=1.000 rmse=0.0465，18/18 单测通过
- 说明：按最小化原则，纯 API/风格改动（`process_with_guess()` 重载、
  marker-building 提取、`make_shared` 风格）已主动跳过

### 阶段二：组件化 + 配置装配重构（设计中，未动代码）

面向对象/现代 C++ 深化，向 `docs/architecture` 目标架构收敛。设计决策已锁定：

- 主链路传感器：**Odin 为主，Mid-70 为可选**（当前 bringup + 实测回归基线均为 Odin）
- 进程拓扑：**`radar_lidar` + `radar_fusion` 合并进单一 component 容器**
  （`radar_algorithm_container`，intra-process 零拷贝，按架构文档）
- 本轮先出**详细设计方案 + 落地计划**，review 通过后再动代码

- [ ] T2.1 `radar_lidar` 组件化：`rclcpp_components_register_node(... PLUGIN "radar::LidarPipeline")`，
      保留 `runtime.cpp` 独立可执行文件作为薄封装（两种形态共存）
- [ ] T2.2 参数批量装配：`pipeline.cpp` 现有 32 个逐条 `get_parameter` 收敛为
      `config::load_from_node(node) -> LocalizationConfig` 装载器，去掉构造函数参数墙
- [ ] T2.3 `radar_fusion` 组件化，与 `radar_lidar` 同容器
- [ ] T2.4 话题契约对齐架构文档：核对 `/lidar/pose_raw` + `/lidar/pose` 命名并按需 remap

阶段二组件化（T2.1/T2.3/T2.4）推迟到阶段三 bringup 编排一并做（组件化价值在
launch 编排，与 bringup 一体，现在单做是半成品）。T2.2 参数批量装配已随
`radar_calibration` 重构一并完成，见下方阶段三记录。

### 阶段三：radar_calibration 重构 + 相机外参自动标定（2026-07-05 完成）

目录重排 + 命名空间统一 + 补相机-雷达自动标定核心：

- [x] 目录重排：新增 `include/radar_calibration/`，头文件从 `src/` 迁出
- [x] 命名空间统一：`Radar::process::model` / `Radar::process::image` /
      `Radar::capture` → `radar::calibration::`
- [x] 删除无用骨架模块：`model_preprocess` / `pointcloud_capture` /
      `pointcloud_process` / `image_preprocess`（新流程用不上，YAGNI）
- [x] 相机外参标定核心：复用第三方库 `direct_visual_lidar_calibration`
      （NID 直接图像-点云配准，target-less），`radar_calibration` 只做编排
      与格式转换，不重新实现配准算法
- [x] 标定流程（`.script/calibrate-camera`，3 步，全程无人工介入）：
      `preprocess_map`（地图 PCD + 相机图像 → `calib.json`）→
      `inject-initial-guess`（注入 `config/initial_guess.yaml` 里的粗略外参
      估算值）→ `calibrate --background`（NID 精配准收敛出 `t_map_camera`）
- [x] 排除 SuperGlue 自动初值路径：需要 torch 重依赖，且模型仅限非商业用途；
      改用直接注入粗略安装几何估算值，零人工介入、零额外依赖
- [x] `radar_calibration_core` 静态库 + `radar_calibration_node` CLI
      （`inject-initial-guess` / `extract-result` 两个子命令），7/7 单测通过
- [x] `build-all` 移除 `direct_visual_lidar_calibration` 的
      `--packages-skip`，验证 colcon 真实编译通过（6 个 CLI 工具产出）；
      `build-radar` 保持跳过（快速日常构建路径）
- [x] 回归零精度损失：radar_lidar 19 + radar_fusion 7 + radar_calibration 8
      测试全通过，synth-scan 逐位一致

PR #33 提交后 CodeRabbit review 修复（2026-07-05）：

- [x] clang-format：`radar_calibration` 新文件 + `radar_lidar` 本次改动触碰的
      文件（`pose.T` → `t_map_lidar` 重命名后对齐宽度变化）
- [x] `package.xml`：`nlohmann-json3-dev`（apt 包名）误填进 `<depend>`，
      改为 rosdep key `nlohmann-json-dev`
- [x] `docs/conventions/cpp-naming.md`：PIMPL 措辞从"本项目不使用 PIMPL"
      改为"算法核心库默认不用"，避免读起来像硬性禁令
- [x] README / architecture：`calib.json` 的 `results.T_lidar_camera` 明确
      标注"第三方库字段名，本项目内部读作 t_map_camera"，避免与项目自身
      `t_目标_源` 命名约定混淆
- [x] README：补 `extract-result` 完整 `ros2 run` 命令示例

待完成（未来）：
- [ ] `radar_camera` 包目前是空壳（`int main(){return 0;}`），尚未接入
      `extract-result` 产出的外参 YAML；需实现 `CameraConfig` / `camera_model`
      （去畸变+投影）/ `detector` / `camera_node`（见架构文档 radar_camera 节）
- [ ] `config/initial_guess.yaml` 当前是占位全零，需实测/估算 RoboMaster
      雷达站相机相对地图系的安装几何（平移 + RPY）后填入
- [ ] `camera_lidar_calibration.cpp` 解析 `T_lidar_camera` 时对 JSON 数组
      类型缺少前置校验（CodeRabbit 提出，非阻塞，留待后续加固）
- [ ] `localization.cpp` 的 `has_initial_pose` 与 `use_lock_strategy` 耦合，
      语义上应可独立生效（CodeRabbit 提出，需与阶段②行为设计一并评估）

## 2. 包间通讯节点梳理（写进 docs）

梳理各 ROS 包之间的通讯契约（话题 / 服务 / 消息类型 / QoS / 坐标系 / frame_id），
写进 `docs/`，方便后续开发对接。

- 范围：radar_lidar / radar_camera / radar_fusion / radar_bridge / radar_calibration / radar_bringup
- 产出：每个包的输入/输出话题表 + 数据流图 + 坐标系约定

# RADAR-LOCATION-LIDAR

基于 **深圳市机智人科技** 纯固态面阵激光雷达 **WS_30PCD_ET3** 开发的激光雷达定位方案。

> 后续将加入基于 **Livox Mid-70** 的定位支持。

---

## Docker 环境

### 创建容器
```bash
docker run -it -d --name RADAR --privileged --restart=always \
  -v /dev:/dev \
  -v /home/pinkpanda/linux-RADAR/RADAR-2026/RADAR-LOCATION-LIDAR:/workspace \
  --network host \
  harryh_radar
```

### 命令解释

| 参数 | 说明 |
|------|------|
| `-it` | 保持 stdin 打开并分配 TTY，支持 `docker attach` 交互 |
| `-d` | 后台运行（detach mode） |
| `--name RADAR` | 容器命名为 `RADAR` |
| `--privileged` | 授予容器特权模式，允许直接访问宿主机硬件设备（LiDAR 等） |
| `--restart=always` | 容器退出后自动重启（含 Docker 守护进程重启时） |
| `-v /dev:/dev` | 挂载宿主机设备目录，使容器可直接访问 `/dev/ttyUSB*` 等串口与传感器 |
| `-v <host>:<container>` | 挂载项目工作目录到容器的 `/workspace` |
| `--network host` | 容器直接使用宿主机网络栈，无需端口映射，低延迟通信 |
| `harryh_radar` | 使用的 Docker 镜像名称 |

---

## Foxglove Bridge 配置

### 安装
```bash
sudo apt update && sudo apt install ros-$ROS_DISTRO-foxglove-bridge -y
```

### 启动
```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

| 参数 | 说明 |
|------|------|
| `ros2 launch` | ROS2 启动工具，用于启动 launch 文件 |
| `foxglove_bridge` | Foxglove 桥接功能包名 |
| `foxglove_bridge_launch.xml` | 启动配置文件，加载桥接节点 |
| `port:=8765` | 指定 WebSocket 服务端口为 `8765`，Foxglove Studio 通过此端口连接 |

---

## WS_30PCD_ET3 开发资料

### 产品手册
- [产品手册](https://gitee.com/shenzhen-smart-people/product_manual.git)

### 驱动包
| 平台 | 地址 |
|------|------|
| ROS | https://gitee.com/shenzhen-smart-people/ws_30pcd_et3_ros.git |
| ROS2 | https://gitee.com/shenzhen-smart-people/ws_30pcd_et3_ros2.git |
| Windows (VS2022) | https://gitee.com/shenzhen-smart-people/ws_30pcd_et3_win10.git |
| Ubuntu (PCL) | https://gitee.com/shenzhen-smart-people/ws_30pcd_et3_pcl.git |
| Ubuntu (Qt) | https://gitee.com/shenzhen-smart-people/ubuntu_qt_sdk.git |

### 结构图纸
- [结构图纸](https://gitee.com/shenzhen-smart-people/structural_drawings.git)

---

## 容器内开发

### 进入容器
```bash
docker exec -it RADAR bash
```

### 编译 & 运行

项目提供了一套脚本简化编译运行流程，位于 `/workspace/script/`，支持 `Release` / `Debug` 两种构建类型：

| 命令 | 说明 |
|------|------|
| `radar --build-pure [Release\|Debug]` | 清理后重新编译 |
| `radar --build-run [Release\|Debug]` | 增量编译并运行节点 |
| `radar --run` | 仅运行节点（不编译） |
| `radar --format` | 格式化 radar_localization_lidar/src/ 下所有 .cpp .hpp |
| `radar --help` | 查看帮助 |

构建类型默认为 `Release`：
- **Release**: `-O3 -march=native -flto -DNDEBUG -DEIGEN_NO_DEBUG`
- **Debug**: `-O3 -march=native -flto -DNDEBUG -DEIGEN_NO_DEBUG -g`（与 Release 优化完全一致，仅多调试符号）

也可以直接调用底层脚本：

```bash
# Release 纯净编译
bash /workspace/script/build_pure.sh Release

# Debug 编译 + 运行
bash /workspace/script/build_run.sh Debug
```

### 手动编译
```bash
cd /workspace/ros_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select radar_localization_lidar
source /workspace/ros_ws/install/setup.bash
ros2 run radar_localization_lidar radar_localization_lidar_node
```

---

## 目录结构

```
RADAR-LOCATION-LIDAR/
├── ros_ws/               # ROS2 工作空间（含 radar_localization_lidar 及第三方依赖）
├── lidar_ros_driver/     # LiDAR 驱动（Livox / WS_30PCD_ET3 / Odin）
├── script/               # 编译运行脚本
├── docs/                 # SLAM 学习资料
└── README.md
```

.\.venv\Scripts\Activate.ps



# 地下停车场 IMU/UWB 车载导航（Windows 测试包）

这是一个不依赖 ROS 的 Windows 原生 Python/PySide6 测试程序，提供：

- 导航地图 JSON 显示：有向车道、车位编号、入口、障碍物、UWB 基站
- 按“入口/当前位置编号 → 停车位编号”的 A* 有向车道规划
- 手机 IMU 预测 + UWB 二维位置校正的 2D EKF
- UWB 马氏距离异常值剔除、规划路径软约束、轨迹实时显示
- UDP 数据接口和内置传感器仿真，可在无设备时完成全流程测试
- DWG/DXF 按图层半自动转导航地图的命令行工具

## 一、5 分钟运行

系统要求：64 位 Windows 10/11、Python 3.10 或更高版本。安装 Python 时勾选 `Add Python to PATH`。

1. 解压本项目，双击 `install.bat`，等待依赖安装完成。
2. 双击 `run.bat`。
3. 默认演示地图会自动加载；选择“入口A”和“A08”，点击“规划路线”。
4. 点击“启动内置仿真”。绿色箭头是 EKF 融合位置，绿色细线是历史轨迹，橙色点是 UWB 原始位置。
5. 鼠标滚轮缩放地图，按住左键拖动地图。

若公司网络不能访问 PyPI，请先配置 pip 镜像，再运行 `install.bat`。

## 二、项目结构

```text
parking_nav_windows/
├─ app.py                    # GUI 入口
├─ install.bat / run.bat     # Windows 安装与启动
├─ config/settings.json      # UDP 端口、EKF、地图匹配参数
├─ maps/demo_map.json        # 可直接运行的地图示例
├─ parking_nav/
│  ├─ models.py              # 统一地图模型
│  ├─ planner.py             # 有向车道 A* 规划
│  ├─ fusion.py              # IMU/UWB EKF
│  ├─ interfaces.py          # UDP 接收
│  ├─ simulator.py           # 内置 IMU/UWB 仿真
│  ├─ canvas.py              # 地图可视化
│  └─ gui.py                 # Windows GUI
├─ tools/dwg_dxf_to_map.py   # CAD 转换
├─ tools/send_sample_udp.py  # UDP 示例发送器
└─ tests/test_core.py         # 核心逻辑测试
```

## 三、真实设备接入

先在程序中点击“启动 UDP IMU/UWB”。默认监听：

- IMU：本机 UDP `19001`
- UWB 二维定位：本机 UDP `19002`
- 数据：UTF-8、单个 UDP 数据报对应一个 JSON 对象
- 时间：Unix 秒，可以是小数，两个数据源必须使用同一时间基准
- 坐标：右手平面坐标，x 向地图东/右，y 向地图北/上，单位米；yaw 逆时针为正

### IMU 包（必须）

```json
{"type":"imu","timestamp":1789780000.123,"ax":0.12,"gyro_z":0.008}
```

| 字段 | 单位 | 含义 |
|---|---:|---|
| `timestamp` | s | Unix 时间或统一的单调时间 |
| `ax` | m/s² | 车辆前向线加速度，已去重力、已变换到车体坐标 |
| `gyro_z` | rad/s | 绕竖直轴角速度，逆时针为正 |

建议手机端先完成：手机坐标系到车体坐标系的固定外参旋转、重力补偿、时间戳保留。若手机只能提供四元数，可在手机端或接口层计算 yaw；正式工程建议扩展 EKF 为包含加速度计偏置的误差状态滤波器。

### UWB 位置包（必须）

```json
{"type":"uwb_position","timestamp":1789780000.200,"x":23.45,"y":18.72,"std":0.45,"quality":0.92}
```

| 字段 | 单位 | 必填 | 含义 |
|---|---:|---:|---|
| `timestamp` | s | 否 | 缺省用接收时间，建议提供 |
| `x`,`y` | m | 是 | 已由 UWB 解算器输出、且已配准到地图坐标系的位置 |
| `std` | m | 否 | 单轴位置标准差，默认 0.6 |
| `quality` | 0~1 | 否 | 质量系数，越小则测量协方差越大 |

当前包接收的是 **UWB 解算后二维坐标**。如果甲方只提供“基站 ID + 距离”或 TDoA 原始值，需要在上游增加多边定位/鲁棒 NLOS 处理，再向此程序发送 `uwb_position`；不要把距离直接填到 x、y。

### UDP 联调

打开 GUI 并启动 UDP 后，在另一个命令行运行：

```bat
.venv\Scripts\python.exe tools\send_sample_udp.py
```

手机与 PC 不在同一台设备时，把发送目标设为 PC 的局域网 IP，并在 Windows Defender 防火墙放行 Python 的 UDP 19001、19002 入站访问。

## 四、地图 JSON

所有几何坐标统一为米。核心结构如下：

```json
{
  "schema_version": "1.0",
  "name": "B1停车场",
  "coordinate_system": "LOCAL_ENU_METERS",
  "lane_network": {
    "lanes": [
      {"id":"L1","points":[[0,0],[10,0]],"width":3.5,"bidirectional":false}
    ]
  },
  "parking_spots": [
    {"id":"A001","position":[8,5],"rotation_deg":90,"width":2.5,"length":5.2,"type":"car"}
  ],
  "entrances": [{"id":"入口1","position":[0,0],"type":"entrance"}],
  "static_obstacles": [{"id":"柱1","polygon":[[3,3],[4,3],[4,4],[3,4]]}],
  "uwb_anchors": [{"id":"UWB01","position":[0,0,2.5]}]
}
```

车道 `points` 的顺序就是允许行驶方向；双向车道设置 `bidirectional=true`。车位与入口通过最近可达车道接入。实际交付时应额外核查车道连接、转弯限制、禁行区、坡道楼层和车辆最小转弯半径。

## 五、DWG/DXF 转换

DWG 是专有二进制格式。本工具采用“DWG 经 ODA File Converter 读取/转 DXF，再由 ezdxf 提取图层实体”的方式。直接 DXF 不需要 ODA。

### 建议甲方整理图层

| 语义 | 默认图层 | 推荐 CAD 实体 |
|---|---|---|
| 有向车道中心线 | `LANE_CENTER` | LINE/LWPOLYLINE，绘制顺序代表行驶方向 |
| 停车位中心 | `PARKING_SLOT` | 带属性的块 INSERT，属性 `ID` 或 `NUMBER` |
| 入口/出口 | `ENTRANCE` | 带属性的块 INSERT，属性 `ID` |
| 障碍物 | 先人工录入 JSON | 闭合多段线；当前转换脚本未自动提取 |

若 CAD 单位为毫米：

```bat
.venv\Scripts\python.exe tools\dwg_dxf_to_map.py input.dwg maps\site.json --scale 0.001
```

若图层名不同：

```bat
.venv\Scripts\python.exe tools\dwg_dxf_to_map.py input.dxf maps\site.json --scale 1.0 --lane-layer ROAD_CENTER --slot-layer CAR_SPACE --entrance-layer GATE
```

转换后先在 GUI 打开 JSON 做目视检查。复杂 DWG 往往包含块嵌套、样条曲线、外部参照、文字而非块属性；这时需要按甲方图层规范扩展转换器，不能期待任意 DWG 全自动获得语义地图。

### 坐标配准

UWB 坐标和 CAD 坐标必须落在同一个地图坐标系。建议现场测量至少 3 个不共线控制点，求二维刚体或相似变换：旋转、平移；若 CAD 比例不可靠再估计尺度。把变换应用到 UWB 解算结果，或预先应用到地图几何，不能两边都变换。

## 六、对原 PDR 代码的判断

原 `imu_localization_node` 不宜直接用于车载导航，需要修改：

1. 原实现是行人步态 PDR：通过加速度峰值检测“走一步”，每步固定 0.65 m；车辆没有稳定步态，模型不成立。
2. 原矩阵注释写“逆时针 90°”，但矩阵第一行实际保留 x，第二行把 x、y 相加，不是正交旋转矩阵，会破坏向量长度。
3. 航向只积分 `gyro_z`，没有估计陀螺零偏，静止时也会持续漂移。
4. 代码把位置直接吸附到规划路径，这会掩盖定位误差；本包仅采用低权重软约束，并保留 UWB 创新门限。
5. 原发布名称含 `ekf`，但代码实际没有 EKF 状态、协方差、预测和量测更新。
6. 车载系统应使用纵向加速度/轮速进行运动预测；仅靠手机 MEMS 加速度两次积分，位置会很快发散。若能取得 OBD/CAN 轮速和方向盘角，优先接入。

本包的 2D EKF 状态为 `[x, y, v, yaw, gyro_bias]`，由 `ax` 和 `gyro_z` 预测，以 UWB `(x,y)` 校正。它适合 Windows 功能联调和接口验证；量产版还应加入手机安装外参在线/离线标定、静止检测、时间同步、UWB NLOS 检测、车辆非完整约束、楼层状态和重定位。

## 七、参数调节

编辑 `config/settings.json`：

- `imu_accel_noise`：手机前向加速度噪声；轨迹抖动或速度发散时适当增大
- `gyro_noise`：角速度噪声
- `gyro_bias_noise`：零偏随机游走
- `uwb_default_std`：UWB 未提供 `std` 时使用
- `uwb_gate_mahalanobis`：二维 UWB 创新门限；9.21 约对应 99% 卡方门限
- `map_match_weight`：0 表示不使用规划路径修正；建议 0~0.15
- `map_match_max_distance`：只有在距路径不远时才启用软约束

## 八、核心测试

安装完成后执行：

```bat
set PYTHONPATH=%CD%
.venv\Scripts\python.exe tests\test_core.py
```

## 九、下一阶段建议

1. 先拿到真实 DWG、图层表、单位、车位块命名和 UWB 坐标定义，完成地图转换与控制点配准。
2. 记录一套时间同步的手机 IMU、UWB、人工真值/里程计数据，离线回放调参。
3. 若是“车辆”而非“行人拿手机”，争取增加 OBD/CAN 轮速；手机 IMU 单独承担短时航位推算，UWB承担绝对位置校正。
4. 正式版本增加日志录制/回放、串口或 WebSocket 接口、多楼层图、路径指引语音和偏航重规划。

## 十、本版本新增功能（面向实物测试）

1. **IMU 与 UWB 已拆分，支持无 UWB 停车场**：UDP 监听拆为“启动UDP IMU / 启动UDP UWB”两个独立按钮；界面“启用UWB校正”复选框可随时切换 IMU+UWB 融合与 IMU-only 模式。无 UWB 时仅靠 IMU 航位推算 + 转弯矫正。
   - 配置：`fusion.uwb_enabled`（默认 true）。

2. **过弯矫正累计误差**：新增 `TurnCorrector`（`fusion.py`）。从规划路径提取每个明显转角（航向变化超过阈值），行驶中累计 EKF 航向变化，**检测到转弯后先进入“转弯中”状态、不立即矫正**；等车辆真正转完目标角度、停止转动后，才把航向对齐到转角后的道路方向、并把位置吸附到转角点，周期性消除 IMU 累计误差。适用 90° 转弯、掉头等场景。
   - 关键：矫正发生在**出弯瞬间**，避免旧版“在累计转角刚过阈值(35°)时就提前把航向摆到目标方向，随后陀螺角速度又被积分一次”导致的出弯过转/欠转与斜向漂移（实测逐渐过 90° 弯，出弯航向从旧版的 ~145° 修正为 ~92°）。
   - 配置（`fusion.turn_correction`）：`enabled`、`threshold_deg`（默认 35，判定“开始转弯”的阈值，过低易误触发）、`corner_tolerance`（转角匹配半径 m）、`snap_position`（是否吸附位置）、`cooldown_dist`（冷却距离 m）、`align_deg`（默认 6，判定“已转够目标角度”的容差）、`stop_rate_deg`（默认 1.5，每步角速度低于此值视为“已停止转动”）。

3. **导航终点改为“车道停车点 + 左右反馈”**：规划到车位时路径不再开到车位中心，而是在车位旁的可行驶车道处终止；GUI 提示“车位在您的左侧/右侧”，并在到达时弹出提示。画布上红圈标出目标车位、红点标出停车点。
   - 实现：`planner.plan_to_parking` 返回 `(path, length, side, spot)`。

4. **传感器连接状态反馈（实物测试用）**：界面顶部实时显示 IMU/UWB 的连接状态（● 绿=已连接 / 红=超时 / 灰=未启动）、数据来源（仿真/UDP）、收到帧数、融合模式与过弯矫正已触发次数。

5. **依赖版本说明**：`requirements.txt` 将 PySide6 限制在 `>=6.7,<6.9`。若用 Anaconda 的 Python 建 venv，PySide6 6.9+ 会因 Anaconda 自带 VC++ 运行库(vcruntime140.dll 14.44)过旧而报 `DLL load failed ... 找不到指定的程序`；本版本已实测 PySide6 6.8.3 可用。

## 已知边界

- 示例地图是功能演示，不代表真实停车场。
- 当前规划器遵循有向中心线，但未做 Hybrid A*、Reeds-Shepp 或车辆包络碰撞检查，不用于自动驾驶轨迹控制。
- 当前 UWB 接口为解算后位置，不包含基站测距解算。
- DWG 自动转换效果取决于甲方 CAD 图层和块属性规范。

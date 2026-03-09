# DOFBOT 視覺抓取工作空間 (Visual Grasping Workspace)

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-BSD-green.svg)](LICENSE)

## 📖 專案簡介

本工作空間為 **DOFBOT 5自由度機械臂** 開發的 ROS2 視覺抓取系統，目標是實現 **Sim-to-Real**（仿真到真機）的完整抓取流程。

### 核心功能
- 🦾 機械臂運動控制（MoveIt2 + 自定義IK求解器）
- 📷 視覺檢測系統（OpenCV 顏色識別）
- 🎯 手眼標定（像素坐標到世界坐標轉換）
- 🤖 真機硬體介面（dofbot_hardware 包）
- 🔄 Sim-to-Real 無縫切換（統一運動介面）

---

## 📁 工作空間結構

```
visual_grasping_ws/
├── src/                          # 源代碼目錄
│   ├── dofbot_control/           # 運動控制包
│   │   └── dofbot_control/
│   │       ├── moveit_interface.py   # MoveIt介面類
│   │       ├── unified_interface.py  # 統一運動介面
│   │       └── pick_place_demo.py    # 抓取演示
│   │
│   ├── dofbot_description/       # 機器人描述包
│   │   ├── urdf/                 # URDF模型文件
│   │   │   └── dofbot.urdf       # 主URDF文件
│   │   ├── meshes/               # STL網格模型
│   │   └── launch/               # 啟動文件
│   │
│   ├── dofbot_moveit_config/     # MoveIt2配置包
│   │   ├── config/               # 配置文件
│   │   │   ├── dofbot.srdf       # MoveIt語義配置
│   │   │   └── kinematics.yaml   # 運動學配置
│   │   └── launch/               # MoveIt啟動文件
│   │
│   ├── dofbot_vision/            # 視覺感知包
│   │   ├── dofbot_vision/
│   │   │   ├── vision_processor.py      # HSV顏色檢測
│   │   │   ├── object_detector_node.py  # 目標檢測節點
│   │   │   ├── coordinate_transform_node.py  # 坐標變換
│   │   │   ├── calibrate_hsv.py         # HSV標定工具
│   │   │   └── calibration_tool.py      # 手眼標定工具
│   │   ├── config/               # HSV配置文件
│   │   └── launch/               # 視覺啟動文件
│   │
│   └── dofbot_hardware/          # 硬體介面包
│       ├── dofbot_hardware/
│       │   ├── arm_driver.py         # 真機驅動
│       │   ├── mock_driver.py        # 模擬驅動
│       │   ├── hardware_node.py      # ROS2生命週期節點
│       │   ├── trajectory_executor.py # 軌跡執行器
│       │   └── safety_monitor.py     # 安全監控
│       ├── config/               # 硬體配置
│       └── launch/               # 硬體啟動文件
│
├── docs/                         # 專案文檔
│   ├── todolist.md               # 詳細任務列表
│   ├── spec.md                   # 技術規格說明
│   └── requirements.md           # 需求文檔
│
├── build/                        # 編譯輸出（自動生成）
├── install/                      # 安裝目錄（自動生成）
└── log/                          # 日誌文件（自動生成）
```

---

## 🚀 快速開始

### 環境要求

| 軟體 | 版本 |
|------|------|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble Hawksbill |
| Python | 3.10+ |

### 安裝依賴

```bash
# 安裝 ROS2 Humble（如果尚未安裝）
# 參考：https://docs.ros.org/en/humble/Installation.html

# 安裝編譯工具
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# 初始化 rosdep（首次使用）
sudo rosdep init
rosdep update

# 安裝專案依賴
cd ~/ros_projects/visual_grasping_ws
rosdep install --from-paths src --ignore-src -r -y

# 安裝 Python 依賴
pip3 install ikpy numpy opencv-python
```

### 編譯工作空間

```bash
# 進入工作空間
cd ~/ros_projects/visual_grasping_ws

# 編譯
colcon build --symlink-install

# 載入環境變數
source install/setup.bash
```

---

## 🎮 使用方法

### 1. 啟動仿真環境（Rviz2 + MoveIt2）

```bash
# 載入環境變數
source ~/ros_projects/visual_grasping_ws/install/setup.bash

# 啟動 MoveIt2 演示
ros2 launch dofbot_moveit_config demo.launch.py
```

啟動後，Rviz2 將顯示 DOFBOT 機械臂模型，您可以通過 MotionPlanning 插件進行交互式規劃。

### 2. 運動控制介面

```python
#!/usr/bin/env python3
import rclpy
from dofbot_control.moveit_interface import MoveItInterface

def main():
    rclpy.init()
    
    # 創建介面實例
    arm = MoveItInterface()
    arm.wait_for_server()
    
    # 方式1: 移動到預定義姿態
    arm.move_to_named_target('home')  # 回到初始位置
    arm.move_to_named_target('ready') # 移動到準備姿態
    
    # 方式2: 移動到指定關節角度（弧度）
    joint_angles = [0.0, 0.5, 0.3, 0.0, 0.0]
    arm.move_to_joint_state(joint_angles)
    
    # 方式3: 移動到笛卡爾坐標（米）
    # 注意：5-DOF 機械臂只控制位置，姿態會自然對齊
    arm.move_to_pose(x=0.15, y=0.0, z=0.20)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3. 運行測試腳本

```bash
# 測試運動控制
ros2 run dofbot_control test_motion

# 測試笛卡爾路徑規劃
ros2 run dofbot_control test_cartesian
```

### 4. 啟動視覺系統

```bash
# 啟動相機
ros2 launch dofbot_vision camera.launch.py

# 啟動視覺檢測節點
ros2 launch dofbot_vision vision.launch.py

# 啟動完整視覺管道（相機 + 檢測 + 坐標變換）
ros2 launch dofbot_vision vision_pipeline.launch.py
```

### 5. HSV 顏色標定工具

```bash
# 啟動互動式標定工具
ros2 run dofbot_vision calibrate-hsv --color green

# 手眼標定工具
ros2 run dofbot_vision calibrate-handeye
```

### 6. 硬體介面系統（Phase 3）

```bash
# 啟動模擬硬體節點（用於測試）
ros2 launch dofbot_hardware hardware.launch.py use_mock:=true

# 啟動真機硬體節點
ros2 launch dofbot_hardware hardware.launch.py use_mock:=false

# 啟動完整系統（硬體 + 安全監控）
ros2 launch dofbot_hardware full_system.launch.py
```

### 7. 統一運動介面

```python
#!/usr/bin/env python3
from dofbot_control.unified_interface import create_interface

# 創建模擬介面
sim_interface = create_interface(mode='simulation')
sim_interface.connect()
sim_interface.move_to_named_pose('home')

# 創建真機介面
hardware_interface = create_interface(mode='hardware')
hardware_interface.connect()
hardware_interface.move_to_named_pose('home')

# 控制夾爪
interface.set_gripper(closed=True)   # 閉合
interface.set_gripper(closed=False)  # 張開
```

### 8. 端到端抓取演示

```bash
# 模擬模式抓取演示
ros2 run dofbot_control pick_place_demo --color green --mode simulation

# 真機模式抓取演示
ros2 run dofbot_control pick_place_demo --color green --mode hardware

# 自定義放置位置
ros2 run dofbot_control pick_place_demo --place-x 0.2 --place-y -0.1
```

---

## 📊 開發進度

### ✅ 第一階段：仿真控制（已完成）

- [x] URDF 模型創建與修復
- [x] MoveIt2 配置（SRDF + kinematics.yaml）
- [x] 運動控制介面開發
- [x] 混合 IK 策略（ikpy + MoveIt2）

### ✅ 第二階段：視覺系統（已完成）

- [x] 相機驅動安裝與配置
- [x] `dofbot_vision` 包開發
- [x] HSV 顏色標定工具
- [x] 目標檢測節點
- [x] 手眼標定工具
- [x] 坐標變換節點

### ✅ 第三階段：真機整合（已完成）

- [x] `dofbot_hardware` 包開發
- [x] DofbotDriver 真機驅動（Arm_Lib）
- [x] MockDofbotDriver 模擬驅動
- [x] ROS2 生命週期硬體節點
- [x] 軌跡執行器與安全監控
- [x] 統一運動介面（Sim-to-Real 切換）
- [x] 端到端抓取演示腳本

### 🚧 第四階段：系統優化（進行中）

- [ ] 完整系統整合測試
- [ ] 真機抓取驗證
- [ ] 性能優化與調參

---

## 📚 文檔索引

| 文檔 | 說明 |
|------|------|
| [`todolist.md`](docs/todolist.md) | 詳細任務清單與進度追蹤 |
| [`spec.md`](docs/spec.md) | 技術設計規格說明 |
| [`requirements.md`](docs/requirements.md) | 功能需求文檔 |

---

## 🔧 技術架構

```
┌─────────────────────────────────────────────────────────────┐
│                     系統架構圖                               │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   [USB攝像頭] ──► /image_raw ──► [視覺節點]                │
│                                        │                    │
│                                        ▼                    │
│                                  /target_pose               │
│                                        │                    │
│                                        ▼                    │
│   ┌───────────────────────────────────────────────────┐    │
│   │              MoveIt Interface                      │    │
│   │  ┌─────────────┐      ┌───────────────────────┐  │    │
│   │  │  ikpy (IK)  │ ───► │  MoveIt2 (Planner)    │  │    │
│   │  │  數值求解    │      │  碰撞檢測/軌跡規劃     │  │    │
│   │  └─────────────┘      └───────────────────────┘  │    │
│   └───────────────────────────────────────────────────┘    │
│                                        │                    │
│                                        ▼                    │
│   ┌───────────────────────────────────────────────────┐    │
│   │           Unified Motion Interface                 │    │
│   │  ┌─────────────────┐    ┌─────────────────────┐  │    │
│   │  │  Mock Driver    │    │  DofbotDriver       │  │    │
│   │  │  (模擬)         │    │  (真機 Arm_Lib)     │  │    │
│   │  └─────────────────┘    └─────────────────────┘  │    │
│   └───────────────────────────────────────────────────┘    │
│                                        │                    │
│                                        ▼                    │
│   ┌───────────────────────────────────────────────────┐    │
│   │           Hardware Interface Layer                 │    │
│   │  ┌─────────────────┐    ┌─────────────────────┐  │    │
│   │  │ Safety Monitor  │    │ Trajectory Executor │  │    │
│   │  │ (安全監控)      │    │ (軌跡執行)          │  │    │
│   │  └─────────────────┘    └─────────────────────┘  │    │
│   └───────────────────────────────────────────────────┘    │
│                                        │                    │
│                                        ▼                    │
│                              [DOFBOT 舵機]                  │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## 🛠️ 常見問題

### Q1: 編譯時找不到包依賴？

```bash
# 確保 ROS2 環境已載入
source /opt/ros/humble/setup.bash

# 更新 rosdep 並安裝依賴
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Q2: Rviz2 中無法顯示機器人模型？

檢查 mesh 文件路徑是否正確：
```bash
# 確認 mesh 文件存在
ls src/dofbot_description/meshes/
```

### Q3: MoveIt 規劃失敗？

- 檢查目標位姿是否在工作空間內
- 確認關節限位是否正確配置
- 查看終端輸出的錯誤代碼

---

## 🤝 貢獻指南

1. Fork 本倉庫
2. 創建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 提交 Pull Request

---

## 📄 許可證

本專案採用 BSD-3-Clause 許可證。

---

## 🙏 致謝

- 原始 DOFBOT ROS1 代碼位於 `reference/dofbot_ros1_source/`
- MoveIt2 團隊提供的運動規劃框架
- ikpy 庫提供的逆運動學求解能力

# stretch_main

最小化的 BehaviorTree.CPP 主程式：收到 `/start` 後 tick tree，呼叫 nav2 標準 `navigate_to_pose` action。

## 套件

| Package | 內容 |
|---|---|
| [`btcpp_ros2_interfaces/`](btcpp_ros2_interfaces/) | Vendored ROS2 interfaces (action / srv / msg) |
| [`behaviortree_ros2/`](behaviortree_ros2/) | Vendored BehaviorTree.CPP ↔ ROS2 wrapper（提供 `RosActionNode` 等基底類別） |
| [`bt_nav/`](nav/) | BT action node `NavigateToPose`，呼叫 `nav2_msgs/action/NavigateToPose` |
| [`bt_engine/`](engine/) | BT engine：載入 XML、註冊 `NavigateToPose`、訂閱 `/start` 後開始 tick |

其他：
- [`docker/Dockerfile.ci`](docker/Dockerfile.ci) — 共用 image（CI 與本地都用同一份）
- [`docker/docker-compose.yaml`](docker/docker-compose.yaml) — 本地 `build` / `dev` 兩個 service
- [`run.sh`](run.sh) — tmux 啟動腳本

## 系統需求

- ROS 2 Humble
- apt：`ros-humble-behaviortree-cpp`、`ros-humble-nav2-msgs`、`ros-humble-navigation2`、`ros-humble-tf2-geometry-msgs`、`ros-humble-generate-parameter-library`

> `behaviortree_ros2` 與 `btcpp_ros2_interfaces` 已 vendored 在 repo 內，不需另外 clone。

## Build

### Native

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to bt_engine
source install/setup.bash
```

### Docker（與 CI 同 image）

```bash
# 一次性 build
docker compose -f docker/docker-compose.yaml run --rm build

# 進開發 shell（workspace 掛在 /ws）
docker compose -f docker/docker-compose.yaml run --rm dev
```

## 執行

### 1. 啟動 nav2 `navigate_to_pose` action server

正常情況：把實機 / Gazebo + nav2 stack 跑起來。

無實機快速驗證：
```bash
ros2 run nav2_util fake_action_server navigate_to_pose
```

### 2. 跑 BT engine（推薦：tmux 兩格）

```bash
./run.sh
```

- **左 pane**：`ros2 run bt_engine bt_engine`，啟動後等 `/start`
- **右 pane**：輸入 `s` + Enter → 發 `std_msgs/Empty` 到 `/start`，tree 開始 tick
- 輸入 `q` + Enter 退出右 pane

### 手動方式（不用 tmux）

terminal 1：
```bash
ros2 run bt_engine bt_engine
```

terminal 2：
```bash
ros2 topic pub --once /start std_msgs/msg/Empty {}
```

## BT XML

預設 tree 在 [`engine/bt/main_tree.xml`](engine/bt/main_tree.xml)：

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence>
      <NavigateToPose x="1.0" y="0.0" yaw="0.0" frame_id="map"/>
    </Sequence>
  </BehaviorTree>
</root>
```

`NavigateToPose` ports：

| Port | Type | Default | 說明 |
|---|---|---|---|
| `x` | double | — | 目標 x（必填） |
| `y` | double | — | 目標 y（必填） |
| `yaw` | double (rad) | `0.0` | 目標朝向 |
| `frame_id` | string | `"map"` | 參考座標系 |

修改 XML 後重 build（CMake install 會 copy 到 `share/bt_engine/bt/`），或直接以參數指向其他 XML：

```bash
ros2 run bt_engine bt_engine --ros-args -p bt_xml_path:=/abs/path/to/your_tree.xml
```

其他可調參數：
- `tree_name` (default `"MainTree"`)
- `tick_rate_hz` (default `20`)

## CI

[`.github/workflows/ci.yml`](.github/workflows/ci.yml) 在每次 push / PR 時：
1. 用 [`docker/Dockerfile.ci`](docker/Dockerfile.ci) build 出最小 image（layer 用 GHA cache）
2. 容器內 `colcon build --packages-up-to bt_engine`

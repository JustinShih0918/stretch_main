# stretch_main

以 BehaviorTree.CPP 為核心的 Stretch 機器人工作區：BT engine 收到 `/start` 後 tick tree，呼叫 nav2 標準 `navigate_to_pose` action。同一套 BT engine 同時驅動 **模擬（Isaac Sim）** 與 **實機（on-robot）** 兩種環境。

## 倉庫結構

所有 colcon 套件都放在 [`src/`](src/) 底下，依用途分組：

| 目錄 | 套件 | 說明 |
|---|---|---|
| [`src/common/`](src/common/) | `btcpp_ros2_interfaces` | Vendored ROS2 interfaces (action / srv / msg) |
| | `behaviortree_ros2` | Vendored BehaviorTree.CPP ↔ ROS2 wrapper（`RosActionNode` 等基底類別） |
| | `bt_nav` | BT action node `NavigateToPose`，呼叫 `nav2_msgs/action/NavigateToPose` |
| | `bt_engine` | BT engine：載入 XML、註冊 `NavigateToPose`、訂閱 `/start` 後 tick |
| [`src/sim/`](src/sim/) | `stretch3_navigation` | Isaac Sim 用 nav2 / SLAM 設定與 launch（移植自 [j3soon/ros2-essentials](https://github.com/j3soon/ros2-essentials) `stretch3_ws`） |
| | `stretch_urdf` | Stretch URDF 產生工具（hello-robot pip 套件，非 colcon 套件） |
| [`src/deploy/`](src/deploy/) | `stretch_nav2` | 實機 nav2 + slam_toolbox + AMCL（vendored 自 [hello-robot/stretch_ros2](https://github.com/hello-robot/stretch_ros2)，Apache-2.0） |

> `src/common/*` 在三種 image 都會 build；`src/sim/*` 只在 sim image build；`src/deploy/*` 只在 deploy image build。套件選擇是用 `--packages-up-to`，與目錄分組無關（colcon 會遞迴掃整個 `src/`）。

## 三種 Docker 環境

| 環境 | 路徑 | 用途 | 指令（在 repo 根目錄執行） |
|---|---|---|---|
| **ci** | [`docker/ci/`](docker/ci/) | 最小化 build / test image（CI 與本地共用） | `docker compose -f docker/ci/docker-compose.yaml run --rm build` |
| **sim** | [`docker/sim/`](docker/sim/) | Isaac Sim 5.1 模擬環境（GPU / X11） | `docker compose -f docker/sim/compose.yaml run --rm stretch3-ws` |
| **deploy** | [`docker/deploy/`](docker/deploy/) | 實機部署 image（nav2 + slam） | `docker compose -f docker/deploy/docker-compose.yaml run --rm build` |

其他：[`run.sh`](run.sh) — tmux 啟動腳本（任一環境 build 後皆可用）。

## ci — build / 驗證

### Native
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to bt_engine
source install/setup.bash
```

### Docker（與 CI 同 image）
```bash
docker compose -f docker/ci/docker-compose.yaml run --rm build   # 一次性 build
docker compose -f docker/ci/docker-compose.yaml run --rm dev      # 開發 shell（/ws）
```

## sim — Isaac Sim 模擬

需要 NVIDIA GPU + nvidia-container-toolkit。

```bash
# 第一次先建立 cache volume 的權限
docker compose -f docker/sim/compose.yaml up -d volume-instantiation
# 進入容器（首次進入會自動 colcon build common + stretch3_navigation）
docker compose -f docker/sim/compose.yaml run --rm stretch3-ws
```

容器內：啟動 Isaac Sim 提供 `world -> odom -> base_link` TF 與 `/laser_scan`、`/odom`，再跑：

```bash
ros2 launch stretch3_navigation navigation.launch.py   # 純 nav2（無 SLAM）
# 或 cartographer.launch.py / rtabmap.launch.py 做 SLAM
```

之後即可用下方「執行 BT engine」驅動 `navigate_to_pose`。

## deploy — 實機部署

在機器人上執行。**Stretch 硬體驅動（`stretch_core`）與校正後 URDF 來自機器人本身的 hello-robot 安裝**，需先 source 成 underlay（詳見 [src/deploy/README.md](src/deploy/README.md)）。

```bash
docker compose -f docker/deploy/docker-compose.yaml run --rm build   # build common + stretch_nav2
docker compose -f docker/deploy/docker-compose.yaml run --rm nav     # ros2 launch stretch_nav2 navigation.launch.py
docker compose -f docker/deploy/docker-compose.yaml run --rm bt      # ros2 run bt_engine bt_engine
```

## 執行 BT engine

無實機 / 無模擬快速驗證，可先起一個假的 action server：
```bash
ros2 run nav2_util fake_action_server navigate_to_pose
```

推薦用 tmux 兩格：
```bash
./run.sh
```
- **左 pane**：`ros2 run bt_engine bt_engine`，啟動後等 `/start`
- **右 pane**：輸入 `s` + Enter → 發 `std_msgs/Empty` 到 `/start`，tree 開始 tick；`q` + Enter 退出

手動方式：
```bash
# terminal 1
ros2 run bt_engine bt_engine
# terminal 2
ros2 topic pub --once /start std_msgs/msg/Empty {}
```

## BT XML

預設 tree 在 [`src/common/bt_engine/bt/main_tree.xml`](src/common/bt_engine/bt/main_tree.xml)：

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

修改 XML 後重 build（CMake install 會 copy 到 `share/bt_engine/bt/`），或以參數指向其他 XML：
```bash
ros2 run bt_engine bt_engine --ros-args -p bt_xml_path:=/abs/path/to/your_tree.xml
```

其他可調參數：`tree_name`（default `"MainTree"`）、`tick_rate_hz`（default `20`）。

## CI

[`.github/workflows/ci.yml`](.github/workflows/ci.yml) 在每次 push / PR：
1. 用 [`docker/ci/Dockerfile.ci`](docker/ci/Dockerfile.ci) build 出最小 image（layer 用 GHA cache）
2. 容器內 `colcon build --packages-up-to bt_engine`

export LD_LIBRARY_PATH=/home/user/isaacsim/kit/python/lib/python3.11/site-packages/nvidia/cusparselt/lib:$LD_LIBRARY_PATH

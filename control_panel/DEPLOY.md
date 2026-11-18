# 實機部署：Stretch 3 + Jetson Thor + 控制面板

把控制面板（`control_panel/`）裝到真的機器人上的步驟，以及常見錯誤怎麼查。
架構與設計原因見 [README.md](README.md)。

> **狀態**：面板的機器人控制（步驟 1-4）已在實機測過。OSG 分頁（步驟 5）只在假機器人
> 與假 OSG（`tests/e2e_fake.sh`）上測過，`thor` image 還沒在 Thor 上 build 過。第一次上機請兩個人：一人拿平板，
> 一人站在機器人旁、手放在實體急停按鈕上。

```
平板/手機 ──Wi-Fi──▶ Thor :8080（panel 容器）──有線 DDS──▶ Stretch（supervisor，原生執行）
                     192.168.100.2                         192.168.100.1
```

---

## 0. 取得程式（兩台都做）

```bash
cd ~/stretch_main
git fetch && git checkout feat/sg && git pull
git submodule update --init --recursive     # 只有 Thor 需要（之後跑 OSG 才用到）
```

## 1. 網路（兩台都做）

1. 有線網路：機器人 `192.168.100.1`、Thor `192.168.100.2`，互相 `ping` 得到。
2. 記下**自己這台**的有線網卡名稱：`ip -brief addr`（Thor 是 `enP2p1s0`）。
3. 放大 DDS 收包緩衝（一次就好；不做的話影像會卡頓或消失，且不會報錯）：

   ```bash
   echo 'net.core.rmem_max=134217728' | sudo tee /etc/sysctl.d/60-cyclonedds.conf
   sudo sysctl --system
   ```

兩端必須一致的三件事：`ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、
DDS 設定檔 `docker/cyclonedds-eth.xml`（`CYCLONEDDS_IFACE` 各自填自己的網卡）。

## 2. 機器人：啟動 supervisor（先用原生方式）

supervisor 會用機器人上 hello-robot 原生安裝的 driver、光達、相機 launch，所以第一次
請直接在機器人上跑，不要用容器。

1. 確認套件都在：

   ```bash
   ros2 pkg list | grep -E "stretch_core|stretch_nav2|slam_toolbox|nav2_map_server"
   ```

   hello-robot 的 workspace 不在 `~/ament_ws` 時：`export HELLO_WS=<路徑>`。
2. 確認沒有其他程式佔用機器人（其他終端機的 driver、官方 launch 都要先關）；
   必要時執行 `stretch_free_robot_process.py`。
3. 在**一般的互動式終端機**啟動（這樣才帶得到 hello-robot 的 `HELLO_FLEET_ID` 等變數）：

   ```bash
   export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=0
   export CYCLONEDDS_URI=$HOME/stretch_main/docker/cyclonedds-eth.xml
   export CYCLONEDDS_IFACE=<機器人的有線網卡>
   python3 ~/stretch_main/control_panel/supervisor/supervisor.py
   ```

   看到 `panel supervisor up: 7 units, maps in ..., logs in ...` 即成功。
   Ctrl-C 會依序關閉它開過的所有模組（導航先、driver 最後）。

地圖存在 `~/stretch_maps/`（`PANEL_MAPS_DIR`），各模組 log 在 `~/.ros/panel_logs/<模組>.log`。

### 之後改成開機自動啟動（手動跑穩了再做）

用 [`supervisor/stretch-panel-supervisor.service`](supervisor/stretch-panel-supervisor.service)
（檔內有安裝指令）。**先改兩處**：

- `CYCLONEDDS_IFACE=enp0s31f6` 是預設猜測值，改成機器人實際的網卡名稱。
- systemd 不會讀 `~/.bashrc`，要自己加上
  `Environment=HELLO_FLEET_ID=stretch-se3-XXXX` 和 `Environment=HELLO_FLEET_PATH=/home/hello-robot/stretch_user`，
  否則 driver 會報 `ERROR: Must be run on a robot.`

（`docker/deploy/docker-compose.yaml` 也有 `supervisor` 容器版本，需要掛入 hello-robot 的
workspace、fleet 目錄與 `/etc/hello-robot`，尚未在機器人上試過。）

## 3. Thor：啟動控制面板

```bash
cd ~/stretch_main
cp docker/thor/.env.example docker/thor/.env
#   編輯 .env：UID/GID 填 `id -u` / `id -g`；ROS_DOMAIN_ID 與機器人相同；
#   CYCLONEDDS_IFACE=enP2p1s0；PANEL_PORT（預設 8080）
docker compose -f docker/thor/compose.yaml --env-file docker/thor/.env build panel
docker compose -f docker/thor/compose.yaml --env-file docker/thor/.env up -d panel
docker logs -f stretch-panel           # 應看到 [panel] http://0.0.0.0:8080
```

只 build `panel` 用的小 image（ros:humble-ros-base + 幾個訊息套件），幾分鐘內完成；
OSG 本體（`thor` image）此時不需要。

確認 DDS 通了（要看到 `/panel/supervisor/state`）：

```bash
docker exec stretch-panel bash -c "source /opt/ros/humble/setup.bash && ros2 topic list | grep panel"
```

平板／手機開 `http://<Thor 的 Wi-Fi IP>:8080`。

## 4. 第一次上機（每步確認了再往下）

| # | 在哪 | 做什麼 | 應該看到 |
|---|---|---|---|
| 1 | 系統 | 底盤驅動 Driver →「啟動」 | 上方出現 `navigation`、電壓；模組顯示「執行中」 |
| 2 | 系統 | 「解除急停」→「歸零 Home」（**手臂、升降會動，周圍淨空**） | 徽章變「已歸零」 |
| 3 | 系統 | 光達 Lidar →「啟動」 | 執行中 |
| 4 | 駕駛 | 速度調 20%，前進、轉彎、放開；再試一次「急停」 | 放開立刻停；急停後推搖桿不會動 |
| 5 | 系統 | 「開始建圖」（依序開 driver → 光達 → 建圖 → 導航） | 地圖分頁出現地圖，紅色雷射點貼著牆 |
| 6 | 駕駛／地圖 | 手動開一圈；地圖分頁輸入名稱 →「儲存地圖」 | 「saved map …」；機器人 `~/stretch_maps/<名稱>.posegraph/.pgm/.yaml` |
| 7 | 系統 | 關閉「導航」「建圖」→ 選地圖 →「載入地圖並導航」 | 定位、導航執行中 |
| 8 | 地圖 | 紅點沒對上牆時：「設定位置」，按在機器人實際位置、拖出朝向、放開 | 紅點貼回牆面 |
| 9 | 地圖 | 「導航目標」：先點一個近的點、拖出朝向 | 藍色路徑，狀態「導航中」→「已到達」 |

操作提示：

- 面板駕駛和 Nav2 都需要 driver 在「導航」模式；Hello 手把需要切到「手把」模式（系統分頁）。
- 推一下搖桿會取消正在進行的導航目標。
- 手指放開、網頁當掉、斷 Wi-Fi、切到背景 → 0.4 s 內停車；driver 自己另有 0.5 s 逾時。
- 面板**沒有登入**：連得到 8080 的人都能開車，請只放在實驗室網路。

---

## 可能的錯誤

### 連線 / DDS

| 現象 | 原因 | 怎麼辦 |
|---|---|---|
| 網頁打不開 | Thor 防火牆擋 8080，或平板不在同一網路 | `sudo ufw allow 8080/tcp`；用 Thor 的 Wi-Fi IP，不是 192.168.100.2 |
| 左上角圓點紅色 | 網頁連不到 gateway | `docker logs stretch-panel`；容器有沒有在跑 |
| 「supervisor 沒有回應」，但下方有列出「有在跑」的模組 | DDS 通，supervisor 沒開 | 回到步驟 2 啟動 supervisor |
| 什麼都偵測不到、`ros2 topic list` 看不到 `/panel/...` | DDS 沒通 | 兩端 `ROS_DOMAIN_ID`、`RMW_IMPLEMENTATION` 一致？`CYCLONEDDS_IFACE` 是**自己這台**的網卡？網路線有接？ |
| 啟動時 `does not match an available interface` | 網卡名稱錯，或網卡沒有連線（沒插線） | `ip -brief addr` 確認名稱與狀態 UP |
| `can't open configuration file` | `CYCLONEDDS_URI` 路徑錯 | 容器內是 `/etc/cyclonedds-eth.xml`；原生是 `~/stretch_main/docker/cyclonedds-eth.xml` |
| `Failed to find a free participant index` | 參與者太多（舊設定檔） | 確認用的是 repo 的 `docker/cyclonedds-eth.xml`（`MaxAutoParticipantIndex 60`） |
| 狀態都看得到，但按鈕、導航都沒反應或逾時 | 某一端在跑 Fast-DDS（topic 能互通，service/action 不行） | 機器人上**每一個**終端機都要 `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；`ros2 topic info -v /tf` 的 GID 開頭 `01.0f` 是 Fast-DDS、`01.10` 是 Cyclone |
| 相機畫面、地圖卡頓或消失，沒有錯誤 | `rmem_max` 沒調 | 步驟 1.3，兩台都要 |

### 模組（系統分頁顯示「失敗」時，下方會有最後幾行 log；完整 log 在 `~/.ros/panel_logs/`）

| 現象 | 原因 | 怎麼辦 |
|---|---|---|
| driver：`ERROR: Must be run on a robot.` | 沒有 `HELLO_FLEET_ID` | supervisor 要從互動式終端機啟動；systemd 要寫 `Environment=HELLO_FLEET_ID=...` |
| driver 啟動即失敗、log 提到 USB／裝置被佔用 | 另一個程式在用機器人 | 關掉其他 driver／launch；`stretch_free_robot_process.py` |
| driver／光達／相機：`package 'stretch_core' not found` | 沒 source 到 hello-robot workspace | `export HELLO_WS=<路徑>`（supervisor 會 source `$HELLO_WS/install/setup.bash`） |
| 「請先啟動 driver」「stop mapping first」 | 依賴／互斥檢查 | 照提示的順序開關；建圖與定位不能同時開 |
| 建圖：地圖一直沒出現 | 光達沒開，或 `/scan_filtered` 沒資料 | 光達模組有在跑？`ros2 topic hz /scan_filtered` |
| 定位：`no saved map 'xxx'` | 該地圖沒有 `.posegraph` | 必須在「建圖」執行中按「儲存地圖」；檔案在 `~/stretch_maps/` |
| 導航模組失敗，log 提到 `params_file` 或參數 | 機器人上官方 `stretch_nav2` 的 `navigation_launch.py` 與 repo 版本不同 | 把 log 貼出來；必要時改 `supervisor/units.yaml` 的 navigation 指令 |
| 相機執行中但駕駛分頁沒畫面 | 沒有壓縮影像 topic（需要 `image_transport_plugins`） | `ros2 topic list | grep compressed`；沒有就在機器人上 `sudo apt install ros-humble-image-transport-plugins` |
| 相機畫面方向不對 | 頭部相機側裝 | 駕駛分頁「旋轉」鈕循環 90°／-90°／0° |
| 「關閉中」很久 | 模組沒理 Ctrl-C | 會自動在 8 s 後 SIGTERM、再 4 s 後 SIGKILL，等一下即可 |

### 操作

| 現象 | 原因 | 怎麼辦 |
|---|---|---|
| 推搖桿不會動 | driver 不在「導航」模式、在急停中、或沒歸零 | 駕駛分頁上方黃條會說明；系統分頁切模式／解除急停 |
| 歸零失敗 | 急停中 | 先「解除急停」 |
| 導航目標「被拒絕」或沒反應 | 導航模組沒開，或沒有 `map` 座標（建圖／定位沒開） | 系統分頁確認導航 + 建圖或定位都在執行中 |
| 地圖上沒有機器人箭頭 | 沒有 `map → base_link` TF | 建圖或定位沒在跑；只有 driver 時只會有 `odom` |
| 定位後紅點對不上牆 | 起始位置錯 | 用「設定位置」手動給位置與朝向 |
| 導航途中位置大跳 | 定位失準 | 停下、重新「設定位置」；不要在建圖和搜尋之間重開 SLAM（OSG `docs/THOR.md`） |

---

## 5. OSG（場景圖搜尋）：從面板操作

控制面板的 **OSG** 分頁可以啟動模型、預檢、選目標、跑建圖趟／搜尋趟、跑滅火器 demo。
說明見 [README.md](README.md)「OSG from the panel」。

### Thor：第一次設定（一次就好）

```bash
cd ~/stretch_main
git submodule update --init --recursive
source ~/.nvidia_api.env          # 滅火器檢查（VLM）要用 NVIDIA_API_KEY；不放進 .env
docker compose -f docker/thor/compose.yaml --env-file docker/thor/.env build thor bridge
docker compose -f docker/thor/compose.yaml --env-file docker/thor/.env up -d ollama bridge thor panel
```

`thor` 很大、build 很久；卡在 BLIP-2 可在 `.env` 設 `WITH_BLIP2=0`（OSG `docs/THOR.md`）。
`thor` 容器現在預設跑面板的 runner（`control_panel/osg_runner`），面板就能啟動 OSG。

下載模型權重（一次）：

```bash
docker exec -it osg-thor bash
python scripts/download_weights.py --profile large
mkdir -p outputs/dualmap_comparison/vendor/DualMap/model && \
  (cd outputs/dualmap_comparison/vendor/DualMap/model && python -c "from ultralytics import FastSAM; FastSAM('FastSAM-s.pt')")
bash scripts/fetch_ascent_weights.sh
exit
docker exec osg-ollama ollama pull qwen2.5:7b
```

### 每次測試

1. **系統**：driver、光達、**相機**、建圖（或定位）、導航都在執行中；driver「導航」模式、已歸零、沒有急停。
2. **OSG → ① 啟動**：感知模型「啟動」，等約 90 秒。
3. **OSG → ② 預檢**：全部 ✅（⚠️ 是建議項，不擋執行）。
4. 選一個：
   - **③ 任務**：選模式與目標（或自己打字）、地圖名稱 → 「建圖趟」；搬動物體 → 同一個地圖名稱「搜尋趟」。
   - **④ Demo 流程**：依序按「執行」：預檢 → 建圖趟 → 搬 #1 後按「繼續」 → 搜尋 #1 並檢查。
5. **進度**：看階段（搜尋中 → 確認中 → 檢查中 → 判定）、判定卡片與 OSG 輸出；地圖分頁有場景圖物體（橘色 = 目標）。

注意：

- **建圖趟和搜尋趟之間不要重開 SLAM**（場景圖存在當時的 `map` 座標系）。真的要重開：
  先在地圖分頁「儲存地圖」，之後用「載入地圖並導航」在同一張地圖上定位。
- OSG 跑的時候不要用面板搖桿：碰搖桿會取消 Nav2 目標。要停用「停止 OSG」或「急停」。
- 每次的完整紀錄在 Thor 的 `src/semantic_nav/object_scene_graph/outputs/<時間>/`；面板顯示的輸出另存在 `outputs/panel_logs/`。

### OSG 的可能錯誤

| 現象 | 原因 | 怎麼辦 |
|---|---|---|
| ①「OSG 容器沒有回應」 | `osg-thor` 沒在跑，或是舊 image 沒有 runner | `up -d thor`；`docker logs osg-thor` 應有 `[osg_runner] http://127.0.0.1:18790` |
| 感知模型「失敗」或預檢 13182/13185 ❌ | 權重沒下載、GPU 記憶體不足 | `docker exec osg-thor tmux attach -t osg_models` 看各視窗；先做「下載模型權重」 |
| 預檢 ollama ❌ | 沒有 `qwen2.5:7b` | `docker exec osg-ollama ollama pull qwen2.5:7b` |
| 預檢 VLM 金鑰 ❌（滅火器模式） | `up` 時環境沒有 `NVIDIA_API_KEY` | `source ~/.nvidia_api.env` 後 `up -d thor`（重建容器） |
| 預檢相機 ❌ | 相機沒開，或機器人沒有 image_transport 壓縮 plugin | 系統分頁啟動相機；機器人上 `sudo apt install ros-humble-image-transport-plugins` |
| 預檢 OSG bridge ❌ | `osg-bridge` 沒在跑或 DDS 不通 | `up -d bridge`；`docker logs osg-bridge` |
| 「target may be letters, digits…」 | 目標含特殊字元 | 只能用字母、數字、中文、空白、`# . -` |
| 「no saved map 'xxx'」 | 這個名字沒跑過建圖趟 | 先跑建圖趟（或從下拉選已存的地圖） |
| 搜尋趟一開始就結束，輸出有 `FileNotFoundError` / Fast-DDS | 地圖不存在，或機器人用了 Fast-DDS | 看輸出最後幾行；RMW 見上面 DDS 表 |
| 一直印 `POSE JUMP` | 定位跑掉 | 停下 OSG，重新「設定位置」；不要重開 SLAM |
| 停在不是目標的東西前面 | 偵測誤判 | 會自動「不是目標，繼續找」（最多 3 次）；持續發生把該次 `outputs/` 給開發者 |
| 判定「無法判定」或錯誤 | VLM 逾時、金鑰錯、畫面太暗 | 判定卡片有錯誤訊息；`outputs/<時間>/inspection.json` |

## 收工

- 系統分頁「全部關閉」，或在機器人上對 supervisor 按 Ctrl-C（會依序關閉所有模組）。
- OSG 分頁「停止 OSG」、感知模型「關閉」。
- Thor：`docker compose -f docker/thor/compose.yaml --env-file docker/thor/.env stop panel thor`

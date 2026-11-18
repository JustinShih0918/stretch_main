# Stretch control panel

A web page for a phone or tablet that runs the Stretch 3: bring its parts up one at a
time, drive it, map, save the map, relocalise on it, send Nav2 goals. No terminal, no
`ros2 launch` with five arguments. The scene graph (OSG) is the next tab.

```
 phone / tablet ──HTTP + WebSocket──▶ ┌─ Jetson Thor ─────────────────┐            ┌─ Stretch 3 ──────────────────────┐
 http://<thor>:8080                   │ panel (gateway/)              │            │ supervisor (supervisor/)          │
                                      │  page, cmd_vel, driver        │  CycloneDDS │  starts / stops the units:       │
                                      │  services, Nav2 goals,        │◀──wired────▶│  driver · lidar · camera ·       │
                                      │  /initialpose, map, TF, scan  │  link       │  mapping · localization · Nav2 · │
                                      │  docker/thor `panel`          │            │  joystick; saves maps             │
                                      └───────────────────────────────┘            └──────────────────────────────────┘
```

## Why it is split this way

hello-robot's launch files each bring up the driver **and** the lidar **and** SLAM or
Nav2 together (`stretch_nav2` `navigation.launch.py`, `offline_mapping`, `online_mapping`),
so going from mapping to navigating meant killing the driver and starting another bundle.
But the driver can change mode without a restart: `/switch_to_navigation_mode`,
`/switch_to_gamepad_mode`, `/switch_to_position_mode`, `/home_the_robot`,
`/stow_the_robot`, `/runstop` are services, and `/stretch/cmd_vel` is obeyed in
navigation mode. So the panel starts the driver once and treats everything else as a
**unit** that comes and goes around it (`supervisor/units.yaml`):

| unit | runs | needs |
|---|---|---|
| `driver` | `stretch_core stretch_driver.launch.py mode:=navigation broadcast_odom_tf:=True` | |
| `lidar` | `stretch_core rplidar.launch.py` (`/scan`, `/scan_filtered`) | |
| `camera` | `stretch_core d435i_high_resolution.launch.py` | |
| `joystick` | `stretch_nav2 teleop_twist.launch.py` (a USB joystick; the Hello gamepad needs no unit, only gamepad mode) | driver |
| `mapping` | `slam_toolbox` mapping mode, `/scan_filtered` | driver, lidar; not with localization |
| `localization` | `slam_toolbox` localization mode on a saved pose graph | driver, lidar, a map; not with mapping |
| `navigation` | `stretch_nav2 navigation_launch.py` (Nav2 without a localiser) | driver, lidar, mapping **or** localization |

`slam_toolbox` provides `map -> odom` in both modes, so one localiser serves mapping and
navigating, and Nav2 runs on top of either. (OSG `docs/THOR.md` has why it is slam_toolbox
and not AMCL on this robot: AMCL jumped 6 m on an unconverged start.)

**Who runs what.** Starting a process needs the robot: the driver, lidar and camera need its
USB devices and hello-robot's install, and ROS 2 has no way to start a process on another
machine. That is the **supervisor's** whole job, on the Stretch. Everything else the
**gateway** on the Thor does directly over DDS, the same DDS the OSG bridge uses
(`docker/cyclonedds-eth.xml`, wired link, `192.168.100.1` robot / `.2` Thor). The page talks
only to the gateway, so a tablet on Wi-Fi needs no ROS and no DDS.

The two talk over three topics, JSON in `std_msgs/String`, so no interface package has to
be built on either side: `/panel/supervisor/request`, `/panel/supervisor/reply`, and the
latched `/panel/supervisor/state` (units, their last log lines, saved maps).

## Running it

**On the Stretch: the supervisor.** It runs hello-robot's own install, so natively first:

```bash
cd ~/stretch_main && git pull
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=$PWD/docker/cyclonedds-eth.xml \
       CYCLONEDDS_IFACE=<the robot's wired NIC>      # `ip -brief addr`
python3 control_panel/supervisor/supervisor.py      # Ctrl-C stops every unit it started
```

or as a service that starts with the robot: `supervisor/stretch-panel-supervisor.service`
(instructions inside). There is also a `supervisor` service in
`docker/deploy/docker-compose.yaml` that mounts hello-robot's workspace, fleet directory and
`/etc/hello-robot`; it has not been tried on the robot yet. Maps go to `~/stretch_maps`
(`PANEL_MAPS_DIR`), unit logs to `~/.ros/panel_logs/<unit>.log`.

**On the Thor: the panel.**

```bash
cp docker/thor/.env.example docker/thor/.env      # UID/GID, CYCLONEDDS_IFACE, PANEL_PORT
docker compose -f docker/thor/compose.yaml --env-file docker/thor/.env up -d panel
```

then open `http://<thor's address>:8080` on the tablet. Without docker, on any machine with
ROS 2 Humble and the DDS settings above: `cd control_panel && python3 -m gateway`.

**Both ends must agree** on `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and the
DDS file. If the page says the supervisor is not answering but lists units as "detected",
DDS works and only the supervisor is down; if nothing is detected, it is DDS (OSG
`docs/ROS2.md` lists the failure modes, e.g. a Fast-DDS robot whose topics flow while its
services do not).

## Using it

- **系統 (System)**: home / stow / stop the arm, release the runstop, the driver mode, one
  switch per unit with its state and, when one dies, its last log lines. **開始建圖** brings
  up driver → lidar → mapping → Nav2 in order; **載入地圖並導航** does driver → lidar →
  localization on the chosen map → Nav2.
- **地圖 (Map)**: the map, the robot, the laser scan over it (red; when it lines up with the
  walls, the robot is localised), and Nav2's plan. *導航目標*: press where to go, drag
  for the heading, release. *設定位置*: the same gesture sets `/initialpose`, which is how a
  robot that starts lost on a saved map is relocalised. **儲存地圖** saves the mapping
  session as a pose graph (what localization loads) plus a `.pgm`/`.yaml` image.
- **駕駛 (Drive)**: the head camera and a joystick. Drive needs the driver in 導航
  (navigation) mode; the page says so when it is not. Keys W A S D on a laptop.
- **急停 STOP** (always on screen): teleop off, the Nav2 goal cancelled, a zero twist, and
  the driver's `/runstop` on. 解除急停 releases it.

### Safety, as built

- A drive command only stands while the page keeps sending it (every 100 ms while a finger
  is down; the gateway drops it after 0.4 s), and the driver has its own 0.5 s cmd_vel
  timeout behind that. A frozen page, a dropped Wi-Fi, a backgrounded tab or a closed
  page all stop the base.
- Speeds are clamped in the gateway (`PANEL_MAX_LINEAR` 0.3 m/s, `PANEL_MAX_ANGULAR`
  1.0 rad/s), not taken from the page.
- Touching the stick cancels a running Nav2 goal: the operator takes over, the two never
  fight over cmd_vel.
- Homing and stowing ask for confirmation; they move the arm and lift.
- The supervisor refuses a unit whose prerequisites are down (`start driver first`) or
  that conflicts with one that is up (`stop mapping first`). Stopping a unit is Ctrl-C to
  its whole process group, then SIGTERM, then SIGKILL, until nothing of it is left.
- There is no login: anyone who can reach the port can drive the robot. Keep it on the
  lab network.

## Tests

```bash
cd control_panel && python3 -m pytest tests -q        # 14 tests, no ROS graph needed
bash control_panel/tests/e2e_fake.sh                  # everything, against a fake robot
```

`e2e_fake.sh` runs the supervisor (with `tests/fake_units.yaml`: the same units, played by
`tools/fake_stretch.py` under the real node, topic, service and action names), the gateway,
and a scripted session through the page's WebSocket: start each unit and the refusals,
gamepad mode ignores the panel, teleop drives and stops on release, a Nav2 goal arrives, a
stick touch cancels a goal, save a map (the real `ros2 service call` and `map_saver_cli`),
localise on it, set the pose, E-stop, stop all. It uses ROS domain 77 with
`ROS_LOCALHOST_ONLY=1`, whatever the shell exports, because the fake publishes
`/stretch/cmd_vel`. `KEEP=1` leaves it up at `http://localhost:18080`.

Not tested yet: the real Stretch, and the two docker images (built from these files, but
not built in the environment this was written in).

## Next: the scene graph

The 場景圖 tab is a placeholder. OSG already publishes its graph on `/osg/*` through the
bridge (`debug_stream.py`); the gateway can subscribe to it, draw the objects and rooms on
the same map, and start a search (`run_robot.py`) from the page.

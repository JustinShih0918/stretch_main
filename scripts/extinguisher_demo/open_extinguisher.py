# Isaac Sim startup script for the extinguisher demo: open the hospital-with-extinguisher
# stage, select the extinguisher's condition, press Play, then keep watching a control
# file so the recorder can switch the condition and reset the scene between
# takes without reloading the stage or clicking Isaac's toolbar.
#
#   EXTINGUISHER_CONDITION=bad ./isaac-sim.sh --exec /home/user/stretch_main/scripts/extinguisher_demo/open_extinguisher.py
#
# The control file ($EXTINGUISHER_CONTROL) holds one line, "<condition> [<token>]":
#   echo good > $EXTINGUISHER_CONTROL         switch the extinguisher, keep playing
#   echo "bad take3" > $EXTINGUISHER_CONTROL  Stop (robot and objects back to their
#                                        authored poses), switch, Play; then write
#                                        "take3" to $EXTINGUISHER_CONTROL.ack
# Applied within half a second. EXTINGUISHER_CLEAN_UI=1 hides every panel but the
# viewport and its overlays, for recording.
#
# A second control file, $EXTINGUISHER_CONTROL.move, moves extinguishers while the
# scene plays (move_extinguisher.sh writes it). One line per move:
#   1 -7.6 1.85 -80        extinguisher #1 (/World/FireExtinguisher) to x, y (m), yaw (deg)
#   2 home                 #2 (/World/FireExtinguisher2) back to where the stage put it
# They have colliders but no rigid body, so a move stays put across Stop / Play.
# EXTINGUISHER_STAGE=.../stretch3_og_hospital_two_extinguishers.usda opens the
# stage with two.
#
# EXTINGUISHER_FOLLOW_CAMERA=close puts the third-person camera 1.7 m behind the robot
# (not 2.33 m) with a wider lens. Inspecting #1 by the waiting chairs, the robot stands
# 2 m from the corridor wall facing away from it, and the stage's camera ends up on
# the far side of that wall (a blank yellow viewport).
import asyncio, os, carb, omni.usd, omni.timeline, omni.kit.app

USD = os.environ.get("EXTINGUISHER_STAGE",
                     "/home/user/stretch_main/isaacsim/assets/stretch3_og_hospital_extinguisher.usda")
CONTROL = os.environ.get("EXTINGUISHER_CONTROL", "/tmp/extinguisher_condition")
CLEAN_UI = os.environ.get("EXTINGUISHER_CLEAN_UI", "0") == "1"
EXTINGUISHER = "/World/FireExtinguisher"
EXTINGUISHERS = {"1": EXTINGUISHER, "2": "/World/FireExtinguisher2"}
CAMERA = "/World/stretch3/base_link/follow_camera"
# (translate in base_link, orient (w, x, y, z): level, pitched 21 deg down, focal mm)
FOLLOW_CAMERAS = {"close": ((-1.7, 0.0, 1.45), (0.582745, 0.400510, -0.400510, -0.582745), 14.0)}


def _set_follow_camera(stage, which):
    from pxr import Gf
    if which not in FOLLOW_CAMERAS:
        carb.log_warn(f"[open_extinguisher] EXTINGUISHER_FOLLOW_CAMERA={which!r}: not one of {sorted(FOLLOW_CAMERAS)}")
        return
    (x, y, z), (w, qx, qy, qz), focal = FOLLOW_CAMERAS[which]
    cam = stage.GetPrimAtPath(CAMERA)
    cam.GetAttribute("xformOp:translate").Set(Gf.Vec3d(x, y, z))
    cam.GetAttribute("xformOp:orient").Set(Gf.Quatd(w, Gf.Vec3d(qx, qy, qz)))
    cam.GetAttribute("focalLength").Set(focal)


def _set_condition(stage, condition):
    vset = stage.GetPrimAtPath(EXTINGUISHER).GetVariantSets().GetVariantSet("condition")
    if condition not in vset.GetVariantNames():
        carb.log_warn(f"[open_extinguisher] unknown condition {condition!r}, keeping {vset.GetVariantSelection()!r}")
        return
    if vset.GetVariantSelection() != condition:
        vset.SetVariantSelection(condition)
    carb.log_warn(f"[open_extinguisher] extinguisher condition = {condition}")


def _clean_ui():
    import omni.ui as ui
    for name in ("Stage", "Layer", "Render Settings", "Property", "Content", "Console"):
        ui.Workspace.show_window(name, False)
    # The viewport's FPS / memory / resolution readout.
    carb.settings.get_settings().set("/persistent/app/viewport/displayOptions", 0)
    # Fill the recorded display; --/app/window/width on the command line loses
    # to the size Isaac persisted last session.
    import omni.appwindow
    omni.appwindow.get_default_app_window().resize(1920, 1080)


def _move(stage, line, home):
    """Apply one line of the .move control file: '<n> <x> <y> [<yaw>]' or '<n> home'."""
    parts = line.split()
    if not parts or parts[0] not in EXTINGUISHERS:
        carb.log_warn(f"[open_extinguisher] move: bad line {line!r}")
        return
    prim = stage.GetPrimAtPath(EXTINGUISHERS[parts[0]])
    if not prim:
        carb.log_warn(f"[open_extinguisher] move: no {EXTINGUISHERS[parts[0]]} on this stage")
        return
    from pxr import Gf

    t, r = prim.GetAttribute("xformOp:translate"), prim.GetAttribute("xformOp:rotateXYZ")
    if parts[1:2] == ["home"]:
        pose = home.get(parts[0])
        if pose is None:
            return
        t.Set(pose[0])
        r.Set(pose[1])
    else:
        x, y = float(parts[1]), float(parts[2])
        z = t.Get()[2]
        t.Set(Gf.Vec3d(x, y, z))
        if len(parts) > 3:
            r.Set(Gf.Vec3d(0.0, 0.0, float(parts[3])))
    carb.log_warn(f"[open_extinguisher] moved #{parts[0]} to {t.Get()} yaw {r.Get()[2]:.0f}")


async def _frames(n):
    app = omni.kit.app.get_app()
    for _ in range(n):
        await app.next_update_async()


async def _go():
    ctx = omni.usd.get_context()
    ok, err = await ctx.open_stage_async(USD)
    carb.log_warn(f"[open_extinguisher] open_stage ok={ok} err={err}")
    while True:
        _, files, total = ctx.get_stage_loading_status()
        if total == 0:
            break
        await _frames(1)
    stage = ctx.get_stage()
    condition = os.environ.get("EXTINGUISHER_CONDITION", "").strip()
    if condition:
        _set_condition(stage, condition)
    shown = stage.GetPrimAtPath(EXTINGUISHER).GetVariantSets().GetVariantSet("condition").GetVariantSelection()
    with open(CONTROL, "w") as f:   # start the control file at the condition shown
        f.write(shown + "\n")
    if os.environ.get("EXTINGUISHER_FOLLOW_CAMERA", "").strip():
        _set_follow_camera(stage, os.environ["EXTINGUISHER_FOLLOW_CAMERA"].strip())
    try:
        from omni.kit.viewport.utility import get_active_viewport
        get_active_viewport().camera_path = CAMERA
    except Exception as exc:  # noqa: BLE001 -- the default camera still renders
        carb.log_warn(f"[open_extinguisher] could not bind {CAMERA}: {exc!r}")
    if CLEAN_UI:
        _clean_ui()
    await _frames(60)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    carb.log_warn("[open_extinguisher] PLAYING")
    last_condition, last_token = shown, ""
    home = {}
    for n, path in EXTINGUISHERS.items():
        prim = stage.GetPrimAtPath(path)
        if prim:
            home[n] = (prim.GetAttribute("xformOp:translate").Get(),
                       prim.GetAttribute("xformOp:rotateXYZ").Get())
    move_file = CONTROL + ".move"

    def _stamp():
        try:
            return os.stat(move_file).st_mtime_ns
        except OSError:
            return None

    # A .move file left by an earlier session is not replayed on startup
    # (EXTINGUISHER_APPLY_MOVES=1 does replay it); one written after is applied.
    move_seen = None if os.environ.get("EXTINGUISHER_APPLY_MOVES", "0") == "1" else _stamp()
    while True:
        await asyncio.sleep(0.5)
        stamp = _stamp()
        if stamp is not None and stamp != move_seen:
            with open(move_file) as f:
                for line in f.read().splitlines():
                    if line.strip():
                        _move(ctx.get_stage(), line, home)
            with open(move_file + ".ack", "w") as f:
                f.write(f"{stamp}\n")
            move_seen = stamp
        try:
            with open(CONTROL) as f:
                parts = f.read().split()
        except OSError:
            continue
        if not parts:
            continue
        want, token = parts[0], (parts[1] if len(parts) > 1 else "")
        if token and token != last_token:
            carb.log_warn(f"[open_extinguisher] reset {token}: Stop, {want}, Play")
            timeline.stop()
            await _frames(30)
            _set_condition(ctx.get_stage(), want)
            await _frames(30)
            timeline.play()
            await _frames(10)
            with open(CONTROL + ".ack", "w") as f:
                f.write(token + "\n")
            carb.log_warn("[open_extinguisher] PLAYING")
        elif want != last_condition:
            _set_condition(ctx.get_stage(), want)
        last_condition, last_token = want, token

asyncio.ensure_future(_go())

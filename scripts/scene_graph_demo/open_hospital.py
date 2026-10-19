# Isaac Sim startup script: open the hospital stage and press Play.
#   ./isaac-sim.sh --exec /home/user/stretch_main/scripts/scene_graph_demo/open_hospital.py
import asyncio, carb, omni.usd, omni.timeline, omni.kit.app

USD = "/home/user/stretch_main/isaacsim/assets/stretch3_og_hospital.usda"

async def _go():
    ctx = omni.usd.get_context()
    ok, err = await ctx.open_stage_async(USD)
    carb.log_warn(f"[open_hospital] open_stage ok={ok} err={err}")
    app = omni.kit.app.get_app()
    while True:
        _, files, total = ctx.get_stage_loading_status()
        if total == 0:
            break
        await app.next_update_async()
    for _ in range(60):
        await app.next_update_async()
    omni.timeline.get_timeline_interface().play()
    carb.log_warn("[open_hospital] PLAYING")

asyncio.ensure_future(_go())

# DualVLN remote inference server

This ROS-free image serves the official
`InternRobotics/InternVLA-N1-DualVLN` checkpoint on port `18082`. InternNav
is pinned to `7a5c62400ac45b313d9b709c740b64191556a242`; the checkpoint snapshot is
pinned in the Dockerfile and compose file.

```bash
docker compose -f docker/dualvln/compose.yaml up -d
curl http://localhost:18082/health
DUALVLN_SERVER_URL=http://remote-host:18082 ./run_vln_demo.sh backend:=dualvln
```

The adapter takes the instruction from `/reset`, rejects stale sessions,
clears history (including the `last_s2_idx` scheduler field upstream's
`reset()` leaves behind), consumes paired RGB/depth in metres with the
request calibration, performs the upstream look-down second pass, and returns
either canonical actions or finite robot-relative `[x_m, y_m]` trajectory
points.

When the language head emits neither waypoint coordinates nor an action token,
upstream yields an empty action list. Both agent outputs are cleared at that
point, so the adapter takes one more step — which re-runs System 2 on the
newest history — before failing with HTTP 500, rather than aborting a
300-second benchmark episode on a single unparseable reply.

`PLAN_STEP_GAP` (default `0`) is the upstream System 2 replanning gap; at `0`
System 2 runs on every other step and System 1 fills in between.

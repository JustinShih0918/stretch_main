# Dynamic-scene experimental protocol

This authored relocation test is separate from the fixed-layout ObjectNav
baseline in `METHODOLOGY.md`.

The dynamic-scene test uses a prior map as a controlled input. In a static
layout, a mapping pass writes a schema-v3 snapshot containing object tracks,
presence beliefs, the occupancy grid, the room segmentation and the ground-plane
height it was mapped at.
After the objects are relocated, the search pass loads that snapshot and
must find the target in the changed layout. On reload, presence log-odds are
capped at `reload_max_log_odds = 1.5` so old sightings do not start the new
episode at full confidence.

The occupancy grid travels with the snapshot. `ycb.map_in_occupancy=false`
restores the tracks and beliefs only, and the search pass rebuilds its own
occupancy from scratch.

(Upstream also ran a cross-anchor arm over ASCENT's per-storey obstacle maps,
with a coverage audit of stairs and targets; that arm, the ASCENT
transcription and multi-floor support were removed from this copy.)

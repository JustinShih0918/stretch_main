#!/usr/bin/env python3
"""Cut the recorded takes into one demo video: title card, a card before each take
saying what the scene really holds, the takes, and an end card with every verdict.

    python3 scripts/extinguisher_demo/compile_demo.py videos/extinguisher_demo/take1_*.mp4 \\
        videos/extinguisher_demo/take2_*.mp4 ... -o videos/extinguisher_demo/fire_extinguisher_demo.mp4

A take is the mp4 run_extinguisher_demo.sh wrote and the .log beside it. The mode and
the extinguisher's real condition come from the file name, the verdict from the run's
inspection.json (the log's last `[robot] ... -> outputs/<run>` line names it), so
the cards say what happened, not what was meant to. Each take is trimmed to start
where this run first reaches the HUD (the half-minute of model
loading before it are a still frame, or the previous take's screen). Everything
the robot does before the inspection -- exploring and mapping, or driving to the
extinguisher -- plays at `--fast` speed with a badge saying so; from just before
the VLM is asked, the take plays in real time, except that a long wait for the
VLM's answer is squeezed to a few seconds, its badge giving the real time.
Writes the 1080p video and a 720p copy beside it for sharing.

A take recorded with SCENE_TITLE / SCENE / TRUTHS / TRUTH_TEXT (the recorder saves
them beside it as take*.json) gets its card from them instead. With TRUTHS
("1=bad,2=good", several extinguishers told apart), the real condition is that of
the extinguisher the run actually inspected (inspection.json's `object`), and the
end card says which one each mission asked for and which one it judged.

`--gallery DIR` (inspect_gallery.py's output) adds the damage gallery after the
first take of a damaged extinguisher: one quick card per defect, the image the VLM
was shown beside its answer and the defect actually staged.

System python3 (PIL, OpenCV) and ffmpeg.
"""
import argparse
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
OSG = HERE.parents[1] / "src" / "semantic_nav" / "object_scene_graph"
W, H, FPS = 1920, 1080, 30
CAMERA_PANEL = (1186, 570, 296, 496)       # demo_hud.py CAMERA, below its header
VLM_PANEL = (1498, 570, 404, 496)          # demo_hud.py VLM, below its header

BG = (14, 16, 20)
TEXT = (232, 232, 232)
DIM = (154, 160, 171)
FIRE = (255, 106, 26)
RED = (229, 72, 77)
GREEN = (48, 164, 108)
YELLOW = (245, 197, 66)

TRUTH = {
    "good": "Ground truth: an intact extinguisher, upright in its stand, hose and handle on",
    "bad": "Ground truth: a damaged extinguisher: corroded body and stand, its hose torn off "
           "and lying on the floor, dry-chemical powder spilled beside it",
}
# Takes are told as days, in the order given: a mapping run, then inspection
# missions that reuse its scene graph.
MODE_TITLE = {
    "map": ("Day {day}  ·  Mapping run",
            "The robot explores the hospital, building an object scene graph (room → surface → "
            "object) as it goes, until it finds the fire extinguisher. Then the VLM inspects it."),
    "search": ("Day {day}  ·  Inspection mission",
               "The robot loads the scene graph from the mapping run. It already knows where the "
               "extinguisher is, so it drives straight there, re-sees it, and the VLM inspects it."),
}

# The damage gallery's conditions (fire_extinguisher_inspect.usda), as staged.
GALLERY = {
    "corroded": ("Corroded", "rust over most of the paint, body and stand. Nothing missing"),
    "hose_detached": ("Hose torn off", "the hose and nozzle torn off the valve, lying on the floor beside it"),
    "discharged": ("Discharged", "dry-chemical powder spilled on the floor beside it, as after use"),
    "fallen": ("Out of its stand", "standing on the floor behind its empty stand, tipped against the wall"),
    "blocked": ("Blocked access", "cardboard boxes stacked in front of it"),
}
GALLERY_CARD_S = 4.0
# The fast-forward ends this long before the VLM is asked, so the robot's last
# turn to face the extinguisher and the start of the inspection play in real time.
FAST_LEAD_S = 1.5
FAST_BADGE = {"map": "EXPLORING & MAPPING", "search": "DRIVING TO THE EXTINGUISHER"}
# A wait for the VLM's answer longer than THINK_SQUEEZE_S (the reasoning
# model has taken 52 s) is shown at whatever speed fits it into THINK_SHOW_S,
# with a badge giving the real time; THINK_LEAD_S of it at either end stays
# real time. Shorter waits play as they happened.
THINK_SQUEEZE_S = 20.0
THINK_SHOW_S = 6.0
THINK_LEAD_S = 2.0
BADGE_XY = (32, 162)                       # inside demo_hud.py's VIEWPORT, top-left


def font(px, bold=False, mono=False):
    name = "DejaVuSansMono" if mono else ("DejaVuSans-Bold" if bold else "DejaVuSans")
    return ImageFont.truetype(f"/usr/share/fonts/truetype/dejavu/{name}.ttf", px)


def wrap(draw, text, fnt, width):
    lines, line = [], ""
    for word in text.split():
        trial = f"{line} {word}".strip()
        if draw.textlength(trial, font=fnt) > width and line:
            lines.append(line)
            line = word
        else:
            line = trial
    return lines + ([line] if line else [])


def card_base(kicker):
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    d.ellipse((160, 170, 196, 206), fill=FIRE)
    d.text((216, 166), kicker, font=font(30, bold=True), fill=DIM)
    return img, d


def title_card(path):
    img, d = card_base("STRETCH 3  ·  ISAAC SIM")
    d.text((160, 250), "Fire Extinguisher Inspection", font=font(96, bold=True), fill=TEXT)
    d.text((164, 380), "Find it with an object scene graph. Judge it with a VLM.", font=font(40), fill=DIM)
    steps = [("1", "Explore and map the hospital as an object scene graph", "room → surface → object, from YOLOE detections"),
             ("2", "Find the fire extinguisher in the graph and drive to it", "frontier search, then Nav2"),
             ("3", "Ask a VLM whether the extinguisher is in good or bad condition",
              "NVIDIA Nemotron 3.5 Super VL: corrosion, hose, handle, placement, discharge, access")]
    y = 520
    for n, head, sub in steps:
        d.ellipse((160, y, 220, y + 60), fill=FIRE)
        d.text((190, y + 30), n, font=font(32, bold=True), fill=BG, anchor="mm")
        d.text((250, y - 2), head, font=font(36, bold=True), fill=TEXT)
        d.text((250, y + 44), sub, font=font(26), fill=DIM)
        y += 140
    img.save(path)


def title_card_instances(path):
    """The title card when the takes are about several extinguishers told apart."""
    img, d = card_base("STRETCH 3  ·  ISAAC SIM")
    d.text((160, 250), "Two Fire Extinguishers", font=font(96, bold=True), fill=TEXT)
    d.text((164, 380), "Tell them apart. Follow the one that moves. Inspect the right one.",
           font=font(40), fill=DIM)
    steps = [("1", "Map the hospital: each extinguisher gets an index as it is found",
              "#1 and #2 in the object scene graph, kept in the saved map"),
             ("2", "Ask for one by name, after moving it",
              "#1 not where the map says: look for it elsewhere, never take #2 for it"),
             ("3", "The VLM judges only the extinguisher the robot stopped at",
              "#1 is corroded, #2 is intact: the verdict shows which one was inspected")]
    y = 520
    for n, head, sub in steps:
        d.ellipse((160, y, 220, y + 60), fill=FIRE)
        d.text((190, y + 30), n, font=font(32, bold=True), fill=BG, anchor="mm")
        d.text((250, y - 2), head, font=font(36, bold=True), fill=TEXT)
        d.text((250, y + 44), sub, font=font(26), fill=DIM)
        y += 140
    img.save(path)


def section_card(path, take):
    head, body = MODE_TITLE[take["mode"]]
    head = head.format(day=take["index"])
    meta = take["meta"]
    head, body = meta.get("scene_title") or head, meta.get("scene") or body
    if take.get("fast"):
        body += (f" The robot's {'exploring' if take['mode'] == 'map' else 'driving'} is shown at "
                 f"{take['fast']:g}× speed, the inspection in real time.")
    kicker = f"TAKE {take['index']}" + (f"  ·  MISSION: {take['target'].upper()}" if take["target"] else "")
    img, d = card_base(kicker)
    hf = font(80 if d.textlength(head, font=font(80, bold=True)) <= 1600 else 64, bold=True)
    d.text((160, 260), head, font=hf, fill=TEXT)
    y = 400
    for line in wrap(d, body, font(38), 1600):
        d.text((164, y), line, font=font(38), fill=DIM)
        y += 54
    col = FIRE if meta.get("truth_text") else GREEN if take["truth"] == "good" else RED
    y += 50
    d.rounded_rectangle((160, y, 1760, y + 120), radius=18, outline=col, width=4)
    lines = wrap(d, meta.get("truth_text") or TRUTH[take["truth"]], font(34), 1520)
    for i, line in enumerate(lines[:2]):
        d.text((196, y + 22 + 42 * i if len(lines) > 1 else y + 40), line, font=font(34), fill=TEXT)
    img.save(path)


def fit_paste(img, im, box):
    """Paste `im` scaled to fit inside `box`, centred."""
    x0, y0, x1, y1 = box
    scale = min((x1 - x0) / im.width, (y1 - y0) / im.height)
    im = im.resize((max(1, int(im.width * scale)), max(1, int(im.height * scale))), Image.LANCZOS)
    img.paste(im, (x0 + (x1 - x0 - im.width) // 2, y0 + (y1 - y0 - im.height) // 2))


def gallery_intro(path, n):
    img, d = card_base("DAMAGE GALLERY")
    d.text((160, 260), "More kinds of damage", font=font(80, bold=True), fill=TEXT)
    body = (f"The same inspection on {n} more extinguishers, one defect each. The robot stays parked "
            "where a mission stops while the extinguisher is swapped in Isaac; YOLOE finds it in the robot's "
            "camera, and Nemotron judges those views with the missions' prompt and checklist. The "
            "VLM is not told which defect was staged.")
    y = 400
    for line in wrap(d, body, font(38), 1600):
        d.text((164, y), line, font=font(38), fill=DIM)
        y += 54
    img.save(path)


def gallery_card(path, case, i, n):
    title, truth = GALLERY.get(case["condition"], (case["condition"], ""))
    r = case["report"]
    img = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(img)
    d.ellipse((60, 46, 84, 70), fill=FIRE)
    d.text((100, 42), f"DAMAGE GALLERY  ·  {i} / {n}", font=font(26, bold=True), fill=DIM)
    for k in range(n):                                       # progress dots
        cx = W - 60 - (n - 1 - k) * 34
        d.ellipse((cx - 9, 49, cx + 9, 67), fill=FIRE if k < i else (60, 64, 72))
    d.text((60, 82), title, font=font(60, bold=True), fill=TEXT)
    d.text((62, 160), "Staged:", font=font(28, bold=True), fill=YELLOW)
    d.text((62 + d.textlength("Staged: ", font=font(28, bold=True)), 160), truth,
           font=font(28), fill=TEXT)
    # Left: the close-up the VLM judged.
    d.text((60, 222), "WHAT THE VLM SAW  ·  ROBOT CAMERA", font=font(22, bold=True), fill=DIM)
    fit_paste(img, case["crop"], (60, 258, 760, 1040))
    # Right: Isaac's view for context, then the answer.
    d.text((800, 222), "ISAAC SIM  ·  THIRD-PERSON", font=font(22, bold=True), fill=DIM)
    vp = case["viewport"]
    vp = vp.crop((0, int(vp.height * 0.06), vp.width, int(vp.height * 0.72)))  # no ceiling, no base
    fit_paste(img, vp, (800, 258, 1860, 680))
    cond = str(r.get("condition", "unknown"))
    col = GREEN if cond == "good" else RED if cond == "bad" else YELLOW
    d.text((800, 716), f"VLM VERDICT  ·  {r.get('model', '')}", font=font(22, bold=True), fill=DIM)
    d.text((800, 744), cond.upper(), font=font(84, bold=True), fill=col)
    x = 800 + d.textlength(cond.upper(), font=font(84, bold=True)) + 40
    if cond in ("good", "bad"):
        d.text((x, 760), f"{float(r.get('confidence', 0)):.0%} confidence", font=font(30), fill=TEXT)
    d.text((x, 800), f"answered in {float(r.get('latency_s', 0)):.1f} s", font=font(30), fill=DIM)
    y = 866
    for issue in (r.get("issues") or [r.get("error") or ""])[:3]:
        d.ellipse((802, y + 12, 816, y + 26), fill=col)
        for j, line in enumerate(wrap(d, issue, font(30), 1020)[:2]):
            d.text((832, y), line, font=font(30), fill=TEXT)
            y += 38
        y += 6
    lines = wrap(d, r.get("summary", ""), font(25), 1040)
    room = max(0, min(2, (1040 - y) // 32))
    if len(lines) > room and room:
        lines = lines[:room - 1] + [lines[room - 1].rstrip(",;:") + " …"]
    for line in lines[:room]:
        d.text((800, y + 4), line, font=font(25), fill=DIM)
        y += 32
    img.save(path)


def badge(path, text):
    f = font(30, bold=True)
    probe = ImageDraw.Draw(Image.new("RGBA", (1, 1)))
    w = int(probe.textlength(text, font=f)) + 44
    img = Image.new("RGBA", (w, 52), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rounded_rectangle((0, 0, w - 1, 51), radius=26, fill=FIRE + (235,))
    d.text((22, 26), text, font=f, fill=BG, anchor="lm")
    img.save(path)


def end_card_instances(path, takes):
    """RESULTS for missions that name one of several extinguishers: what was
    asked for, which one was inspected, and what the VLM said about it."""
    img, d = card_base("RESULTS")
    d.text((160, 250), "Which one, and what the VLM said", font=font(72, bold=True), fill=TEXT)
    cols = [(164, "take"), (300, "mission"), (520, "inspected"), (780, "real"), (960, "verdict"),
            (1210, "scenario")]
    y = 390
    for x, head in cols:
        d.text((x, y), head, font=font(26, bold=True), fill=DIM)
    y += 50
    for t in takes:
        r = t["report"] or {}
        cond = str(r.get("condition", "none"))
        col = GREEN if cond == "good" else RED if cond == "bad" else YELLOW
        asked = f"#{t['want']}" if t["want"] else "any"
        got = f"#{t['got']}" if t["got"] else "—"
        right = t["got"] is not None and (not t["want"] or t["got"] == t["want"])
        d.text((164, y), str(t["index"]), font=font(34), fill=TEXT)
        d.text((300, y), asked, font=font(34, bold=True), fill=TEXT)
        d.text((520, y), got + ("  ✓" if right else "  ✗"), font=font(34, bold=True),
               fill=GREEN if right else RED)
        d.text((780, y), t["truth"].upper(), font=font(34, bold=True),
               fill=GREEN if t["truth"] == "good" else RED)
        verdict = cond.upper() + (f"  {float(r.get('confidence', 0)):.0%}" if cond in ("good", "bad") else "")
        d.text((960, y), verdict, font=font(34, bold=True), fill=col)
        lines = wrap(d, t["meta"].get("scene_title", ""), font(26), 560)
        for i, line in enumerate(lines[:2]):
            d.text((1210, y + 4 + 32 * i), line, font=font(26), fill=DIM)
        y += max(76, 32 * min(2, len(lines)) + 30)
    model = next((t["report"].get("model") for t in takes if t["report"]), "")
    d.text((164, H - 120), f"VLM: {model}  ·  scene graph + search: Object_scene_graph (OSG)",
           font=font(24, mono=True), fill=DIM)
    img.save(path)


def end_card(path, takes, gallery=None):
    img, d = card_base("RESULTS")
    d.text((160, 250), "What the VLM said", font=font(80, bold=True), fill=TEXT)
    y = 400
    d.text((164, y), "take", font=font(26, bold=True), fill=DIM)
    d.text((420, y), "real condition", font=font(26, bold=True), fill=DIM)
    d.text((720, y), "verdict", font=font(26, bold=True), fill=DIM)
    d.text((1000, y), "what it saw", font=font(26, bold=True), fill=DIM)
    y += 50
    for t in takes:
        r = t["report"] or {}
        cond = str(r.get("condition", "none"))
        col = GREEN if cond == "good" else RED if cond == "bad" else YELLOW
        d.text((164, y), f"{t['index']}  {t['mode']}", font=font(34), fill=TEXT)
        d.text((420, y), t["truth"].upper(), font=font(34, bold=True),
               fill=GREEN if t["truth"] == "good" else RED)
        verdict = cond.upper() + (f"  {float(r.get('confidence', 0)):.0%}" if cond in ("good", "bad") else "")
        d.text((720, y), verdict, font=font(34, bold=True), fill=col)
        issues = "; ".join(r.get("issues") or []) or (r.get("summary") or r.get("error") or "")
        lines = wrap(d, issues, font(26), 760)
        for i, line in enumerate(lines[:3]):
            d.text((1000, y + 4 + 32 * i), line, font=font(26), fill=DIM)
        y += max(80, 32 * min(3, len(lines)) + 30)
    if gallery:
        verdicts = [str(c["report"].get("condition")) for c in gallery]
        d.line((164, y + 6, 1760, y + 6), fill=(48, 52, 60), width=2)
        d.text((164, y + 30), f"Damage gallery: {len(gallery)} single-defect extinguishers, "
               f"VLM said BAD on {verdicts.count('bad')} / {len(gallery)}",
               font=font(34, bold=True), fill=TEXT)
        names = ", ".join(GALLERY.get(c["condition"], (c["condition"],))[0].lower() for c in gallery)
        d.text((164, y + 80), names, font=font(26), fill=DIM)
    model = next((t["report"].get("model") for t in takes if t["report"]), "")
    d.text((164, H - 120), f"VLM: {model}  ·  scene graph + search: Object_scene_graph (OSG)",
           font=font(24, mono=True), fill=DIM)
    img.save(path)


def probe_duration(path):
    out = subprocess.run(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of",
                          "csv=p=0", str(path)], capture_output=True, text=True, check=True)
    return float(out.stdout.strip())


def mission_start(path):
    """Seconds into the take when this run reaches the HUD.

    Until then the HUD shows whatever it showed last -- an empty panel after a
    fresh start, or the previous take's final screen, verdict and all -- so the
    take starts at the first frame whose robot-camera panel differs from the
    opening one: the run's first image, or the HUD clearing for the new mission.
    """
    cap = cv2.VideoCapture(str(path))
    fps = cap.get(cv2.CAP_PROP_FPS) or FPS
    x, y, w, h = CAMERA_PANEL
    first = None
    i = 0
    while True:
        ok = cap.grab()
        if not ok:
            return 0.0
        if i % int(fps // 4) == 0:
            _, frame = cap.retrieve()
            panel = frame[y:y + h, x:x + w].astype(np.int16)
            if first is None:
                first = panel
            elif np.abs(panel - first).mean() > 8:
                return i / fps
        i += 1


def load_take(index, path):
    m = re.match(r"take\d+_(map|search)_(.+)_\d{8}_\d{6}\.mp4$", path.name)
    if not m:
        sys.exit(f"{path.name}: not a run_extinguisher_demo.sh take (take<N>_<mode>_<condition>_<stamp>.mp4)")
    report = None
    log = path.with_suffix(".log")
    runs = re.findall(r"^\[robot\] \d+ steps, .* -> (outputs/\S+)$", log.read_text(), re.M) if log.exists() else []
    if runs and (OSG / runs[-1] / "inspection.json").exists():
        report = json.loads((OSG / runs[-1] / "inspection.json").read_text())
    meta = json.loads(path.with_suffix(".json").read_text()) if path.with_suffix(".json").exists() else {}
    index_of = lambda text: int(k.group(1)) if (k := re.search(r"#(\d+)", str(text or ""))) else None  # noqa: E731
    want, got = index_of(meta.get("target")), index_of((report or {}).get("object"))
    truths = dict(kv.split("=", 1) for kv in meta.get("truths", "").split(",") if "=" in kv)
    # The real condition of the extinguisher judged (or, with no verdict, asked for).
    truth = truths.get(str(got or want)) or ("good" if m.group(2) == "good" else "bad")
    return {"index": index, "path": path, "mode": m.group(1), "truth": truth, "report": report,
            "meta": meta, "target": meta.get("target", ""), "want": want, "got": got,
            "instances": bool(truths)}


def inspection_start(path, start):
    """Seconds into the take when the VLM is asked, or None if it never is.

    demo_hud.py's VLM panel turns orange (FIRE) when the mission reaches
    "inspecting" -- the "Analysing..." line and the panel's accent -- and has no
    orange before that, so this is the first frame from `start` on with orange
    in that panel.
    """
    cap = cv2.VideoCapture(str(path))
    fps = cap.get(cv2.CAP_PROP_FPS) or FPS
    x, y, w, h = VLM_PANEL
    every = int(fps // 4)
    i = 0
    while cap.grab():
        t = i / fps
        if t >= start and i % every == 0:
            _, frame = cap.retrieve()
            p = frame[y:y + h, x:x + w].astype(np.int16)
            fire = (p[..., 2] > 220) & (np.abs(p[..., 1] - 106) < 40) & (p[..., 0] < 80)
            if int(fire.sum()) >= 200:
                return t
        i += 1
    return None


def verdict_start(path, asked):
    """Seconds into the take when the VLM's verdict appears, or None.

    The VLM panel's headline, right of the thumbnail, says "Analysing..." in
    orange while the model thinks and the verdict in green (GOOD), red (BAD) or
    yellow (none) once it answers. The thumbnail beside it shows the red
    extinguisher, so only the headline is looked at.
    """
    cap = cv2.VideoCapture(str(path))
    fps = cap.get(cv2.CAP_PROP_FPS) or FPS
    x, y, _, _ = VLM_PANEL
    colours = np.array([GREEN, RED, YELLOW])[:, ::-1].astype(np.int16)      # BGR
    every = int(fps // 4)
    i = 0
    while cap.grab():
        t = i / fps
        if t > asked and i % every == 0:
            _, frame = cap.retrieve()
            p = frame[y + 8:y + 90, x + 170:x + 400].astype(np.int16)
            hit = (np.abs(p[:, :, None, :] - colours[None, None]).max(-1) < 40).any(-1)
            if int(hit.sum()) >= 300:
                return t
        i += 1
    return None


def load_gallery(root):
    order = [c["condition"] for c in json.loads((root / "summary.json").read_text())]
    cases = []
    for cond in order:
        d = root / cond
        if not (d / "inspection.json").exists() or not (d / "inspection_view0.jpg").exists():
            print(f"gallery: {cond} has no inspection, skipped")
            continue
        cases.append({"condition": cond, "report": json.loads((d / "inspection.json").read_text()),
                      "crop": Image.open(d / "inspection_view0.jpg").convert("RGB"),
                      "viewport": Image.open(d / "viewport.jpg").convert("RGB")})
    return cases


def segment(src, out, start=0.0, end=None, duration=None, fade_in=0.6, fade_out=0.6,
            still=False, speed=1.0, overlay=None):
    """One piece of the cut, encoded alike so the pieces concatenate by copy.
    A clip runs from `start` to `end` (default: its end) at `speed`;
    `overlay` is a PNG laid over it at BADGE_XY."""
    args = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y"]
    if still:
        args += ["-loop", "1", "-framerate", str(FPS), "-t", f"{duration:.2f}", "-i", str(src)]
    else:
        end = probe_duration(src) if end is None else end
        args += ["-ss", f"{start:.2f}", "-t", f"{end - start:.2f}", "-i", str(src)]
        duration = (end - start) / speed
    chain = (f"setpts=(PTS-STARTPTS)/{speed:g}," if speed != 1 else "") + f"scale={W}:{H},fps={FPS}"
    fades = ([f"fade=t=in:st=0:d={fade_in}"] if fade_in else []) + \
            ([f"fade=t=out:st={duration - fade_out:.2f}:d={fade_out}"] if fade_out else [])
    if overlay:
        args += ["-loop", "1", "-i", str(overlay), "-filter_complex",
                 f"[0:v]{chain}[v];[v][1:v]overlay={BADGE_XY[0]}:{BADGE_XY[1]}:shortest=1,"
                 + ",".join(fades + ["format=yuv420p"]) + "[o]", "-map", "[o]"]
    else:
        args += ["-vf", ",".join([chain] + fades + ["format=yuv420p"])]
    args += ["-an", "-c:v", "libx264", "-preset", "medium", "-crf", "18", "-r", str(FPS), str(out)]
    subprocess.run(args, check=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("takes", nargs="+", type=Path)
    ap.add_argument("-o", "--out", type=Path, required=True)
    ap.add_argument("--fast", type=float, default=4.0,
                    help="speed of everything before the inspection: exploring, driving (1 = off)")
    ap.add_argument("--gallery", type=Path, help="inspect_gallery.py's output directory")
    args = ap.parse_args()
    takes = [load_take(i + 1, p) for i, p in enumerate(args.takes)]
    gallery = load_gallery(args.gallery) if args.gallery else []
    gallery_after = next((t["index"] for t in takes if t["truth"] == "bad"), takes[-1]["index"])
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        parts = []
        several = any(t["instances"] for t in takes)
        (title_card_instances if several else title_card)(tmp / "title.png")
        segment(tmp / "title.png", tmp / "00_title.mp4", duration=7, still=True)
        parts.append(tmp / "00_title.mp4")
        for mode, what in FAST_BADGE.items():
            badge(tmp / f"badge_{mode}.png", f"▶▶  {args.fast:g}× SPEED  ·  {what}")
        for t in takes:
            start = mission_start(t["path"])
            # The take as pieces (from, to, speed, badge): the driving fast,
            # the moment the VLM is asked in real time, a long wait for its
            # answer squeezed to THINK_SHOW_S, the verdict in real time.
            end = probe_duration(t["path"])
            pieces = [(start, end, 1.0, None)]
            notes = []
            asked = inspection_start(t["path"], start) if args.fast > 1 else None
            if asked is not None and asked - FAST_LEAD_S - start > 5:
                t["fast"] = args.fast
                fast_badge = tmp / f"badge_{t['mode']}.png"
                if t["want"]:
                    fast_badge = tmp / f"badge_want{t['index']}.png"
                    badge(fast_badge, f"▶▶  {args.fast:g}× SPEED  ·  "
                          f"{'MAPPING' if t['mode'] == 'map' else 'SEARCHING FOR'} #{t['want']}")
                pieces = [(start, asked - FAST_LEAD_S, args.fast, fast_badge),
                          (asked - FAST_LEAD_S, end, 1.0, None)]
                notes.append(f"{asked - FAST_LEAD_S - start:.0f}s before the inspection at {args.fast:g}x")
            answered = verdict_start(t["path"], asked) if asked is not None else None
            if answered is not None:
                wait_from, wait_to = asked + THINK_LEAD_S, answered - THINK_LEAD_S
                if answered - asked > THINK_SQUEEZE_S:
                    speed = round((wait_to - wait_from) / THINK_SHOW_S, 1)
                    badge(tmp / f"badge_think{t['index']}.png",
                          f"▶▶  {speed:g}× SPEED  ·  VLM THINKING, {answered - asked:.0f} s IN ALL")
                    head, tail = pieces[:-1], pieces[-1]
                    pieces = head + [(tail[0], wait_from, 1.0, None),
                                     (wait_from, wait_to, speed, tmp / f"badge_think{t['index']}.png"),
                                     (wait_to, tail[1], 1.0, None)]
                    notes.append(f"the VLM's {answered - asked:.0f}s answer at {speed:g}x")
            section_card(tmp / f"card{t['index']}.png", t)
            segment(tmp / f"card{t['index']}.png", tmp / f"{t['index']:02d}a.mp4", duration=6, still=True)
            parts.append(tmp / f"{t['index']:02d}a.mp4")
            for k, (a, b, speed, overlay) in enumerate(pieces):
                out = tmp / f"{t['index']:02d}b{k}.mp4"
                segment(t["path"], out, start=a, end=b, speed=speed, overlay=overlay,
                        fade_in=0.6 if k == 0 else 0, fade_out=0.6 if k == len(pieces) - 1 else 0)
                parts.append(out)
            r = t["report"] or {}
            print(f"take {t['index']}: {t['mode']}, {t['target'] or 'any'}, inspected {r.get('object')}, "
                  f"real {t['truth']}, VLM {r.get('condition')} "
                  f"{r.get('confidence')}, trimmed {start:.1f}s of loading"
                  + "".join(f", {n}" for n in notes))
            if gallery and t["index"] == gallery_after:
                gallery_intro(tmp / "g00.png", len(gallery))
                segment(tmp / "g00.png", tmp / f"{t['index']:02d}g00.mp4", duration=5, still=True)
                parts.append(tmp / f"{t['index']:02d}g00.mp4")
                for k, case in enumerate(gallery, 1):
                    gallery_card(tmp / f"g{k:02d}.png", case, k, len(gallery))
                    segment(tmp / f"g{k:02d}.png", tmp / f"{t['index']:02d}g{k:02d}.mp4",
                            duration=GALLERY_CARD_S, still=True, fade_in=0.25, fade_out=0.25)
                    parts.append(tmp / f"{t['index']:02d}g{k:02d}.mp4")
                print(f"gallery after take {t['index']}: "
                      + ", ".join(f"{c['condition']} {c['report'].get('condition')}" for c in gallery))
        if several:
            end_card_instances(tmp / "end.png", takes)
        else:
            end_card(tmp / "end.png", takes, gallery)
        segment(tmp / "end.png", tmp / "99_end.mp4", duration=10, still=True)
        parts.append(tmp / "99_end.mp4")
        (tmp / "list.txt").write_text("".join(f"file '{p}'\n" for p in parts))
        args.out.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0",
                        "-i", str(tmp / "list.txt"), "-c", "copy", "-movflags", "+faststart", str(args.out)],
                       check=True)
    small = args.out.with_name(args.out.stem + "_720p.mp4")
    subprocess.run(["ffmpeg", "-hide_banner", "-loglevel", "error", "-y", "-i", str(args.out),
                    "-vf", "scale=1280:-2", "-c:v", "libx264", "-preset", "slow", "-crf", "26",
                    "-movflags", "+faststart", str(small)], check=True)
    print(f"wrote {args.out} ({probe_duration(args.out):.0f} s) and {small}")


if __name__ == "__main__":
    main()

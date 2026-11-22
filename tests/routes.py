"""Run the production detector against the original GTA Z-up LAe2 collision OBJ."""
import subprocess
import sys
from pathlib import Path

runner, scene = sys.argv[1:3]
road_a, road_b = (2519, -1815, 13.2), (2519, -1788, 13.2)
open_end, inside = (2790, -1423, 16.1), (2712, -1423, 16.1)
routes = {}


def route(label, a, b, speed, fps=60):
    routes[label] = (a, b, speed, fps)


for fps in (25, 60, 144):
    route(f"bridge-{fps}", (2870,-1615,12), (2870,-1565,12), 120, fps)
    route(f"lamp-{fps}", road_a, road_b, 110, fps)
    route(f"return-{fps}", road_b, road_a, 110, fps)
    route(f"enter-{fps}", open_end, inside, 110, fps)
    route(f"exit-{fps}", inside, open_end, 110, fps)
for kmh in (45, 80, 162, 190):
    route(f"speed-{kmh}", road_a, road_b, kmh)
route("far", (2524.5, -1815, 13.2), (2524.5, -1788, 13.2), 110)
route("covered-parking", (2330, -1272, 22.4), (2330, -1220, 22.4), 110)
commands = ["route " + " ".join(map(str, (name, *a, *b, speed, fps)))
            for name, (a, b, speed, fps) in routes.items()]
rows = subprocess.run([runner, scene], input="\n".join(commands) + "\n",
                      text=True, capture_output=True, check=True).stdout.splitlines()
hits = {name: [] for name in routes}
totals = {}
for row in rows:
    fields = row.split()
    if fields[0] == "total":
        totals[fields[1]] = list(map(int, fields[2:]))
    else:
        _, name, kind, metres, gain, pan, model, *motion = fields
        hits[name].append((kind, float(metres), float(gain), float(pan), model, *map(float, motion)))


def lamp(name):
    found = [h for h in hits[name] if h[4] == "lamppost3_405"]
    assert len(found) == 1, (name, found)
    return found[0]


for fps in (25, 60, 144):
    assert [h[0] for h in hits[f"bridge-{fps}"] if h[0] in ("bridge","tunnel","exit")] == ["bridge","exit"]
    assert lamp(f"lamp-{fps}")[3] < -.9 and lamp(f"return-{fps}")[3] > .9
    assert any(h[0] == "large" for h in hits[f"lamp-{fps}"])
    assert [h[0] for h in hits[f"enter-{fps}"] if h[0] in ("bridge","tunnel","exit")] == ["tunnel"]
    assert [h[0] for h in hits[f"exit-{fps}"] if h[0] in ("bridge","tunnel","exit")] == ["exit"]
assert totals["speed-45"] == [0, 0, 0, 0, 0]
assert lamp("speed-80")[2] < lamp("lamp-60")[2] < lamp("speed-162")[2]
assert lamp("speed-162")[2] == lamp("speed-190")[2]
assert not any(h[4]=="lamppost3_405" and h[2]>=lamp("lamp-60")[2] for h in hits["far"])
assert not hits["covered-parking"]
print(f"{len(routes)} LAe2 passes checked at 25, 60 and 144 FPS.")
if len(sys.argv) == 4:
    sections = [("lamp-60", .55), ("return-60", 2.55), ("enter-60", 4.65)]
    cues = []
    for name, start in sections:
        for kind, metres, gain, pan, _, *motion in hits[name]:
            time = start + metres / (routes[name][2] / 3.6)
            cues.append((time, ["small", "large", "tunnel"].index(kind), gain, pan, *motion))
    Path(sys.argv[3]).write_text("".join("%.5f %d %.5f %.5f %.4f %.4f %.4f %.4f %d\n" % cue for cue in sorted(cues)))

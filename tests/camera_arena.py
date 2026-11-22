"""Sweep GTA's chase camera around four real LAe2 locations using scene_runner."""
import math
import subprocess
import sys

locations = {
    "bridge": (2870, -1588, 12),
    "street": (2519, -1803, 13.2),
    "tunnel": (2730, -1423, 16.1),
    "parking": (2330, -1246, 22.4),
}
commands, cases = [], []
for label, (x, y, z) in locations.items():
    for radius in (4.7, 5.068, 5.436, 5.9):
        for pitch in (-25, 0, 25, 55):
            for yaw in range(0, 360, 15):
                a, b = math.radians(yaw), math.radians(pitch)
                height = z + 1.66
                end = (x + radius*math.cos(a)*math.cos(b), y + radius*math.sin(a)*math.cos(b),
                       height + radius*math.sin(b))
                commands.append("camera " + " ".join(f"{v:.7f}" for v in (x, y, height, *end)))
                cases.append((label, radius))
run = subprocess.run(sys.argv[1:3], input="\n".join(commands)+"\n", text=True,
                     capture_output=True, check=True)
lines = run.stdout.splitlines()
assert len(lines) == len(cases)
clipped = dict.fromkeys(locations, 0)
for (label, radius), line in zip(cases, lines):
    name, stop, contact = line.split()
    stop, contact = float(stop), float(contact)
    expected = radius if name == "open" else max(0, min(radius, contact-.24))
    assert abs(expected-stop) < .0006, (line, expected)
    clipped[label] += name != "open" and stop < radius-.01
assert sum(clipped.values()) > 0 and sum(clipped.values()) < len(lines)
print(f"{len(lines)} camera rays across LAe2. Clipped by geometry: {clipped}.")

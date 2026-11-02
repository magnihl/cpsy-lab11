#!/usr/bin/env python3
"""Run convoy_check's simulation many ways and draw it as one HTML page.

  python3 tools/convoy_view.py ~/Projects/raunkerfi/linefollower/convoy/master-*.jsonl

For every recorded master lap and every packet gap pattern (the real track
runs in logs/, and the same gaps moved along the track) it drives the slave
once per fix in METHODS: the code as it is now (straight chord), an arc, and
three changes to what the master sends. The page shows the master's true
path, the lost packets, where the slave drove with each fix, and how far off
the path it was over time. Writes logs/convoy_view.html.
"""

import contextlib
import glob
import io
import json
import os
import statistics
import sys

# paths from the command line, resolved before moving into the repo folder
LAPS = [os.path.abspath(p) for p in sys.argv[1:]]
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
os.chdir(ROOT)

import convoy_check as cc  # noqa: E402

OUT = "logs/convoy_view.html"
SHIFTS = (0, 20, 40, 60, 80)  # move the real gaps this far along the track, cm
SEEDS = (1, 2, 3, 4, 5)       # random 20 percent loss, the old check, as a control
GAP = 60
SIZE = 360                    # plot width and height in px
BAD_CM = 5.0                  # off the path by more than this counts as a bad run

# name, what changes, simulate() options, colour, drawn in the top view
METHODS = [
    ("now", "the slave as it is: straight chord across a gap", {}, "--now", True),
    ("arc", "slave bridges a gap with an arc", {"arc": True}, "--arc", False),
    ("x, y", "master also sends its own x, y", {"master": "xy"}, "--xy", True),
    ("history", "master also resends its last 2 s of (s, heading)",
     {"master": "history"}, "--hist", True),
    ("both", "master sends x, y and its last 2 s", {"master": "both"}, "--both", False),
]


def gap_patterns():
    """Name and lost stretches for every real slave log, moved along the track."""
    out = []
    for log in sorted(glob.glob("logs/convoy_slave_*_run*.csv")):
        cut = cc.real_gaps(log)
        if not cut:
            continue
        run = os.path.basename(log).split("_")[2]
        for shift in SHIFTS:
            name = f"run {run}" + (f", moved {shift} cm" if shift else "")
            out.append((name, [(a + shift, b + shift) for a, b in cut]))
    return out


def quiet(fn, *args, **kw):
    """The slave prints when it hears the master again; keep the page run quiet."""
    with contextlib.redirect_stdout(io.StringIO()):
        return fn(*args, **kw)


def worst(r):
    return max(r["off"]) if r["off"] else 0.0


def frame(results):
    """Scale and offset so the track fits the square plot."""
    pts = results[0]["truth"] + [p for r in results for p in r["drove"]]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    # a slave that drove away would squeeze the track to a dot: clip the view
    # to the track plus a margin, the runaway leaves the picture
    tx, ty = [p[0] for p in results[0]["truth"]], [p[1] for p in results[0]["truth"]]
    margin = 0.25 * max(max(tx) - min(tx), max(ty) - min(ty))
    x0, x1 = max(min(xs), min(tx) - margin), min(max(xs), max(tx) + margin)
    y0, y1 = max(min(ys), min(ty) - margin), min(max(ys), max(ty) + margin)
    span = max(x1 - x0, y1 - y0, 1.0)
    scale = (SIZE - 20) / span
    ox = 10 - x0 * scale + (SIZE - 20 - (x1 - x0) * scale) / 2
    oy = 10 + y1 * scale + (SIZE - 20 - (y1 - y0) * scale) / 2
    return scale, ox, oy


def polyline(points, scale, ox, oy, colour, width):
    step = max(1, len(points) // 600)
    d = " ".join(f"{ox + x * scale:.1f},{oy - y * scale:.1f}" for x, y in points[::step])
    return (f'<polyline points="{d}" fill="none" stroke="var({colour})" '
            f'stroke-width="{width}" stroke-linejoin="round"/>')


def plot(results, truth_s):
    """Top view: true path, lost packets, where each drawn fix drove."""
    scale, ox, oy = frame(results)
    first = results[0]
    lost = []
    for s in first["dropped"][::2]:
        k = cc.bisect_left(truth_s, s)
        if k < len(first["truth"]):
            x, y = first["truth"][k]
            lost.append(f'<circle cx="{ox + x * scale:.1f}" cy="{oy - y * scale:.1f}" '
                        f'r="2.2" fill="var(--lost)"/>')
    lines = [polyline(r["drove"], scale, ox, oy, m[3], 1.6)
             for r, m in zip(results, METHODS) if m[4]]
    return (f'<svg viewBox="0 0 {SIZE} {SIZE}" class="top" style="overflow:hidden">'
            + polyline(first["truth"], scale, ox, oy, "--truth", 6)
            + "".join(lost) + "".join(lines) + "</svg>")


def chart(results):
    """Off the path over time, one line per drawn fix."""
    w, h = SIZE, 120
    drawn = [(r, m) for r, m in zip(results, METHODS) if m[4] and r["times"]]
    t_end = max(r["times"][-1] for r, _ in drawn)
    top = min(max(20.0, max(worst(r) for r, _ in drawn)), 60.0)

    def line(r, colour):
        step = max(1, len(r["off"]) // 500)
        d = " ".join(f"{30 + t / t_end * (w - 40):.1f},{h - 15 - min(o, top) / top * (h - 25):.1f}"
                     for t, o in list(zip(r["times"], r["off"]))[::step])
        return f'<polyline points="{d}" fill="none" stroke="var({colour})" stroke-width="1.4"/>'

    ticks = "".join(
        f'<line x1="30" x2="{w - 10}" y1="{h - 15 - v / top * (h - 25):.1f}" '
        f'y2="{h - 15 - v / top * (h - 25):.1f}" class="grid"/>'
        f'<text x="26" y="{h - 12 - v / top * (h - 25):.1f}" class="tick">{v:.0f}</text>'
        for v in range(0, int(top) + 1, 10 if top > 25 else 5))
    return (f'<svg viewBox="0 0 {w} {h}" class="chart">{ticks}'
            + "".join(line(r, m[3]) for r, m in drawn)
            + f'<text x="{w - 10}" y="{h - 2}" class="tick">{t_end:.0f} s</text>'
            + '<text x="32" y="10" class="label">cm off the path</text></svg>')


def card(lap, name, results, truth_s):
    numbers = ", ".join(f"<span style='color:var({m[3]})'>{m[0]} {worst(r):.1f}</span>"
                        for r, m in zip(results, METHODS))
    return (f"<figure><figcaption><b>lap {lap}</b>, packets lost like {name}<br>"
            f"worst cm off the path: {numbers}</figcaption>"
            f"{plot(results, truth_s)}{chart(results)}</figure>")


def main(laps):
    patterns = gap_patterns()
    rows, cards, worst_runs = [], [], []
    for lap in laps:
        packets = [json.loads(line) for line in open(lap)]
        lap_name = os.path.basename(lap).replace("master-", "").replace(".jsonl", "")
        path = cc.cs.Path()
        for p in packets:
            path.add(p["s_cm"], p["heading"])
        for name, cut in patterns:
            results = [quiet(cc.simulate, packets, GAP, cut=cut, **m[2]) for m in METHODS]
            rows.append((lap_name, name, results))
            if "moved" not in name:
                cards.append(card(lap_name, name, results, path.s))
            else:
                worst_runs.append((worst(results[0]), card(lap_name, name, results, path.s)))
        for seed in SEEDS:
            results = [quiet(cc.simulate, packets, GAP, loss=0.2, seed=seed, **m[2])
                       for m in METHODS]
            rows.append((lap_name, f"random 20% lost, seed {seed}", results))
        print(f"{lap_name}: done")

    real = [r for r in rows if r[1].startswith("run")]
    stats = []
    for i, m in enumerate(METHODS):
        w = [worst(r[2][i]) for r in real]
        stats.append((m, max(w), statistics.median(w), sum(1 for x in w if x > BAD_CM)))
    print(f"\n{len(real)} runs with real gap patterns, {len(rows) - len(real)} with random loss")
    print(f"{'fix':10} {'worst':>7} {'median':>7} {'runs over 5 cm':>15}")
    for m, mx, med, bad in stats:
        print(f"{m[0]:10} {mx:7.1f} {med:7.1f} {bad:15d}")

    head = "".join(f"<th class='num'>{m[0]}</th>" for m in METHODS)
    table = "".join(
        f"<tr class='{'ctrl' if r[1].startswith('random') else ''}'><td>{r[0]}</td><td>{r[1]}</td>"
        + "".join(f"<td class='num {'bad' if worst(x) > BAD_CM else ''}'>{worst(x):.1f}</td>"
                  for x in r[2]) + "</tr>" for r in rows)
    stat_rows = "".join(
        f"<tr><td><i class='sw' style='background:var({m[3]})'></i><b>{m[0]}</b></td><td>{m[1]}</td>"
        f"<td class='num'>{mx:.1f}</td><td class='num'>{med:.1f}</td>"
        f"<td class='num'>{bad} of {len(real)}</td></tr>" for m, mx, med, bad in stats)
    worst_runs.sort(key=lambda w: -w[0])
    best = min(stats, key=lambda s: (s[3], s[1]))
    drawn_key = "".join(f'<span><i class="sw" style="background:var({m[3]})"></i>{m[0]}</span>'
                        for m in METHODS if m[4])

    html = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Convoy slave sim</title>
<style>
:root {{ --bg:#fbfaf7; --fg:#1d1d1b; --muted:#6b6b66; --line:#e3e0d8;
  --truth:#d9d4c7; --lost:#e8a33d; --now:#c8423b; --arc:#2f6db5; --xy:#8a4fbf;
  --hist:#2e8b57; --both:#0f766e; }}
@media (prefers-color-scheme: dark) {{ :root {{ --bg:#17181a; --fg:#e8e6e1;
  --muted:#9a988f; --line:#2c2e31; --truth:#3a3c40; --lost:#e0a050; --now:#ef6a60;
  --arc:#6aa3e8; --xy:#b48ae0; --hist:#5cc48a; --both:#3fb8ad; }} }}
body {{ background:var(--bg); color:var(--fg); margin:0; font:15px/1.5 system-ui, sans-serif; }}
main {{ max-width:1180px; margin:0 auto; padding:24px 16px 60px; }}
h1 {{ font-size:22px; margin:0 0 4px; }} h2 {{ font-size:17px; margin:36px 0 8px; }}
p {{ color:var(--muted); max-width:780px; margin:4px 0; }}
.sw {{ display:inline-block; width:18px; height:4px; vertical-align:middle;
  margin-right:8px; border-radius:2px; }}
.key span {{ display:inline-block; margin-right:16px; font-size:14px; }}
.grid-cards {{ display:grid; grid-template-columns:repeat(auto-fill,minmax(340px,1fr)); gap:18px; }}
figure {{ margin:0; border:1px solid var(--line); border-radius:8px; padding:12px; }}
figcaption {{ font-size:13px; color:var(--muted); margin-bottom:6px; }}
svg {{ width:100%; height:auto; display:block; }}
.chart {{ margin-top:6px; }}
.grid {{ stroke:var(--line); stroke-width:1; }}
.tick, .label {{ fill:var(--muted); font-size:10px; }} .tick {{ text-anchor:end; }}
.summary {{ font-size:16px; color:var(--fg); margin:14px 0; padding:10px 14px;
  border-left:4px solid var(--hist); background:color-mix(in srgb,var(--hist) 9%,transparent); }}
.tablewrap {{ overflow-x:auto; }}
table {{ border-collapse:collapse; font-size:13px; width:100%; }}
th, td {{ text-align:left; padding:5px 10px; border-bottom:1px solid var(--line); white-space:nowrap; }}
.num {{ text-align:right; font-variant-numeric:tabular-nums; }}
.bad {{ color:var(--now); font-weight:600; }}
tr.ctrl td {{ color:var(--muted); }}
</style></head><body><main>
<h1>Convoy slave against Mani's recorded laps</h1>
<p>The slave follows {GAP} cm behind Mani's recorded laps. Packets are lost the way the
real track runs lost them (logs/convoy_slave_*: the same stretches of track, also moved
along it), and with 20 percent random loss as a control. Every run is done once per fix.</p>
<div class="summary">Best: <b>{best[0][0]}</b> ({best[0][1]}): worst {best[1]:.1f} cm off the
path, {best[3]} of {len(real)} runs over {BAD_CM:.0f} cm. The code now: worst
{stats[0][1]:.1f} cm, {stats[0][3]} runs over {BAD_CM:.0f} cm.</div>
<h2>Each fix over {len(real)} runs with real gap patterns</h2>
<div class="tablewrap"><table><tr><th>fix</th><th>what changes</th><th class="num">worst, cm</th>
<th class="num">median, cm</th><th class="num">runs over {BAD_CM:.0f} cm</th></tr>{stat_rows}</table></div>
<p class="key" style="margin-top:14px"><span><i class="sw" style="background:var(--truth);height:8px"></i>master's true path</span>
<span><i class="sw" style="background:var(--lost)"></i>lost packets</span>
{drawn_key} (arc and both are in the numbers, not drawn)</p>
<h2>Top view per lap, real gap patterns</h2>
<div class="grid-cards">{"".join(cards)}</div>
<h2>The 6 worst runs of the code now, gaps moved along the track</h2>
<div class="grid-cards">{"".join(c for _, c in worst_runs[:6])}</div>
<h2>Every run, worst cm off the path</h2>
<div class="tablewrap"><table><tr><th>lap</th><th>packets lost</th>{head}</tr>{table}</table></div>
</main></body></html>
"""
    with open(OUT, "w") as f:
        f.write(html)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    if not LAPS:
        sys.exit(__doc__)
    main(LAPS)

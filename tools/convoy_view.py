#!/usr/bin/env python3
"""Run convoy_check's simulation many ways and draw it as one HTML page.

  python3 tools/convoy_view.py ~/Projects/raunkerfi/linefollower/convoy/master-*.jsonl

For every recorded master lap and every packet gap pattern (the real track
runs in logs/, and the same gaps moved along the track) it drives the slave
twice: bridging lost stretches with a straight chord (as now) and with an
arc. The page shows the master's true path, the slave's map of it, where the
slave drove, the lost packets, and how far off the path it was over time.
Writes logs/convoy_view.html.
"""

import contextlib
import glob
import io
import json
import os
import sys

# paths from the command line, resolved before moving into the repo folder
LAPS = [os.path.abspath(p) for p in sys.argv[1:]]
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
os.chdir(ROOT)

import convoy_check as cc  # noqa: E402

OUT = "logs/convoy_view.html"
SHIFTS = (0, 40, 80)       # move the real gaps this far along the track, cm
SEEDS = (1, 2, 3, 4, 5)    # random 20 percent loss, the old check, as a control
GAP = 60
SIZE = 360                 # plot width and height in px


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


def svg_path(points, scale, ox, oy, colour, width, dash=""):
    step = max(1, len(points) // 600)
    pts = points[::step]
    d = " ".join(f"{ox + x * scale:.1f},{oy - y * scale:.1f}" for x, y in pts)
    dash = f' stroke-dasharray="{dash}"' if dash else ""
    return (f'<polyline points="{d}" fill="none" stroke="{colour}" '
            f'stroke-width="{width}"{dash}/>')


def plot(chord, arc):
    """Top view: true path, the chord map, where each version drove, lost packets."""
    pts = chord["truth"] + chord["drove"] + arc["drove"] + chord["map"]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    scale = (SIZE - 20) / span
    ox = 10 - min(xs) * scale + (SIZE - 20 - (max(xs) - min(xs)) * scale) / 2
    oy = 10 + max(ys) * scale + (SIZE - 20 - (max(ys) - min(ys)) * scale) / 2
    lost = []
    # mark lost packets on the true path by their distance along it
    s_points = chord["truth_s"]
    for s in chord["dropped"][::2]:
        k = cc.bisect_left(s_points, s)
        if k < len(chord["truth"]):
            x, y = chord["truth"][k]
            lost.append(f'<circle cx="{ox + x * scale:.1f}" cy="{oy - y * scale:.1f}" '
                        f'r="2.2" fill="var(--lost)"/>')
    return (f'<svg viewBox="0 0 {SIZE} {SIZE}" class="top">'
            + svg_path(chord["truth"], scale, ox, oy, "var(--truth)", 6)
            + "".join(lost)
            + svg_path(chord["map"], scale, ox, oy, "var(--chord)", 1, "3 3")
            + svg_path(chord["drove"], scale, ox, oy, "var(--chord)", 1.6)
            + svg_path(arc["drove"], scale, ox, oy, "var(--arc)", 1.6)
            + "</svg>")


def chart(chord, arc, top=20.0):
    """Off the path over time, chord and arc."""
    w, h = SIZE, 110
    t_end = max((chord["times"] or [1])[-1], (arc["times"] or [1])[-1])
    top = max(top, worst(chord), worst(arc))

    def line(r, colour):
        if not r["off"]:
            return ""
        step = max(1, len(r["off"]) // 500)
        d = " ".join(f"{30 + t / t_end * (w - 40):.1f},{h - 15 - o / top * (h - 25):.1f}"
                     for t, o in list(zip(r["times"], r["off"]))[::step])
        return f'<polyline points="{d}" fill="none" stroke="{colour}" stroke-width="1.4"/>'

    ticks = "".join(
        f'<line x1="30" x2="{w - 10}" y1="{h - 15 - v / top * (h - 25):.1f}" '
        f'y2="{h - 15 - v / top * (h - 25):.1f}" class="grid"/>'
        f'<text x="26" y="{h - 12 - v / top * (h - 25):.1f}" class="tick">{v:.0f}</text>'
        for v in (0, 5, 10, 15, 20) if v <= top)
    return (f'<svg viewBox="0 0 {w} {h}" class="chart">{ticks}'
            + line(chord, "var(--chord)") + line(arc, "var(--arc)")
            + f'<text x="{w - 10}" y="{h - 2}" class="tick" text-anchor="end">'
              f'{t_end:.0f} s</text>'
            + '<text x="32" y="10" class="label">cm off the path</text></svg>')


def main(laps):
    patterns = gap_patterns()
    rows, panels, worst_runs = [], [], []
    for lap in laps:
        packets = [json.loads(line) for line in open(lap)]
        lap_name = os.path.basename(lap).replace("master-", "").replace(".jsonl", "")
        path = cc.cs.Path()
        for p in packets:
            path.add(p["s_cm"], p["heading"])
        truth_s = path.s
        for name, cut in patterns:
            chord = quiet(cc.simulate, packets, GAP, cut=cut)
            arc = quiet(cc.simulate, packets, GAP, cut=cut, arc=True)
            chord["truth_s"] = truth_s
            rows.append((lap_name, name, worst(chord), worst(arc),
                         min(chord["gaps"]), max(chord["gaps"]),
                         min(arc["gaps"]), max(arc["gaps"]),
                         chord["state"], arc["state"]))
            if not name.count("moved"):
                panels.append((lap_name, name, chord, arc))
            else:
                worst_runs.append((max(worst(chord), worst(arc)), lap_name, name, chord, arc))
        for seed in SEEDS:
            chord = quiet(cc.simulate, packets, GAP, loss=0.2, seed=seed)
            arc = quiet(cc.simulate, packets, GAP, loss=0.2, seed=seed, arc=True)
            rows.append((lap_name, f"random 20% lost, seed {seed}", worst(chord),
                         worst(arc), min(chord["gaps"]), max(chord["gaps"]),
                         min(arc["gaps"]), max(arc["gaps"]),
                         chord["state"], arc["state"]))

    print(f"{'lap':16} {'packets lost':32} {'chord max':>9} {'arc max':>8}")
    for r in rows:
        print(f"{r[0]:16} {r[1]:32} {r[2]:8.1f}  {r[3]:7.1f}")

    real = [r for r in rows if r[1].startswith("run")]
    better = sum(1 for r in real if r[3] < r[2] - 0.2)
    same = sum(1 for r in real if abs(r[3] - r[2]) <= 0.2)
    worse = len(real) - better - same
    summary = (f"With the real gap patterns, the arc was better in {better}, "
               f"the same in {same} and worse in {worse} of {len(real)} runs. "
               f"Worst off the path: chord {max(r[2] for r in real):.1f} cm, "
               f"arc {max(r[3] for r in real):.1f} cm.")
    print(summary)

    table = "".join(
        f"<tr class='{'ctrl' if r[1].startswith('random') else ''}'>"
        f"<td>{r[0]}</td><td>{r[1]}</td>"
        f"<td class='num {'bad' if r[2] > 5 else ''}'>{r[2]:.1f}</td>"
        f"<td class='num {'bad' if r[3] > 5 else ''}'>{r[3]:.1f}</td>"
        f"<td class='num'>{r[4]:.0f} to {r[5]:.0f}</td>"
        f"<td class='num'>{r[6]:.0f} to {r[7]:.0f}</td>"
        f"<td>{r[8]} / {r[9]}</td></tr>" for r in rows)
    worst_runs.sort(key=lambda w: -w[0])
    worst_cards = "".join(
        f"<figure><figcaption><b>lap {lap}</b>, packets lost like {name}<br>"
        f"worst off the path: <span class='c'>chord {worst(c):.1f} cm</span>, "
        f"<span class='a'>arc {worst(a):.1f} cm</span></figcaption>"
        f"{plot(c, a)}{chart(c, a)}</figure>"
        for _, lap, name, c, a in worst_runs[:3])
    cards = "".join(
        f"<figure><figcaption><b>lap {lap}</b>, packets lost like {name}<br>"
        f"worst off the path: <span class='c'>chord {worst(c):.1f} cm</span>, "
        f"<span class='a'>arc {worst(a):.1f} cm</span></figcaption>"
        f"{plot(c, a)}{chart(c, a)}</figure>"
        for lap, name, c, a in panels)

    html = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Convoy slave sim</title>
<style>
:root {{ --bg:#fbfaf7; --fg:#1d1d1b; --muted:#6b6b66; --line:#e3e0d8;
  --truth:#d9d4c7; --lost:#e8a33d; --chord:#c8423b; --arc:#2f6db5; }}
@media (prefers-color-scheme: dark) {{ :root {{ --bg:#17181a; --fg:#e8e6e1;
  --muted:#9a988f; --line:#2c2e31; --truth:#3a3c40; --lost:#e0a050;
  --chord:#ef6a60; --arc:#6aa3e8; }} }}
body {{ background:var(--bg); color:var(--fg); margin:0;
  font:15px/1.5 system-ui, sans-serif; }}
main {{ max-width:1180px; margin:0 auto; padding:24px 16px 60px; }}
h1 {{ font-size:22px; margin:0 0 4px; }} h2 {{ font-size:17px; margin:36px 0 8px; }}
p {{ color:var(--muted); max-width:760px; margin:4px 0; }}
.key span {{ display:inline-block; margin-right:16px; }}
.key i {{ display:inline-block; width:22px; height:4px; vertical-align:middle;
  margin-right:6px; border-radius:2px; }}
.grid-cards {{ display:grid; grid-template-columns:repeat(auto-fill,minmax(340px,1fr));
  gap:18px; }}
figure {{ margin:0; border:1px solid var(--line); border-radius:8px; padding:12px; }}
figcaption {{ font-size:13px; color:var(--muted); margin-bottom:6px; }}
.c {{ color:var(--chord); }} .a {{ color:var(--arc); }}
svg {{ width:100%; height:auto; display:block; }}
.chart {{ margin-top:6px; }}
.grid {{ stroke:var(--line); stroke-width:1; }}
.tick, .label {{ fill:var(--muted); font-size:10px; }} .chart .tick {{ text-anchor:end; }}
.summary {{ font-size:16px; color:var(--fg); margin:14px 0; padding:10px 14px;
  border-left:4px solid var(--arc); background:color-mix(in srgb,var(--arc) 8%,transparent); }}
.tablewrap {{ overflow-x:auto; }}
table {{ border-collapse:collapse; font-size:13px; width:100%; }}
th, td {{ text-align:left; padding:5px 10px; border-bottom:1px solid var(--line);
  white-space:nowrap; }}
.num {{ text-align:right; font-variant-numeric:tabular-nums; }}
.bad {{ color:var(--chord); font-weight:600; }}
tr.ctrl td {{ color:var(--muted); }}
</style></head><body><main>
<h1>Convoy slave against Mani's recorded laps</h1>
<p>The slave follows {GAP} cm behind. Each run is done twice: lost stretches
bridged with a straight chord (the code now) and with an arc. Packets are lost
like on the real track runs (logs/convoy_slave_*), also moved along the track,
and with 20 percent random loss as a control.</p>
<div class="summary">{summary}</div>
<p class="key"><span><i style="background:var(--truth);height:8px"></i>master's true path</span>
<span><i style="background:var(--lost)"></i>lost packets</span>
<span><i style="background:var(--chord)"></i>slave, chord (dashed: its map)</span>
<span><i style="background:var(--arc)"></i>slave, arc</span></p>
<h2>Top view per lap, real gap patterns</h2>
<div class="grid-cards">{cards}</div>
<h2>The 3 worst runs, gaps moved along the track</h2>
<p>The same real gaps, but falling on a different part of the track.</p>
<div class="grid-cards">{worst_cards}</div>
<h2>Every run</h2>
<div class="tablewrap"><table><tr><th>lap</th><th>packets lost</th>
<th class="num">chord max off, cm</th><th class="num">arc max off, cm</th>
<th class="num">chord gap, cm</th><th class="num">arc gap, cm</th>
<th>end state chord / arc</th></tr>{table}</table></div>
</main></body></html>
"""
    with open(OUT, "w") as f:
        f.write(html)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    if not LAPS:
        sys.exit(__doc__)
    main(LAPS)

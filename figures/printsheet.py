# /// script
# dependencies = ["matplotlib"]
# ///
"""Lab 11 figures and exhibits on A4 sheets, at notebook size.

Each item is 119 x 84 mm with its caption printed underneath, so nothing
has to be copied by hand. Print at 100 percent, not "fit to page", then
cut and paste into the notebook.

Every number comes from the run logs in ../logs, except where a row says
otherwise.

Run on the laptop, from this folder:  uv run printsheet.py
"""

import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.patches import Rectangle

LOGS = Path(__file__).resolve().parent.parent / "logs"
A4 = (8.27, 11.69)
FIG_W, FIG_H = 4.7, 3.3
INK, MUTED, RULE = "#0b0b0b", "#52514e", "#c9c8c3"

# config.py values the calibration replaced
OLD_CM_PER_PULSE = 15.08 / 693
RULER_TRACK_CM = 10.8

CAPTIONS = {
    "11A": (
        "Exhibit 11A. Calibration of my rover, calibrate.py, 29 Oct 2026, steps 1\n"
        "to 3. Roll: about 2 m at 30 rpm, tape measure beside the path. The two\n"
        "dropped rolls hit the tape mark and turned, so the wheels slipped.\n"
        "Circle: 20 / 40 rpm, 2 turns. The effective track is 22 percent above\n"
        "the ruler: the tyres slip sideways in a turn. Check: 100 cm by encoder\n"
        "against the tape. All three read 0.5 to 0.8 percent far, under 8 mm."),
}


def last_row(name):
    with open(LOGS / name, newline="") as f:
        return list(csv.DictReader(f))[-1]


def exhibit_11a(ax):
    rows = []
    rolls = [("131551", "dropped, tape"), ("131906", "dropped, tape"),
             ("132335", "1"), ("132759", "2"), ("133022", "3")]
    kept = []
    for stamp, run in rolls:
        r = last_row(f"calibrate_roll_{stamp}.csv")
        entered = r["entered"].split()
        tape = f"{entered[0]}, {entered[1] if len(entered) > 1 else '-'}"
        left, right = (float(x) for x in r["result"].split())
        if run.isdigit():
            kept.append((left, right))
        rows.append(["roll", run, f"{r['left_pulses']} / {r['right_pulses']}",
                     tape, f"{left:.5f} / {right:.5f}"])
    mean_l = sum(k[0] for k in kept) / len(kept)
    mean_r = sum(k[1] for k in kept) / len(kept)
    rows.append(["roll", "mean 1-3", "", "",
                 f"{mean_l:.5f} / {mean_r:.5f}"])
    rows.append(["roll", "old", "", "15.08 / 693", f"{OLD_CM_PER_PULSE:.5f}"])

    tracks = []
    for stamp in ("133404", "133504", "133548", "133705", "133752", "133835"):
        r = last_row(f"calibrate_circle_{stamp}.csv")
        track = float(r["result"].split()[2])
        tracks.append(track)
        rows.append(["circle", r["side"],
                     f"{r['left_pulses']} / {r['right_pulses']}",
                     f"{float(r['entered']):.0f} turns", f"{track:.2f} cm"])
    rows.append(["circle", "mean", "", "",
                 f"{sum(tracks) / len(tracks):.2f} cm"])
    rows.append(["circle", "ruler", "", "", f"{RULER_TRACK_CM:.1f} cm"])

    # check 1 crashed on the side input before writing its log, so its
    # numbers are from the terminal. check 3 had -2 typed for forward
    checks = [("1, terminal", 4940, 4940, 100.0, -2.0)]
    for stamp, run in (("134428", "2"), ("134630", "3, typo fixed")):
        r = last_row(f"calibrate_check_{stamp}.csv")
        side = float(r["entered"].split()[1])
        checks.append((run, int(r["left_pulses"]), int(r["right_pulses"]),
                       100.0, side))
    for run, lp, rp, forward, side in checks:
        encoder = (lp * mean_l + rp * mean_r) / 2
        tape = math.hypot(forward, side)
        error = (encoder - tape) / tape * 100
        rows.append(["check", run, f"{lp} / {rp}",
                     f"{forward:.0f}, {side:g}",
                     f"{encoder:.1f} cm, {error:+.2f} %"])

    head = ["test", "run", "pulses L / R", "tape fwd, side",
            "result L / R"]
    ax.axis("off")
    table = ax.table(cellText=rows, colLabels=head, loc="center",
                     cellLoc="center", colLoc="center",
                     colWidths=[0.11, 0.2, 0.2, 0.19, 0.3])
    table.auto_set_font_size(False)
    table.set_fontsize(6.2)
    table.scale(1, 0.92)
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor(RULE)
        cell.set_linewidth(0.4)
        if row == 0:
            cell.set_text_props(fontweight="bold", color=INK)
            cell.set_facecolor("#efeee9")
            continue
        run = rows[row - 1][1]
        if run.startswith("dropped"):
            cell.set_text_props(color=MUTED)
        if run.startswith("mean"):
            cell.set_text_props(fontweight="bold")


def page(pdf, items, title):
    fig = plt.figure(figsize=A4, dpi=300)
    left = (A4[0] - FIG_W) / 2 / A4[0]
    width = FIG_W / A4[0]
    height = FIG_H / A4[1]
    for i, (key, draw) in enumerate(items):
        bottom = 1 - (0.06 + (i + 1) * height + i * 0.16)
        draw(fig.add_axes([left, bottom, width, height]))
        fig.add_artist(Rectangle((left, bottom), width, height,
                                 transform=fig.transFigure, facecolor="none",
                                 edgecolor=RULE, linewidth=0.4,
                                 linestyle=(0, (4, 3))))
        fig.text(left, bottom - 0.015, CAPTIONS[key], fontsize=7.5,
                 color=INK, va="top", ha="left", linespacing=1.5)
    fig.text(left, 0.975, title, fontsize=7, color=MUTED, va="top")
    pdf.savefig(fig)
    return fig


with PdfPages("lab11-print-sheet.pdf") as pdf:
    first = page(pdf, [("11A", exhibit_11a)],
                 "CPSY Lab 11 figures, page 1, cut along the dashed edges")
    first.savefig("lab11-print-sheet-1.png", dpi=110)
print("wrote lab11-print-sheet.pdf")

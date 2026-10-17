# -*- coding: utf-8 -*-
"""iPoster ekran grafikleri (4 Ekim 2026): 16:9, buyuk punto, az etiket.

Renk: dataviz referans paleti (dogrulanmis, degistirilmedi) - sira: mavi, turuncu, aqua.
Durum (karsilandi / karsilanmadi): ayrilmis good/critical renkleri + ikon + etiket.
Iki farkli olcu ayni eksende gosterilmez (irtifa ve itki ayri paneller).
Cikti: cikti/G*.png (1600x900)
"""
import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

sys.stdout.reconfigure(encoding="utf-8")
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "graphics")
DATA = os.path.join(HERE, "..", "..", "data")

# ---- referans palet (dataviz references/palette.md, acik mod)
SURF, INK, INK2, MUTED = "#fcfcfb", "#0b0b0b", "#52514e", "#898781"
GRID, BASE = "#e1e0d9", "#c3c2b7"
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
GOOD, CRIT = "#0ca30c", "#d03b3b"

plt.rcParams.update({
    "font.family": ["Segoe UI", "Segoe UI Symbol", "DejaVu Sans"],
    "font.size": 15, "axes.labelsize": 15, "axes.titlesize": 19, "xtick.labelsize": 13,
    "ytick.labelsize": 13, "legend.fontsize": 13,
    "axes.edgecolor": BASE, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
    "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False,
    "figure.facecolor": SURF, "axes.facecolor": SURF, "savefig.facecolor": SURF,
    "savefig.dpi": 200,
})


def kaydet(fig, ad):
    p = os.path.join(OUT, ad)
    fig.savefig(p)
    plt.close(fig)
    print("yazildi", ad)


# ------------------------------------------------------------------ G1 sayilarla arac
def g1():
    tiles = [("1.20 m", "tall and wide\n(landing gear deployed)"),
             ("29 kg", "wet mass\n(26 kg dry + 3 kg air)"),
             ("600 N", "throttled cold-gas thrust,\none nozzle on a ±6° gimbal"),
             ("300 bar", "dry air in a 9 L tank\n(about 3 kg)"),
             ("≈1371 N·s", "estimated total impulse\n(real-gas blowdown model)"),
             ("USD 3,700", "approximate hardware cost,\nbuilt by undergraduates")]
    fig = plt.figure(figsize=(8, 4.5))
    for i, (big, small) in enumerate(tiles):
        r, c = divmod(i, 3)
        x0, y0 = 0.03 + c * 0.325, 0.53 - r * 0.47
        fig.patches.append(FancyBboxPatch((x0, y0), 0.30, 0.42, boxstyle="round,pad=0,rounding_size=0.02",
                                          transform=fig.transFigure, facecolor="#ffffff", edgecolor=GRID, lw=1.2))
        fig.text(x0 + 0.02, y0 + 0.27, big, fontsize=28, fontweight="bold", color=INK)
        fig.text(x0 + 0.02, y0 + 0.06, small, fontsize=12.5, color=INK2, linespacing=1.3)
    kaydet(fig, "G1_vehicle_numbers.png")


# ------------------------------------------------------------------ G2 impuls butcesi
def g2():
    W_WET = 28.98 * 9.81
    # toplam ve agirlik ustu: makaledeki ayni tablolardan (sekil_uret.py) -> 1371 ve ~947
    total, above = 1371.0, 947.0
    hil = 792.0                      # HIL inis kaydinin integrali (makale 6.2, Sekil 14)
    items = [("HIL descent from 8 m", hil), ("5 s hover, 26 kg", 25.939 * 9.81 * 5), ("5 s hover, 33.8 kg", 331.6 * 5)]
    fig, ax = plt.subplots(figsize=(8, 4.5))
    fig.subplots_adjust(left=0.25, right=0.95, top=0.80, bottom=0.17)
    y = np.arange(len(items))[::-1]
    for yi, (lab, v) in zip(y, items):
        ok = v <= above
        col = GOOD if ok else "#9aa3ab"
        ax.add_patch(FancyBboxPatch((0, yi - 0.28), v, 0.56, boxstyle="round,pad=0,rounding_size=12",
                                    mutation_aspect=0.02, facecolor=col, edgecolor="none"))
        ax.text(v + 25, yi, "%d N·s  %s" % (round(v), "✓ within budget" if ok else "larger tank"),
                va="center", fontsize=14, color=INK, fontweight="bold", zorder=5,
                bbox=dict(facecolor=SURF, edgecolor="none", pad=2.0))
    ax.set_axisbelow(True)
    ax.axvline(total, color=INK, lw=1.6, ls="--", zorder=3)
    ax.text(total + 20, len(items) - 0.38, "total ≈1371", ha="left", fontsize=13, color=INK)
    ax.axvline(above, color=INK2, lw=1.6, ls=":", zorder=3)
    ax.text(above - 20, len(items) - 0.38, "above weight ≈950", ha="right", fontsize=13, color=INK2)
    ax.set_yticks(y)
    ax.set_yticklabels([i[0] for i in items], fontsize=14, color=INK)
    ax.set_xlim(0, 2450)
    ax.set_ylim(-0.6, len(items) - 0.2)
    ax.set_xlabel("Impulse needed [N·s]")
    ax.grid(axis="y", visible=False)
    ax.tick_params(axis="y", length=0)
    fig.suptitle("Stored impulse, not thrust, limits the mission", x=0.03, ha="left", fontsize=19,
                 fontweight="bold", color=INK, y=0.95)
    kaydet(fig, "G2_impulse_budget.png")


# ------------------------------------------------------------------ G3 HIL inisi
def g3():
    p = os.path.join(DATA, "hil", "hil_landing_run001_detail.csv")
    r = list(csv.reader(open(p, encoding="utf-8-sig"), delimiter=";"))
    c = {k: i for i, k in enumerate(r[0])}
    d = [x for x in r[1:] if len(x) == len(r[0])]
    g = lambda k: np.array([float(x[c[k]].replace(",", ".")) for x in d])
    t, z, T = g("saniye"), g("irtifa_m"), g("itki_N")
    m = t <= 2.62
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 4.5), sharex=True, gridspec_kw={"height_ratios": [1.25, 1]})
    fig.subplots_adjust(left=0.10, right=0.97, top=0.86, bottom=0.13, hspace=0.18)
    a1.plot(t[m], z[m], color=S1, lw=2.2)
    a1.axhline(5, color=MUTED, lw=1.2, ls=":")
    a1.text(1.55, 5.25, "thrust allowed below 5 m", fontsize=12, color=INK2)
    a1.annotate("descent arrested 0.13 m above the ground", xy=(1.88, 0.15), xytext=(1.25, 2.9), fontsize=12.5,
                color=INK, arrowprops=dict(arrowstyle="->", color=INK2, lw=1.1))
    a1.set_ylabel("Altitude [m]")
    a1.set_ylim(-0.3, 8.6)
    a2.plot(t[m], T[m], color=S2, lw=2.2)
    a2.axhline(600, color=MUTED, lw=1.2, ls=":")
    a2.text(0.02, 620, "600 N limit", fontsize=12, color=INK2)
    a2.set_ylabel("Thrust [N]")
    a2.set_ylim(-30, 760)
    a2.set_xlabel("Time from release [s]")
    a2.set_xlim(0, 2.62)
    fig.suptitle("HIL landing from 8 m: braking at the 600 N thrust limit", x=0.03, ha="left",
                 fontsize=18, fontweight="bold", color=INK, y=0.97)
    kaydet(fig, "G3_HIL_descent.png")


if __name__ == "__main__":
    for f in (g1, g2, g3):
        f()

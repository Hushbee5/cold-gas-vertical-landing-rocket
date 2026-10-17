# -*- coding: utf-8 -*-
"""Ingilizce etiketli, yayin kalitesinde sekil uretimi (gercek olcum/hesap verisinden).

Uretilen dosyalar ../sekiller/F0*.png
Calistir: python sekil_uret.py
"""
import sys
import os
import csv
import shutil
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import openpyxl

sys.stdout.reconfigure(encoding="utf-8")

OUT = "../figures/paper"
XLSX = "../data/propulsion/thrust_time_tables.xlsx"

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif"],
    "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 9.5,
    "legend.fontsize": 8, "xtick.labelsize": 8, "ytick.labelsize": 8,
    "axes.grid": True, "grid.alpha": 0.3, "grid.linestyle": "--",
    "figure.dpi": 300, "savefig.dpi": 300, "savefig.bbox": "tight",
    "mathtext.default": "regular",
})

# ---------------------------------------------------------------- data
wb = openpyxl.load_workbook(XLSX, data_only=True)
cases = {}
for sheet, label in [("Itki_Zaman_300N", "300 N"),
                     ("Itki_Zaman_400N", "400 N"),
                     ("Itki_Zaman_600N", "600 N")]:
    rows = [(r[0], r[1]) for r in wb[sheet].iter_rows(min_row=2, values_only=True)
            if r[0] is not None and r[1] is not None]
    cases[label] = (np.array([r[0] for r in rows]),
                    np.array([r[1] for r in rows]))

STYLE = {"300 N": "-", "400 N": "--", "600 N": "-."}

# ------------------------------------------------- F01 thrust vs time
fig, ax = plt.subplots(figsize=(3.4, 2.3))
for label, (t, F) in cases.items():
    ax.plot(t, F, STYLE[label], lw=1.3, label=label + " command")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Thrust [N]")
ax.set_xlim(0, 8)
ax.set_ylim(0, 660)
ax.legend(frameon=False, loc="upper right")
fig.savefig(os.path.join(OUT, "F01_thrust_time.png"))
plt.close(fig)

# --------------------------------------------- F02 cumulative impulse
totals = []
fig, ax = plt.subplots(figsize=(3.4, 2.3))
for label, (t, F) in cases.items():
    I = np.concatenate([[0.0], np.cumsum(np.diff(t) * (F[:-1] + F[1:]) / 2.0)])
    totals.append(I[-1])
    ax.plot(t, I, STYLE[label], lw=1.3, label=label + " command")
I_BUDGET = float(np.mean(totals))
ax.axhline(I_BUDGET, color="0.35", lw=0.9, ls=":")
ax.text(0.4, I_BUDGET + 50, "usable total impulse " + str(round(I_BUDGET)) + " N·s",
        fontsize=7.5, color="0.25")
ax.set_xlabel("Time [s]")
ax.set_ylabel("Cumulative impulse [N·s]")
ax.set_xlim(0, 18)
ax.set_ylim(0, 1650)
ax.legend(frameon=False, loc="lower right")
fig.savefig(os.path.join(OUT, "F02_cumulative_impulse.png"))
plt.close(fig)

# ------------------------------------------------- F03 impulse budget
g = 9.81
# HIL inis kosusu (3 Ekim 2026): 600 N x 1.42 s (852 N s) ust siniri yerine kayittan integre
# edilen impuls. Kaynak AHR_DIKEY_INIS/S-TEK-HIL-ayrinti-no-001.csv (veri/ altina kopyalanir).
HIL_ARCH = "../data/hil/hil_landing_run001_detail.csv"
_r = list(csv.reader(open(HIL_ARCH, encoding="utf-8-sig"), delimiter=";"))
_c = {k: i for i, k in enumerate(_r[0])}
_d = [x for x in _r[1:] if len(x) == len(_r[0])]
_num = lambda x, k: float(x[_c[k]].replace(",", "."))
I_HIL = float(np.trapezoid([_num(x, "itki_N") for x in _d], [_num(x, "saniye") for x in _d]))
missions = [
    ("HIL descent\nfrom 8 m (33.8 kg)", I_HIL),
    ("Hover 5 s\n254 N (25.9 kg)", 25.939 * g * 5.0),
    ("Hover 5 s\n332 N (33.8 kg)", 331.6 * 5.0),
    ("HIL hover run\n332 N, 25.4 s", 331.6 * 25.42),
]
fig, ax = plt.subplots(figsize=(3.4, 2.5))
labels = [m[0] for m in missions]
values = [m[1] for m in missions]
# agirligin ustundeki impuls (3 Ekim 2026): itki 28.98 kg islak agirligi asarken verilen kisim,
# uc komutun ortalamasi (~947 N s). Yesil/kirmizi ayrimi bu esige gore.
W_WET = 28.98 * g
I_W = float(np.mean([np.trapezoid(np.where(F >= W_WET, F, 0.0), t) for (t, F) in cases.values()]))
colors = ["#2F6E4E" if v <= I_W else "#9E3320" for v in values]
y = np.arange(len(labels))
ax.barh(y, values, color=colors, height=0.6)
ax.axvline(I_BUDGET, color="0.2", lw=1.1, ls="--")
ax.text(I_BUDGET * 1.05, -0.60, "total " + str(round(I_BUDGET)) + " N·s",
        fontsize=7.5, color="0.2")
ax.axvline(I_W, color="0.35", lw=1.0, ls=":")
ax.text(I_W * 0.96, -0.60, "above weight " + str(round(I_W)), fontsize=7.0, color="0.3", ha="right")
for i, v in enumerate(values):
    # beyaz zemin: butce cizgisine yakin degerler (1272) cizginin altinda kalip "272" okunmasin
    ax.text(v * 1.04, i, str(round(v)), va="center", fontsize=7.5, zorder=3,
            bbox=dict(facecolor="white", edgecolor="none", pad=0.4))
ax.set_yticks(y)
ax.set_yticklabels(labels, fontsize=7.5)
ax.invert_yaxis()
ax.set_xlabel("Required impulse [N·s]")
ax.set_xscale("log")
ax.set_xlim(500, 25000)
ax.grid(axis="y", alpha=0.0)
fig.savefig(os.path.join(OUT, "F03_impulse_budget.png"))
plt.close(fig)

# ------------------------------------- F04 nozzle exit temperature
GAMMA, T0 = 1.4, 293.15
M = np.linspace(1.0, 5.0, 400)
Te = T0 / (1.0 + (GAMMA - 1.0) / 2.0 * M ** 2)
Me_design = 4.07
Te_design = T0 / (1.0 + (GAMMA - 1.0) / 2.0 * Me_design ** 2)

# Tank adyabatik genlesmeyle sogur: 150 bar anindaki gaz sicakligi bosalma
# modelinden 246.1 K (300 K baslangic; izentropik deger 300*(0.5)^0.286 ile ayni)
T0_COOL = 246.1
Te_cool = T0_COOL / (1.0 + (GAMMA - 1.0) / 2.0 * M ** 2)
Te_cool_design = T0_COOL / (1.0 + (GAMMA - 1.0) / 2.0 * Me_design ** 2)

fig, ax = plt.subplots(figsize=(3.4, 2.3))
ax.plot(M, Te, "-", lw=1.4, color="#1F5C7A", label="inlet 293 K (ambient)")
ax.plot(M, Te_cool, "--", lw=1.3, color="#1F5C7A", label="inlet 246 K (tank at 150 bar)")
ax.axhline(90.2, color="#9E3320", lw=0.9, ls="--")
ax.axhline(77.4, color="#9E3320", lw=0.9, ls=":")
ax.text(1.05, 95, "O2 boiling point, 90.2 K", fontsize=7, color="#9E3320")
ax.text(1.05, 60, "N2 boiling point, 77.4 K", fontsize=7, color="#9E3320")
ax.plot([Me_design], [Te_design], "o", ms=5, color="#16202B")
ax.plot([Me_design], [Te_cool_design], "s", ms=4.5, color="#16202B")
ax.annotate("design point, Me = 4.07\nTe = " + str(round(Te_design)) + " K / "
            + str(round(Te_cool_design)) + " K",
            xy=(Me_design, Te_design), xytext=(2.35, 150), fontsize=7,
            arrowprops=dict(arrowstyle="->", lw=0.7, color="#16202B"))
ax.legend(frameon=False, loc="upper right", fontsize=6.6)
ax.set_xlabel("Exit Mach number [-]")
ax.set_ylabel("Exit static temperature [K]")
ax.set_xlim(1, 5)
ax.set_ylim(0, 270)
fig.savefig(os.path.join(OUT, "F04_nozzle_exit_temperature.png"))
plt.close(fig)

# ------------------------------------------- report
print("Usable total impulse : %.1f N·s" % I_BUDGET)
print("HIL descent impulse  : %.1f N·s" % I_HIL)
print("Above-weight impulse : %.1f N·s" % I_W)
print("Nozzle exit temp     : %.1f K at Me = %.2f" % (Te_design, Me_design))
print("Generated figures:")
for name in sorted(os.listdir(OUT)):
    if name.startswith("F0"):
        kb = os.path.getsize(os.path.join(OUT, name)) // 1024
        print("  %-36s %4d KB" % (name, kb))

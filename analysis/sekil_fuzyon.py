# -*- coding: utf-8 -*-
"""Ayse Sila'nin fuzyon dogrulama grafigini Ingilizce etiketlerle yeniden yazar.
Kaynak: Fuzyon_Dogrulama_Verisi.docx icindeki gomulu PNG (1350x630).
Uretilen: ../sekiller/F10_fusion_validation.png
"""
import os
import sys
from PIL import Image, ImageDraw, ImageFont

sys.stdout.reconfigure(encoding="utf-8")
FIG = "../figures/paper"
SRC = os.path.join(FIG, "fusion_bench_source.png")


def font(sz, bold=False):
    for n in (("timesbd.ttf" if bold else "times.ttf"), "arial.ttf", "DejaVuSans.ttf"):
        try:
            return ImageFont.truetype(n, sz)
        except OSError:
            continue
    return ImageFont.load_default()


im = Image.open(SRC).convert("RGB")
W, H = im.size
d = ImageDraw.Draw(im)

WHITE = (255, 255, 255)

# ---- baslik (ust orta) ----------------------------------------------
d.rectangle([0, 0, W, 58], fill=WHITE)
f = font(26)
txt = "Fusion validation - static bench test (1375 packets, ~50 Hz)"
d.text(((W - d.textlength(txt, font=f)) / 2, 10), txt, font=f, fill=(0, 0, 0))

# ---- efsane kutusu (sol ust) ----------------------------------------
# kutu cercevesi korunur, sadece metinler degistirilir
f2 = font(21)
LEG = [(148, 72, "Raw barometer"),
       (148, 103, "Raw LiDAR"),
       (148, 134, "Kalman fused estimate")]
d.rectangle([88, 60, 505, 172], fill=WHITE)
for x, y, s in LEG:
    d.text((x, y), s, font=f2, fill=(0, 0, 0))
# renk cizgileri yeniden cizilir
for y, col in [(84, (57, 106, 177)), (115, (218, 124, 48)), (146, (62, 150, 81))]:
    d.line([(96, y), (136, y)], fill=col, width=3)

# ---- x ekseni etiketi (alt orta) ------------------------------------
d.rectangle([0, H - 62, W, H], fill=WHITE)
f3 = font(23)
txt = "Time [s]"
d.text(((W - d.textlength(txt, font=f3)) / 2, H - 40), txt, font=f3, fill=(0, 0, 0))

# ---- y ekseni etiketi (sol, dondurulmus) ----------------------------
d.rectangle([0, 60, 52, H - 62], fill=WHITE)
lab = Image.new("RGB", (200, 32), WHITE)
ld = ImageDraw.Draw(lab)
ld.text((0, 0), "Altitude [m]", font=f3, fill=(0, 0, 0))
lab = lab.rotate(90, expand=True)
im.paste(lab, (2, int(H / 2) - 100))

out = os.path.join(FIG, "F10_fusion_validation.png")
im.save(out)
print("yazildi:", os.path.basename(out), im.size,
      os.path.getsize(out) // 1024, "KB")

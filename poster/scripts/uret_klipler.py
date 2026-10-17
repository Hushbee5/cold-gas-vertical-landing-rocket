# -*- coding: utf-8 -*-
"""iPoster klipleri (4 Ekim 2026): kirp, kisalt, 16:9'a oturt, alt yazi bindir.

Cikti: cikti/*.mp4 (1280x720, 30 fps, H.264, sessiz, faststart) ve hero GIF.
Alt yazilar PIL ile seffaf PNG olarak cizilip videonun ustune bindirilir.
Calistir: python uret_klipler.py [klip_adi ...]
"""
import os
import subprocess
import sys

from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from araclar_video import FF  # noqa: E402

KOK = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
OUT = os.path.join(KOK, "media")
TMP = os.path.join(KOK, "tmp")
os.makedirs(OUT, exist_ok=True)
os.makedirs(TMP, exist_ok=True)
# Raw test videos are not in this repository (people are visible in them).
# Set RAW_VIDEO_DIR to a folder that mirrors the original layout to regenerate the clips.
DESK = os.environ.get("RAW_VIDEO_DIR", "raw_videos")
W, H = 1280, 720

FONT_B = next(p for p in (r"C:\Windows\Fonts\segoeuib.ttf", r"C:\Windows\Fonts\arialbd.ttf") if os.path.exists(p))
FONT_R = next(p for p in (r"C:\Windows\Fonts\segoeui.ttf", r"C:\Windows\Fonts\arial.ttf") if os.path.exists(p))


def altyazi(path, baslik, alt=None):
    """Alt kenarda yari saydam bant: kalin baslik + istege bagli ince alt satir."""
    im = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    fb = ImageFont.truetype(FONT_B, 36)
    fr = ImageFont.truetype(FONT_R, 27)
    h_band = 70 if not alt else 112
    d.rectangle([0, H - h_band, W, H], fill=(10, 20, 28, 175))
    d.rectangle([0, H - h_band, 10, H], fill=(47, 110, 78, 255))   # sol vurgu cizgisi
    d.text((34, H - h_band + 14), baslik, font=fb, fill=(255, 255, 255, 255))
    if alt:
        d.text((34, H - h_band + 62), alt, font=fr, fill=(214, 226, 232, 255))
    im.save(path)


def geom_dikey(crop=None):
    """Dikey goruntuyu 16:9'a bulanik arka planla oturtan filtre zinciri."""
    c = (crop + ",") if crop else ""
    return ("[0:v]%ssplit[a][b];[a]scale=%d:%d:force_original_aspect_ratio=increase,crop=%d:%d,"
            "boxblur=24:6,eq=brightness=-0.06[bg];[b]scale=-2:%d[fg];[bg][fg]overlay=(W-w)/2:(H-h)/2[g]"
            % (c, W, H, W, H, H))


def geom_yatay(crop=None):
    c = (crop + ",") if crop else ""
    return "[0:v]%sscale=%d:%d:force_original_aspect_ratio=increase,crop=%d:%d[g]" % (c, W, H, W, H)


def parca(src, t0, sure, geom, cap_png, out, hiz=1.0):
    """Tek parca: kirp/oturt -> hiz -> alt yazi -> 30 fps H.264."""
    setpts = "setpts=PTS/%g," % hiz if hiz != 1.0 else ""
    cikis = sure / hiz
    # dongudeki PNG girdisi sonsuz akistir: hem girdiye hem ciktiya sure sinirlari, overlay shortest=1
    fc = geom + ";[g]%sfps=30[gv];[gv][1:v]overlay=0:0:shortest=1,format=yuv420p[v]" % setpts
    cmd = [FF, "-hide_banner", "-loglevel", "error", "-y", "-ss", str(t0), "-t", str(sure), "-i", src,
           "-loop", "1", "-t", "%.3f" % (cikis + 0.5), "-i", cap_png, "-filter_complex", fc, "-map", "[v]", "-an",
           "-c:v", "libx264", "-crf", "22", "-preset", "medium", "-r", "30", "-t", "%.3f" % cikis,
           "-movflags", "+faststart", out]
    subprocess.run(cmd, check=True)


def birlestir(parcalar, out):
    lst = os.path.join(TMP, "liste.txt")
    with open(lst, "w", encoding="utf-8") as f:
        for p in parcalar:
            f.write("file '%s'\n" % p.replace("\\", "/"))
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", "-f", "concat", "-safe", "0", "-i", lst,
                    "-c", "copy", "-movflags", "+faststart", out], check=True)


def klip(ad, src, segs, geom):
    """segs: [(t0, sure, baslik, alt, hiz), ...]"""
    parcalar = []
    for i, (t0, sure, bas, alt, hiz) in enumerate(segs):
        cap = os.path.join(TMP, "%s_%d.png" % (ad, i))
        altyazi(cap, bas, alt)
        p = os.path.join(TMP, "%s_%d.mp4" % (ad, i))
        parca(src, t0, sure, geom, cap, p, hiz)
        parcalar.append(p)
    out = os.path.join(OUT, ad + ".mp4")
    birlestir(parcalar, out)
    print("%-22s %5.1f MB" % (ad + ".mp4", os.path.getsize(out) / 1e6))


HIL = DESK + r"\vertical rocket iac 2026\S-TEK AHR DOSYALARI\S-TEK DİKEY İNİŞLİ ROKET TAKIMI HIL TEST VİDEOSU.mp4"
KLIPLER = {
    "02_HIL": (HIL, geom_dikey("crop=588:1080:662:0"), [
        (68, 11, "Hardware-in-the-loop", "Real flight computer and servos, simulated vehicle", 1.0),
        (141, 10, "Same software on the vehicle avionics", "Servos and valve drive in the loop", 1.0),
        (183, 7, "50 Hz lock-step with the simulated plant", "5 defects found that simulation could not show", 1.0),
    ]),
    "07_avionics_board": (HIL, geom_dikey("crop=588:1080:662:0"), [
        (14, 9, "BORTE-I avionics board", "ESP32-S3, IMU, barometer, LiDAR, servo and valve drivers", 1.0),
    ]),
    "03_leak_timelapse": (r"D:\sizdirmazlik\sizdirmazlik_test_1_20x_sayacli.mp4", geom_yatay(), [
        (0, 174.9, "Leak test: 180 bar held for one hour", "Pressure loss below 5 % in all five runs  (time-lapse)", 11.66),
    ]),
    "04_soap_check": (r"D:\murat itki ahr\köpük sürmeli sızdırmazlık testi murat 2 dakika 50 saniye.mp4", geom_dikey(), [
        (21, 11, "Soap-film check of every fitting", None, 1.0),
    ]),
    "05_firing": (DESK + r"\stek pnömatik videoları\tek lüle itki performans testi\IMG_0979.MOV",
                  geom_yatay("crop=2160:1215:0:2250"), [
        (26, 10, "Single-nozzle firing at about 152 bar", "Flow and thrust in 5 of 5 runs; cold air leaves no visible plume", 1.0),
    ]),
    "06_drop_tests": (DESK + r"\Stek ahr\s-tek_inis_takimi_yapisal_butunluk_testi.mp4", geom_dikey(), [
        (16.5, 4.0, "Drop test 1: bare chassis, 0.4 m", None, 1.0),
        (18.5, 1.0, "Drop test 1: impact", "slow motion", 0.3),
        (35.5, 4.0, "Drop test 2: fully integrated, 0.4 m", None, 1.0),
        (37.4, 1.0, "Drop test 2: impact", "slow motion", 0.3),
        (39.4, 0.6, "No cracking, distortion or loosening", None, 1.0),
    ]),
}


def hero_gif():
    src = DESK + r"\vertical rocket iac 2026\S-TEK Kritik Tasarım Raporu\Patlatma Animasyonları\Roket 2026 Montaj.mp4"
    pal = os.path.join(TMP, "pal.png")
    base = "fps=15,scale=960:-1:flags=lanczos"
    pp = "[0:v]%s,split[f][r0];[r0]reverse[r];[f][r]concat=n=2:v=1:a=0" % base
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", "-i", src, "-filter_complex",
                    pp + ",palettegen=max_colors=128:stats_mode=full", pal], check=True)
    out = os.path.join(OUT, "01_hero_assembly.gif")
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", "-i", src, "-i", pal, "-filter_complex",
                    pp + "[x];[x][1:v]paletteuse=dither=bayer:bayer_scale=4", "-loop", "0", out], check=True)
    mp4 = os.path.join(OUT, "01_hero_assembly.mp4")
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", "-i", src, "-filter_complex",
                    "[0:v]fps=30,scale=1280:-2:flags=lanczos,split[f][r0];[r0]reverse[r];[f][r]concat=n=2:v=1:a=0,"
                    "pad=1280:720:(ow-iw)/2:(oh-ih)/2:white,format=yuv420p[v]",
                    "-map", "[v]", "-an", "-c:v", "libx264", "-crf", "20", "-movflags", "+faststart", mp4], check=True)
    print("%-22s %5.1f MB | mp4 %.1f MB" % ("01_hero_assembly.gif", os.path.getsize(out) / 1e6, os.path.getsize(mp4) / 1e6))


if __name__ == "__main__":
    secim = sys.argv[1:] or (["hero"] + list(KLIPLER))
    for ad in secim:
        if ad == "hero":
            hero_gif()
        else:
            src, geom, segs = KLIPLER[ad]
            klip(ad, src, segs, geom)

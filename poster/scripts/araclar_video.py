# -*- coding: utf-8 -*-
"""iPoster video yardimcilari: sure okuma ve kare tablosu (contact sheet)."""
import re, subprocess, sys
import imageio_ffmpeg
FF = imageio_ffmpeg.get_ffmpeg_exe()

def bilgi(path):
    p = subprocess.run([FF, "-hide_banner", "-i", path], capture_output=True, text=True, encoding="utf-8", errors="replace")
    m = re.search(r"Duration: (\d+):(\d+):([\d.]+)", p.stderr)
    dur = int(m.group(1)) * 3600 + int(m.group(2)) * 60 + float(m.group(3)) if m else None
    v = re.search(r"Video: (\w+).*?, (\d{2,5})x(\d{2,5})", p.stderr)
    rot = re.search(r"rotate\s*:\s*(-?\d+)|displaymatrix: rotation of (-?[\d.]+)", p.stderr)
    return dur, (v.group(1), int(v.group(2)), int(v.group(3))) if v else None, (rot.group(1) or rot.group(2)) if rot else None

def tablo(path, out, n=8, cols=4, w=360):
    dur, _, _ = bilgi(path)
    fps = n / max(dur, 0.1)
    rows = (n + cols - 1) // cols
    vf = "fps=%.6f,scale=%d:-2,drawtext=fontfile='C\:/Windows/Fonts/arial.ttf':text='%%{pts\:hms}':x=5:y=5:fontsize=18:fontcolor=yellow:box=1:boxcolor=black@0.6,tile=%dx%d" % (fps, w, cols, rows)
    subprocess.run([FF, "-hide_banner", "-loglevel", "error", "-y", "-i", path, "-vf", vf, "-frames:v", "1", out], check=True)
    return dur

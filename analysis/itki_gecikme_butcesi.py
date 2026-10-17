# -*- coding: utf-8 -*-
"""Itki gecikme butcesi — Bolum 5.5 ve Denklem (21) icin (24 Eylul 2026).

Yumusak inis profili (Bolum 2.1): arac vincle hr = 8-10 m'ye kaldirilir, birakilir.
Arsivlenen inis mantigi (rocket_firmware_inisHIL_.ino, LANDING_Z_ACTIVATION_M 5.0):
kestirim za = 5 m'nin ustundeyken motor KAPALI -> 5 m'ye kadar serbest dusus.
Itki, 5 m gecildikten tau saniye sonra sabit bir basamak olarak geldigi varsayilir
(arac icin EN IYIMSER durum: gercek bir itki rampasi daha fazlasini gerektirir).

  va   = sqrt(2 g (hr - za))            5 m'deki hiz
  vtau = va + g tau                     itki geldigindeki hiz
  htau = za - va tau - g tau^2 / 2      itki geldigindeki yukseklik
  Treq = m (g + vtau^2 / (2 htau))      yerde durdurmak icin en kucuk sabit itki  (21)

Kutle 28.98 kg (Tablo 1, islak). Itki siniri 600 N (yazilim, THRUST_MAX_N),
tam acik 1112 N (300 bar). Karsilastirma icin HIL kutlesi 33.8 kg da basilir.
Calistir: python itki_gecikme_butcesi.py
"""
import math
import sys

sys.stdout.reconfigure(encoding="utf-8")

G = 9.81
ZA = 5.0
M_WET = 28.98
M_HIL = 33.8
T_LIMIT = 600.0
T_FULL = 1112.0


def treq(hr, tau, m):
    va = math.sqrt(2 * G * (hr - ZA))
    vt = va + G * tau
    ht = ZA - va * tau - 0.5 * G * tau ** 2
    if ht <= 0:
        return math.inf, vt, ht
    return m * (G + vt ** 2 / (2 * ht)), vt, ht


def tau_max(hr, thrust, m):
    lo, hi = 0.0, 1.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if treq(hr, mid, m)[0] <= thrust:
            lo = mid
        else:
            hi = mid
    return lo


for m, etiket in ((M_WET, "arac (islak)"), (M_HIL, "HIL modeli")):
    print("=== %s: %.2f kg ===" % (etiket, m))
    for hr in (8.0, 10.0):
        va = math.sqrt(2 * G * (hr - ZA))
        print("  birakma %.0f m: 5 m'de hiz %.2f m/s" % (hr, va))
        for tau in (0.0, 0.1, 0.2, 0.3):
            T, vt, ht = treq(hr, tau, m)
            print("    tau %.1f s: kalan %.2f m, hiz %.2f m/s, Treq %s"
                  % (tau, ht, vt, "%.0f N" % T if math.isfinite(T) else "durdurulamaz"))
        print("    azami gecikme: %.3f s (600 N), %.3f s (1112 N)"
              % (tau_max(hr, T_LIMIT, m), tau_max(hr, T_FULL, m)))
    print()

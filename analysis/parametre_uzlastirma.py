# -*- coding: utf-8 -*-
"""
S-TEK Dikey Inisli Roket - IAC 2026 bildirisi
Master parametre uzlastirma ve denetim betigi.

Raporlar arasi celiskili degerleri (atalet, kutle, gaz kutlesi, impuls)
tek bir dogrulanabilir kaynaga indirger.  Ciktisi master_parametreler.md
tablosunu besler.

Calistir:  python parametre_uzlastirma.py
"""
import sys
import numpy as np
from scipy.optimize import brentq

sys.stdout.reconfigure(encoding="utf-8")

# ----------------------------------------------------------------------
# 1) Peng-Robinson ile tank gaz kutlesi   (B3)
# ----------------------------------------------------------------------
# Sabitler pnomatik detay hesaplari raporundaki degerlerle ayni tutuldu.
Mw = 0.028965          # kg/mol   kuru hava
Ru = 8.31446           # J/(mol K)
Tc = 132.53            # K
Pc = 37.86e5           # Pa
omega = 0.0335

a_pr = 0.45723553 * (Ru * Tc) ** 2 / Pc
b_pr = 0.077796074 * Ru * Tc / Pc
kappa = 0.37464 + 1.54226 * omega - 0.26992 * omega ** 2


def alpha_pr(T):
    return (1.0 + kappa * (1.0 - np.sqrt(T / Tc))) ** 2


def p_pr(T, vm):
    """Peng-Robinson basinci [Pa], vm = molar hacim [m3/mol]."""
    return Ru * T / (vm - b_pr) - a_pr * alpha_pr(T) / (vm ** 2 + 2 * b_pr * vm - b_pr ** 2)


def gaz_kutlesi(P, T, V):
    """Verilen P [Pa], T [K], V [m3] icin gercek gaz kutlesi [kg] ve Z."""
    vm_ig = Ru * T / P
    vm = brentq(lambda v: p_pr(T, v) - P, b_pr * 1.0001, vm_ig * 50.0, maxiter=200)
    rho = Mw / vm
    Z = P * vm / (Ru * T)
    return rho * V, rho, Z


V_TANK = 0.009         # m3  (9 L)
P_DOLUM = 300e5        # Pa
P_TEST = 180e5         # Pa  - AHR: tup yetkili personelce ancak 180 bar'a dolduruldu

print("=" * 68)
print("1) TANK GAZ KUTLESI  (Peng-Robinson gercek gaz)")
print("=" * 68)
for T in (293.15, 300.0):
    m, rho, Z = gaz_kutlesi(P_DOLUM, T, V_TANK)
    m_ig = P_DOLUM * V_TANK / ((Ru / Mw) * T)
    print(f"  300 bar, T={T:6.2f} K : m = {m:.3f} kg   rho = {rho:6.2f} kg/m3   Z = {Z:.4f}"
          f"   (ideal gaz {m_ig:.3f} kg)")
m180, _, _ = gaz_kutlesi(P_TEST, 293.15, V_TANK)
print(f"  180 bar, T=293.15 K : m = {m180:.3f} kg   <- AHR sizdirmazlik testi dolum seviyesi")
m_gaz, _, _ = gaz_kutlesi(P_DOLUM, 293.15, V_TANK)
print(f"\n  KTR-Ek benzetiminde kullanilan  : 5.000 kg")
print(f"  Gercek deger (300 bar/293.15 K) : {m_gaz:.3f} kg")
print(f"  NOT: benzetim gaz kutlesini %{100*(5.0/m_gaz-1):.0f} fazla almis")

# ----------------------------------------------------------------------
# 2) Toplam impuls butcesi ve gorev denetimi   (B4)
# ----------------------------------------------------------------------
print()
print("=" * 68)
print("2) IMPULS BUTCESI DENETIMI")
print("=" * 68)

XLSX = "../data/propulsion/thrust_time_tables.xlsx"

try:
    import openpyxl
    wb = openpyxl.load_workbook(XLSX, data_only=True)
    impulslar = {}
    for sh in ("Itki_Zaman_600N", "Itki_Zaman_400N", "Itki_Zaman_300N"):
        d = [(r[0], r[1]) for r in wb[sh].iter_rows(min_row=2, values_only=True)
             if r[0] is not None and r[1] is not None]
        I = sum((d[i + 1][0] - d[i][0]) * (d[i][1] + d[i + 1][1]) / 2 for i in range(len(d) - 1))
        impulslar[sh] = I
        print(f"  {sh:18s} : toplam impuls = {I:7.1f} N.s   (sure {d[-1][0]:.2f} s)")
    I_TOPLAM = float(np.mean(list(impulslar.values())))
except Exception as exc:                                    # tablo yoksa elle deger
    print(f"  [uyari] xlsx okunamadi ({exc}); I_TOPLAM elle giriliyor")
    I_TOPLAM = 1385.0

print(f"\n  KULLANILABILIR TOPLAM IMPULS  I_top = {I_TOPLAM:.0f} N.s")

g = 9.81
M_BUTCE = 25.939        # kg  - AHR kutle butcesi
M_HIL = 33.8            # kg  - HIL plant betigi

print(f"\n  Gorev profili denetimi (I_gerekli <= {I_TOPLAM:.0f} N.s olmali):")
gorevler = [
    ("Yumusak inis  (600 N x 1.42 s)",            600.0, 1.42),
    ("Hover 5 s     (331.6 N, m=33.8 kg)",        331.6, 5.00),
    ("Hover AHR HIL (331.6 N x ~25.4 s)",         331.6, 25.42),
    ("Hover 5 s     (254.5 N, m=25.939 kg)",  M_BUTCE * g, 5.00),
]
for ad, T, sure in gorevler:
    I = T * sure
    ok = "UYGUN" if I <= I_TOPLAM else f"ASILDI ({I/I_TOPLAM:.1f}x)"
    print(f"    {ad:42s} I = {I:7.0f} N.s   -> {ok}")

print(f"\n  Teorik azami hover suresi:")
for m in (M_BUTCE, M_HIL):
    T_denge = m * g
    print(f"    m = {m:6.3f} kg -> denge itkisi {T_denge:6.1f} N -> "
          f"t_max = I_top/T = {I_TOPLAM/T_denge:.2f} s")

# ----------------------------------------------------------------------
# 3) Atalet tensoru makullugu   (B1)
# ----------------------------------------------------------------------
print()
print("=" * 68)
print("3) ATALET TENSORU MAKULLUK KONTROLU")
print("=" * 68)
L_BOY, D_DIS = 1.18, 1.18          # m  (AHR s.4-5: 118 cm x 118 cm)

def jirasyon(I, m):
    return np.sqrt(I / m)

print(f"  Arac gabarisi: boy {L_BOY} m, genislik {D_DIS} m, kutle {M_BUTCE} kg")
print(f"  Fiziksel ust sinir: hicbir kutle elemani AM'den {D_DIS/2:.2f} m'den uzak olamaz")
print(f"  -> jirasyon yaricapi k <= {D_DIS/2:.2f} m, yani I <= {M_BUTCE*(D_DIS/2)**2:.2f} kg.m2\n")

for kaynak, I_vec in [("OTR / KTR s.3", [29.3, 8.4, 29.1]),
                      ("AHR s.3",       [1.913, 1.995, 0.845])]:
    ks = [jirasyon(I, M_BUTCE) for I in I_vec]
    verdict = "FIZIKSEL OLARAK IMKANSIZ" if max(ks) > D_DIS / 2 else "makul"
    print(f"  {kaynak:14s} I = {I_vec}  ->  k = "
          f"[{ks[0]:.3f}, {ks[1]:.3f}, {ks[2]:.3f}] m   {verdict}")
print(f"\n  HIL plant betigi ATALET_KGM2 = 1.995  -> AHR degerleriyle tutarli")

# ----------------------------------------------------------------------
# 4) Uzlastirilmis master parametreler
# ----------------------------------------------------------------------
print()
print("=" * 68)
print("4) UZLASTIRILMIS MASTER DEGERLER")
print("=" * 68)
M_KURU = M_BUTCE - 0.0
M_ISLAK = M_BUTCE + m_gaz
print(f"  Kuru kutle (gaz haric, butceye gore)   : {M_KURU:.3f} kg")
print(f"  Gaz kutlesi (300 bar / 293.15 K)       : {m_gaz:.3f} kg")
print(f"  Kalkis (islak) kutlesi                 : {M_ISLAK:.3f} kg")
print(f"  Denge (hover) itkisi @ islak kutle     : {M_ISLAK*g:.1f} N")
print(f"  Atalet tensoru (AHR/CAD)               : Ixx=1.913  Iyy=1.995  Izz=0.845 kg.m2")
print(f"  Kullanilabilir toplam impuls           : {I_TOPLAM:.0f} N.s")
print(f"  Tasarim itkisi / azami itki            : 600 N / 1090 N (vana 90 deg)")
print(f"  Lule: Dt=5.66 mm  De=19.13 mm  eps=11.41  Me=4.07  Ve=672.61 m/s")
print()
print("  NOT: Kutle butcesindeki 25.939 kg 'Tank' satiri 5200 g ile tup DONANIMINI")
print("       icerir; gaz kutlesi butceye hic girmemis.  HIL betigindeki 33.8 kg")
print("       ise hicbir belgeyle eslesmiyor - takimca dogrulanmali.")

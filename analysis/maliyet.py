# -*- coding: utf-8 -*-
"""Arac donanim maliyeti — IAC 2026 bildirisi Bolum 6.4 icin.

Kaynak onceligi:
  1) 'S-TEK Dikey Genel Malzeme Listesi 2 temmuz ardakaan asya v2.xlsx'
     (AHR konfigurasyonuna uyan guncel liste; sayfa toplamlari esas alinir)
  2) KTR maliyet tablosu — yalnizca v2'de yeniden listelenmemis AMA hala
     aracta bulunan kalemler (hat donanimi, pistonlar, karbon kumas, kelepce)

Haric tutulanlar:
  - Basinc tupu: yarisma organizatoru (TEKNOFEST) tarafindan saglandi
  - KTR'deki V-port vana ve motor: AHR'de igne vana + NEMA 34 ile degistirildi
  - Prototipleme sarf malzemeleri (SLA/ABS recine, izopropil alkol)
  - Birebir tekrar eden haç rakor (v2 fiyati kullanilir)

Doviz: v2 listesinin kendi toplamindan cikan kur (360 USD motor kalemi).
Bekleyen: kaucuk ayak ve PCB uretim maliyeti; toplam fiyatlandirilmis donanim ara toplamidir.
Calistir: python maliyet.py
"""
import sys

sys.stdout.reconfigure(encoding="utf-8")

# ---- 1) v2 listesi: sayfa toplamlari (TL) ----------------------------
V2 = {
    "Avionics": 44298.21,          # 16 kalem, CS-D808 vana surucusu dahil
    # igne vana kesinlesti: 10.500 TL (v2 listesindeki "30000 ?" tahmini gecersiz)
    "Propulsion": 48307.19 - 30000 + 10500,
    "Structure": 57332.26,         # karbon borular, epoksi, rulman, gijon, kaucuk, aluminyum
}
# Kur, v2 listesinin KENDI (duzeltilmemis) pnomatik toplamindan geri hesaplanir:
# sayfa toplami 48.307,19 = 30.000 + 948 + 600 + 360 USD x kur
V2_PROP_SHEET_TOTAL = 48307.19
V2_PROP_TL_ITEMS = 30000 + 948 + 600
USD_MOTOR = 360.0
RATE_TL_PER_USD = (V2_PROP_SHEET_TOTAL - V2_PROP_TL_ITEMS) / USD_MOTOR

# ---- 2) KTR'den aktarilan, aracta hala bulunan kalemler (TL) ----------
KTR_CARRY = {
    "Propulsion": {
        "Ball valve, manual vent (2)": 6000.00,
        "High-pressure hose": 1500.00,
        "Fittings": 1000.00,
        "Pressure/temperature sensor": 3464.81,
    },
    "Structure": {
        "Landing-gear dampers (pistons)": 2100.00,
        "Carbon-fibre fabric (bulkheads)": 23750.00,
        "Clamps": 1748.00,
    },
}

# ---- 3) bekleyen kalemler ---------------------------------------------
# Baris'tan kesinlesen ek kalemler (TL)
CONFIRMED_EXTRA = {
    "Propulsion": {
        "Main nozzle (custom manufacture)": 1000.00,
        "Ball-valve actuator (TVC-type servo)": 1637.42,
    },
}
PENDING = {"Rubber-foot cost confirmation": None, "PCB manufacture": None}

rows = []
total = 0.0
for sub in ("Avionics", "Propulsion", "Structure"):
    tl = (V2[sub] + sum(KTR_CARRY.get(sub, {}).values())
          + sum(CONFIRMED_EXTRA.get(sub, {}).values()))
    rows.append((sub, tl))
    total += tl

print("Ima edilen kur (v2 listesi): %.2f TL/USD" % RATE_TL_PER_USD)
print()
print("%-12s %14s %10s" % ("Alt sistem", "TL", "USD"))
for sub, tl in rows:
    print("%-12s %14s %10s" % (sub, "{:,.0f}".format(tl), "{:,.0f}".format(tl / RATE_TL_PER_USD)))
print("%-12s %14s %10s" % ("TOPLAM", "{:,.0f}".format(total), "{:,.0f}".format(total / RATE_TL_PER_USD)))
print()
print("Bekleyen:", ", ".join(PENDING) or "yok")
for sub, tl in rows:
    print("  %-12s pay %%%.1f" % (sub, 100*tl/total))
print("  karbon boru payi %%%.1f" % (100*45490.46/total))

# manuscript icin disari aktar
RESULT = {
    "rate": RATE_TL_PER_USD,
    "rows": rows,
    "total_tl": total,
    "total_usd": total / RATE_TL_PER_USD,
}

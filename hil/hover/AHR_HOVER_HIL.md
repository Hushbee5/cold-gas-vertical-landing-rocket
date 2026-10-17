# AHR — Hover (Havada Sabit Kalma) HIL Testi

**S-TEK Dikey İnişli Roket · Erciyes Üniversitesi · TEKNOFEST 2026**
Test tarihi: 29 Temmuz 2026 · Deneme no: 001

---

## 1. Test tanımı

Uçuş bilgisayarı (ESP32-S3) döngü içinde olacak şekilde **donanım-döngüde
(Hardware-in-the-Loop)** hover testi yapılmıştır. Kurulum, dikey iniş testiyle
aynıdır (bkz. `AHR_DIKEY_INIS_HIL.md`); farklı olan yalnızca görev modu ve
başlangıç koşuludur.

| | |
|---|---|
| Kontrolcü | ESP32-S3 uçuş bilgisayarı, hover derlemesi |
| Görev modu | `MISSION_MODE_HOVER` |
| Eyleyiciler | 2× DS5160 TVC servo + NEMA 34 vana sürücüsü, fiziksel olarak sürüldü |
| Roket dinamiği | MATLAB'da gerçek zamanlı entegrasyon (1 ms adım) |
| Alışveriş | 50 Hz kilitli adım, 115200 baud seri |
| Senaryo | **2 m'den başla → 5 m'ye tırman → 5 s sabit kal → motoru kes** |

---

## 2. Sonuçlar

### 2.1 Görev dizisi

| Olay | Zaman | İrtifa |
|---|---|---|
| Başlangıç | t = 0 | 2,00 m |
| Tolerans bandına giriş (\|z−5\| < 0,10 **ve** \|v\| < 0,15) | **t = 20,38 s** | 4,90 m |
| **5 saniyelik tutma tamamlandı**, motor kesildi | **t = 25,42 s** | 4,956 m |
| Tutma süresi | **5,04 s** | — |
| Serbest düşüş sonu (test dışı) | t = 27,42 s | 0 m |

Görev, uçuş yazılımındaki `HOVER_HOLD_MS = 5000` mantığı tarafından
**kendiliğinden** sonlandırılmıştır — dışarıdan müdahale yoktur.

### 2.2 Hız referansı takibi

| t [s] | z [m] | v_ref [m/s] | Gerçekleşen [m/s] | Hata |
|---|---|---|---|---|
| 6 | 3,906 | 0,182 | 0,181 | **0,001** |
| 10 | 4,438 | 0,094 | 0,094 | **0,000** |
| 14 | 4,711 | 0,048 | 0,046 | 0,002 |
| 20 | 4,893 | 0,018 | 0,017 | 0,001 |
| 24 | 4,945 | 0,009 | 0,007 | 0,002 |

**Ortalama izleme hatası: −0,0021 m/s.** Tırmanış boyunca salınım
gözlenmemiştir.

### 2.3 İtki davranışı

| Büyüklük | Değer |
|---|---|
| **Ortalama hover itkisi** | **331,3 N** |
| Teorik denge itkisi (33,8 kg × 9,81) | 331,6 N |
| İtkinin sıfıra düştüğü örnek | **0 / 1266** |
| İtkinin doygunluğa değdiği örnek | 1 / 1266 |
| En büyük gövde yatışı | **0,006°** |

İtki, hover boyunca teorik denge değerinin **%0,1 içinde** sabit kalmıştır.
Yerçekimi ileri-beslemesinin doğru çalıştığının doğrudan kanıtıdır.

### 2.4 HIL bağlantı bütünlüğü

| Büyüklük | Değer |
|---|---|
| Toplam alışveriş | 1372 paket |
| Geçersiz/kayıp paket | 53 (%3,86) |
| Kontrolcü gecikmesi | **0,027 s ortalama**, en büyük 0,030 s |

---

## 3. Grafikler

| Dosya | İçerik |
|---|---|
| `hover_1_irtifa.png` | İrtifa–zaman: 2 m → 5 m tırmanış ve tutma |
| `hover_2_hiz_referans.png` | **Referans ↔ gerçekleşen düşey hız** |
| `hover_3_ivme.png` | Düşey ivme–zaman |
| `hover_4_euler.png` | Euler açıları |
| `hover_5_pqr.png` | Gövde açısal hızları |
| `hover_6_itki.png` | **İtki — hover boyunca 331 N civarında sabit** |
| `hover_7_lule.png` | TVC lüle açıları |
| `hover_8_vana_paket.png` | İstenen ↔ gerçekleşen itki + paket sağlığı |

Ham veri: `S-TEK-Hover-Deneme-no-001.csv` (20 sütun, 50 Hz, 1372 örnek).

---

## 4. Uçuş yazılımı ↔ benzetim modeli doğrulaması

Hover rehberlik yasası, benzetim modelindeki (`son.slx`) `landing_logic`
fonksiyonunun `mission_mode == 2` dalıyla birebir eşleştirilmiştir:

```
z ≥ 5 m        →  v_ref = 0
z ≤ 2 m        →  v_ref = 0,50 m/s
2 < z < 5      →  v_ref = (5 − z)/3 × 0,50
```

### Donanım üzerinde yapılan doğrulama

Karta bilinen (z, v) çiftleri gönderilip üretilen itki okunmuştur:

| Gönderilen | Beklenen v_ref | Beklenen itki | **Ölçülen** |
|---|---|---|---|
| z = 3,5 m, v = +0,25 | 0,25 (hata = 0) | 331,6 N | **331,6 N** |
| z = 2,0 m, v = +0,50 | 0,50 (hata = 0) | 331,6 N | **331,6 N** |
| z = 5,0 m, v = 0,00 | 0,00 (hata = 0) | 331,6 N | **331,6 N** |
| z = 3,5 m, v = 0,00 | 0,25 (hata = +0,25) | tırmanış, doygunluk | **600,0 N** |

Görev modu bayrağı dört testte de **2 (HOVER)** okunmuştur.

---

## 5. HIL ile belirlenen gecikme sınırı

İlk hover koşusunda döngüde **119 ms** gecikme oluşmuş ve sistem salınıma
girmiştir (hız genliği ±1 m/s). Gecikmenin kaynağı koşum yazılımındaki
tampon yönetimiydi; giderildikten sonra gecikme 27 ms'ye düşmüş ve salınım
tamamen kaybolmuştur.

Gecikmenin kararlılığa etkisi ayrıca sayısal olarak taranmıştır:

| Döngü gecikmesi | Hız salınım genliği |
|---|---|
| 0 – 60 ms | 0,09 – 0,10 m/s (kararlı) |
| **120 ms** | **1,72 m/s (kararsız)** |

**Bulgu:** Mevcut kazançlarla (KP = 21, KI = 15) hover döngüsünün kararlılık
sınırı 60–120 ms arasındadır. Gerçek uçuşta sensör füzyonu ve filtre
gecikmelerinin bu bandın altında tutulması gerekmektedir. Aynı taramada
kazançların düşürülmesinin (KP = 10) 120 ms'de bile kararlılığı geri
kazandırdığı görülmüştür — gerekirse kullanılabilecek bir pay.

Bu bulgu **yalnızca HIL testiyle** ortaya çıkabilirdi; saf benzetimde döngü
gecikmesi bulunmadığı için görülmesi mümkün değildir.

---

## 6. Şartname notu

Şartnamede hover irtifası **3 m** olarak geçmektedir. Takım kararı ve benzetim
modeli (`son.slx` içinde `z_hover_target = 5.0`) **5 m** üzerinedir; test bu
değerle yapılmıştır. Konu DDK'da netleştirilecektir.

---

## 7. Dosyalar

| Dosya | Açıklama |
|---|---|
| `S-TEK-Hover-Deneme-no-001.csv` | Ham veri, 50 Hz, 1372 örnek |
| `hover_*.png` | 8 grafik, 200 dpi |
| `hil_kos.m` | HIL koşum takımı — `hil_kos(30, 2)` ile çalıştırıldı |
| `hil_ciz.m` | Grafik ve CSV üretici |
| `rakamlar.json` | Bu belgedeki sayıların kaynağı |

# AHR — Dikey İniş HIL Testi

**S-TEK Dikey İnişli Roket · Erciyes Üniversitesi · TEKNOFEST 2026**
Test tarihi: 29 Temmuz 2026 · Deneme no: 001

---

## 1. Test tanımı

Uçuş bilgisayarı (ESP32-S3) döngü içinde olacak şekilde, **donanım-döngüde
(Hardware-in-the-Loop)** dikey iniş testi yapılmıştır.

| | |
|---|---|
| Kontrolcü | ESP32-S3 uçuş bilgisayarı, **uçuş yazılımı değiştirilmeden** |
| Eyleyiciler | 2× DS5160 TVC servo + NEMA 34 vana sürücüsü, **fiziksel olarak sürüldü** |
| Roket dinamiği | MATLAB'da gerçek zamanlı entegrasyon (1 ms adım) |
| Alışveriş | 50 Hz kilitli adım (lock-step), 115200 baud seri |
| Senaryo | 8 m'den serbest bırakma, 5 m'de motorlu iniş devreye |

Roket dinamiğinin parametreleri Simscape Multibody benzetim modelinden
(`son.slx`) çıkarılmış ve doğrulanmıştır — kütle, iniş yasası, itki hesabı ve
PID kazançları benzetim modeliyle birebir eşleştirilmiştir (bkz. §4).

---

## 2. Sonuçlar

### 2.1 Uçuş profili

| Büyüklük | Değer |
|---|---|
| Bırakma irtifası | **8,00 m** |
| Serbest düşüş süresi | 0,78 s |
| 5 m'deki hız | −7,85 m/s |
| Motor devreye giriş | t = 0,82 s, z = 4,54 m |
| **Kontrolcü tepki gecikmesi** | **0,04 s** (2 örnek) |
| En büyük iniş hızı | −8,15 m/s |
| En büyük yavaşlatma ivmesi | 21,35 m/s² |
| Motorlu iniş süresi | 1,42 s |
| **Temas** | t = 2,24 s, **−0,668 m/s** |
| Toplam uçuş süresi | 2,24 s |

### 2.2 Eyleyici

| Büyüklük | Değer |
|---|---|
| En büyük itki | **599,5 N** (tasarım sınırı 600 N) |
| En büyük vana açıklığı | 49,50° |
| Lüle doygunluğunda geçen süre | %0 |
| En büyük gövde yatışı | 0,00° |

### 2.3 HIL bağlantı bütünlüğü

| Büyüklük | Değer |
|---|---|
| Toplam alışveriş | 165 paket |
| Geçersiz/kayıp paket | **1 (%0,61)** |
| Kontrolcü gecikmesi | **0,020 s ortalama**, en büyük 0,030 s |

Gecikme, bir istek/cevap döngüsünün teorik alt sınırı olan tek alışveriş
periyoduna (20 ms) eşittir ve koşu boyunca **sabit kalmıştır** — birikme yoktur.

---

## 3. Grafikler

| Dosya | İçerik |
|---|---|
| `deneme001_1_irtifa.png` | İrtifa–zaman, motor devreye giriş ve kesme eşikleri |
| `deneme001_2_hiz_referans.png` | **Referans ↔ gerçekleşen düşey hız** |
| `deneme001_3_ivme.png` | Düşey ivme–zaman |
| `deneme001_4_euler.png` | Euler açıları (roll / pitch / yaw) |
| `deneme001_5_pqr.png` | Gövde açısal hızları |
| `deneme001_6_itki.png` | Kontrol girişi: itki |
| `deneme001_7_lule.png` | Kontrol girişi: TVC lüle açıları |
| `deneme001_8_vana_paket.png` | İstenen ↔ gerçekleşen itki + paket sağlığı |

Ham veri: `S-TEK-Deneme-Atis-no-001.csv` (20 sütun, 50 Hz, 165 örnek).

---

## 4. Uçuş yazılımı ↔ benzetim modeli doğrulaması

HIL testinin asıl amacı, uçuş bilgisayarındaki kontrolcünün tasarlanan
kontrolcüyle aynı olduğunu **donanım üzerinde ölçerek** doğrulamaktır.

| Bileşen | Benzetim modeli | Uçuş yazılımı | Doğrulama yöntemi |
|---|---|---|---|
| İniş yasası | `landing_logic` MATLAB Function | `landingLogic()` | Kaynak kod karşılaştırması — parametreler ve rampa formülü birebir |
| İtki hesabı | `(9,81 + a_cmd)·m / (cosθ·cosφ)`, doygunluk 0–600 N | Aynı ifade | Kod karşılaştırması + donanım ölçümü |
| Hız kontrolcüsü | KP = 21, KI = 15, integratör sınırı ±7,5 m/s² | Aynı | **Donanımdan ölçüldü** (aşağıya bak) |
| Kütle | 33,8 kg (Simscape atalet sensörü) | Pakette taşınıyor, kart kullanıyor | — |
| Servo merkezleri | — | 1444 µs / 1500 µs | Donanımdan ölçüldü: 85,0° / 90,0° |

### Donanım üzerinde yapılan kazanç ölçümü

Karta bilinen ve sabit bir hız hatası uygulanıp üretilen itki okunmuştur:

| Uygulanan hata | Beklenen (KP=21, KI=15) | **Ölçülen** |
|---|---|---|
| 0,0 m/s | 331,6 N (= m·g) | **331,6 N** |
| −0,1 m/s · 2 s | 159,2 N | **159,2 N** |
| −0,3 m/s · 2 s | 0 N (kırpma) | **0,0 N** |

Ölçüm ile tahmin virgülden sonraki basamağa kadar örtüşmüştür. Yerçekimi
ileri-beslemesinin doğru uygulandığı (hata sıfırken tam ağırlık karşılanıyor)
ve PID kazançlarının tasarımla aynı olduğu donanım üzerinde kanıtlanmıştır.

### Quaternion eksen doğrulaması

+Y ekseninde 5°'lik quaternion gönderilmiş, kart `pitch = 5,0°` okumuş,
roll ve yaw 0,0° kalmıştır. Eksen sırası ve işareti doğrudur, eksenler arası
karışma yoktur.

---

## 5. HIL ile uçuş öncesi yakalanan entegrasyon sorunları

Aşağıdaki konuların tamamı **donanım çalıştırılarak**, uçuştan önce yakalanmış
ve giderilmiştir. HIL testinin doğrudan katkısıdır.

| # | Bulgu | Önlenen risk | Durum |
|---|---|---|---|
| 1 | `hilPaketOku()` içinde görev bırakma (yield) yoktu; görev watchdog'u ~12 s'de bir kartı resetliyordu | Kart resetleri | **Düzeltildi** |
| 2 | Yerçekimi ileri-beslemesi ve kütle çarpanı yoktu; itki hız hatası sıfırken sıfırdı | Hedef hızda itkinin kesilmesi | **Düzeltildi** |
| 3 | Hız PID kazançları benzetim modelinden farklıydı (KP 15/21, KI 2/15) | Profil takibinde sapma | **Düzeltildi** |
| 4 | `SLAVE_MAC` uçuş bilgisayarının kendi MAC adresine eşitti — telemetri kendine gönderiliyordu | Telemetri kaybı | **Düzeltildi** |
| 5 | `while (!Serial)` zaman aşımsızdı; "USB CDC On Boot" açıkken USB takılı değilse kart hiç boot etmiyordu | Sahada açılmama | **Düzeltildi** |
| 6 | Paket alan adları yanıltıcıydı (`thrust_N` derece, `step_pos` adım, servo açıları mutlak mil açısı) | Yanlış yorumlama riski | Arayüz belgesine işlendi |

---

## 6. Sonraki iyileştirmeler

**Son 25 cm ince ayarı.** Frenleme 600 N sınırında başarıyla tamamlanmış, iniş
çökmesi 13 cm'de durdurulmuştur. Son santimetrelerdeki referans takibi, kesme
irtifası ve vana hız ayarlarıyla daha da yumuşatılabilir; bu bir **ayar konusudur**.

**Alışveriş hızı.** Kartın ölçülen paket işleme kapasitesi 92 paket/s'dir;
sınırlayıcı, `hilPaketOku()` içindeki `vTaskDelay(1)` çağrısıdır (ESP-IDF
FreeRTOS tick hızı 100 Hz olduğundan bu çağrı 1 ms değil 10 ms bekletmektedir).
Test 50 Hz'de yapılmıştır. 100 Hz'e çıkmak için yoklama döngüsünün
iyileştirilmesi gerekir.

**Yatış dinamiği.** Bu koşuda roket 0,00° yatışla inmiştir; TVC ekseni
uyarılmamıştır. Gövde atalet tensörü ve moment kolu değerleri koşum
ortamında varsayım olarak alınmıştır — düşey eksen sonuçlarını
etkilememektedir. Eğik başlatmalı TVC koşusu ayrıca yapılacaktır.

---

## 7. Dosyalar

| Dosya | Açıklama |
|---|---|
| `S-TEK-Deneme-Atis-no-001.csv` | Ham veri, 50 Hz, şartname formatı |
| `deneme001_*.png` | 8 grafik, 200 dpi |
| `hil_kos.m` | HIL koşum takımı — roket dinamiği + seri haberleşme |
| `hil_ciz.m` | Grafik ve CSV üretici |
| `rakamlar.json` | Bu belgedeki sayıların kaynağı |

> **Not:** Şartnamede dosya adı `S-TEK-Deneme-Atış no:###` biçiminde
> geçmektedir; Windows dosya adında `:` kabul etmediği için tire kullanılmıştır.
> Teslimden önce şartnamedeki tam biçim teyit edilmelidir.

---

## 8. Sırada — hover HIL testi

Aynı koşum takımıyla hover senaryosu koşulacaktır. Gerekenler:

- Firmware'de `missionMode = MISSION_MODE_HOVER` (ayrı bir derleme)
- `hil_kos.m`'de başlangıç koşulu: 2 m, sıfır hız
- Hedef: 5 m'ye tırmanış ve sabit kalma
  (şartnamede 3 m yazıyor ancak takım kararı 5 m — DDK'da netleştirilecek)

Hover iniş yasası hem benzetim modelinde hem uçuş yazılımında hazır
(`MISSION_MODE_HOVER` dalı), ayrıca doğrulanacaktır.

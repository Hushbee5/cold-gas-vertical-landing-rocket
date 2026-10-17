/*
 * ============================================================
 *  Dikey İniş Roketi — TAM YORUMLU SÜRÜM (v32 — HOVER TEST + HIL PATCH)
 *  v31 HOVER PATCH'in birebir aynısı; üzerine HIL (Hardware-in-the-Loop)
 *  modu eklendi. Eklenen HER YER "// >>> HIL PATCH" / "// <<< HIL PATCH"
 *  ile işaretlendi (HOVER PATCH işaretleri de korunmuştur).
 *
 *  TASARIM İLKESİ: kontrol zincirinde (landingLogic -> velocityPID ->
 *  computeAngleCorrectedThrust -> computeValveTargetDegFromThrust ->
 *  updateValve, runTVC) VE hover tetik/abort/hold-timer mantığında
 *  TEK BİR SATIR bile değişmiyor. Değişen yalnızca zincire giren veri:
 *  gerçek sensörler yerine Simulink'in gönderdiği hil_sensor_t paketi.
 *
 *  HIL_MODE=1 iken:
 *    • BNO055/BMP390/TFmini init edilmez, Kalman predict/update ATLANIR
 *      (Simulink zaten füzyonlanmış z/vz veriyor).
 *    • sensorTaskFunc() döngüsü artık sabit 100 Hz timer değil,
 *      Simulink paketine KİLİTLİ (lock-step): hilPaketOku() bloklayıcı,
 *      her yeni paket bir kontrol adımı tetikler. dt, paketteki
 *      simülasyon zamanından (hs.t) hesaplanır.
 *    • Quaternion (q[w,x,y,z]) -> Euler çevrimi quatToEuler() ile
 *      yapılır, BNO055'in getVector(VECTOR_EULER) çıktısıyla AYNI
 *      eksen sırasına (x=yaw, y=roll, z=pitch) oturtulur.
 *    • Servo/vana çıktısı hem GERÇEK donanıma yazılır (updateValve,
 *      safeWriteServo — değişmedi) hem de hil_cmd_t olarak Simulink'e
 *      geri yollanır.
 *    • Hover tetik/abort komutları HÂLÂ ESP-NOW üzerinden (yer
 *      istasyonundan) geliyor — HIL_SERIAL'dan BAĞIMSIZ, değişmedi.
 *
 *  !! HIL_SERIAL = Serial0 (UART0, kartın COM jakındaki CH343 köprüsü).
 *     Arduino IDE'de Tools -> USB CDC On Boot -> Enabled ZORUNLU. !!
 * ============================================================
 */

// ─── KÜTÜPHANELER ──────────────────────────────────────────────
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <BasicLinearAlgebra.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <SPI.h>
#include <SD.h>

// >>> HIL PATCH: S-TEK HIL Protokolü (tek dosya için doğrudan gömülü,
//     ayrı .h dosyasına gerek yok)
#include <stdint.h>
#define HIL_HDR1  0xAA
#define HIL_HDR2  0x55
#define HIL_SENSOR_LEN  48
#define HIL_CMD_LEN     24
#pragma pack(push, 1)
typedef struct {
    uint8_t hdr1;      /* 0xAA */
    uint8_t hdr2;      /* 0x55 */
    uint8_t seq;       /* sira no */
    float   t;         /* [s]     simulasyon zamani      */
    float   q[4];      /* [-]     quaternion  w,x,y,z    */
    float   w[3];      /* [rad/s] jiro  p,q,r            */
    float   z;         /* [m]     irtifa                 */
    float   vz;        /* [m/s]   dusey hiz (+ yukari)   */
    float   mass;      /* [kg]    anlik kutle            */
    uint8_t chk;       /* XOR                            */
} hil_sensor_t;        /* 48 bayt */
#pragma pack(pop)
#pragma pack(push, 1)
typedef struct {
    uint8_t hdr1;         /* 0xAA */
    uint8_t hdr2;         /* 0x55 */
    uint8_t seq;          /* cevaplanan sensor paketinin seq'i */
    float   thrust_N;     /* [N]      hesaplanan itki          */
    float   step_pos;     /* [adim]   valfe giden adim pozisyonu */
    float   tvc_pitch;    /* [derece] TVC pitch komutu         */
    float   tvc_yaw;      /* [derece] TVC yaw komutu           */
    float   status;       /* [-] 0=off 1=landing 2=hover (+hata) */
    uint8_t chk;          /* XOR */
} hil_cmd_t;              /* 24 bayt */
#pragma pack(pop)
static inline uint8_t hil_checksum(const uint8_t *buf, int len)
{
    uint8_t c = 0;
    for (int i = 2; i < len - 1; i++) c ^= buf[i];
    return c;
}

// HIL modu açma/kapama:
//   1 = Simulink'ten simüle sensör verisiyle çalış
//   0 = gerçek uçuş kodu (orijinal v31 HOVER davranışı)
#define HIL_MODE   1

#if HIL_MODE
  #define HIL_SERIAL        Serial0   // UART0 = kartın COM jakındaki CH343 köprü çipi
  #define HIL_SERIAL_BAUD   115200
  #define HIL_TIMEOUT_MS    200    // bu sürede yeni paket gelmezse adımı atla
#endif
// <<< HIL PATCH

using namespace BLA;  // BLA::Matrix<> yerine sadece Matrix<> yazabilmek için (kısaltma)

// ════════════════════════════════════════════════════════════
//  SLAVE (YER İSTASYONU) MAC ADRESİ
// ════════════════════════════════════════════════════════════
uint8_t SLAVE_MAC[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // set to your ground-station ESP32 MAC address

// ─── DONANIM SABİTLERİ — Uçuş Bilgisayarı ─────────────────────
#define BNO055_I2C_ADDR   0x28
#define BMP390_I2C_ADDR   0x77
#define LIDAR_SERIAL      Serial1
#define LIDAR_RX_PIN      18
#define LIDAR_TX_PIN      17
#define LIDAR_BAUD        115200

#define LOOP_HZ           100
#define DT                (1.0f / LOOP_HZ)

#define TELEMETRY_SEND_DIVIDER   2
#define WIFI_CHANNEL      1
#define I2C_SDA_PIN       8
#define I2C_SCL_PIN       9
#define I2C_CLOCK_HZ      400000

// ─── SD KART AYARLARI (SPI) ────────────────────────────────────
#define SD_CS_PIN     10
#define SD_SCK_PIN    12
#define SD_MISO_PIN   13
#define SD_MOSI_PIN   11
#define SD_FLUSH_EVERY_N_WRITES   10

// ─── EĞİM GÜVENLİK EŞİĞİ ────────────────────────────────────────
#define TILT_LIMIT_DEG   45.0f
#define TILT_LIMIT_RAD   (TILT_LIMIT_DEG * DEG_TO_RAD)

// ─── LiDAR AYKIRI-DEĞER (OUTLIER) REDDİ ────────────────────────
#define LIDAR_JUMP_LIMIT_M   0.5f
#define LIDAR_OUTLIER_MAX_CONSECUTIVE   5

// ─── BAROMETRE ZEMİN KALİBRASYONU ──────────────────────────────
#define BARO_CAL_SKIP_SAMPLES   3
#define BARO_CAL_AVG_SAMPLES    20

// ─── KOMUT TİPLERİ (ESP-NOW ÜZERİNDEN GELEN) ───────────────────
#define CMD_DEPLOY        0x01
#define CMD_DEPLOY_STOP   0x02

// >>> HOVER PATCH: uzaktan hover tetiği için yeni komut tipleri
#define CMD_HOVER_START    0x03
#define CMD_HOVER_ABORT    0x04
// <<< HOVER PATCH

// ════════════════════════════════════════════════════════════
//  TVC SERVO AYARLARI (Pitch/Roll gimbal)
// ════════════════════════════════════════════════════════════
#define PITCH_SERVO_PIN    47
#define ROLL_SERVO_PIN     14

#define SAFE_MIN_PULSE_US     600
#define SAFE_MAX_PULSE_US     2400
#define US_PER_DEGREE         11.11

#define LEDC_FREQ_HZ              50
#define LEDC_RESOLUTION_BITS     12
#define LEDC_PERIOD_US            20000.0
#define LEDC_MAX_DUTY             ((1 << LEDC_RESOLUTION_BITS) - 1)

float PITCH_CENTER_PULSE_US = 1444.0;
float ROLL_CENTER_PULSE_US  = 1500.0;

float PITCH_GAIN = 1.0;
float ROLL_GAIN  = 1.0;

int PITCH_DIR = -1;
int ROLL_DIR  = +1;

#define MAX_PITCH_GIMBAL_ANGLE   15.0
#define MAX_ROLL_GIMBAL_ANGLE    10.0
#define PID_INTEGRAL_LIMIT       10.0

float PITCH_KP = 2.0;
float PITCH_KI = 0.0;
float PITCH_KD = 0.5;

float ROLL_KP  = 1.2;
float ROLL_KI  = 0.0;
float ROLL_KD  = 0.2;

#define PITCH_DERIVATIVE_FILTER_ALPHA   0.15
#define ROLL_DERIVATIVE_FILTER_ALPHA    0.08

#define PITCH_ANGLE_FILTER_ALPHA        0.2
#define ROLL_ANGLE_FILTER_ALPHA         0.1

#define PITCH_DEADBAND_DEG   2.0
#define ROLL_DEADBAND_DEG    2.0

#define MAX_PITCH_SLEW_DEG_PER_LOOP   1.5
#define MAX_ROLL_SLEW_DEG_PER_LOOP    1.5

// ════════════════════════════════════════════════════════════
//  İTKİ VANASI — NEMA34 + CWDS86C (STEP/DIR sürücü)
// ════════════════════════════════════════════════════════════
#define VALVE_STEP_PIN   4
#define VALVE_DIR_PIN    5

#define VALVE_STEPS_PER_REV     800
#define VALVE_STEPS_PER_DEGREE  (VALVE_STEPS_PER_REV / 360.0)

#define VALVE_CLOSED_DEG        0.0
#define VALVE_OPEN_DEG           90.0

#define VALVE_STEP_PULSE_US        5
#define MAX_VALVE_STEPS_PER_LOOP   30

// ════════════════════════════════════════════════════════════
//  MOTORLU İNİŞ REHBERLİĞİ (LANDING GUIDANCE) AYARLARI
// ════════════════════════════════════════════════════════════
#define LANDING_Z_ACTIVATION_M   5.0f
#define LANDING_Z_CUTOFF_M       0.9245f

// >>> HOVER PATCH: hover hedef irtifası 3m'den 5m'ye güncellendi
#define LANDING_Z_HOVER_M        5.0f
// <<< HOVER PATCH
#define LANDING_KZ_HOVER         0.5f
#define LANDING_V_HOVER_UP_MAX    0.60f
#define LANDING_V_HOVER_DOWN_MAX -0.60f
#define LANDING_HOVER_DEADBAND    0.03f

// >>> HOVER PATCH v2: landingLogic() hover dalı için yeni, doğrusal
//     tırmanma profili (eski LANDING_KZ_HOVER'lı P-kontrolcü 4m'de
//     benzetim modelinin 3 katı hızla tırmanıyordu)
#define HOVER_Z_TARGET   5.0f
#define HOVER_Z_START    2.0f
#define HOVER_V_CLIMB    0.50f
// <<< HOVER PATCH v2

#define LANDING_Z_FLARE_START     1.50f
#define LANDING_Z_TOUCH_ZONE      0.15f
#define LANDING_V_LAND_FAST      -1.20f
#define LANDING_V_LAND_TOUCH     -0.15f

#define LANDING_VZ_CUTOFF_MIN    -0.20f
#define LANDING_VZ_CUTOFF_MAX     0.10f

#define MISSION_MODE_OFF      0
#define MISSION_MODE_LANDING  1
#define MISSION_MODE_HOVER    2

// >>> HOVER PATCH: missionMode runtime'da tetikle değişiyor — Core1
//     (ESP-NOW) yazıyor, Core0 (sensorTaskFunc) okuyor, volatile.
volatile int missionMode = MISSION_MODE_OFF;
// <<< HOVER PATCH

// >>> HOVER PATCH: hover görev tetiği + 5 saniyelik hold-timer
static volatile bool hoverTriggerReceived = false;
static volatile bool hoverAbortReceived   = false;

#define HOVER_HOLD_MS        5000
#define HOVER_ALT_TOLERANCE  0.10f
#define HOVER_VEL_TOLERANCE  0.15f

static uint32_t hoverHoldStartMs = 0;
static bool     hoverComplete    = false;
// <<< HOVER PATCH

float VEL_KP = 21.0f;              // 15.0f idi
float VEL_KI = 15.0f;              // 2.0f  idi
float VEL_KD = 0.0f;               // 1.0f  idi
#define VEL_DERIVATIVE_FILTER_ALPHA  0.2f
#define VEL_INTEGRAL_LIMIT  0.5f   // 30.0f idi

#define ROCKET_MASS_KG        33.8f
#define THRUST_MAX_N          600.0f
#define THRUST_TO_VALVE_GAIN  (90.0f/1090.0f)    // 1.0f idi
#define MIN_COS_PRODUCT   0.3f

// ════════════════════════════════════════════════════════════
//  ESP-NOW PAKET YAPILARI
// ════════════════════════════════════════════════════════════
typedef struct __attribute__((packed)) TelemetryPacket {
    uint32_t packet_id;
    float    roll;
    float    pitch;
    float    yaw;
    float    h_baro;
    float    h_lidar;
    float    h_fused;
    float    v_descent;
    float    temperature;
    float    pitchServoDeg;
    float    rollServoDeg;
    uint8_t  status;
} TelemetryPacket;

typedef struct __attribute__((packed)) AckPacket {
    uint32_t ack_id;
    float    slave_batt;
    int8_t   rssi;
    uint8_t  slave_status;
} AckPacket;

typedef struct __attribute__((packed)) CommandPacket {
    uint32_t cmd_id;
    uint8_t  cmd_type;
} CommandPacket;

// ─── KALMAN FİLTRESİ YAPISI ─────────────────────────────────────
struct KalmanFilter {
    Matrix<3, 1> x;
    Matrix<3, 3> P;
    Matrix<3, 3> A;
    Matrix<3, 1> B;
    Matrix<2, 3> C;
    Matrix<3, 3> Q;
    Matrix<2, 2> R;
};

KalmanFilter kf;

// ─── PID SINIFI ──────────────────────────────────────────────
struct PIDController {
    float kp, ki, kd;
    float derivativeFilterAlpha;
    float integral           = 0.0;
    float prevError          = 0.0;
    float filteredDerivative = 0.0;
    float integralLimit      = PID_INTEGRAL_LIMIT;

    float compute(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;

        integral += error * dt;
        integral = constrain(integral, -integralLimit, integralLimit);

        float rawDerivative = (dt > 0) ? (error - prevError) / dt : 0.0;

        filteredDerivative = derivativeFilterAlpha * rawDerivative
                            + (1.0 - derivativeFilterAlpha) * filteredDerivative;

        prevError = error;

        return kp * error + ki * integral + kd * filteredDerivative;
    }

    void reset() {
        integral = 0.0;
        prevError = 0.0;
        filteredDerivative = 0.0;
    }
};

PIDController pitchPID = { PITCH_KP, PITCH_KI, PITCH_KD, PITCH_DERIVATIVE_FILTER_ALPHA };
PIDController rollPID  = { ROLL_KP,  ROLL_KI,  ROLL_KD,  ROLL_DERIVATIVE_FILTER_ALPHA  };
PIDController velocityPID = { VEL_KP, VEL_KI, VEL_KD, VEL_DERIVATIVE_FILTER_ALPHA };

// ─── SERVO FİLTRE / DURUM DEĞİŞKENLERİ ─────────────────────────
float filteredPitchAngle  = 0.0;
float filteredRollAngle   = 0.0;
bool  servoFilterInit     = false;

float lastPitchOffsetDeg = 0.0;
float lastRollOffsetDeg  = 0.0;

long  currentValveSteps = 0;
float currentValveDeg   = 0.0;

float currentPitchServoDeg = 0.0;
float currentRollServoDeg  = 0.0;

// ─── SENSÖR NESNELERİ ────────────────────────────────────────
Adafruit_BNO055 bno(55, BNO055_I2C_ADDR);
Adafruit_BMP3XX bmp;

// ─── GLOBAL DEĞİŞKENLER ──────────────────────────────────────
float groundAltitude = 0.0f;
float baroAltitude   = 0.0f;
float lidarDist      = 0.0f;
float lidarAltCorr   = 0.0f;
int   lidarFlux      = 0;
bool  lidarValid     = false;
float accZ           = 0.0f;
float tiltAngleRad   = 0.0f;
float temperature    = 0.0f;

static float lastAcceptedLidarAlt = 0.0f;
static bool  hasAcceptedLidarAlt  = false;

// ─── TAHLİYE DURUMU ──────────────────────────────────────────
static volatile bool deployCommandReceived = false;
static volatile bool deployStopCommandReceived = false;
static bool          deploymentActive      = false;
static uint32_t      lastCmdId             = 0;

// ─── ESP-NOW GLOBAL DEĞİŞKENLERİ ────────────────────────────────
static TelemetryPacket outPacket   = {};
static AckPacket       inAck       = {};
static CommandPacket   inCmd       = {};
static uint32_t        sentCount   = 0;
static uint32_t        lostCount   = 0;
static esp_now_peer_info_t peerInfo = {};

static volatile bool     sendPending      = false;
static volatile bool     lastSendFailed   = false;
static volatile uint32_t lastFailedPktId  = 0;

static volatile bool     ackAvailable     = false;
static volatile bool     cmdAvailable     = false;
static volatile bool     unexpectedLenFlag = false;
static volatile int      unexpectedLen     = 0;
static int8_t             lastSlaveRssi    = 0;

// ─── SD KART LOG DURUMU ───────────────────────────────────────
static File     logFile;
static bool     sdReady          = false;
static uint32_t sdWriteCount     = 0;
static uint32_t sdTotalRows      = 0;

// ════════════════════════════════════════════════════════════
//  ÇİFT ÇEKİRDEK MİMARİSİ — PAYLAŞILAN VERİ
// ════════════════════════════════════════════════════════════
struct SharedTelemetryState {
    float roll, pitch, yaw;
    float h_baro, h_lidar, h_fused;
    float v_descent, temperature;
    float pitchServoDeg, rollServoDeg;
    float valveDeg;
    float vRef;
    float fzCmd;
    float thrustCmd;
    bool  motorOn;
    bool  useLidar;
    bool  tiltSafe;
    // >>> HOVER PATCH
    bool  hoverActive;
    bool  hoverComplete;
    // <<< HOVER PATCH
};

static SharedTelemetryState sharedState = {};
static SemaphoreHandle_t    telemetryMutex = NULL;
static TaskHandle_t         sensorTaskHandle = NULL;

static volatile bool  tiltWarningPending = false;
static volatile float tiltWarnRoll = 0.0f;
static volatile float tiltWarnPitch = 0.0f;
static volatile bool  lidarOutlierPending = false;
static volatile float lidarOutlierRaw = 0.0f;
static volatile float lidarOutlierLastAccepted = 0.0f;

// ─── FONKSİYON PROTOTİPLERİ ──────────────────────────────────
void  setupKalman();
void  setupEspNow();
void  setupTVC();
void  sendTelemetry(float roll, float pitch, float yaw, float h_baro, float h_lidar,
                     float h_fused, float v_descent, float temp, float pitchServoDeg,
                     float rollServoDeg, bool useLidar, bool tiltSafe);
void  sensorTaskFunc(void *pvParameters);
void  runTVC(const imu::Vector<3> &euler, bool tiltSafe, float dt);
void  predict(float accel_meas);
void  update(float baro_h, float lidar_h, bool use_lidar);
bool  readLiDAR(float &dist_m, int &flux);
float readBaro();
float readIMU();
void  calibrateBaro(int samples);
void  setupSDCard();
void  writeLogRow(uint32_t ms, float h_fused, float v_descent, float h_baro, float h_lidar,
                   float roll, float pitch, float yaw, float temp, float pitchServoDeg,
                   float rollServoDeg, bool useLidar, bool tiltSafe, float valveDeg,
                   float vRef, bool motorOn, uint32_t lost);
float getTiltAngleFromEuler(const imu::Vector<3> &euler);
float correctLidarAlt(float rawDist_m, float tiltRad);
bool  rejectLidarOutlier(float newAlt, float &acceptedAlt);
void  executeDeployment();
void  executeDeploymentStop();
int   angleOffsetToPulseUs(float centerPulseUs, float offsetDegrees);
float applyDeadband(float angleDeg, float deadbandDeg);
float applySlewLimit(float targetDeg, float &prevDeg, float maxStepDeg);
void  setupValve();
void  updateValve(float targetDeg);
void  valveStepOnce(bool forward);
void  safeWriteServo(int pin, float centerPulseUs, float offsetDegrees, float maxAngle, float &outDeg);
float pulseUsToAngleDeg(float pulseUs);
float landingLogic(float z, float vz, bool &motorOn);
float computeValveTargetDegFromThrust(float thrustCmd, bool motorOn);
float computeAngleCorrectedThrust(float Fz, float thetaRad, float phiRad);

// >>> HIL PATCH
#if HIL_MODE
bool  hilPaketOku(hil_sensor_t &s);
void  hilKomutGonder(const hil_cmd_t &cIn);
void  quatToEuler(float w, float x, float y, float z, float &yawDeg, float &rollDeg, float &pitchDeg);
#endif
// <<< HIL PATCH

// ════════════════════════════════════════════════════════════
//  ESP-NOW CALLBACK: Gönderim Durumu
// ════════════════════════════════════════════════════════════
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    sendPending = false;
    if (status != ESP_NOW_SEND_SUCCESS) {
        lostCount++;
        lastSendFailed  = true;
        lastFailedPktId = outPacket.packet_id;
    }
}

// ════════════════════════════════════════════════════════════
//  ESP-NOW CALLBACK: Veri Alındı
// ════════════════════════════════════════════════════════════
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int data_len) {

    if (info && info->rx_ctrl) {
        lastSlaveRssi = info->rx_ctrl->rssi;
    }

    if (data_len == sizeof(AckPacket)) {
        memcpy(&inAck, data, sizeof(AckPacket));
        ackAvailable = true;

    } else if (data_len == sizeof(CommandPacket)) {
        CommandPacket tmp;
        memcpy(&tmp, data, sizeof(CommandPacket));

        if (tmp.cmd_id != lastCmdId) {
            lastCmdId = tmp.cmd_id;
            inCmd     = tmp;

            if (inCmd.cmd_type == CMD_DEPLOY) {
                deployCommandReceived = true;
                cmdAvailable = true;
            } else if (inCmd.cmd_type == CMD_DEPLOY_STOP) {
                deployStopCommandReceived = true;
                cmdAvailable = true;
            // >>> HOVER PATCH
            } else if (inCmd.cmd_type == CMD_HOVER_START) {
                hoverTriggerReceived = true;
                cmdAvailable = true;
            } else if (inCmd.cmd_type == CMD_HOVER_ABORT) {
                hoverAbortReceived = true;
                cmdAvailable = true;
            }
            // <<< HOVER PATCH
        }

    } else {
        unexpectedLenFlag = true;
        unexpectedLen     = data_len;
    }
}

// ════════════════════════════════════════════════════════════
//  executeDeployment() / executeDeploymentStop()
// ════════════════════════════════════════════════════════════
void executeDeployment() {
    Serial.println("[TAHLİYE] executeDeployment() çalıştırıldı — mekanizma henüz tanımlı değil.");
}

void executeDeploymentStop() {
    Serial.println("[TAHLİYE] executeDeploymentStop() çalıştırıldı — mekanizma henüz tanımlı değil.");
}

// ════════════════════════════════════════════════════════════
//  setupEspNow()
// ════════════════════════════════════════════════════════════
void setupEspNow() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    delay(100);
    esp_err_t wifiStartRes = esp_wifi_start();
    if (wifiStartRes != ESP_OK && wifiStartRes != ESP_ERR_WIFI_STATE) {
        Serial.printf("[HATA] esp_wifi_start(): %s\n", esp_err_to_name(wifiStartRes));
    }
    delay(50);

    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_ps(WIFI_PS_NONE);

    esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                                        WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
    esp_wifi_set_max_tx_power(84);

    Serial.printf("[ESP-NOW] Master MAC : %s\n", WiFi.macAddress().c_str());
    if (WiFi.macAddress() == "00:00:00:00:00:00") {
        Serial.println("[UYARI] MAC adresi hala sıfır görünüyor! Wi-Fi radyosu düzgün başlamamış olabilir.");
    }
    Serial.println("[INFO] Bu adresi Slave kodundaki MASTER_MAC dizisine yazdığından emin ol.");

    if (esp_now_init() != ESP_OK) {
        Serial.println("[HATA] esp_now_init() başarısız!");
        ESP.restart();
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    memcpy(peerInfo.peer_addr, SLAVE_MAC, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("[HATA] Slave peer eklenemedi! MAC doğru mu?");
        ESP.restart();
    }

    Serial.println("[OK] ESP-NOW hazır.");
}

// ════════════════════════════════════════════════════════════
//  setupTVC()
// ════════════════════════════════════════════════════════════
void setupTVC() {
    bool pitchOk = ledcAttach(PITCH_SERVO_PIN, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
    bool rollOk  = ledcAttach(ROLL_SERVO_PIN,  LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);

    if (!pitchOk || !rollOk) {
        Serial.println("[HATA] ledcAttach basarisiz! Pin/kanal kontrol et.");
    }

    safeWriteServo(PITCH_SERVO_PIN, PITCH_CENTER_PULSE_US, 0.0, MAX_PITCH_GIMBAL_ANGLE, currentPitchServoDeg);
    safeWriteServo(ROLL_SERVO_PIN,  ROLL_CENTER_PULSE_US,  0.0, MAX_ROLL_GIMBAL_ANGLE,  currentRollServoDeg);

    Serial.println("[OK] TVC servolar (native LEDC) merkeze alındı.");
}

// ════════════════════════════════════════════════════════════
//  angleOffsetToPulseUs() / applyDeadband() / pulseUsToAngleDeg()
// ════════════════════════════════════════════════════════════
int angleOffsetToPulseUs(float centerPulseUs, float offsetDegrees) {
    float pulse = centerPulseUs + (offsetDegrees * US_PER_DEGREE);
    return (int)constrain(pulse, SAFE_MIN_PULSE_US, SAFE_MAX_PULSE_US);
}

float applyDeadband(float angleDeg, float deadbandDeg) {
    if (fabs(angleDeg) < deadbandDeg) return 0.0;
    return angleDeg;
}

float applySlewLimit(float targetDeg, float &prevDeg, float maxStepDeg) {
    float delta = targetDeg - prevDeg;
    delta = constrain(delta, -maxStepDeg, maxStepDeg);
    prevDeg = prevDeg + delta;
    return prevDeg;
}

// ════════════════════════════════════════════════════════════
//  İTKİ VANASI (NEMA34 + CWDS86C, PUL/DIR PULSE) FONKSİYONLARI
// ════════════════════════════════════════════════════════════
void valveStepOnce(bool forward) {
    digitalWrite(VALVE_DIR_PIN, forward ? HIGH : LOW);

    digitalWrite(VALVE_STEP_PIN, HIGH);
    delayMicroseconds(VALVE_STEP_PULSE_US);
    digitalWrite(VALVE_STEP_PIN, LOW);
    delayMicroseconds(VALVE_STEP_PULSE_US);
}

// ════════════════════════════════════════════════════════════
//  landingLogic()
// ════════════════════════════════════════════════════════════
float landingLogic(float z, float vz, bool &motorOn) {
    motorOn = false;
    if (z < 0.0f) z = 0.0f;

    if (missionMode == MISSION_MODE_HOVER) {
        // >>> HOVER PATCH v2: eski LANDING_KZ_HOVER'lı P-kontrolcü 4m'de
        //     benzetim modelinin 3 katı hızla tırmanıyordu — yerine
        //     doğrusal tırmanma profili geldi.
        motorOn = true;
        if (z >= HOVER_Z_TARGET)      return 0.0f;
        else if (z <= HOVER_Z_START)  return HOVER_V_CLIMB;
        else {
            float oran = (HOVER_Z_TARGET - z) / (HOVER_Z_TARGET - HOVER_Z_START);
            oran = constrain(oran, 0.0f, 1.0f);
            return oran * HOVER_V_CLIMB;
        }
        // <<< HOVER PATCH v2

    } else if (missionMode == MISSION_MODE_LANDING) {

        if (z > LANDING_Z_ACTIVATION_M) {
            motorOn = false;
            return 0.0f;
        }

        if (z <= LANDING_Z_CUTOFF_M && vz >= LANDING_VZ_CUTOFF_MIN && vz <= LANDING_VZ_CUTOFF_MAX) {
            motorOn = false;
            return 0.0f;
        }

        motorOn = true;

        if (z > LANDING_Z_FLARE_START) {
            return LANDING_V_LAND_FAST;

        } else if (z > LANDING_Z_TOUCH_ZONE) {
            float ratio = (z - LANDING_Z_TOUCH_ZONE) / (LANDING_Z_FLARE_START - LANDING_Z_TOUCH_ZONE);
            ratio = constrain(ratio, 0.0f, 1.0f);
            return LANDING_V_LAND_TOUCH + ratio * (LANDING_V_LAND_FAST - LANDING_V_LAND_TOUCH);

        } else {
            return LANDING_V_LAND_TOUCH;
        }

    } else {
        motorOn = false;
        return 0.0f;
    }
}

float computeAngleCorrectedThrust(float Fz, float thetaRad, float phiRad) {
    float cosProduct = cosf(thetaRad) * cosf(phiRad);
    cosProduct = fmaxf(cosProduct, MIN_COS_PRODUCT);
    return Fz / cosProduct;
}

float computeValveTargetDegFromThrust(float thrustCmd, bool motorOn) {
    if (!motorOn) return VALVE_CLOSED_DEG;
    float deg = thrustCmd * THRUST_TO_VALVE_GAIN;
    return constrain(deg, (float)VALVE_CLOSED_DEG, (float)VALVE_OPEN_DEG);
}

void updateValve(float targetDeg) {
    long targetSteps = (long)round(targetDeg * VALVE_STEPS_PER_DEGREE);

    long diff = targetSteps - currentValveSteps;
    if (diff == 0) return;

    long stepsThisLoop = min(labs(diff), (long)MAX_VALVE_STEPS_PER_LOOP);
    bool opening = diff > 0;

    for (long i = 0; i < stepsThisLoop; i++) {
        valveStepOnce(opening);
        currentValveSteps += opening ? 1 : -1;
    }

    currentValveDeg = currentValveSteps / VALVE_STEPS_PER_DEGREE;
}

void setupValve() {
    pinMode(VALVE_STEP_PIN, OUTPUT);
    pinMode(VALVE_DIR_PIN, OUTPUT);
    digitalWrite(VALVE_STEP_PIN, LOW);
    digitalWrite(VALVE_DIR_PIN, LOW);

    long targetSteps = (long)round(VALVE_CLOSED_DEG * VALVE_STEPS_PER_DEGREE);
    bool opening = targetSteps > currentValveSteps;

    long stepsToMove = labs(targetSteps - currentValveSteps);
    for (long i = 0; i < stepsToMove; i++) {
        valveStepOnce(opening);
    }
    currentValveSteps = targetSteps;
    currentValveDeg = currentValveSteps / VALVE_STEPS_PER_DEGREE;

    Serial.printf("[OK] Vana güvenli baslangic acikligina alindi: %.1f derece (%ld step).\n", currentValveDeg, targetSteps);
}

float pulseUsToAngleDeg(float pulseUs) {
    return (pulseUs - 500.0) / US_PER_DEGREE;
}

// ════════════════════════════════════════════════════════════
//  safeWriteServo()
// ════════════════════════════════════════════════════════════
void safeWriteServo(int pin, float centerPulseUs, float offsetDegrees, float maxAngle, float &outDeg) {
    float clampedOffset = constrain(offsetDegrees, -maxAngle, maxAngle);
    int pulseUs = angleOffsetToPulseUs(centerPulseUs, clampedOffset);
    uint32_t duty = (uint32_t)((pulseUs / LEDC_PERIOD_US) * LEDC_MAX_DUTY);
    ledcWrite(pin, duty);
    outDeg = pulseUsToAngleDeg((float)pulseUs);
}

// ════════════════════════════════════════════════════════════
//  runTVC()
// ════════════════════════════════════════════════════════════
void runTVC(const imu::Vector<3> &euler, bool tiltSafe, float dt) {
    float rollAngle  = euler.y();
    float pitchAngle = euler.z();

    if (!tiltSafe) {
        safeWriteServo(PITCH_SERVO_PIN, PITCH_CENTER_PULSE_US, 0.0, MAX_PITCH_GIMBAL_ANGLE, currentPitchServoDeg);
        safeWriteServo(ROLL_SERVO_PIN,  ROLL_CENTER_PULSE_US,  0.0, MAX_ROLL_GIMBAL_ANGLE,  currentRollServoDeg);
        pitchPID.reset();
        rollPID.reset();
        servoFilterInit = false;
        lastPitchOffsetDeg = 0.0;
        lastRollOffsetDeg  = 0.0;
        return;
    }

    if (!servoFilterInit) {
        filteredPitchAngle = pitchAngle;
        filteredRollAngle  = rollAngle;
        servoFilterInit = true;
    }

    filteredPitchAngle = PITCH_ANGLE_FILTER_ALPHA * pitchAngle + (1.0 - PITCH_ANGLE_FILTER_ALPHA) * filteredPitchAngle;
    filteredRollAngle  = ROLL_ANGLE_FILTER_ALPHA  * rollAngle  + (1.0 - ROLL_ANGLE_FILTER_ALPHA)  * filteredRollAngle;

    float pitchForPID = applyDeadband(filteredPitchAngle, PITCH_DEADBAND_DEG);
    float rollForPID  = applyDeadband(filteredRollAngle,  ROLL_DEADBAND_DEG);

    float pitchOutput = pitchPID.compute(0.0, pitchForPID, dt);
    float rollOutput  = rollPID.compute(0.0, rollForPID, dt);

    float pitchOffsetDegTarget = PITCH_DIR * PITCH_GAIN * pitchOutput;
    float rollOffsetDegTarget  = ROLL_DIR  * ROLL_GAIN  * rollOutput;

    float pitchOffsetDeg = applySlewLimit(pitchOffsetDegTarget, lastPitchOffsetDeg, MAX_PITCH_SLEW_DEG_PER_LOOP);
    float rollOffsetDeg  = applySlewLimit(rollOffsetDegTarget,  lastRollOffsetDeg,  MAX_ROLL_SLEW_DEG_PER_LOOP);

    safeWriteServo(PITCH_SERVO_PIN, PITCH_CENTER_PULSE_US, pitchOffsetDeg, MAX_PITCH_GIMBAL_ANGLE, currentPitchServoDeg);
    safeWriteServo(ROLL_SERVO_PIN,  ROLL_CENTER_PULSE_US,  rollOffsetDeg,  MAX_ROLL_GIMBAL_ANGLE,  currentRollServoDeg);
}

// ════════════════════════════════════════════════════════════
//  sendTelemetry()
// ════════════════════════════════════════════════════════════
void sendTelemetry(float roll, float pitch, float yaw, float h_baro, float h_lidar,
                    float h_fused, float v_descent, float temp, float pitchServoDeg,
                    float rollServoDeg, bool useLidar, bool tiltSafe) {

    if (sendPending) {
        return;
    }

    outPacket.packet_id   = ++sentCount;

    outPacket.roll        = roll;
    outPacket.pitch       = pitch;
    outPacket.yaw         = yaw;

    outPacket.h_baro      = h_baro;
    outPacket.h_lidar     = h_lidar;
    outPacket.h_fused     = h_fused;

    outPacket.v_descent   = v_descent;
    outPacket.temperature = temp;

    outPacket.pitchServoDeg = pitchServoDeg;
    outPacket.rollServoDeg  = rollServoDeg;

    uint8_t st = 0;
    st |= 0x01;
    if (useLidar)          st |= 0x02;
    if (tiltSafe)          st |= 0x04;
    if (deploymentActive)  st |= 0x08;
    outPacket.status = st;

    sendPending = true;
    esp_err_t res = esp_now_send(SLAVE_MAC,
                                  (uint8_t *)&outPacket,
                                  sizeof(TelemetryPacket));
    if (res != ESP_OK) {
        sendPending = false;
        Serial.printf("[HATA] esp_now_send: %s\n", esp_err_to_name(res));
    }
}

// ════════════════════════════════════════════════════════════
//  getTiltAngleFromEuler()
// ════════════════════════════════════════════════════════════
float getTiltAngleFromEuler(const imu::Vector<3> &euler) {
    float pitchRad = (float)euler.z() * DEG_TO_RAD;
    float rollRad  = (float)euler.y() * DEG_TO_RAD;
    float cosTheta = constrain(cosf(pitchRad) * cosf(rollRad), -1.0f, 1.0f);
    return acosf(cosTheta);
}

float correctLidarAlt(float rawDist_m, float tiltRad) {
    return rawDist_m * cosf(tiltRad);
}

bool rejectLidarOutlier(float newAlt, float &acceptedAlt) {
    static int consecutiveRejects = 0;

    if (!hasAcceptedLidarAlt) {
        acceptedAlt = newAlt;
        hasAcceptedLidarAlt = true;
        consecutiveRejects = 0;
        return true;
    }

    float delta = fabsf(newAlt - acceptedAlt);

    if (delta > LIDAR_JUMP_LIMIT_M) {
        consecutiveRejects++;
        if (consecutiveRejects >= LIDAR_OUTLIER_MAX_CONSECUTIVE) {
            acceptedAlt = newAlt;
            consecutiveRejects = 0;
            return true;
        }
        return false;
    }

    acceptedAlt = newAlt;
    consecutiveRejects = 0;
    return true;
}

// >>> HIL PATCH
#if HIL_MODE
// ════════════════════════════════════════════════════════════
//  hilPaketOku() — Simulink'ten 0xAA 0x55 ile başlayan 48 baytlık
//  hil_sensor_t paketini okur (readLiDAR() ile aynı state machine
//  senkronizasyon mantığı). BLOKLAYICI: paket gelene (ya da
//  HIL_TIMEOUT_MS'e) kadar bekler — HIL modunda sensorTaskFunc()
//  döngüsü sabit 100 Hz timer YERİNE Simulink'in adımına kilitlenir.
// ════════════════════════════════════════════════════════════
bool hilPaketOku(hil_sensor_t &s) {
    static uint8_t buf[HIL_SENSOR_LEN];
    static int idx = 0;
    bool bulundu = false;                       // <<< YENI

    uint32_t startMs = millis();

    while (millis() - startMs < HIL_TIMEOUT_MS) {
        while (HIL_SERIAL.available()) {
            uint8_t b = HIL_SERIAL.read();

            if (idx == 0) {
                if (b != HIL_HDR1) continue;
            } else if (idx == 1) {
                if (b != HIL_HDR2) { idx = 0; continue; }
            }
            buf[idx++] = b;

            if (idx == HIL_SENSOR_LEN) {
                idx = 0;
                if (hil_checksum(buf, HIL_SENSOR_LEN) == buf[HIL_SENSOR_LEN - 1]) {
                    memcpy(&s, buf, HIL_SENSOR_LEN);
                    bulundu = true;             // <<< return ETME, tamponu bosalt
                }
            }
        }
        if (bulundu) return true;               // <<< en YENI paketle don
        static int bosSayac = 0;
        if (++bosSayac >= 20) { bosSayac = 0; vTaskDelay(1); }    // watchdog
        else { delayMicroseconds(200); }                          // hizli yoklama
    }
    return false;   // timeout — bu adimda yeni paket gelmedi
}

// ════════════════════════════════════════════════════════════
//  hilKomutGonder() — hesaplanan komutu checksum'layip Simulink'e
//  geri yollar.
// ════════════════════════════════════════════════════════════
void hilKomutGonder(const hil_cmd_t &cIn) {
    hil_cmd_t c = cIn;
    c.hdr1 = HIL_HDR1;
    c.hdr2 = HIL_HDR2;
    c.chk  = hil_checksum((const uint8_t *)&c, HIL_CMD_LEN);
    HIL_SERIAL.write((const uint8_t *)&c, HIL_CMD_LEN);
}

// ════════════════════════════════════════════════════════════
//  quatToEuler() — BNO055 getVector(VECTOR_EULER) çıktısıyla AYNI
//  eksen sırasına (x=yaw, y=roll, z=pitch, derece) çevirir.
//  hil_sensor_t.q = (w,x,y,z).
//  !! EKSEN SIRASI/İŞARETİ Simulink tarafındaki quaternion tanımına
//     göre DOĞRULANMALI. !!
// ════════════════════════════════════════════════════════════
void quatToEuler(float w, float x, float y, float z, float &yawDeg, float &rollDeg, float &pitchDeg) {
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    float rollRad = atan2f(sinr_cosp, cosr_cosp);

    float sinp = 2.0f * (w * y - z * x);
    sinp = constrain(sinp, -1.0f, 1.0f);
    float pitchRad = asinf(sinp);

    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    float yawRad = atan2f(siny_cosp, cosy_cosp);

    rollDeg  = rollRad  * RAD_TO_DEG;
    pitchDeg = pitchRad * RAD_TO_DEG;
    yawDeg   = yawRad   * RAD_TO_DEG;
}
#endif // HIL_MODE
// <<< HIL PATCH

// ════════════════════════════════════════════════════════════
//  SETUP() — cihaz açıldığında BİR KERE çalışır
// ════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println("\n=== Roket — Kalman + ESP-NOW + Tahliye + TVC + Inis Rehberligi + HOVER TEST + HIL (v32) ===");

    setupTVC();
    setupValve();
#if !HIL_MODE
    setupEspNow();
#endif

    // >>> HIL PATCH
#if HIL_MODE
    Serial.println("[HIL] HIL_MODE=1 -> gercek sensorler (BNO055/BMP390/TFmini) ATLANIYOR.");
    HIL_SERIAL.begin(HIL_SERIAL_BAUD);   // pin argümanı YOK — UART0'ı varsayılan 43/44'te bırakır
    Serial.printf("[OK] HIL seri portu hazir (baud=%d).\n", HIL_SERIAL_BAUD);
    groundAltitude = 0.0f;   // HIL'de Simulink zaten mutlak/göre irtifa veriyor
#else
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);
    LIDAR_SERIAL.begin(LIDAR_BAUD, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    Serial.println("[OK] TFmini Plus UART başlatıldı.");

    if (!bno.begin()) {
        Serial.println("[HATA] BNO055 bulunamadı!");
        while (1) delay(10);
    }
    delay(500);
    bno.setExtCrystalUse(true);
    Serial.println("[OK] BNO055 başlatıldı.");

    if (!bmp.begin_I2C(BMP390_I2C_ADDR)) {
        Serial.println("[HATA] BMP390 bulunamadı!");
        while (1) delay(10);
    }
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    Serial.println("[OK] BMP390 başlatıldı.");

    Serial.println("[INFO] Zemin irtifası kalibre ediliyor...");
    calibrateBaro(BARO_CAL_SKIP_SAMPLES + BARO_CAL_AVG_SAMPLES + 5);
    Serial.printf("[OK] Zemin irtifası: %.2f m (ilk %d örnek atlandı, sonraki %d örnek ortalandı)\n",
                  groundAltitude, BARO_CAL_SKIP_SAMPLES, BARO_CAL_AVG_SAMPLES);
#endif
    // <<< HIL PATCH

    setupKalman();
    pitchPID.reset();
    rollPID.reset();
    velocityPID.reset();
    velocityPID.integralLimit = VEL_INTEGRAL_LIMIT;
    Serial.println("[OK] Kalman filtresi, TVC PID ve itki/hız PID hazır.");

    setupSDCard();

    Serial.println("---");
    Serial.println("ms, h_fused, v_descent, h_baro, h_lidar, roll, pitch, yaw, temp, pitchServo, rollServo, LiDAR_OK, kayıp");

    // >>> HOVER PATCH
    Serial.printf("[HOVER] Görev modu OFF ile başlıyor. Hedef irtifa: %.1f m, hold süresi: %lu ms.\n",
                  (float)LANDING_Z_HOVER_M, (unsigned long)HOVER_HOLD_MS);
    Serial.println("[HOVER] Yer istasyonundan CMD_HOVER_START (0x03) komutu bekleniyor...");
    // <<< HOVER PATCH

    telemetryMutex = xSemaphoreCreateMutex();
    if (telemetryMutex == NULL) {
        Serial.println("[HATA] telemetryMutex oluşturulamadı! Sistem durduruluyor.");
        while (1) delay(10);
    }

    BaseType_t taskRes = xTaskCreatePinnedToCore(
        sensorTaskFunc,
        "SensorTask",
        8192,
        NULL,
        1,
        &sensorTaskHandle,
        0
    );

    if (taskRes != pdPASS) {
        Serial.println("[HATA] sensorTaskFunc görevi oluşturulamadı!");
        while (1) delay(10);
    }

    Serial.println("[OK] sensorTaskFunc Core 0'da başlatıldı.");

    // >>> HIL PATCH: ESP-NOW kapalı olduğu için CMD_HOVER_START tetiği
    //     hiç gelemez — HIL testinde hover'ı otomatik başlatıyoruz.
#if HIL_MODE
    missionMode = MISSION_MODE_HOVER;
    Serial.println("[HIL] Hover otomatik baslatildi.");
#endif
    // <<< HIL PATCH
}

// ════════════════════════════════════════════════════════════
//  sensorTaskFunc() — CORE 0'DA ÇALIŞAN GÖREV
//  v32: HIL_MODE=1 iken veri kaynağı Simulink paketi, döngü lock-step.
//       HIL_MODE=0 iken orijinal v31 HOVER davranışı (sabit 100 Hz).
// ════════════════════════════════════════════════════════════
void sensorTaskFunc(void *pvParameters) {
#if !HIL_MODE
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1000 / LOOP_HZ);
#else
    static float lastSimTime = -1.0f;
    // Boot sirasinda birikmis BAYAT paketleri at. Simulink portu acinca
    // kart resetleniyor ve setup() ~1 s suruyor; o sirada gelen paketler
    // islenirse kart kalici olarak 1 s geride kaliyor.
    while (HIL_SERIAL.available()) HIL_SERIAL.read();
#endif

    for (;;) {

        // >>> HIL PATCH
#if HIL_MODE
        // ── 1) Simulink'ten sensör paketini oku (BLOKLAYICI, lock-step) ──
        hil_sensor_t hs;
        if (!hilPaketOku(hs)) {
            continue;   // timeout ya da checksum hatasi: bu adimi atla
        }

        float dtSim = (lastSimTime < 0.0f) ? DT : (hs.t - lastSimTime);
        if (dtSim <= 0.0f) dtSim = DT;
        lastSimTime = hs.t;

        float yawDeg, rollDeg, pitchDeg;
        quatToEuler(hs.q[0], hs.q[1], hs.q[2], hs.q[3], yawDeg, rollDeg, pitchDeg);
        imu::Vector<3> euler(yawDeg, rollDeg, pitchDeg);

        // GERÇEK SENSÖR YERİNE HIL paketindeki değerler kullanılıyor.
        // hs.z/hs.vz zaten Simulink tarafında füzyonlanmış "gerçek" durum
        // olduğu için Kalman predict/update ATLANIYOR.
        kf.x(0) = hs.z;
        kf.x(1) = hs.vz;

        baroAltitude = hs.z;
        lidarAltCorr = hs.z;
        lidarValid   = true;
        bool useLidar = true;
        temperature  = 20.0f;   // HIL paketinde yok, log/telemetri placeholder

        tiltAngleRad  = getTiltAngleFromEuler(euler);
        bool tiltSafe = (fabs(euler.y()) <= TILT_LIMIT_DEG) && (fabs(euler.z()) <= TILT_LIMIT_DEG);
#else
        // ── 1) SENSÖR OKUMALARI ──────────────────────────────
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

        accZ         = readIMU();
        baroAltitude = readBaro();
        lidarValid   = readLiDAR(lidarDist, lidarFlux);

        // ── 2) EĞİM + LiDAR DÜZELTMESİ ───────────────────────
        tiltAngleRad = getTiltAngleFromEuler(euler);
        float rawLidarAltCorr = correctLidarAlt(lidarDist, tiltAngleRad);

        bool lidarAccepted = true;
        if (lidarValid) {
            lidarAccepted = rejectLidarOutlier(rawLidarAltCorr, lastAcceptedLidarAlt);
        }
        lidarAltCorr = lastAcceptedLidarAlt;

        // ── 3) KALMAN TAHMİN (PREDICT) ADIMI ─────────────────
        predict(accZ);

        // ── 4) GÜVENLİK KARARLARI ────────────────────────────
        bool tiltSafe = (fabs(euler.y()) <= TILT_LIMIT_DEG) && (fabs(euler.z()) <= TILT_LIMIT_DEG);

        bool useLidar = lidarValid
                        && lidarAccepted
                        && (lidarDist < 10.0f)
                        && (lidarFlux > 100)
                        && tiltSafe;

        if (!tiltSafe) {
            tiltWarningPending = true;
            tiltWarnRoll  = (float)euler.y();
            tiltWarnPitch = (float)euler.z();
        }
        if (lidarValid && !lidarAccepted) {
            lidarOutlierPending = true;
            lidarOutlierRaw = rawLidarAltCorr;
            lidarOutlierLastAccepted = lastAcceptedLidarAlt;
        }

        // ── 5) KALMAN GÜNCELLEME (UPDATE) ADIMI ──────────────
        update(baroAltitude, lidarAltCorr, useLidar);
#endif
        // <<< HIL PATCH

        // ═══════════════════════════════════════════════════
        //  BURADAN SONRASI — UÇUŞ/HOVER KODUYLA TAMAMEN AYNI.
        //  HIL modunda değişen TEK ŞEY yukarıdaki veri kaynağıydı.
        // ═══════════════════════════════════════════════════

        // ── 6) TVC SERVO PID ──────────────────────────────────
#if HIL_MODE
        runTVC(euler, tiltSafe, dtSim);
#else
        runTVC(euler, tiltSafe, DT);
#endif

        // ── 6b) MOTORLU İNİŞ REHBERLİĞİ + AÇI-DÜZELTMELİ İTKİ ──
        float currentAltitude = (float)kf.x(0);
        float vzUpPositive    = (float)kf.x(1);

        // >>> HOVER PATCH: tetik / abort / 5 saniyelik hold-timer mantığı.
        //     HIL modundan tamamen bağımsız — hover komutları ESP-NOW
        //     üzerinden (yer istasyonu) geliyor, HIL_SERIAL'dan değil.
        if (hoverAbortReceived) {
            hoverAbortReceived = false;
            missionMode = MISSION_MODE_OFF;
            hoverHoldStartMs = 0;
            hoverComplete = false;
        }

        if (hoverTriggerReceived) {
            hoverTriggerReceived = false;
            if (missionMode == MISSION_MODE_OFF && !hoverComplete) {
                missionMode = MISSION_MODE_HOVER;
                hoverHoldStartMs = 0;
                Serial.println("[HOVER] Tetik alındı, HOVER moduna geçildi.");
            }
        }

        if (missionMode == MISSION_MODE_HOVER && !hoverComplete) {
            bool atTarget = (fabsf(currentAltitude - LANDING_Z_HOVER_M) < HOVER_ALT_TOLERANCE)
                          && (fabsf(vzUpPositive) < HOVER_VEL_TOLERANCE);

            if (atTarget) {
                if (hoverHoldStartMs == 0) {
                    hoverHoldStartMs = millis();
                } else if (millis() - hoverHoldStartMs >= HOVER_HOLD_MS) {
                    hoverComplete = true;
                    missionMode = MISSION_MODE_OFF;
                    Serial.println("[HOVER] 5 saniye tamamlandı, motor kapatılıyor.");
                }
            } else {
                hoverHoldStartMs = 0;
            }
        }
        // <<< HOVER PATCH

        bool  motorOn = false;
        float vRef    = landingLogic(currentAltitude, vzUpPositive, motorOn);
        float Fz         = 0.0f;
        float thrustCmd   = 0.0f;
        float valveTargetDeg;

        if (!tiltSafe) {
            motorOn = false;
            velocityPID.reset();
            valveTargetDeg = VALVE_CLOSED_DEG;

        } else if (motorOn) {
#if HIL_MODE
            Fz = velocityPID.compute(vRef, vzUpPositive, dtSim);
#else
            Fz = velocityPID.compute(vRef, vzUpPositive, DT);
#endif
            float thetaRad = (float)euler.z() * DEG_TO_RAD;
            float phiRad   = (float)euler.y() * DEG_TO_RAD;

            // Fz artik bir IVME komutu [m/s^2]. Yercekimini karsila,
            // kutleyle carp -> cikis NEWTON.
#if HIL_MODE
            float kutle = hs.mass;
            if (kutle < 1.0f || kutle > 200.0f) kutle = ROCKET_MASS_KG;
#else
            float kutle = ROCKET_MASS_KG;
#endif
            thrustCmd = kutle * computeAngleCorrectedThrust(Fz + 9.81f, thetaRad, phiRad);
            if (thrustCmd > THRUST_MAX_N) thrustCmd = THRUST_MAX_N;
            if (thrustCmd < 0.0f)         thrustCmd = 0.0f;

            valveTargetDeg = computeValveTargetDegFromThrust(thrustCmd, true);

        } else {
            velocityPID.reset();
            valveTargetDeg = computeValveTargetDegFromThrust(0.0f, false);
        }

        updateValve(valveTargetDeg);   // HIL'de de vana/servolar fiziksel olarak hareket eder

        // >>> HIL PATCH: hesaplanan komutu Simulink'e geri yolla
#if HIL_MODE
        hil_cmd_t cmd = {};
        cmd.seq       = hs.seq;
        cmd.thrust_N  = thrustCmd;
        cmd.step_pos  = (float)currentValveSteps;
        cmd.tvc_pitch = currentPitchServoDeg;
        cmd.tvc_yaw   = currentRollServoDeg;   // donanımda yaw yok, roll TVC buraya map'lendi
        cmd.status    = (float)missionMode;
        hilKomutGonder(cmd);
#endif
        // <<< HIL PATCH

        // ── 7) PAYLAŞILAN DURUMU GÜNCELLE (mutex korumalı) ───
        if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(5)) == pdTRUE) {

            sharedState.roll          = (float)euler.y();
            sharedState.pitch         = (float)euler.z();
            sharedState.yaw           = (float)euler.x();
            sharedState.h_baro        = baroAltitude;
            sharedState.h_lidar       = lidarAltCorr;
            sharedState.h_fused       = currentAltitude;
            sharedState.v_descent     = -vzUpPositive;
            sharedState.temperature   = temperature;
            sharedState.pitchServoDeg = currentPitchServoDeg;
            sharedState.rollServoDeg  = currentRollServoDeg;
            sharedState.useLidar      = useLidar;
            sharedState.tiltSafe      = tiltSafe;
            sharedState.valveDeg      = currentValveDeg;
            sharedState.vRef          = vRef;
            sharedState.fzCmd         = Fz;
            sharedState.thrustCmd     = thrustCmd;
            sharedState.motorOn       = motorOn;
            // >>> HOVER PATCH
            sharedState.hoverActive   = (missionMode == MISSION_MODE_HOVER);
            sharedState.hoverComplete = hoverComplete;
            // <<< HOVER PATCH
            xSemaphoreGive(telemetryMutex);
        }

#if !HIL_MODE
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
#endif
    }
}

// ════════════════════════════════════════════════════════════
//  ANA DÖNGÜ (CORE 1) — ESP-NOW, komut işleme, Serial/SD log
// ════════════════════════════════════════════════════════════
void loop() {
    static uint32_t lastTime = 0;
    uint32_t now = millis();
    if (now - lastTime < (1000 / LOOP_HZ)) return;
    lastTime = now;

#if !HIL_MODE
    if (lastSendFailed) {
        lastSendFailed = false;
        Serial.printf("[ESP-NOW] FAIL — paket #%lu iletilemedi (kayıp: %lu)\n",
                      lastFailedPktId, lostCount);
    }
    if (ackAvailable) {
        ackAvailable = false;
        Serial.printf("[ACK #%lu] Slave batt=%.2fV  RSSI=%ddBm  status=0x%02X\n",
                      inAck.ack_id, inAck.slave_batt, inAck.rssi, inAck.slave_status);
    }
    if (cmdAvailable) {
        cmdAvailable = false;
        Serial.printf("[KOMUT] cmd_id=%lu, cmd_type=0x%02X işlendi.\n", inCmd.cmd_id, inCmd.cmd_type);
    }
    if (unexpectedLenFlag) {
        unexpectedLenFlag = false;
        Serial.printf("[ESP-NOW] Beklenmeyen paket boyutu: %d\n", unexpectedLen);
    }
    if (sendPending) {
        Serial.println("[UYARI] Önceki ESP-NOW gönderimi hâlâ bekleniyor, bu döngü atlanacak.");
    }
#endif
    if (tiltWarningPending) {
        tiltWarningPending = false;
        Serial.printf("[UYARI] Eğim limiti aşıldı (Roll: %.1f°, Pitch: %.1f°, limit: %.0f°) — LiDAR devre dışı + TVC merkezde + itki kesildi\n",
                      tiltWarnRoll, tiltWarnPitch, TILT_LIMIT_DEG);
    }
    if (lidarOutlierPending) {
        lidarOutlierPending = false;
        Serial.printf("[UYARI] LiDAR aykırı değer reddedildi (ham: %.3f m, son geçerli: %.3f m)\n",
                      lidarOutlierRaw, lidarOutlierLastAccepted);
    }

    if (deployCommandReceived) {
        deployCommandReceived = false;
        if (!deploymentActive) {
            deploymentActive = true;
            executeDeployment();
        }
    }
    if (deployStopCommandReceived) {
        deployStopCommandReceived = false;
        if (deploymentActive) {
            deploymentActive = false;
            executeDeploymentStop();
        }
    }

    SharedTelemetryState snapshot;
    bool gotSnapshot = false;
    if (xSemaphoreTake(telemetryMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        snapshot = sharedState;
        gotSnapshot = true;
        xSemaphoreGive(telemetryMutex);
    }
    if (!gotSnapshot) {
        return;
    }

#if !HIL_MODE
    static uint32_t telemetryLoopCounter = 0;
    telemetryLoopCounter++;
    if (telemetryLoopCounter >= TELEMETRY_SEND_DIVIDER) {
        telemetryLoopCounter = 0;
        sendTelemetry(snapshot.roll, snapshot.pitch, snapshot.yaw,
                      snapshot.h_baro, snapshot.h_lidar, snapshot.h_fused,
                      snapshot.v_descent, snapshot.temperature,
                      snapshot.pitchServoDeg, snapshot.rollServoDeg,
                      snapshot.useLidar, snapshot.tiltSafe);
    }
#endif

    static uint32_t sayac = 0;
    bool bas = true;
#if HIL_MODE
    bas = (++sayac % 50 == 0);      // 100 Hz -> 2 Hz
#endif
    if (bas) {
        Serial.printf("%lu, %.3f, %.3f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %d, %lu, vRef=%.2f, Fz=%.2f, F=%.2f, motor=%d, valve=%.1f, hover=%d, hoverDone=%d\n",
                      now,
                      snapshot.h_fused,
                      snapshot.v_descent,
                      snapshot.h_baro,
                      snapshot.h_lidar,
                      snapshot.roll,
                      snapshot.pitch,
                      snapshot.yaw,
                      snapshot.temperature,
                      snapshot.pitchServoDeg,
                      snapshot.rollServoDeg,
                      (int)snapshot.useLidar,
                      lostCount,
                      snapshot.vRef,
                      snapshot.fzCmd,
                      snapshot.thrustCmd,
                      (int)snapshot.motorOn,
                      snapshot.valveDeg,
                      (int)snapshot.hoverActive,
                      (int)snapshot.hoverComplete);
    }

    writeLogRow(now, snapshot.h_fused, snapshot.v_descent, snapshot.h_baro, snapshot.h_lidar,
                snapshot.roll, snapshot.pitch, snapshot.yaw, snapshot.temperature,
                snapshot.pitchServoDeg, snapshot.rollServoDeg, snapshot.useLidar,
                snapshot.tiltSafe, snapshot.valveDeg, snapshot.vRef, snapshot.motorOn, lostCount);
}

// ════════════════════════════════════════════════════════════
//  setupKalman()
// ════════════════════════════════════════════════════════════
void setupKalman() {
    kf.x = {10.0f, 0.0f, 0.0f};

    kf.A = {1.0f, DT,   0.0f,
            0.0f, 1.0f, -DT,
            0.0f, 0.0f,  1.0f};

    kf.B = {0.5f * DT * DT,
            DT,
            0.0f};

    kf.C = {1.0f, 0.0f, 0.0f,
            1.0f, 0.0f, 0.0f};

    kf.P = {1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f,
            0.0f, 0.0f, 1.0f};

    kf.Q = {0.01f, 0.0f,  0.0f,
            0.0f,  0.01f, 0.0f,
            0.0f,  0.0f,  0.001f};

    kf.R = {0.09f, 0.0f,
            0.0f,  0.001f};
}

// ════════════════════════════════════════════════════════════
//  predict()
// ════════════════════════════════════════════════════════════
void predict(float accel_meas) {
    kf.x = kf.A * kf.x + kf.B * accel_meas;
    kf.P = kf.A * kf.P * (~kf.A) + kf.Q;
}

// ════════════════════════════════════════════════════════════
//  update()
// ════════════════════════════════════════════════════════════
void update(float baro_h, float lidar_h, bool use_lidar) {
    kf.R(1, 1) = use_lidar ? 0.001f : 999.0f;

    Matrix<2, 1> z;
    z(0) = baro_h;
    z(1) = use_lidar ? lidar_h : (float)kf.x(0);

    Matrix<2, 2> S = kf.C * kf.P * (~kf.C) + kf.R;

    Matrix<2, 2> S_inv;
    if (!Invert(S, S_inv)) {
        Serial.println("[UYARI] S matrisi terslenemedi.");
        return;
    }

    Matrix<3, 2> K = kf.P * (~kf.C) * S_inv;

    kf.x = kf.x + K * (z - kf.C * kf.x);

    kf.P = (Eye<3, 3>() - K * kf.C) * kf.P;
}

// ════════════════════════════════════════════════════════════
//  Sensör Okuma Fonksiyonları
// ════════════════════════════════════════════════════════════
float readIMU() {
    imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    return (float)linAccel.z();
}

float readBaro() {
    if (!bmp.performReading()) return baroAltitude;

    temperature = bmp.temperature;
    return bmp.readAltitude(1013.25f) - groundAltitude;
}

bool readLiDAR(float &dist_m, int &flux) {
    static uint8_t buf[9];
    static int     idx = 0;

    while (LIDAR_SERIAL.available()) {
        uint8_t byte = LIDAR_SERIAL.read();

        if (idx == 0 && byte != 0x59) continue;
        if (idx == 1 && byte != 0x59) { idx = 0; continue; }

        buf[idx++] = byte;

        if (idx == 9) {
            idx = 0;

            uint8_t checksum = 0;
            for (int i = 0; i < 8; i++) checksum += buf[i];
            if (checksum != buf[8]) return false;

            dist_m = (buf[2] | (buf[3] << 8)) / 100.0f;
            flux   =  buf[4] | (buf[5] << 8);
            return true;
        }
    }
    return false;
}

// ════════════════════════════════════════════════════════════
//  calibrateBaro()
// ════════════════════════════════════════════════════════════
void calibrateBaro(int samples) {
    float sum       = 0.0f;
    int   count     = 0;
    int   validSeen = 0;

    for (int i = 0; i < samples; i++) {
        if (bmp.performReading()) {
            validSeen++;
            if (validSeen > BARO_CAL_SKIP_SAMPLES) {
                sum += bmp.readAltitude(1013.25f);
                count++;
                if (count >= BARO_CAL_AVG_SAMPLES) break;
            }
        }
        delay(20);
    }
    groundAltitude = (count > 0) ? (sum / count) : 0.0f;
}

// ════════════════════════════════════════════════════════════
//  setupSDCard()
// ════════════════════════════════════════════════════════════
void setupSDCard() {
    SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

    if (!SD.begin(SD_CS_PIN, SPI)) {
        Serial.println("[HATA] SD kart başlatılamadı! Loglama devre dışı, uçuşun geri kalanı SD'siz devam edecek.");
        sdReady = false;
        return;
    }

    char filename[32];
    int  fileIndex = 0;
    do {
        snprintf(filename, sizeof(filename), "/log%04d.csv", fileIndex);
        fileIndex++;
    } while (SD.exists(filename) && fileIndex < 10000);

    logFile = SD.open(filename, FILE_WRITE);
    if (!logFile) {
        Serial.printf("[HATA] SD log dosyası açılamadı: %s\n", filename);
        sdReady = false;
        return;
    }

    logFile.println("ms,h_fused,v_descent,h_baro,h_lidar,roll,pitch,yaw,temp,pitchServo,rollServo,LiDAR_OK,tiltSafe,valveDeg,vRef,motorOn,kayip");
    logFile.flush();

    sdReady = true;
    Serial.printf("[OK] SD kart hazır, log dosyası: %s\n", filename);
}

// ════════════════════════════════════════════════════════════
//  writeLogRow()
// ════════════════════════════════════════════════════════════
void writeLogRow(uint32_t ms, float h_fused, float v_descent, float h_baro, float h_lidar,
                  float roll, float pitch, float yaw, float temp, float pitchServoDeg,
                  float rollServoDeg, bool useLidar, bool tiltSafe, float valveDeg,
                  float vRef, bool motorOn, uint32_t lost) {
    if (!sdReady) return;

    logFile.printf("%lu,%.3f,%.3f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%.1f,%.2f,%d,%lu\n",
                    ms, h_fused, v_descent, h_baro, h_lidar, roll, pitch, yaw, temp,
                    pitchServoDeg, rollServoDeg, (int)useLidar, (int)tiltSafe, valveDeg,
                    vRef, (int)motorOn, lost);

    sdWriteCount++;
    if (sdWriteCount >= SD_FLUSH_EVERY_N_WRITES) {
        sdWriteCount = 0;
        logFile.flush();
    }
    sdTotalRows++;
}

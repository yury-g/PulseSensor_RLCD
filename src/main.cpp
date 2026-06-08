#include <Arduino.h>
#include <esp_arduino_version.h>
#include <Wire.h>

// PulseSensorPlayground owns the 500 Hz ESP32 sampler. Keep foreground UI work
// after readPulseSensor() so the detector always gets serviced first.
#define USE_ARDUINO_INTERRUPTS true
#include <PulseSensorPlayground.h>

#include "rlcd_st7305.h"

#ifndef APP_VERSION
#define APP_VERSION "0.4.43-rlcd"
#endif

#ifndef PULSE_PIN
#define PULSE_PIN 1
#endif

#ifndef RAW_SIGNAL_DIAGNOSTICS
#define RAW_SIGNAL_DIAGNOSTICS 0
#endif

#define PULSE_THRESHOLD 550
#define NO_BEAT_TIMEOUT 3000
#define PULSE_DYNAMIC_THRESHOLD_MIN 180
#define PULSE_DYNAMIC_THRESHOLD_MAX 900
#define MIN_QUALIFIED_BPM 40
#define MAX_QUALIFIED_BPM 180
#define MIN_QUALIFIED_IBI 333
#define MAX_QUALIFIED_IBI 1500
#define MIN_QUALIFIED_AMPLITUDE 20
#define SIGNAL_QUALITY_STEPS 12
#define LOCK_QUALITY_STEPS 10
#define LOCK_QUALIFIED_BEATS 4
#define LOCK_GRACE_BAD_BEATS 4
#define LOCK_HOLD_GRACE_MS 4200
#define ACQUISITION_CADENCE_TOLERANCE_PERCENT 35
#define ACQUISITION_CADENCE_MIN_IBI_PERCENT 70
#define PEAK_TO_PEAK_RECOVERY_ENABLED 1
#define PEAK_TO_PEAK_MIN_RANGE 70
#define PEAK_TO_PEAK_STRONG_RANGE 135
#define PEAK_TO_PEAK_MIN_AMPLITUDE 10
#define PEAK_TO_PEAK_ACQUIRE_MIN_SCORE 5
#define PEAK_TO_PEAK_LOCKED_MIN_SCORE 4
#define PEAK_TO_PEAK_PRELOCK_CADENCE_MIN_STREAK 1
#define PEAK_TO_PEAK_FIRST_BEAT_SCORE 8
#define PEAK_TO_PEAK_LOCKED_IBI_TOLERANCE_PERCENT 35
#define PEAK_TO_PEAK_LOCKED_MIN_IBI_TOLERANCE_MS 180
#define PEAK_TO_PEAK_LOCKED_MIN_IBI_PERCENT 70
#define PEAK_RECOVERY_IBI_TOLERANCE_PERCENT 28
#define PEAK_RECOVERY_MIN_IBI_TOLERANCE_MS 120
#define PEAK_RECOVERY_MIN_RANGE 80
#define PEAK_RECOVERY_MIN_AMPLITUDE 12
#define LOCKED_CLIPPING_SCORE_LIMIT 100
#define REARM_SIGNAL_RANGE 120
#define REARM_NO_BEAT_MS 1600
#define REARM_COOLDOWN_MS 1600
#define SIGNAL_COACH_FLAT_RANGE 90
#define SIGNAL_COACH_FLAT_AMPLITUDE 12
#define SIGNAL_COACH_STEADY_AMPLITUDE MIN_QUALIFIED_AMPLITUDE
#define SIGNAL_ACQUISITION_MIN_RANGE 40
#define SIGNAL_ACQUISITION_FULL_RANGE 220
#define SIGNAL_ACQUISITION_MAX_SCORE_BEFORE_LOCK 11
#define SIGNAL_MOTION_ARTIFACT_RANGE 980
#define AMPLITUDE_METER_MAX 120
#define CLIPPING_SCORE_DECAY_MS 20

static constexpr int PIN_BOOT = 0;
static constexpr int PIN_KEY = 18;
static constexpr int PIN_BATTERY_ADC = 4;
static constexpr float BATTERY_ADC_SCALE = 3.0f;
static constexpr float BATTERY_EMPTY_VOLTAGE = 3.0f;
static constexpr float BATTERY_FULL_VOLTAGE = 4.12f;
static constexpr int PIN_I2C_SDA = 13;
static constexpr int PIN_I2C_SCL = 14;
static constexpr uint8_t BATTERY_GAUGE_ADDR = 0x55;
static constexpr uint8_t BATTERY_SOC_REG = 0x2C;
static constexpr unsigned long BATTERY_SAMPLE_MS = 5000;
static constexpr uint8_t SHTC3_ADDR = 0x70;
static constexpr uint16_t SHTC3_WAKEUP = 0x3517;
static constexpr uint16_t SHTC3_SLEEP = 0xB098;
static constexpr uint16_t SHTC3_MEASURE_T_RH = 0x7866;
static constexpr unsigned long ENVIRONMENT_SAMPLE_MS = 5000;
static constexpr int HEART_MIN_SIZE = 9;
static constexpr int HEART_MAX_SIZE = 18;
static constexpr int LED_PEAK_HOLD_MS = 90;
static constexpr int LED_FADE_MS = 620;
static constexpr int DASHBOARD_DRAW_MS = 80;
static constexpr int WAVEFORM_SAMPLE_MS = 20;

static constexpr int GRAPH_LEFT = 14;
static constexpr int GRAPH_TOP = 46;
static constexpr int GRAPH_WIDTH = 372;
static constexpr int GRAPH_HEIGHT = 138;
static constexpr int SIGNAL_STRIP_Y = 190;
static constexpr int SIGNAL_STRIP_H = 20;
static constexpr int PANEL_Y = 218;
static constexpr int PANEL_H = 66;
static constexpr int BPM_PANEL_X = 14;
static constexpr int IBI_PANEL_X = 210;
static constexpr int SIGNAL_PANEL_X = 274;
static constexpr int METRIC_PANEL_W = 176;
static constexpr int SIGNAL_PANEL_W = 116;
static constexpr int LABEL_TEXT_SIZE = 2;
static constexpr int GRAPH_LABEL_TEXT_SIZE = 2;

enum SignalCoachState {
  COACH_SIGNAL_SEARCH,
  COACH_CLIPPED,
  COACH_TOO_FLAT,
  COACH_HOLD_STEADY,
  COACH_GOOD_WAVE,
  COACH_LOCKING,
  COACH_QUALIFIED
};

enum DisplayMode {
  DISPLAY_MONO_DARK,
  DISPLAY_MONO_LIGHT,
  DISPLAY_COLOR_DARK,
  DISPLAY_COLOR_LIGHT,
  DISPLAY_MODE_COUNT
};

struct RearLedColor {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

const RearLedColor REAR_LED_HEARTBEAT = {255, 0, 0};
const RearLedColor REAR_LED_LOCKING = {255, 255, 0};

struct BeatDecision {
  bool qualified;
  bool strictAccepted;
  bool peakToPeakAccepted;
  bool recovered;
  bool accepted;
  const char* acceptReason;
};

struct PulseSignalState {
  int currentSignal = 512;
  int displayBPM = 0;
  int displayIBI = 0;
  int pulseAmplitude = 0;
  int minSignal = 512;
  int maxSignal = 512;
  int activePulseThreshold = PULSE_THRESHOLD;
  unsigned long lastBeatTime = 0;
  unsigned long lastQualifiedBeatTime = 0;
  unsigned long lastSerialPrint = 0;
  unsigned long lastRawDiagnosticPrint = 0;
  unsigned long lastDetectorRearmTime = 0;
  unsigned long rearmPausedAt = 0;
  bool lockedSignal = false;
  bool pulseSensorReady = false;
  bool rearmResumePending = false;
  bool insideBeatWindow = false;
  int signalQuality = 0;
  int qualifiedBeatStreak = 0;
  int unqualifiedBeatStreak = 0;
  int peakToPeakScore = 0;
  int clippedSampleScore = 0;
  int rearmCount = 0;
  bool clippingSinceRangeReset = false;
  const char* lastLockDropReason = "none";
  const char* lastBeatAcceptReason = "none";
  bool rawDiagnosticsHeaderPrinted = false;
  bool rawDiagnosticsBeatPending = false;
  const char* rawDiagnosticsBeatAcceptReason = "none";
};

struct DashboardState {
  DisplayMode displayMode = DISPLAY_MONO_LIGHT;
  bool needsRedraw = true;
  unsigned long lastDraw = 0;
  unsigned long lastWaveSample = 0;
  unsigned long beatPulseStartTime = 0;
  bool beatPulseActive = false;
  int ledBrightness = 0;
  int lastSignalHarmonyQuality = 0;
  int waveformY[GRAPH_WIDTH];
  uint8_t waveformMarker[GRAPH_WIDTH];
  int waveformWriteX = 0;
  bool waveformBeatMarkerPending = false;
  bool waveformBeatMarkerPendingAccepted = false;
};

struct BatteryState {
  bool ready = false;
  int percent = -1;
  float voltage = 0.0f;
  int rawAdc = 0;
  unsigned long lastSample = 0;
};

struct EnvironmentState {
  bool ready = false;
  int temperatureF = 0;
  int humidityPercent = 0;
  unsigned long lastSample = 0;
};

RlcdSt7305 display;
PulseSensorPlayground pulseSensor;
PulseSignalState pulseState;
DashboardState dashboard;
BatteryState batteryState;
EnvironmentState environmentState;
bool batteryAdcReady = false;

int& currentSignal = pulseState.currentSignal;
int& displayBPM = pulseState.displayBPM;
int& displayIBI = pulseState.displayIBI;
int& pulseAmplitude = pulseState.pulseAmplitude;
int& minSignal = pulseState.minSignal;
int& maxSignal = pulseState.maxSignal;
int& activePulseThreshold = pulseState.activePulseThreshold;
unsigned long& lastBeatTime = pulseState.lastBeatTime;
unsigned long& lastQualifiedBeatTime = pulseState.lastQualifiedBeatTime;
unsigned long& lastSerialPrint = pulseState.lastSerialPrint;
unsigned long& lastRawDiagnosticPrint = pulseState.lastRawDiagnosticPrint;
unsigned long& lastDetectorRearmTime = pulseState.lastDetectorRearmTime;
unsigned long& rearmPausedAt = pulseState.rearmPausedAt;
bool& lockedSignal = pulseState.lockedSignal;
bool& pulseSensorReady = pulseState.pulseSensorReady;
bool& rearmResumePending = pulseState.rearmResumePending;
bool& insideBeatWindow = pulseState.insideBeatWindow;
int& signalQuality = pulseState.signalQuality;
int& qualifiedBeatStreak = pulseState.qualifiedBeatStreak;
int& unqualifiedBeatStreak = pulseState.unqualifiedBeatStreak;
int& peakToPeakScore = pulseState.peakToPeakScore;
int& clippedSampleScore = pulseState.clippedSampleScore;
int& rearmCount = pulseState.rearmCount;
bool& clippingSinceRangeReset = pulseState.clippingSinceRangeReset;
const char*& lastLockDropReason = pulseState.lastLockDropReason;
const char*& lastBeatAcceptReason = pulseState.lastBeatAcceptReason;
bool& rawDiagnosticsHeaderPrinted = pulseState.rawDiagnosticsHeaderPrinted;
bool& rawDiagnosticsBeatPending = pulseState.rawDiagnosticsBeatPending;
const char*& rawDiagnosticsBeatAcceptReason = pulseState.rawDiagnosticsBeatAcceptReason;

void setupPulseSensor();
void setupBatteryGauge();
void setupBatteryAdc();
void updateBatteryStatus();
bool readBatteryRegisterWord(uint8_t reg, uint16_t& value);
bool readBatteryAdcVoltage(float& voltage, int& rawAdc);
int batteryPercentFromVoltage(float voltage);
void updateEnvironmentStatus();
bool readShtc3(float& temperatureC, float& humidity);
bool sendI2cCommand(uint8_t address, uint16_t command);
bool shtc3CrcOk(const uint8_t* data, uint8_t checksum);
void drawPulseSensorInitErrorScreen();
void readPulseSensor();
void printRawSignalDiagnostics();
int detectorThresholdForCurrentRange();
void retunePulseDetectorThreshold();
bool isQualifiedBeat(int bpm, int ibi, int amplitude, bool wasLocked);
bool isPlausibleBeatTiming(int bpm, int ibi);
bool isAcquisitionCadenceMatch(int ibi);
bool isLockedCadenceMatch(int ibi);
bool isPeakToPeakCadenceMatch(int ibi);
bool isPeakCadenceRecoveryBeat(int bpm, int ibi, int amplitude);
bool signalIsRecentlyClipped();
bool signalRangeIsMotionArtifact();
bool signalLooksCleanForAcquisition();
bool signalLooksCleanForBeat(bool wasLocked);
int peakToPeakScoreForCurrentSignal();
bool isPeakToPeakCandidateBeat(int bpm, int ibi, int amplitude, bool wasLocked);
BeatDecision decideBeat(int bpm, int ibi, int amplitude, bool wasLocked);
void updateClippingScore();
void resetSignalRangeWindow();
int acquisitionScoreForCurrentSignal();
void updateSignalAcquisitionScore();
void dropSignalLock(const char* reason);
int signalCoachState();
const char* signalCoachText();
int amplitudeMeterSegments(int amplitude);
void maybeRearmDetector();
void rearmPulseDetector(const char* reason);
void resetSignalAcquisitionWindow();
void updatePendingPulseDetectorRearm();
void updateSignalRange();
void triggerRearLedPulse(RearLedColor color);
void triggerBeatEffects();
void startSignalHarmony(int quality);
void stopSignalHarmony();
void captureWaveformSample();
void resetDashboardState();
void drawDashboardIfDue();
void drawDashboard();
void drawHeader();
void drawBatteryStatus();
void drawEnvironmentStatus();
void drawGraphFrame();
void drawWaveformHistory();
void drawPanels();
void drawSignalStrip();
void drawMetricPanel(int x, int y, int w, int h, const char* label, int value, const char* unit, bool valid);
void drawSignalPanel();
void drawQualitySegments(int x, int y);
void drawAmplitudeMeter(int x, int y, int amplitude);
void drawBeatHeart(int centerX, int centerY);
void fillHeartShape(int centerX, int centerY, int size, uint16_t color);
void drawBoldText(const char* text, int x, int y, int textSize, uint16_t color, uint16_t bg);
void drawCenteredText(const char* text, int x, int y, int w, int textSize, uint16_t color, uint16_t bg);
void drawDottedHLine(int x, int y, int w, uint16_t color, int step);
void drawDottedVLine(int x, int y, int h, uint16_t color, int step);
void readButtons();
void cycleDisplayMode();
uint16_t screenBgColor();
uint16_t textColor();
uint16_t panelBgColor();
uint16_t signalSearchColor();
uint16_t signalLockColor();
uint16_t inactiveColor();
const char* displayModeName();
int signalToGraphY(int signal);
int ledPulseEnvelopeBrightness(unsigned long age);
void updateBeatPulse();
void maybePrintSerialStatus();

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("PulseSensor RLCD dashboard prototype");
  Serial.printf("Board=ESP32-S3-RLCD-4.2 display=ST7305 400x300 pulsePin=GPIO%d\n", PULSE_PIN);

  pinMode(PIN_BOOT, INPUT_PULLUP);
  pinMode(PIN_KEY, INPUT_PULLUP);

  bool displayReady = display.begin();
  display.setTextWrap(false);
  if (!displayReady) {
    Serial.println("ST7305 framebuffer allocation failed");
  }

  resetDashboardState();
  setupBatteryGauge();
  setupPulseSensor();
  drawDashboardIfDue();
}

void loop() {
  if (!pulseSensorReady) {
    drawPulseSensorInitErrorScreen();
    delay(250);
    return;
  }

  // SIGNAL FIRST - DO NOT MOVE BELOW UI or button work.
  readPulseSensor();

  captureWaveformSample();
  updateBatteryStatus();
  updateEnvironmentStatus();
  readButtons();
  updateBeatPulse();
  drawDashboardIfDue();
  printRawSignalDiagnostics();
  maybePrintSerialStatus();
}

void setupBatteryGauge() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  setupBatteryAdc();
  updateBatteryStatus();
  updateEnvironmentStatus();
}

void setupBatteryAdc() {
  analogSetPinAttenuation(PIN_BATTERY_ADC, ADC_11db);
  batteryAdcReady = true;
}

void updateBatteryStatus() {
  unsigned long now = millis();
  if (batteryState.lastSample != 0 && now - batteryState.lastSample < BATTERY_SAMPLE_MS) return;
  batteryState.lastSample = now;

  uint16_t stateOfCharge = 0;
  if (readBatteryRegisterWord(BATTERY_SOC_REG, stateOfCharge) && stateOfCharge <= 100) {
    int nextPercent = constrain(static_cast<int>(stateOfCharge), 0, 100);
    if (!batteryState.ready || nextPercent != batteryState.percent || batteryState.voltage != 0.0f) {
      dashboard.needsRedraw = true;
    }
    batteryState.ready = true;
    batteryState.percent = nextPercent;
    batteryState.voltage = 0.0f;
    batteryState.rawAdc = 0;
    Serial.printf("Battery gauge: %d%%\n", batteryState.percent);
    return;
  }

  float adcVoltage = 0.0f;
  int rawAdc = 0;
  if (readBatteryAdcVoltage(adcVoltage, rawAdc)) {
    int nextPercent = batteryPercentFromVoltage(adcVoltage);
    if (!batteryState.ready ||
        nextPercent != batteryState.percent ||
        abs(static_cast<int>((adcVoltage - batteryState.voltage) * 100.0f)) >= 2) {
      dashboard.needsRedraw = true;
      Serial.printf("Battery ADC: raw=%d voltage=%.2fV level=%d%%\n", rawAdc, adcVoltage, nextPercent);
    }
    batteryState.ready = true;
    batteryState.percent = nextPercent;
    batteryState.voltage = adcVoltage;
    batteryState.rawAdc = rawAdc;
    return;
  }

  if (batteryState.ready || batteryState.percent != -1) {
    dashboard.needsRedraw = true;
  }
  batteryState.ready = false;
  batteryState.percent = -1;
}

bool readBatteryRegisterWord(uint8_t reg, uint16_t& value) {
  Wire.beginTransmission(BATTERY_GAUGE_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(BATTERY_GAUGE_ADDR, static_cast<uint8_t>(2)) != 2) return false;

  uint8_t low = Wire.read();
  uint8_t high = Wire.read();
  value = static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
  return true;
}

bool readBatteryAdcVoltage(float& voltage, int& rawAdc) {
  if (!batteryAdcReady) return false;

  int raw = analogRead(PIN_BATTERY_ADC);
  int millivolts = analogReadMilliVolts(PIN_BATTERY_ADC);
  if (millivolts > 0) {
    voltage = 0.001f * static_cast<float>(millivolts) * BATTERY_ADC_SCALE;
  } else {
    int maxRaw = (1 << 10) - 1;
    voltage = (static_cast<float>(raw) / static_cast<float>(maxRaw)) * 3.3f * BATTERY_ADC_SCALE;
  }

  rawAdc = raw;
  return voltage >= 0.05f && voltage <= 5.2f;
}

int batteryPercentFromVoltage(float voltage) {
  if (voltage <= BATTERY_EMPTY_VOLTAGE) return 0;
  if (voltage >= BATTERY_FULL_VOLTAGE) return 100;
  float level = ((voltage - BATTERY_EMPTY_VOLTAGE) / (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE)) * 100.0f;
  return constrain(static_cast<int>(roundf(level)), 0, 100);
}

void updateEnvironmentStatus() {
  unsigned long now = millis();
  if (environmentState.lastSample != 0 && now - environmentState.lastSample < ENVIRONMENT_SAMPLE_MS) return;
  environmentState.lastSample = now;

  float temperatureC = 0.0f;
  float humidity = 0.0f;
  if (readShtc3(temperatureC, humidity)) {
    int nextTempF = static_cast<int>(roundf((temperatureC * 9.0f / 5.0f) + 32.0f));
    int nextHumidity = constrain(static_cast<int>(roundf(humidity)), 0, 100);
    if (!environmentState.ready ||
        nextTempF != environmentState.temperatureF ||
        nextHumidity != environmentState.humidityPercent) {
      dashboard.needsRedraw = true;
    }
    environmentState.ready = true;
    environmentState.temperatureF = nextTempF;
    environmentState.humidityPercent = nextHumidity;
    return;
  }

  if (environmentState.ready) {
    dashboard.needsRedraw = true;
  }
  environmentState.ready = false;
}

bool readShtc3(float& temperatureC, float& humidity) {
  if (!sendI2cCommand(SHTC3_ADDR, SHTC3_WAKEUP)) return false;
  delay(2);
  if (!sendI2cCommand(SHTC3_ADDR, SHTC3_MEASURE_T_RH)) return false;
  delay(20);

  if (Wire.requestFrom(SHTC3_ADDR, static_cast<uint8_t>(6)) != 6) {
    sendI2cCommand(SHTC3_ADDR, SHTC3_SLEEP);
    return false;
  }

  uint8_t bytes[6];
  for (uint8_t i = 0; i < sizeof(bytes); i++) {
    bytes[i] = Wire.read();
  }
  sendI2cCommand(SHTC3_ADDR, SHTC3_SLEEP);

  if (!shtc3CrcOk(bytes, bytes[2]) || !shtc3CrcOk(&bytes[3], bytes[5])) return false;

  uint16_t rawTemp = (static_cast<uint16_t>(bytes[0]) << 8) | bytes[1];
  uint16_t rawHumidity = (static_cast<uint16_t>(bytes[3]) << 8) | bytes[4];
  temperatureC = (175.0f * rawTemp / 65536.0f) - 45.0f - 4.0f;
  humidity = 100.0f * rawHumidity / 65536.0f;
  return true;
}

bool sendI2cCommand(uint8_t address, uint16_t command) {
  Wire.beginTransmission(address);
  Wire.write(static_cast<uint8_t>(command >> 8));
  Wire.write(static_cast<uint8_t>(command & 0xFF));
  return Wire.endTransmission() == 0;
}

bool shtc3CrcOk(const uint8_t* data, uint8_t checksum) {
  uint8_t crc = 0xFF;
  for (uint8_t byteIndex = 0; byteIndex < 2; byteIndex++) {
    crc ^= data[byteIndex];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31) : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc == checksum;
}

void setupPulseSensor() {
  analogReadResolution(10);
  pinMode(PULSE_PIN, INPUT);

  pulseSensor.analogInput(PULSE_PIN);
  activePulseThreshold = PULSE_THRESHOLD;
  pulseSensor.setThreshold(activePulseThreshold);
  pulseSensorReady = pulseSensor.begin();

  unsigned long now = millis();
  lastBeatTime = now;
  lastQualifiedBeatTime = now;
  resetSignalRangeWindow();

  if (!pulseSensorReady) {
    Serial.println("PulseSensor initialization failed");
  }
}

void drawPulseSensorInitErrorScreen() {
  static bool errorDrawn = false;
  if (errorDrawn || !display.ready()) return;
  errorDrawn = true;

  display.clear(screenBgColor());
  display.setTextColor(textColor(), screenBgColor());
  display.setTextSize(3);
  display.setCursor(24, 58);
  display.print("PulseSensor");
  display.setCursor(24, 92);
  display.print("init failed");
  display.setTextSize(2);
  display.setCursor(24, 146);
  display.print("Check signal wire on");
  display.setCursor(24, 170);
  display.printf("GPIO%d, 3.3V, GND", PULSE_PIN);
  display.setTextSize(1);
  display.setCursor(24, 224);
  display.print(APP_VERSION);
  display.display();
}

int detectorThresholdForCurrentRange() {
  int low = constrain(minSignal, 0, 1023);
  int high = constrain(maxSignal, 0, 1023);
  if (high < low) {
    int swapValue = high;
    high = low;
    low = swapValue;
  }

  if (high - low >= SIGNAL_ACQUISITION_MIN_RANGE) {
    return constrain((low + high) / 2, PULSE_DYNAMIC_THRESHOLD_MIN, PULSE_DYNAMIC_THRESHOLD_MAX);
  }

  return PULSE_THRESHOLD;
}

void retunePulseDetectorThreshold() {
  activePulseThreshold = detectorThresholdForCurrentRange();
  pulseSensor.setThreshold(activePulseThreshold);
}

void readPulseSensor() {
  if (rearmResumePending) {
    updatePendingPulseDetectorRearm();
    return;
  }

  currentSignal = pulseSensor.getLatestSample();
  pulseAmplitude = pulseSensor.getPulseAmplitude();
  insideBeatWindow = pulseSensor.isInsideBeat();
  updateClippingScore();
  updateSignalRange();
  peakToPeakScore = peakToPeakScoreForCurrentSignal();
  maybeRearmDetector();
  updateSignalAcquisitionScore();

  if (pulseSensor.sawStartOfBeat()) {
    unsigned long now = millis();
    bool wasLocked = lockedSignal;
    int bpm = pulseSensor.getBeatsPerMinute();
    int ibi = pulseSensor.getInterBeatIntervalMs();
    BeatDecision decision = decideBeat(bpm, ibi, pulseAmplitude, wasLocked);

    lastBeatTime = now;

    if (decision.accepted) {
      displayBPM = bpm;
      displayIBI = ibi;
      lastQualifiedBeatTime = now;
      unqualifiedBeatStreak = 0;
      lastLockDropReason = "none";
      lastBeatAcceptReason = decision.acceptReason;
      qualifiedBeatStreak++;
      if (qualifiedBeatStreak > LOCK_QUALIFIED_BEATS) qualifiedBeatStreak = LOCK_QUALIFIED_BEATS;
    } else if (wasLocked) {
      lastBeatAcceptReason = "reject";
      unqualifiedBeatStreak++;
      if (wasLocked && unqualifiedBeatStreak <= LOCK_GRACE_BAD_BEATS &&
          now - lastQualifiedBeatTime <= LOCK_HOLD_GRACE_MS) {
        qualifiedBeatStreak = LOCK_QUALIFIED_BEATS;
      } else {
        dropSignalLock("grace expired");
      }
    } else {
      lastBeatAcceptReason = "reject";
      qualifiedBeatStreak = 0;
      unqualifiedBeatStreak = 0;
    }

    lockedSignal = qualifiedBeatStreak >= LOCK_QUALIFIED_BEATS;
    updateSignalAcquisitionScore();
    rawDiagnosticsBeatPending = true;
    rawDiagnosticsBeatAcceptReason = lastBeatAcceptReason;
    dashboard.waveformBeatMarkerPending = true;
    dashboard.waveformBeatMarkerPendingAccepted = decision.accepted;

    if (decision.accepted) {
      if (lockedSignal) {
        triggerBeatEffects();
      } else {
        triggerRearLedPulse(REAR_LED_LOCKING);
      }
    }
  }

  unsigned long now = millis();
  if (lockedSignal && now - lastQualifiedBeatTime > LOCK_HOLD_GRACE_MS) {
    dropSignalLock("grace expired");
  }

  if (now - lastQualifiedBeatTime > NO_BEAT_TIMEOUT) {
    dropSignalLock("no beat timeout");
  }
}

void printRawSignalDiagnostics() {
  if (!RAW_SIGNAL_DIAGNOSTICS) return;

  unsigned long now = millis();
  if (!rawDiagnosticsHeaderPrinted) {
    Serial.println("rawDiag,ms,signal,amp,bpm,ibi,locked,quality,p2p,range,clip,inside,beat,accept,drop,qStreak,badStreak");
    rawDiagnosticsHeaderPrinted = true;
  }

  if (now - lastRawDiagnosticPrint < 20 && !rawDiagnosticsBeatPending) return;
  lastRawDiagnosticPrint = now;

  int beat = rawDiagnosticsBeatPending ? 1 : 0;
  const char* acceptReason = rawDiagnosticsBeatPending ? rawDiagnosticsBeatAcceptReason : "none";
  Serial.printf("rawDiag,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%s,%s,%d,%d\n",
                now,
                currentSignal,
                pulseAmplitude,
                displayBPM,
                displayIBI,
                lockedSignal ? 1 : 0,
                signalQuality,
                peakToPeakScore,
                maxSignal - minSignal,
                clippedSampleScore,
                insideBeatWindow ? 1 : 0,
                beat,
                acceptReason,
                lastLockDropReason,
                qualifiedBeatStreak,
                unqualifiedBeatStreak);

  rawDiagnosticsBeatPending = false;
  rawDiagnosticsBeatAcceptReason = "none";
}

bool isQualifiedBeat(int bpm, int ibi, int amplitude, bool wasLocked) {
  if (!isPlausibleBeatTiming(bpm, ibi)) return false;
  if (amplitude < MIN_QUALIFIED_AMPLITUDE) return false;
  if (maxSignal - minSignal < SIGNAL_COACH_FLAT_RANGE) return false;
  if (!signalLooksCleanForBeat(wasLocked)) return false;
  return true;
}

bool isPlausibleBeatTiming(int bpm, int ibi) {
  if (bpm < MIN_QUALIFIED_BPM || bpm > MAX_QUALIFIED_BPM) return false;
  if (ibi < MIN_QUALIFIED_IBI || ibi > MAX_QUALIFIED_IBI) return false;
  return true;
}

bool isAcquisitionCadenceMatch(int ibi) {
  if (qualifiedBeatStreak <= 0 || displayIBI <= 0) return true;
  if (ibi < (displayIBI * ACQUISITION_CADENCE_MIN_IBI_PERCENT) / 100) return false;
  int ibiTolerance = max(PEAK_RECOVERY_MIN_IBI_TOLERANCE_MS,
                         (displayIBI * ACQUISITION_CADENCE_TOLERANCE_PERCENT) / 100);
  return abs(ibi - displayIBI) <= ibiTolerance;
}

bool isLockedCadenceMatch(int ibi) {
  if (displayIBI <= 0) return false;
  int ibiTolerance = max(PEAK_RECOVERY_MIN_IBI_TOLERANCE_MS,
                         (displayIBI * PEAK_RECOVERY_IBI_TOLERANCE_PERCENT) / 100);
  return abs(ibi - displayIBI) <= ibiTolerance;
}

bool isPeakToPeakCadenceMatch(int ibi) {
  if (displayIBI <= 0) return false;
  if (ibi < (displayIBI * PEAK_TO_PEAK_LOCKED_MIN_IBI_PERCENT) / 100) return false;
  int ibiTolerance = max(PEAK_TO_PEAK_LOCKED_MIN_IBI_TOLERANCE_MS,
                         (displayIBI * PEAK_TO_PEAK_LOCKED_IBI_TOLERANCE_PERCENT) / 100);
  return abs(ibi - displayIBI) <= ibiTolerance;
}

bool isPeakCadenceRecoveryBeat(int bpm, int ibi, int amplitude) {
  if (!lockedSignal) return false;
  if (!isPlausibleBeatTiming(bpm, ibi)) return false;
  if (!isLockedCadenceMatch(ibi)) return false;

  int liveRange = maxSignal - minSignal;
  bool signalStillMoving = liveRange >= PEAK_RECOVERY_MIN_RANGE ||
                           amplitude >= PEAK_RECOVERY_MIN_AMPLITUDE;
  if (!signalStillMoving) return false;

  if (!signalLooksCleanForBeat(true)) return false;
  return true;
}

bool signalIsRecentlyClipped() {
  return clippedSampleScore > 18;
}

bool signalRangeIsMotionArtifact() {
  return maxSignal - minSignal > SIGNAL_MOTION_ARTIFACT_RANGE;
}

bool signalLooksCleanForAcquisition() {
  return !signalIsRecentlyClipped() && !signalRangeIsMotionArtifact();
}

bool signalLooksCleanForBeat(bool wasLocked) {
  if (signalRangeIsMotionArtifact()) return false;
  if (!wasLocked) return !signalIsRecentlyClipped();
  return clippedSampleScore <= LOCKED_CLIPPING_SCORE_LIMIT;
}

int peakToPeakScoreForCurrentSignal() {
  if (!signalLooksCleanForAcquisition()) return 0;

  int liveRange = maxSignal - minSignal;
  int rangeScore = map(constrain(liveRange,
                                 PEAK_TO_PEAK_MIN_RANGE,
                                 PEAK_TO_PEAK_STRONG_RANGE),
                       PEAK_TO_PEAK_MIN_RANGE,
                       PEAK_TO_PEAK_STRONG_RANGE,
                       0,
                       4);
  int amplitudeScore = map(constrain(pulseAmplitude,
                                     PEAK_TO_PEAK_MIN_AMPLITUDE,
                                     AMPLITUDE_METER_MAX),
                           PEAK_TO_PEAK_MIN_AMPLITUDE,
                           AMPLITUDE_METER_MAX,
                           0,
                           3);
  int cleanScore = clippedSampleScore <= 4 ? 2 : (clippedSampleScore <= 18 ? 1 : 0);
  int beatWindowScore = insideBeatWindow ? 1 : 0;
  int score = rangeScore + amplitudeScore + cleanScore + beatWindowScore;

  if (liveRange < PEAK_TO_PEAK_MIN_RANGE && pulseAmplitude < PEAK_TO_PEAK_MIN_AMPLITUDE) {
    score = 0;
  }

  return constrain(score, 0, 10);
}

bool isPeakToPeakCandidateBeat(int bpm, int ibi, int amplitude, bool wasLocked) {
  if (!isPlausibleBeatTiming(bpm, ibi)) return false;
  if (!signalLooksCleanForBeat(wasLocked)) return false;

  int requiredScore = wasLocked ? PEAK_TO_PEAK_LOCKED_MIN_SCORE : PEAK_TO_PEAK_ACQUIRE_MIN_SCORE;
  if (peakToPeakScore < requiredScore) return false;

  if (wasLocked) return isPeakToPeakCadenceMatch(ibi);
  if (qualifiedBeatStreak < PEAK_TO_PEAK_PRELOCK_CADENCE_MIN_STREAK &&
      peakToPeakScore < PEAK_TO_PEAK_FIRST_BEAT_SCORE) {
    return false;
  }
  if (!isAcquisitionCadenceMatch(ibi)) return false;
  return amplitude >= PEAK_TO_PEAK_MIN_AMPLITUDE || (maxSignal - minSignal) >= PEAK_TO_PEAK_MIN_RANGE;
}

BeatDecision decideBeat(int bpm, int ibi, int amplitude, bool wasLocked) {
  BeatDecision decision;
  decision.qualified = isQualifiedBeat(bpm, ibi, amplitude, wasLocked);
  decision.strictAccepted = decision.qualified;
  decision.peakToPeakAccepted = PEAK_TO_PEAK_RECOVERY_ENABLED &&
                                !decision.strictAccepted &&
                                isPeakToPeakCandidateBeat(bpm, ibi, amplitude, wasLocked);
  decision.recovered = !decision.strictAccepted &&
                       !decision.peakToPeakAccepted &&
                       wasLocked &&
                       isPeakCadenceRecoveryBeat(bpm, ibi, amplitude);
  decision.accepted = decision.strictAccepted || decision.peakToPeakAccepted || decision.recovered;
  decision.acceptReason = decision.accepted ?
                          (decision.strictAccepted ? "strict" :
                           (decision.peakToPeakAccepted ? "peak2peak" : "peak-cadence")) :
                          "reject";
  return decision;
}

void updateClippingScore() {
  static unsigned long lastClipDecayMs = 0;
  unsigned long now = millis();
  bool clipped = currentSignal <= 8 || currentSignal >= 1015;

  if (clipped) {
    clippingSinceRangeReset = true;
    clippedSampleScore += 8;
    if (clippedSampleScore > 100) clippedSampleScore = 100;
    lastClipDecayMs = now;
    return;
  }

  if (clippedSampleScore > 0 && now - lastClipDecayMs >= CLIPPING_SCORE_DECAY_MS) {
    clippedSampleScore--;
    lastClipDecayMs = now;
    if (clippedSampleScore == 0 && clippingSinceRangeReset) {
      resetSignalRangeWindow();
    }
  }
}

int acquisitionScoreForCurrentSignal() {
  if (lockedSignal) return SIGNAL_QUALITY_STEPS;
  if (!signalLooksCleanForAcquisition()) return 0;

  int liveRange = maxSignal - minSignal;
  int rangeScore = map(constrain(liveRange,
                                 SIGNAL_ACQUISITION_MIN_RANGE,
                                 SIGNAL_ACQUISITION_FULL_RANGE),
                       SIGNAL_ACQUISITION_MIN_RANGE,
                       SIGNAL_ACQUISITION_FULL_RANGE,
                       0,
                       5);
  int amplitudeScore = map(constrain(pulseAmplitude,
                                     SIGNAL_COACH_FLAT_AMPLITUDE,
                                     AMPLITUDE_METER_MAX),
                           SIGNAL_COACH_FLAT_AMPLITUDE,
                           AMPLITUDE_METER_MAX,
                           0,
                           4);
  int cleanScore = clippedSampleScore <= 4 ? 2 : (clippedSampleScore <= 18 ? 1 : 0);
  int beatWindowScore = insideBeatWindow ? 1 : 0;
  int streakScore = qualifiedBeatStreak * 2;
  int peakScore = PEAK_TO_PEAK_RECOVERY_ENABLED ? min(2, peakToPeakScore / 3) : 0;
  int score = rangeScore + amplitudeScore + cleanScore + beatWindowScore + streakScore + peakScore;

  if (liveRange < SIGNAL_ACQUISITION_MIN_RANGE && pulseAmplitude < SIGNAL_COACH_FLAT_AMPLITUDE) {
    score = min(score, 2);
  }

  return constrain(score, 0, SIGNAL_ACQUISITION_MAX_SCORE_BEFORE_LOCK);
}

void updateSignalAcquisitionScore() {
  int previousQuality = signalQuality;
  signalQuality = acquisitionScoreForCurrentSignal();

  if (signalQuality <= 1) {
    dashboard.lastSignalHarmonyQuality = 0;
  }

  if (!lockedSignal &&
      signalQuality > previousQuality &&
      signalQuality > dashboard.lastSignalHarmonyQuality) {
    startSignalHarmony(signalQuality);
    dashboard.lastSignalHarmonyQuality = signalQuality;
  }
}

void dropSignalLock(const char* reason) {
  bool hadSignalState = lockedSignal ||
                        displayBPM > 0 ||
                        displayIBI > 0 ||
                        qualifiedBeatStreak > 0 ||
                        unqualifiedBeatStreak > 0;
  if (hadSignalState) {
    lastLockDropReason = reason;
  }

  lockedSignal = false;
  qualifiedBeatStreak = 0;
  unqualifiedBeatStreak = 0;
  lastBeatAcceptReason = "none";
  displayBPM = 0;
  displayIBI = 0;
  updateSignalAcquisitionScore();
}

int signalCoachState() {
  int liveRange = maxSignal - minSignal;

  if (lockedSignal) return COACH_QUALIFIED;
  if (!signalLooksCleanForAcquisition()) return COACH_CLIPPED;
  if (liveRange < SIGNAL_COACH_FLAT_RANGE || pulseAmplitude < SIGNAL_COACH_FLAT_AMPLITUDE) {
    return COACH_TOO_FLAT;
  }
  if (pulseAmplitude < SIGNAL_COACH_STEADY_AMPLITUDE) return COACH_HOLD_STEADY;
  if (signalQuality >= LOCK_QUALITY_STEPS / 2) return COACH_LOCKING;
  if (liveRange >= REARM_SIGNAL_RANGE) return COACH_GOOD_WAVE;
  return COACH_SIGNAL_SEARCH;
}

const char* signalCoachText() {
  switch (signalCoachState()) {
    case COACH_QUALIFIED:
      return "QUALIFIED BEAT";
    case COACH_CLIPPED:
      return "ADJUST SENSOR";
    case COACH_TOO_FLAT:
      return "TOO FLAT";
    case COACH_HOLD_STEADY:
      return "HOLD STEADY";
    case COACH_GOOD_WAVE:
      return "GOOD WAVE";
    case COACH_LOCKING:
      return "LOCKING";
    default:
      return "SIGNAL SEARCH";
  }
}

int amplitudeMeterSegments(int amplitude) {
  amplitude = constrain(amplitude, 0, AMPLITUDE_METER_MAX);
  return map(amplitude, 0, AMPLITUDE_METER_MAX, 0, 10);
}

void maybeRearmDetector() {
  unsigned long now = millis();
  int liveRange = maxSignal - minSignal;
  bool signalLooksAlive = liveRange >= REARM_SIGNAL_RANGE && !signalIsRecentlyClipped();
  bool detectorIsQuiet = (now - lastBeatTime) >= REARM_NO_BEAT_MS;
  bool rearmCooledDown = (now - lastDetectorRearmTime) >= REARM_COOLDOWN_MS;

  if (!lockedSignal && signalLooksAlive && detectorIsQuiet && rearmCooledDown) {
    rearmPulseDetector("alive signal without beat event");
    resetSignalAcquisitionWindow();
  }
}

void rearmPulseDetector(const char* reason) {
  Serial.print("Re-arming PulseSensor detector: ");
  Serial.println(reason);

  retunePulseDetectorThreshold();
  pulseSensor.pause();
  rearmPausedAt = millis();
  rearmResumePending = true;

  lastDetectorRearmTime = millis();
  lastBeatTime = millis();
  lastQualifiedBeatTime = millis();
  signalQuality = 0;
  dashboard.lastSignalHarmonyQuality = 0;
  qualifiedBeatStreak = 0;
  unqualifiedBeatStreak = 0;
  peakToPeakScore = 0;
  displayBPM = 0;
  displayIBI = 0;
  lockedSignal = false;
  lastLockDropReason = reason;
  lastBeatAcceptReason = "none";
  rearmCount++;
}

void updatePendingPulseDetectorRearm() {
  if (!rearmResumePending) return;
  if (millis() - rearmPausedAt < 8) return;
  pulseSensor.resume();
  rearmResumePending = false;
}

void resetSignalAcquisitionWindow() {
  stopSignalHarmony();
  resetSignalRangeWindow();
  clippedSampleScore = 0;
  peakToPeakScore = 0;
  insideBeatWindow = false;
}

void resetSignalRangeWindow() {
  minSignal = currentSignal - 40;
  maxSignal = currentSignal + 40;
  clippingSinceRangeReset = false;
}

void updateSignalRange() {
  static unsigned long lastDecay = 0;

  if (millis() - lastDecay >= 100) {
    lastDecay = millis();
    minSignal = min(minSignal + 4, currentSignal);
    maxSignal = max(maxSignal - 4, currentSignal);
  }

  minSignal = min(minSignal, currentSignal);
  maxSignal = max(maxSignal, currentSignal);

  if (maxSignal - minSignal < 80) {
    int center = currentSignal;
    minSignal = center - 40;
    maxSignal = center + 40;
  }
}

void triggerRearLedPulse(RearLedColor color) {
  (void)color;
  dashboard.beatPulseStartTime = millis();
  dashboard.beatPulseActive = true;
  dashboard.ledBrightness = 200;
}

void triggerBeatEffects() {
  triggerRearLedPulse(REAR_LED_HEARTBEAT);
  dashboard.ledBrightness = 255;
}

void startSignalHarmony(int quality) {
  (void)quality;
}

void stopSignalHarmony() {
}

void captureWaveformSample() {
  unsigned long now = millis();
  if (now - dashboard.lastWaveSample < WAVEFORM_SAMPLE_MS) return;
  dashboard.lastWaveSample = now;

  dashboard.waveformY[dashboard.waveformWriteX] = signalToGraphY(currentSignal);
  if (dashboard.waveformBeatMarkerPending) {
    dashboard.waveformMarker[dashboard.waveformWriteX] =
        dashboard.waveformBeatMarkerPendingAccepted ? 2 : 1;
    dashboard.waveformBeatMarkerPending = false;
  } else {
    dashboard.waveformMarker[dashboard.waveformWriteX] = 0;
  }

  dashboard.waveformWriteX++;
  if (dashboard.waveformWriteX >= GRAPH_WIDTH) {
    dashboard.waveformWriteX = 0;
  }
}

void resetDashboardState() {
  int centerY = GRAPH_TOP + GRAPH_HEIGHT / 2;
  for (int i = 0; i < GRAPH_WIDTH; i++) {
    dashboard.waveformY[i] = centerY;
    dashboard.waveformMarker[i] = 0;
  }
  dashboard.waveformWriteX = 0;
  dashboard.waveformBeatMarkerPending = false;
  dashboard.waveformBeatMarkerPendingAccepted = false;
  dashboard.needsRedraw = true;
}

void drawDashboardIfDue() {
  if (!display.ready()) return;
  unsigned long now = millis();
  if (!dashboard.needsRedraw && now - dashboard.lastDraw < DASHBOARD_DRAW_MS) return;
  dashboard.lastDraw = now;
  dashboard.needsRedraw = false;
  drawDashboard();
  display.display();
}

void drawDashboard() {
  display.clear(screenBgColor());
  drawHeader();
  drawGraphFrame();
  drawWaveformHistory();
  drawSignalStrip();
  drawPanels();
}

void drawHeader() {
  uint16_t bg = screenBgColor();
  uint16_t fg = textColor();

  char statusRow[64];
  snprintf(statusRow, sizeof(statusRow), "PulseSensor.com | %s", APP_VERSION);
  display.fillRect(12, 8, 274, 18, bg);
  display.setTextSize(1);
  display.setTextColor(fg, bg);
  display.setCursor(14, 13);
  display.print(statusRow);
  drawBatteryStatus();
  drawEnvironmentStatus();

  drawBeatHeart(226, 26);
}

void drawEnvironmentStatus() {
  uint16_t fg = textColor();
  uint16_t bg = screenBgColor();

  char envText[24];
  if (environmentState.ready) {
    snprintf(envText,
             sizeof(envText),
             "DEV TMP %dF  HUM %d%%",
             environmentState.temperatureF,
             environmentState.humidityPercent);
  } else {
    snprintf(envText, sizeof(envText), "DEV TMP --F  HUM --%%");
  }

  const int envRight = 386;
  int envW = strlen(envText) * 6;
  display.fillRect(236, 27, 152, 11, bg);
  drawBoldText(envText, envRight - envW, 29, 1, fg, bg);
}

void drawBatteryStatus() {
  uint16_t fg = textColor();
  uint16_t bg = screenBgColor();
  int percent = batteryState.ready ? batteryState.percent : -1;

  const int iconX = 362;
  const int iconY = 11;
  const int iconW = 24;
  const int iconH = 12;
  const int nubW = 3;

  display.fillRect(286, iconY - 2, 102, iconH + 4, bg);

  char batteryText[16];
  if (percent >= 0) {
    snprintf(batteryText, sizeof(batteryText), "DEV BAT %d%%", percent);
  } else {
    snprintf(batteryText, sizeof(batteryText), "DEV BAT --%%");
  }
  drawBoldText(batteryText, 286, 13, 1, fg, bg);

  display.drawRect(iconX, iconY, iconW, iconH, fg);
  display.fillRect(iconX + iconW, iconY + 3, nubW, iconH - 6, fg);

  if (percent >= 0) {
    int fillW = map(percent, 0, 100, 0, iconW - 4);
    if (percent > 0 && fillW == 0) fillW = 1;
    display.fillRect(iconX + 2, iconY + 2, fillW, iconH - 4, fg);
  } else {
    display.drawFastHLine(iconX + 5, iconY + iconH / 2, iconW - 10, fg);
  }
}

void drawGraphFrame() {
  uint16_t fg = textColor();
  uint16_t bg = screenBgColor();

  display.drawRect(GRAPH_LEFT - 4, GRAPH_TOP - 6, GRAPH_WIDTH + 8, GRAPH_HEIGHT + 10, fg);

  for (int x = 46; x < GRAPH_WIDTH; x += 46) {
    drawDottedVLine(GRAPH_LEFT + x, GRAPH_TOP, GRAPH_HEIGHT, fg, 7);
  }
  for (int y = 32; y < GRAPH_HEIGHT; y += 32) {
    drawDottedHLine(GRAPH_LEFT, GRAPH_TOP + y, GRAPH_WIDTH, fg, 7);
  }

  int thresholdY = signalToGraphY(activePulseThreshold);
  for (int x = 0; x < GRAPH_WIDTH; x += 6) {
    display.drawFastVLine(GRAPH_LEFT + x, thresholdY - 1, 3, fg);
  }

  display.setTextColor(fg, bg);
  drawBoldText("LIVE", GRAPH_LEFT + 8, GRAPH_TOP + 7, GRAPH_LABEL_TEXT_SIZE, fg, bg);

  char thresholdText[12];
  snprintf(thresholdText, sizeof(thresholdText), "THR%d", activePulseThreshold);
  int thresholdW = strlen(thresholdText) * 6 * GRAPH_LABEL_TEXT_SIZE;
  drawBoldText(thresholdText,
               GRAPH_LEFT + GRAPH_WIDTH - thresholdW - 8,
               GRAPH_TOP + 7,
               GRAPH_LABEL_TEXT_SIZE,
               fg,
               bg);

  const char* status = signalCoachText();
  int statusW = strlen(status) * 6 * GRAPH_LABEL_TEXT_SIZE;
  int statusX = GRAPH_LEFT + GRAPH_WIDTH - statusW - 10;
  int statusY = GRAPH_TOP + GRAPH_HEIGHT - 22;
  display.fillRect(statusX - 4, statusY - 2, statusW + 8, 20, bg);
  drawBoldText(status, statusX, statusY, GRAPH_LABEL_TEXT_SIZE, fg, bg);
}

void drawWaveformHistory() {
  uint16_t fg = textColor();
  uint16_t trace = lockedSignal ? signalLockColor() : signalSearchColor();

  for (int i = 1; i < GRAPH_WIDTH; i++) {
    if (i == dashboard.waveformWriteX) continue;
    int x0 = GRAPH_LEFT + i - 1;
    int x1 = GRAPH_LEFT + i;
    int y0 = dashboard.waveformY[i - 1];
    int y1 = dashboard.waveformY[i];
    display.drawLine(x0, y0, x1, y1, trace);
    if (lockedSignal) {
      display.drawLine(x0, y0 + 1, x1, y1 + 1, trace);
    }
  }

  for (int i = 0; i < GRAPH_WIDTH; i++) {
    uint8_t marker = dashboard.waveformMarker[i];
    if (marker == 0) continue;
    int x = GRAPH_LEFT + i;
    int y = dashboard.waveformY[i];
    if (marker == 2) {
      display.fillCircle(x, y, 5, fg);
    } else {
      display.drawCircle(x, y, 3, fg);
      display.drawPixel(x, y, fg);
    }
  }

  int cursorX = GRAPH_LEFT + dashboard.waveformWriteX;
  display.drawFastVLine(cursorX, GRAPH_TOP + 1, GRAPH_HEIGHT - 2, fg);
}

void drawPanels() {
  drawMetricPanel(BPM_PANEL_X, PANEL_Y, METRIC_PANEL_W, PANEL_H, "BPM", displayBPM, "", lockedSignal);
  drawMetricPanel(IBI_PANEL_X, PANEL_Y, METRIC_PANEL_W, PANEL_H, "IBI", displayIBI, "ms", lockedSignal);
}

void drawSignalStrip() {
  uint16_t fg = textColor();
  uint16_t bg = screenBgColor();
  uint16_t signalColor = lockedSignal ? signalLockColor() : signalSearchColor();

  display.fillRect(GRAPH_LEFT - 4, SIGNAL_STRIP_Y - 2, GRAPH_WIDTH + 8, SIGNAL_STRIP_H + 4, bg);
  display.drawRect(GRAPH_LEFT - 4, SIGNAL_STRIP_Y - 2, GRAPH_WIDTH + 8, SIGNAL_STRIP_H + 4, fg);

  char signalText[32];
  snprintf(signalText,
           sizeof(signalText),
           "SIG GP%d | %s | A%03d",
           PULSE_PIN,
           lockedSignal ? "LOCK" : "SEARCH",
           constrain(pulseAmplitude, 0, 999));
  drawBoldText(signalText, GRAPH_LEFT + 6, SIGNAL_STRIP_Y + 6, 1, fg, bg);

  const int tickCount = 6;
  const int tickW = 12;
  const int tickH = 8;
  const int tickGap = 4;
  const int ticksW = tickCount * tickW + (tickCount - 1) * tickGap;
  const int ticksX = GRAPH_LEFT + GRAPH_WIDTH - ticksW - 6;
  const int ticksY = SIGNAL_STRIP_Y + 6;
  int activeTicks = constrain((signalQuality + 1) / 2, 0, tickCount);
  for (int i = 0; i < tickCount; i++) {
    int x = ticksX + i * (tickW + tickGap);
    if (i < activeTicks) {
      display.fillRect(x, ticksY, tickW, tickH, signalColor);
    } else {
      display.drawRect(x, ticksY, tickW, tickH, inactiveColor());
    }
  }
}

void drawMetricPanel(int x, int y, int w, int h, const char* label, int value, const char* unit, bool valid) {
  uint16_t fg = textColor();
  uint16_t bg = screenBgColor();

  display.drawRect(x, y, w, h, fg);
  display.setTextColor(fg, bg);
  drawBoldText(label, x + 10, y + 8, LABEL_TEXT_SIZE, fg, bg);

  char valueText[8];
  if (valid) {
    snprintf(valueText, sizeof(valueText), "%d", value);
  } else {
    snprintf(valueText, sizeof(valueText), "--");
  }

  int valueSize = strcmp(label, "IBI") == 0 ? 3 : 4;
  int valueY = strcmp(label, "IBI") == 0 ? y + 31 : y + 25;
  int valueW = strlen(valueText) * 6 * valueSize;
  int valueX = x + max(0, (w - valueW) / 2);
  drawBoldText(valueText, valueX, valueY, valueSize, fg, bg);

  if (valid && unit[0] != '\0') {
    drawBoldText(unit, x + w - 31, y + 10, 1, fg, bg);
  }
}

void drawSignalPanel() {
  uint16_t fg = textColor();
  uint16_t bg = screenBgColor();

  display.setTextColor(fg, bg);
  char signalLabel[24];
  snprintf(signalLabel,
           sizeof(signalLabel),
           "GP%d %s A%03d",
           PULSE_PIN,
           lockedSignal ? "LOCK" : "SEARCH",
           constrain(pulseAmplitude, 0, 999));
  drawBoldText(signalLabel, SIGNAL_PANEL_X, PANEL_Y + 7, 1, fg, bg);
  drawQualitySegments(SIGNAL_PANEL_X, PANEL_Y + 32);
  drawAmplitudeMeter(SIGNAL_PANEL_X, PANEL_Y + 56, pulseAmplitude);
}

void drawQualitySegments(int x, int y) {
  uint16_t fg = lockedSignal ? signalLockColor() : signalSearchColor();
  for (int i = 0; i < SIGNAL_QUALITY_STEPS; i++) {
    int bx = x + i * 8;
    if (i < signalQuality) {
      display.fillRect(bx, y, 6, 16, fg);
    }
  }
}

void drawAmplitudeMeter(int x, int y, int amplitude) {
  uint16_t fg = lockedSignal ? signalLockColor() : signalSearchColor();
  int segments = amplitudeMeterSegments(amplitude);
  for (int i = 0; i < 10; i++) {
    int bx = x + i * 5;
    if (i < segments) {
      display.fillRect(bx, y, 3, 7, fg);
    } else {
      display.drawRect(bx, y, 3, 7, inactiveColor());
    }
  }
}

void drawBeatHeart(int centerX, int centerY) {
  uint16_t fg = textColor();
  int size = map(dashboard.ledBrightness, 0, 255, HEART_MIN_SIZE, HEART_MAX_SIZE);
  size = constrain(size, HEART_MIN_SIZE, HEART_MAX_SIZE);

  fillHeartShape(centerX, centerY, size + 2, fg);
  if (dashboard.ledBrightness > 18 || lockedSignal) {
    fillHeartShape(centerX, centerY, size, fg);
  } else {
    fillHeartShape(centerX, centerY, size, screenBgColor());
  }
}

void fillHeartShape(int centerX, int centerY, int size, uint16_t color) {
  display.fillCircle(centerX - size / 2, centerY - size / 3, size / 2, color);
  display.fillCircle(centerX + size / 2, centerY - size / 3, size / 2, color);
  display.fillTriangle(centerX - size, centerY - size / 4,
                       centerX + size, centerY - size / 4,
                       centerX, centerY + size, color);
}

void drawBoldText(const char* text, int x, int y, int textSize, uint16_t color, uint16_t bg) {
  display.setTextSize(textSize);
  display.setTextColor(color, bg);
  display.setCursor(x, y);
  display.print(text);
  display.setTextColor(color);
  display.setCursor(x + 1, y);
  display.print(text);
  display.setTextColor(color, bg);
}

void drawCenteredText(const char* text, int x, int y, int w, int textSize, uint16_t color, uint16_t bg) {
  int textW = strlen(text) * 6 * textSize;
  int cursorX = x + max(0, (w - textW) / 2);
  display.setTextSize(textSize);
  display.setTextColor(color, bg);
  display.setCursor(cursorX, y);
  display.print(text);
}

void drawDottedHLine(int x, int y, int w, uint16_t color, int step) {
  for (int px = x; px < x + w; px += step) {
    display.drawPixel(px, y, color);
  }
}

void drawDottedVLine(int x, int y, int h, uint16_t color, int step) {
  for (int py = y; py < y + h; py += step) {
    display.drawPixel(x, py, color);
  }
}

void readButtons() {
  static bool keyWasPressed = false;
  static unsigned long lastKeyChange = 0;

  bool keyPressed = digitalRead(PIN_KEY) == LOW;
  unsigned long now = millis();
  if (keyPressed && !keyWasPressed && now - lastKeyChange > 180) {
    cycleDisplayMode();
    lastKeyChange = now;
  }
  keyWasPressed = keyPressed;
}

void cycleDisplayMode() {
  dashboard.displayMode =
      dashboard.displayMode == DISPLAY_MONO_LIGHT ? DISPLAY_MONO_DARK : DISPLAY_MONO_LIGHT;
  dashboard.needsRedraw = true;
}

uint16_t screenBgColor() {
  return dashboard.displayMode == DISPLAY_MONO_LIGHT ? RLCD_WHITE : RLCD_BLACK;
}

uint16_t textColor() {
  return dashboard.displayMode == DISPLAY_MONO_LIGHT ? RLCD_BLACK : RLCD_WHITE;
}

uint16_t panelBgColor() {
  return screenBgColor();
}

uint16_t signalSearchColor() {
  return textColor();
}

uint16_t signalLockColor() {
  return textColor();
}

uint16_t inactiveColor() {
  return textColor();
}

const char* displayModeName() {
  switch (dashboard.displayMode) {
    case DISPLAY_MONO_DARK:
      return "M DARK";
    case DISPLAY_MONO_LIGHT:
      return "M LIGHT";
    default:
      return "MONO";
  }
}

int signalToGraphY(int signal) {
  if (minSignal == maxSignal) {
    return GRAPH_TOP + GRAPH_HEIGHT / 2;
  }

  int y = map(signal, minSignal, maxSignal, GRAPH_TOP + GRAPH_HEIGHT - 8, GRAPH_TOP + 8);
  return constrain(y, GRAPH_TOP + 8, GRAPH_TOP + GRAPH_HEIGHT - 8);
}

int ledPulseEnvelopeBrightness(unsigned long age) {
  if (age <= LED_PEAK_HOLD_MS) return 255;

  unsigned long fadeAge = age - LED_PEAK_HOLD_MS;
  if (fadeAge >= LED_FADE_MS) return 0;

  uint32_t progress = (fadeAge * 255UL) / LED_FADE_MS;
  uint32_t smooth = (progress * progress * (765UL - (2UL * progress))) / (255UL * 255UL);
  return 255 - smooth;
}

void updateBeatPulse() {
  if (!dashboard.beatPulseActive) {
    dashboard.ledBrightness = lockedSignal ? 72 : 0;
    return;
  }

  unsigned long age = millis() - dashboard.beatPulseStartTime;
  dashboard.ledBrightness = ledPulseEnvelopeBrightness(age);
  if (dashboard.ledBrightness == 0) {
    dashboard.beatPulseActive = false;
  }
}

void maybePrintSerialStatus() {
  if (RAW_SIGNAL_DIAGNOSTICS) return;
  if (millis() - lastSerialPrint < 500) return;
  lastSerialPrint = millis();
  Serial.printf("signal=%d amp=%d bpm=%d ibi=%d locked=%d quality=%d p2p=%d range=%d clip=%d qStreak=%d badStreak=%d accept=%s drop=%s mode=%s\n",
                currentSignal, pulseAmplitude, displayBPM, displayIBI,
                lockedSignal ? 1 : 0, signalQuality, peakToPeakScore,
                maxSignal - minSignal, clippedSampleScore,
                qualifiedBeatStreak, unqualifiedBeatStreak,
                lastBeatAcceptReason,
                lastLockDropReason,
                displayModeName());
}

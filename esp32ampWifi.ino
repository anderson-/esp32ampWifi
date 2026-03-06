#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "esp32-hal-rmt.h"
#include "esp_timer.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = STASSID;
const char* password = STAPSK;

constexpr uint8_t PIN_OFFICE = 2;
constexpr uint8_t PIN_LIVINGROOM = 3;
constexpr uint8_t PIN_AUDIO = 4;
constexpr uint8_t PIN_LED = 10;

constexpr uint32_t AUDIO_TICK_US = 100000;
constexpr uint32_t POWER_GUARD_MS = 2000;
constexpr uint32_t CONNECTED_PULSE_MS = 700;
constexpr uint32_t EVENT_BOOST_MS = 10;
constexpr uint32_t WEB_FLASH_MS = 180;
constexpr uint32_t DEFAULT_STANDBY_SEC = 15 * 60;

WebServer server(80);
Preferences preferences;

enum class ZoneState : uint8_t {
  Mute,
  LivingRoom,
  Office,
  Both,
};

struct RgbColor {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
  
  RgbColor() : red(0), green(0), blue(0) {}
  RgbColor(uint8_t r, uint8_t g, uint8_t b) : red(r), green(g), blue(b) {}
  
  bool operator==(const RgbColor& other) const {
    return red == other.red && green == other.green && blue == other.blue;
  }
  
  bool operator!=(const RgbColor& other) const {
    return !(*this == other);
  }
};

void writeRgbPixel(uint8_t pin, uint8_t red, uint8_t green, uint8_t blue, uint8_t address = 0) {
  static int initializedPin = -1;
  static bool rgbReady = false;
  if (initializedPin != pin) {
    rgbReady = rmtInit(pin, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000UL);
    if (rgbReady) {
      rmtSetEOT(pin, LOW);
      initializedPin = pin;
    }
  }
  if (!rgbReady) return;

  rmt_data_t symbols[24];
  const rmt_data_t val[2] = {rmt_data_t{8, 1, 5, 0}, rmt_data_t{4, 1, 9, 0}};
  uint8_t symbolIndex = 0;
  for (uint8_t channel : {green, red, blue})
    for (uint8_t bitMask = 0x80; bitMask != 0; bitMask >>= 1)
      symbols[symbolIndex++] = val[!(channel & bitMask)];
  rmtWrite(pin, symbols, RMT_SYMBOLS_OF(symbols), RMT_WAIT_FOR_EVER);
}

struct DetectionSnapshot {
  bool audioPresent;
  bool inputPulse;
  uint32_t lastInputMs;
};

// Forward declarations
const char* toString(ZoneState state);
bool parseZoneState(const String& value, ZoneState& out);

const char* toString(ZoneState state) {
  switch (state) {
    case ZoneState::LivingRoom: return "livingroom";
    case ZoneState::Office: return "office";
    case ZoneState::Both: return "both";
    default: return "mute";
  }
}

bool parseZoneState(const String& value, ZoneState& out) {
  if (value == "livingroom") {
    out = ZoneState::LivingRoom;
    return true;
  }
  if (value == "office") {
    out = ZoneState::Office;
    return true;
  }
  if (value == "both") {
    out = ZoneState::Both;
    return true;
  }
  if (value == "mute") {
    out = ZoneState::Mute;
    return true;
  }
  return false;
}

class AudioDetector {
 public:
  void begin() {
    instance_ = this;
    pinMode(PIN_AUDIO, INPUT);
    for (uint8_t i = 0; i < kWindowSize; ++i) {
      window_[i] = 0;
    }

    esp_timer_create_args_t timerArgs = {};
    timerArgs.callback = &AudioDetector::onTimerThunk;
    timerArgs.arg = this;
    timerArgs.dispatch_method = ESP_TIMER_TASK;
    timerArgs.name = "audio_detector";
    if (esp_timer_create(&timerArgs, &timer_) == ESP_OK) {
      esp_timer_start_periodic(timer_, AUDIO_TICK_US);
    }
  }

  DetectionSnapshot snapshot() {
    DetectionSnapshot result;
    portENTER_CRITICAL(&lock_);
    result.audioPresent = audioPresent_;
    result.inputPulse = inputPulseLatched_;
    result.lastInputMs = lastInputMs_;
    inputPulseLatched_ = false;
    portEXIT_CRITICAL(&lock_);
    return result;
  }

 private:
  static constexpr uint8_t kWindowSize = 10;
  static AudioDetector* instance_;

  volatile uint8_t window_[kWindowSize] = {0};
  volatile uint8_t windowIndex_ = 0;
  volatile uint8_t windowSum_ = 0;
  volatile bool interruptEnabled_ = false;
  volatile bool disableInterruptRequested_ = false;
  volatile bool rawPulseLatched_ = false;
  volatile bool inputPulseLatched_ = false;
  volatile bool audioPresent_ = false;
  volatile uint32_t lastInputMs_ = 0;
  esp_timer_handle_t timer_ = nullptr;
  portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;

  static void IRAM_ATTR onAudioInterrupt() {
    if (instance_ != nullptr) {
      instance_->handleInterrupt();
    }
  }

  static void onTimerThunk(void* arg) {
    static_cast<AudioDetector*>(arg)->onTimer();
  }

  void IRAM_ATTR handleInterrupt() {
    portENTER_CRITICAL_ISR(&lock_);
    const uint8_t idx = windowIndex_;
    if (window_[idx] == 0) {
      window_[idx] = 1;
      ++windowSum_;
    }
    lastInputMs_ = static_cast<uint32_t>(micros() / 1000ULL);
    rawPulseLatched_ = true;
    disableInterruptRequested_ = true;
    portEXIT_CRITICAL_ISR(&lock_);
  }

  void onTimer() {
    portENTER_CRITICAL(&lock_);

    if (disableInterruptRequested_) {
      detachInterrupt(PIN_AUDIO);
      interruptEnabled_ = false;
      disableInterruptRequested_ = false;
    }

    const uint8_t nextIndex = (windowIndex_ + 1) % kWindowSize;
    const uint8_t oldValue = window_[nextIndex];
    if (oldValue > 0) {
      windowSum_ -= oldValue;
    }
    window_[nextIndex] = 0;
    windowIndex_ = nextIndex;

    if (!interruptEnabled_) {
      attachInterrupt(PIN_AUDIO, &AudioDetector::onAudioInterrupt, RISING);
      interruptEnabled_ = true;
    }

    const bool nextAudioPresent = windowSum_ >= 2;
    const bool nextInputPulse = rawPulseLatched_;
    rawPulseLatched_ = false;
    inputPulseLatched_ = inputPulseLatched_ || nextInputPulse;
    audioPresent_ = nextAudioPresent;

    portEXIT_CRITICAL(&lock_);
  }
};

AudioDetector* AudioDetector::instance_ = nullptr;

class AmplifierController {
 public:
  void begin() {
    pinMode(PIN_LIVINGROOM, INPUT_PULLUP);
    pinMode(PIN_OFFICE, INPUT_PULLUP);
    applyPhysical(ZoneState::Mute);
  }

  void applyPhysical(ZoneState state) {
    switch (state) {
      case ZoneState::LivingRoom:
        setRelay(PIN_LIVINGROOM, true);
        setRelay(PIN_OFFICE, false);
        break;
      case ZoneState::Office:
        setRelay(PIN_LIVINGROOM, false);
        setRelay(PIN_OFFICE, true);
        break;
      case ZoneState::Both:
        setRelay(PIN_LIVINGROOM, true);
        setRelay(PIN_OFFICE, true);
        break;
      case ZoneState::Mute:
      default:
        setRelay(PIN_LIVINGROOM, false);
        setRelay(PIN_OFFICE, false);
        break;
    }
    physicalState_ = state;
  }

  ZoneState physicalState() const {
    return physicalState_;
  }

 private:
  ZoneState physicalState_ = ZoneState::Mute;

  void setRelay(uint8_t pin, bool active) {
    if (active) {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);
    } else {
      pinMode(pin, INPUT_PULLUP);
    }
  }
};

class LedAnimator {
 public:
  void begin() {
    writeRgbPixel(PIN_LED, 0, 0, 0);
  }

  void startConnectedPulse(uint32_t nowMs) {
    connectedPulseUntilMs_ = nowMs + CONNECTED_PULSE_MS;
  }

  void triggerBoost(uint32_t nowMs) {
    if (nowMs + EVENT_BOOST_MS > boostUntilMs_) {
      boostUntilMs_ = nowMs + EVENT_BOOST_MS;
    }
  }

  void triggerWebFlash(uint32_t nowMs) {
    if (nowMs + WEB_FLASH_MS > webFlashUntilMs_) {
      webFlashUntilMs_ = nowMs + WEB_FLASH_MS;
    }
  }

  void render(uint32_t nowMs, bool wifiConnected, const RgbColor& baseColor) {
    RgbColor output;

    if (!wifiConnected) {
      output = scaled(RgbColor(0, 0, 255), slowPulse(nowMs, 0.04f, 0.22f));
    } else if (nowMs < connectedPulseUntilMs_) {
      output = scaled(RgbColor(0, 0, 255), fastPulse(nowMs, 0.08f, 0.35f));
    } else if (nowMs < webFlashUntilMs_) {
      output = scaled(RgbColor(0, 0, 255), 0.50f);
    } else {
      const float brightness = nowMs < boostUntilMs_ ? 0.10f : 0.05f;
      output = scaled(baseColor, brightness);
    }

    if (output != lastColor_) {
      writeRgbPixel(PIN_LED, output.red, output.green, output.blue);
      lastColor_ = output;
    }
  }

 private:
  RgbColor lastColor_ = RgbColor(0, 0, 0);
  uint32_t connectedPulseUntilMs_ = 0;
  uint32_t boostUntilMs_ = 0;
  uint32_t webFlashUntilMs_ = 0;

  static RgbColor scaled(const RgbColor& color, float brightness) {
    brightness = constrain(brightness, 0.0f, 1.0f);
    return RgbColor(
      static_cast<uint8_t>(color.red * brightness),
      static_cast<uint8_t>(color.green * brightness),
      static_cast<uint8_t>(color.blue * brightness)
    );
  }

  static float slowPulse(uint32_t nowMs, float minValue, float maxValue) {
    const float phase = (nowMs % 2200UL) / 2200.0f;
    const float wave = 0.5f - 0.5f * cosf(phase * 2.0f * PI);
    return minValue + (maxValue - minValue) * wave;
  }

  static float fastPulse(uint32_t nowMs, float minValue, float maxValue) {
    const float phase = (nowMs % 280UL) / 280.0f;
    const float wave = 0.5f - 0.5f * cosf(phase * 2.0f * PI);
    return minValue + (maxValue - minValue) * wave;
  }
};

struct RuntimeState {
  ZoneState preferredState = ZoneState::Mute;
  ZoneState stateBeforeStandby = ZoneState::Mute;
  uint32_t standbyTimeoutSec = DEFAULT_STANDBY_SEC;
  bool audioPresent = false;
  bool standbyActive = false;
  bool wifiConnected = false;
  bool previousWifiConnected = false;
  bool saveStatePending = false;
  bool saveStandbyPending = false;
  uint32_t lastAudioMs = 0;
  uint32_t lastInputMs = 0;
  uint32_t powerGuardUntilMs = 0;
};

AudioDetector detector;
AmplifierController amplifier;
LedAnimator led;
RuntimeState runtime;

RgbColor mixColor(const RgbColor& a, const RgbColor& b, float ratio) {
  ratio = constrain(ratio, 0.0f, 1.0f);
  return RgbColor(
    static_cast<uint8_t>(a.red + (b.red - a.red) * ratio),
    static_cast<uint8_t>(a.green + (b.green - a.green) * ratio),
    static_cast<uint8_t>(a.blue + (b.blue - a.blue) * ratio)
  );
}

RgbColor idleGradientColor(uint32_t nowMs) {
  if (runtime.standbyTimeoutSec == 0) {
    return RgbColor(255, 0, 0);
  }

  const uint32_t elapsedMs = nowMs > runtime.lastAudioMs ? nowMs - runtime.lastAudioMs : 0;
  const float progress = constrain(static_cast<float>(elapsedMs) / (runtime.standbyTimeoutSec * 1000.0f), 0.0f, 1.0f);

  if (progress < 0.5f) {
    return mixColor(RgbColor(0, 255, 0), RgbColor(255, 255, 0), progress / 0.5f);
  }
  return mixColor(RgbColor(255, 255, 0), RgbColor(255, 0, 0), (progress - 0.5f) / 0.5f);
}

RgbColor currentBaseColor(uint32_t nowMs) {
  if (!runtime.wifiConnected) {
    return RgbColor(0, 0, 255);
  }
  if (runtime.audioPresent) {
    return RgbColor(0, 255, 0);
  }
  if (runtime.standbyActive || runtime.preferredState == ZoneState::Mute) {
    return RgbColor(255, 0, 0);
  }
  return idleGradientColor(nowMs);
}

void persistIfNeeded() {
  if (runtime.saveStatePending) {
    preferences.putString("state", toString(runtime.preferredState));
    runtime.saveStatePending = false;
  }
  if (runtime.saveStandbyPending) {
    preferences.putUInt("stby_sec", runtime.standbyTimeoutSec);
    runtime.saveStandbyPending = false;
  }
}

ZoneState desiredPhysicalState(uint32_t nowMs) {
  if (nowMs < runtime.powerGuardUntilMs) {
    return ZoneState::Mute;
  }
  if (runtime.standbyActive) {
    return ZoneState::Mute;
  }
  return runtime.preferredState;
}

void applyPhysicalStateIfNeeded(uint32_t nowMs) {
  const ZoneState target = desiredPhysicalState(nowMs);
  if (target != amplifier.physicalState()) {
    amplifier.applyPhysical(target);
    led.triggerBoost(nowMs);
  }
}

void processDetection(uint32_t nowMs) {
  DetectionSnapshot snapshot = detector.snapshot();

  if (snapshot.inputPulse) {
    runtime.lastInputMs = snapshot.lastInputMs;
    led.triggerBoost(nowMs);
  }

  const bool previousAudio = runtime.audioPresent;
  runtime.audioPresent = snapshot.audioPresent;

  if (runtime.audioPresent) {
    runtime.lastAudioMs = nowMs;
    if (!previousAudio) {
      runtime.standbyActive = false;
      led.triggerBoost(nowMs);
    }
    return;
  }

  if (previousAudio) {
    runtime.lastAudioMs = nowMs;
  }

  if (!runtime.standbyActive && runtime.lastAudioMs > 0) {
    const uint32_t silentMs = nowMs - runtime.lastAudioMs;
    if (silentMs >= runtime.standbyTimeoutSec * 1000UL) {
      runtime.stateBeforeStandby = runtime.preferredState;
      runtime.standbyActive = true;
      led.triggerBoost(nowMs);
    }
  }
}

String statePayload() {
  return String("{\"state\":\"") + toString(runtime.preferredState) + "\"}";
}

void handleRoot() {
  server.send(200, "text/plain", "ESP32-C3 Audio Switch");
}

void handleGetState() {
  server.send(200, "application/json", statePayload());
}

void handleSetState() {
  if (!server.hasArg("state")) {
    server.send(400, "text/plain", "Missing state parameter");
    return;
  }

  ZoneState nextState;
  if (!parseZoneState(server.arg("state"), nextState)) {
    server.send(400, "text/plain", "Invalid state");
    return;
  }

  runtime.preferredState = nextState;
  runtime.stateBeforeStandby = nextState;
  runtime.standbyActive = false;
  runtime.saveStatePending = true;
  const uint32_t nowMs = millis();
  led.triggerBoost(nowMs);
  led.triggerWebFlash(nowMs);
  applyPhysicalStateIfNeeded(nowMs);

  server.send(200, "application/json", statePayload());
}

void handleStandby() {
  if (server.method() == HTTP_GET) {
    server.send(200, "application/json", String("{\"standby_timeout_sec\":") + runtime.standbyTimeoutSec + "}");
    return;
  }

  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method not allowed");
    return;
  }

  if (!server.hasArg("timeout")) {
    server.send(400, "text/plain", "Missing timeout parameter");
    return;
  }

  const uint32_t newTimeout = static_cast<uint32_t>(server.arg("timeout").toInt());
  if (newTimeout < 1 || newTimeout > 86400UL) {
    server.send(400, "text/plain", "Timeout out of range");
    return;
  }

  runtime.standbyTimeoutSec = newTimeout;
  runtime.saveStandbyPending = true;
  const uint32_t nowMs = millis();
  led.triggerWebFlash(nowMs);
  server.send(200, "application/json", String("{\"standby_timeout_sec\":") + runtime.standbyTimeoutSec + "}");
}

void setupWifi() {
  WiFi.begin(ssid, password);
}

void setupOTA() {
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}

void setupWeb() {
  server.on("/", handleRoot);
  server.on("/state", HTTP_GET, handleGetState);
  server.on("/state", HTTP_POST, handleSetState);
  server.on("/standby", HTTP_GET, handleStandby);
  server.on("/standby", HTTP_POST, handleStandby);
  server.begin();
}

void loadSettings() {
  preferences.begin("audio-mode", false);

  ZoneState loadedState = ZoneState::Mute;
  parseZoneState(preferences.getString("state", "mute"), loadedState);
  runtime.preferredState = loadedState;
  runtime.stateBeforeStandby = loadedState;
  runtime.standbyTimeoutSec = preferences.getUInt("stby_sec", DEFAULT_STANDBY_SEC);
}

void updateWifiStatus(uint32_t nowMs) {
  runtime.wifiConnected = WiFi.status() == WL_CONNECTED;
  if (runtime.wifiConnected && !runtime.previousWifiConnected) {
    led.startConnectedPulse(nowMs);
  }
  runtime.previousWifiConnected = runtime.wifiConnected;
}

void setup() {
  Serial.begin(115200);
  amplifier.begin();
  led.begin();
  loadSettings();
  detector.begin();
  runtime.powerGuardUntilMs = millis() + POWER_GUARD_MS;
  setupWifi();
  setupOTA();
  setupWeb();
}

void loop() {
  const uint32_t nowMs = millis();

  updateWifiStatus(nowMs);
  processDetection(nowMs);
  applyPhysicalStateIfNeeded(nowMs);
  persistIfNeeded();
  led.render(nowMs, runtime.wifiConnected, currentBaseColor(nowMs));
  server.handleClient();
  ArduinoOTA.handle();
}

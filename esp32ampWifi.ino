#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include "esp32-hal-rmt.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = STASSID;
const char* password = STAPSK;

constexpr uint8_t PIN_OFFICE = 2;
constexpr uint8_t PIN_LIVINGROOM = 3;
constexpr uint8_t PIN_AUDIO = 4;
constexpr uint8_t PIN_LED = 10;

constexpr uint32_t POWER_GUARD_MS = 2000;
constexpr uint32_t WEB_FLASH_MS = 180;
constexpr uint32_t DEFAULT_STANDBY_SEC = 15 * 60;

constexpr uint16_t AUDIO_WINDOW_MS = 50;
constexpr uint16_t AUDIO_PULSE_THRESHOLD = 30;
constexpr uint32_t EVENT_BOOST_MS = 5;

WebServer   server(80);
Preferences preferences;

// --- Audio detection state ---
volatile uint8_t pulse_counts[AUDIO_WINDOW_MS] = {0};
volatile uint32_t total_pulse_sum = 0;
volatile uint32_t last_pulse_ms = 0;
uint32_t last_processed_ms = 0;

// --- LED state ---
volatile uint32_t led_boost_until_ms = 0;
uint32_t web_flash_until_ms = 0;

enum class ZoneState : uint8_t { Mute, LivingRoom, Office, Both };

struct RgbColor {
  uint8_t red, green, blue;
  RgbColor(uint8_t r = 0, uint8_t g = 0, uint8_t b = 0) : red(r), green(g), blue(b) {}
  bool operator==(const RgbColor& o) const { return red == o.red && green == o.green && blue == o.blue; }
  bool operator!=(const RgbColor& o) const { return !(*this == o); }
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
  uint32_t powerGuardUntilMs = 0;
};

RuntimeState runtime;

// ─────────────────────────────────────────────
// RGB LED
// ─────────────────────────────────────────────

void writeRgbPixel(uint8_t pin, uint8_t red, uint8_t green, uint8_t blue) {
  static int  initializedPin = -1;
  static bool rgbReady = false;
  if (initializedPin != pin) {
    rgbReady = rmtInit(pin, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000UL);
    if (rgbReady) { rmtSetEOT(pin, LOW); initializedPin = pin; }
  }
  if (!rgbReady) return;
  rmt_data_t symbols[24];
  const rmt_data_t val[2] = { rmt_data_t{8, 1, 5, 0}, rmt_data_t{4, 1, 9, 0} };
  uint8_t i = 0;
  for (uint8_t ch : {green, red, blue})
    for (uint8_t mask = 0x80; mask; mask >>= 1)
      symbols[i++] = val[!(ch & mask)];
  rmtWrite(pin, symbols, RMT_SYMBOLS_OF(symbols), RMT_WAIT_FOR_EVER);
}

// ─────────────────────────────────────────────
// Audio detection
// ─────────────────────────────────────────────

void IRAM_ATTR onAudioPulse() {
  uint32_t now_ms = millis();
  uint16_t idx = now_ms % AUDIO_WINDOW_MS;
  if (pulse_counts[idx] < 255) {
    pulse_counts[idx]++;
    total_pulse_sum++;
  }
  last_pulse_ms      = now_ms;
  led_boost_until_ms = now_ms + EVENT_BOOST_MS;
}

void setupAudioDetection() {
  pinMode(PIN_AUDIO, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_AUDIO), onAudioPulse, RISING);
  last_processed_ms = millis();
}

void updateAudioDetection(uint32_t now_ms) {
  // Advance sliding window, capped to avoid long loops after delays
  uint32_t steps = now_ms - last_processed_ms;
  if (steps > AUDIO_WINDOW_MS) steps = AUDIO_WINDOW_MS;
  while (steps--) {
    last_processed_ms++;
    uint16_t old_idx = last_processed_ms % AUDIO_WINDOW_MS;
    total_pulse_sum -= pulse_counts[old_idx];
    pulse_counts[old_idx] = 0;
  }
  last_processed_ms = now_ms;

  // Full silence for a whole window → hard reset
  if (now_ms - last_pulse_ms > AUDIO_WINDOW_MS && total_pulse_sum > 0) {
    total_pulse_sum = 0;
    memset((void*)pulse_counts, 0, sizeof(pulse_counts));
  }

  runtime.audioPresent = (total_pulse_sum >= AUDIO_PULSE_THRESHOLD);
}

// ─────────────────────────────────────────────
// Amplifier relays
// ─────────────────────────────────────────────

ZoneState physicalState = ZoneState::Mute;

void setRelay(uint8_t pin, bool active) {
  if (active) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
  else { pinMode(pin, INPUT_PULLUP); }
}

void applyPhysical(ZoneState state) {
  switch (state) {
    case ZoneState::LivingRoom: 
      setRelay(PIN_LIVINGROOM, true);  setRelay(PIN_OFFICE, false); 
      break;
    case ZoneState::Office: 
      setRelay(PIN_LIVINGROOM, false); setRelay(PIN_OFFICE, true);  
      break;
    case ZoneState::Both: 
      setRelay(PIN_LIVINGROOM, true);  setRelay(PIN_OFFICE, true);  
      break;
    default: 
      setRelay(PIN_LIVINGROOM, false); setRelay(PIN_OFFICE, false); 
      break;
  }
  physicalState = state;
}

void setupAmplifier() {
  pinMode(PIN_LIVINGROOM, INPUT_PULLUP);
  pinMode(PIN_OFFICE, INPUT_PULLUP);
  applyPhysical(ZoneState::Mute);
}

// ─────────────────────────────────────────────
// LED rendering
// ─────────────────────────────────────────────

RgbColor mixColor(const RgbColor& a, const RgbColor& b, float t) {
  t = constrain(t, 0.0f, 1.0f);
  return RgbColor(
    a.red   + (b.red   - a.red)   * t,
    a.green + (b.green - a.green) * t,
    a.blue  + (b.blue  - a.blue)  * t
  );
}

RgbColor idleGradientColor(uint32_t nowMs) {
  if (runtime.standbyTimeoutSec == 0) {
    return RgbColor(255, 0, 0);
  }
  const uint32_t elapsedMs = nowMs > runtime.lastAudioMs ? nowMs - runtime.lastAudioMs : 0;
  const float progress = constrain(
    static_cast<float>(elapsedMs) / (runtime.standbyTimeoutSec * 1000.0f), 0.0f, 1.0f);
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

void renderStatusLed(uint32_t nowMs) {
  RgbColor output;
  if (nowMs < web_flash_until_ms) {
    output = RgbColor(0, 0, 50);
  } else {
    RgbColor color = currentBaseColor(nowMs);
    float brightness = (nowMs < led_boost_until_ms) ? 0.2f : 0.02f;
    output = RgbColor(color.red * brightness, color.green * brightness, color.blue * brightness);
  }

  static RgbColor lastColor;
  if (output != lastColor) {
    writeRgbPixel(PIN_LED, output.red, output.green, output.blue);
    lastColor = output;
  }
}

// ─────────────────────────────────────────────
// ZoneState helpers
// ─────────────────────────────────────────────

const char* toString(ZoneState state) {
  switch (state) {
    case ZoneState::LivingRoom: return "livingroom";
    case ZoneState::Office:     return "office";
    case ZoneState::Both:       return "both";
    default:                    return "mute";
  }
}

bool parseZoneState(const String& value, ZoneState& out) {
  if (value == "livingroom") { out = ZoneState::LivingRoom; return true; }
  if (value == "office")     { out = ZoneState::Office;     return true; }
  if (value == "both")       { out = ZoneState::Both;       return true; }
  if (value == "mute")       { out = ZoneState::Mute;       return true; }
  return false;
}

// ─────────────────────────────────────────────
// Audio + standby state machine
// ─────────────────────────────────────────────

void processAudioState(uint32_t nowMs) {
  const bool previousAudio = runtime.audioPresent;
  updateAudioDetection(nowMs);

  if (runtime.audioPresent) {
    runtime.lastAudioMs = nowMs;
    if (!previousAudio) runtime.standbyActive = false;
  } else {
    if (previousAudio) runtime.lastAudioMs = nowMs;
    if (!runtime.standbyActive && runtime.lastAudioMs > 0) {
      if (nowMs - runtime.lastAudioMs >= runtime.standbyTimeoutSec * 1000UL) {
        runtime.stateBeforeStandby = runtime.preferredState;
        runtime.standbyActive = true;
      }
    }
  }
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

ZoneState desiredPhysicalState() {
  if (millis() < runtime.powerGuardUntilMs) {
    return ZoneState::Mute;
  }
  if (runtime.standbyActive) {
    return ZoneState::Mute;
  }
  return runtime.preferredState;
}

void applyPhysicalStateIfNeeded() {
  const ZoneState target = desiredPhysicalState();
  if (target != physicalState) {
    applyPhysical(target);
  }
}

// ─────────────────────────────────────────────
// Web API
// ─────────────────────────────────────────────

String statePayload() {
  return String("{\"state\":\"") + toString(runtime.preferredState) + "\"}";
}

void notifyWebActivity() {
  web_flash_until_ms = millis() + WEB_FLASH_MS;
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
  runtime.preferredState     = nextState;
  runtime.stateBeforeStandby = nextState;
  runtime.standbyActive      = false;
  runtime.saveStatePending   = true;
  notifyWebActivity();
  applyPhysicalStateIfNeeded();
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
  runtime.standbyTimeoutSec  = newTimeout;
  runtime.saveStandbyPending = true;
  notifyWebActivity();
  server.send(200, "application/json", String("{\"standby_timeout_sec\":") + runtime.standbyTimeoutSec + "}");
}

// ─────────────────────────────────────────────
// Setup & loop
// ─────────────────────────────────────────────

void setupWifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void setupOTA() {
  ArduinoOTA.onStart([]() {
    writeRgbPixel(PIN_LED, 255, 165, 0);
  });
  ArduinoOTA.onEnd([]() {
    writeRgbPixel(PIN_LED, 0, 255, 0);
  });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    static uint8_t prev = 255;
    uint8_t prog = (p * 255) / t;
    if (prog == prev) return;
    prev = prog;
    uint8_t block = prog / 25, pos = prog % 25, peak = block * 25;
    uint8_t bright = (block % 2 == 0) ? (pos * peak) / 24 : peak - (pos * peak) / 24;
    writeRgbPixel(PIN_LED, 0, (prog * bright) / 255, ((255 - prog) * bright) / 255);
  });
  ArduinoOTA.onError([](ota_error_t e) {
    writeRgbPixel(PIN_LED, 255, 0, 0);
    delay(5000);
    ESP.restart();
  });
  ArduinoOTA.begin();
}

void setupWeb() {
  server.on("/",        handleRoot);
  server.on("/state",   HTTP_GET,  handleGetState);
  server.on("/state",   HTTP_POST, handleSetState);
  server.on("/standby", HTTP_GET,  handleStandby);
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

void updateWifiStatus() {
  const bool wasConnected = runtime.wifiConnected;
  runtime.wifiConnected = (WiFi.status() == WL_CONNECTED);
  if (!wasConnected && runtime.wifiConnected) {
    writeRgbPixel(PIN_LED, 0, 255, 0); delay(500); // WiFi connected
  }
}

void setup() {
  writeRgbPixel(PIN_LED, 30, 30, 30); delay(300); // Boot indicator
  writeRgbPixel(PIN_LED, 0, 0, 0); delay(300);
  
  setupAmplifier();
  loadSettings();
  setupAudioDetection();
  runtime.powerGuardUntilMs = millis() + POWER_GUARD_MS;
  
  writeRgbPixel(PIN_LED, 0, 0, 50); // WiFi connecting
  setupWifi();
  setupOTA();
  setupWeb();
}

void loop() {
  uint32_t nowMs = millis();
  updateWifiStatus();
  processAudioState(nowMs);
  applyPhysicalStateIfNeeded();
  persistIfNeeded();
  renderStatusLed(nowMs);
  server.handleClient();
  ArduinoOTA.handle();
}
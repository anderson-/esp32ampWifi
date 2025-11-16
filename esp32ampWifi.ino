#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <FastLED.h>
#include "esp_timer.h"


#define PIN_LIVINGROOM 3
#define PIN_OFFICE     2
#define PIN_AUDIO      4
#define LED_PIN        10

WebServer server(80);
Preferences prefs;

CRGB leds[1];

volatile uint8_t audioWindow[10];
volatile uint8_t windowIndex = 0;
volatile uint8_t audioSum = 0;

volatile bool interruptEnabled = false;
volatile bool disableInterruptRequested = false;

volatile uint32_t lastAudioMicros = 0;
bool hasAudio = false;
bool standbyActive = false;

String currentState = "mute";           // estado físico atual
String persistedState = "mute";         // último estado salvo na flash
String savedStateBeforeStandby = "mute";

esp_timer_handle_t periodic_timer;

CRGB nextLedColor = CRGB::Green;
portMUX_TYPE colorLock = portMUX_INITIALIZER_UNLOCKED;

volatile bool savePending = false;
volatile bool standbySavePending = false;
portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

uint32_t standbyTimeoutSec = 15 * 60;  // tempo padrão (15min)

// ------------------------------------------------------------

void setPinMode(int pin, bool active) {
  if (active) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  } else {
    pinMode(pin, INPUT_PULLUP);
  }
}

void applyState(const String& state, bool persist = true) {
  if (state == "livingroom") {
    setPinMode(PIN_LIVINGROOM, true);
    setPinMode(PIN_OFFICE, false);
  } else if (state == "office") {
    setPinMode(PIN_LIVINGROOM, false);
    setPinMode(PIN_OFFICE, true);
  } else if (state == "both") {
    setPinMode(PIN_LIVINGROOM, true);
    setPinMode(PIN_OFFICE, true);
  } else {
    setPinMode(PIN_LIVINGROOM, false);
    setPinMode(PIN_OFFICE, false);
  }
  currentState = state;
  if (persist) savePending = true;
}

String getState() {
  // retorna o estado salvo, não o físico
  return persistedState;
}

// ------------------------------------------------------------
// Endpoints

void handleRoot() {
  server.send(200, "text/plain", "ESP32-C3 Audio Switch");
}

void handleGetState() {
  String resp = "{\"state\":\"" + getState() + "\"}";
  server.send(200, "application/json", resp);
}

void handleSetState() {
  if (!server.hasArg("state")) {
    server.send(400, "text/plain", "Missing state parameter");
    return;
  }
  String newState = server.arg("state");
  if (newState != "livingroom" && newState != "office" &&
      newState != "both" && newState != "mute") {
    server.send(400, "text/plain", "Invalid state");
    return;
  }

  // aplica imediatamente e agenda persistência no loop
  persistedState = newState;
  applyState(newState, true);
  standbyActive = false; // sair de standby ao comando manual

  server.send(200, "application/json", "{\"state\":\"" + persistedState + "\"}");
}

void handleStandbyConfig() {
  if (server.method() == HTTP_GET) {
    String resp = "{\"standby_timeout_sec\":" + String(standbyTimeoutSec) + "}";
    server.send(200, "application/json", resp);
  } else if (server.method() == HTTP_POST) {
    if (!server.hasArg("timeout")) {
      server.send(400, "text/plain", "Missing timeout parameter");
      return;
    }
    uint32_t newTimeout = server.arg("timeout").toInt();
    if (newTimeout <= 1 || newTimeout > 24UL * 3600UL) {
      server.send(400, "text/plain", "Timeout out of range");
      return;
    }
    standbyTimeoutSec = newTimeout;
    standbySavePending = true; // salva no loop() para evitar concorrência no NVS
    String resp = "{\"standby_timeout_sec\":" + String(standbyTimeoutSec) + "}";
    server.send(200, "application/json", resp);
  } else {
    server.send(405, "text/plain", "Method not allowed");
  }
}

// ------------------------------------------------------------

void IRAM_ATTR audioISR() {
  uint8_t idx = windowIndex;
  if (audioWindow[idx] == 0) {
    audioWindow[idx] = 1;
    audioSum++;
  }
  lastAudioMicros = micros();
  disableInterruptRequested = true;
}

// ------------------------------------------------------------

void periodicTimerCallback(void* arg) {
  portENTER_CRITICAL(&spinlock);

  if (disableInterruptRequested) {
    detachInterrupt(PIN_AUDIO);
    interruptEnabled = false;
    disableInterruptRequested = false;
  }

  uint8_t next = (windowIndex + 1) % 10;
  uint8_t old = audioWindow[next];
  if (old) audioSum -= old;
  audioWindow[next] = 0;
  windowIndex = next;

  // Re-enable audio interrupt even during standby so we can wake on audio
  if (!interruptEnabled) {
    audioWindow[windowIndex] = 0;
    attachInterrupt(PIN_AUDIO, audioISR, RISING);
    interruptEnabled = true;
  }

  bool newHasAudio = (audioSum >= 2);
  portEXIT_CRITICAL(&spinlock);

  if (newHasAudio) {
    lastAudioMicros = micros();
    if (!hasAudio) {
      hasAudio = true;
      if (standbyActive) {
        applyState(savedStateBeforeStandby);
        standbyActive = false;
      } else {
        // aplica o estado salvo apenas quando há áudio
        if (persistedState != "mute") applyState(persistedState);
      }
    }
  } else {
    if (hasAudio) hasAudio = false;
    uint32_t now = micros();
    uint32_t elapsed = now - lastAudioMicros;
    uint32_t timeoutMicros = standbyTimeoutSec * 1000000UL;

    if (!standbyActive && lastAudioMicros != 0 && elapsed >= timeoutMicros) {
      savedStateBeforeStandby = persistedState;
      applyState("mute", false);
      standbyActive = true;
    }
  }

  portENTER_CRITICAL(&colorLock);
  nextLedColor = hasAudio ? CRGB::Green : CRGB::Red;
  portEXIT_CRITICAL(&colorLock);
}

// ------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LIVINGROOM, INPUT_PULLUP);
  pinMode(PIN_OFFICE, INPUT_PULLUP);
  pinMode(PIN_AUDIO, INPUT);

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, 1);
  leds[0] = CRGB::Blue;
  FastLED.show();

  prefs.begin("audio-mode", false);
  persistedState = prefs.getString("state", "mute");
  // NVS keys have a 15-char limit; use a short key
  standbyTimeoutSec = prefs.getUInt("stby_sec", 15 * 60);
  Serial.printf("Standby timeout loaded: %lu sec\n", (unsigned long)standbyTimeoutSec);

  // sempre começa mudo (físico)
  applyState("mute", false);
  currentState = "mute";
  savedStateBeforeStandby = persistedState;

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());

  server.on("/", handleRoot);
  server.on("/state", HTTP_GET, handleGetState);
  server.on("/state", HTTP_POST, handleSetState);
  server.on("/standby", HTTP_GET, handleStandbyConfig);
  server.on("/standby", HTTP_POST, handleStandbyConfig);
  server.begin();

  for (int i = 0; i < 10; ++i) audioWindow[i] = 0;
  windowIndex = 0;
  audioSum = 0;
  interruptEnabled = false;
  lastAudioMicros = 0;
  hasAudio = false;
  standbyActive = false;

  esp_timer_create_args_t periodic_timer_args = {};
  periodic_timer_args.callback = &periodicTimerCallback;
  periodic_timer_args.arg = NULL;
  periodic_timer_args.dispatch_method = ESP_TIMER_TASK;
  periodic_timer_args.name = "audio_periodic";
  esp_err_t _timer_err = esp_timer_create(&periodic_timer_args, &periodic_timer);
  if (_timer_err != ESP_OK) {
    Serial.printf("esp_timer_create failed: %d\n", (int)_timer_err);
  } else {
    esp_timer_start_periodic(periodic_timer, 100000);
    Serial.println("Audio detection timer started (10Hz).");
  }
}

// ------------------------------------------------------------

void loop() {
  server.handleClient();

  CRGB localColor;
  portENTER_CRITICAL(&colorLock);
  localColor = nextLedColor;
  portEXIT_CRITICAL(&colorLock);

  if (leds[0] != localColor) {
    leds[0] = localColor;
    FastLED.show();
  }

  if (savePending) {
    prefs.putString("state", currentState);
    savePending = false;
  }

  if (standbySavePending) {
    size_t w = prefs.putUInt("stby_sec", standbyTimeoutSec);
    Serial.printf("Standby timeout saved to NVS: %lu sec (bytes=%u)\n",
                  (unsigned long)standbyTimeoutSec, (unsigned)w);
    standbySavePending = false;
  }
}

#define BLYNK_TEMPLATE_ID   "TMPLxxxxxxxx"
#define BLYNK_TEMPLATE_NAME "Home"
#define BLYNK_AUTH_TOKEN    "YOUR_NEW_BLYNK_AUTH_TOKEN"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>


#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// =====================================================
// WIFI SETTINGS
// =====================================================
char ssid[] = "OnMyLevel";
char pass[] = "YOUR_WIFI_PASSWORD"; 
// =====================================================
// RELAY AND SWITCH PINS
// =====================================================
const uint8_t relayPins[4]  = {25, 26, 32, 33};
const uint8_t switchPins[4] = {13, 17, 27, 15};
const uint8_t vPins[4]      = {V1, V2, V3, V4};

// Active-LOW relays.
// false = OFF, true = ON.
bool relayState[4] = {false, false, false, false};

// Physical switch debounce
int lastRaw[4] = {HIGH, HIGH, HIGH, HIGH};
int stableState[4] = {HIGH, HIGH, HIGH, HIGH};
unsigned long debounceStart[4] = {0, 0, 0, 0};
const unsigned long debounceMs = 45;

// =====================================================
// BLYNK STARTUP SYNCHRONISATION
// =====================================================
// On the first Blynk connection after boot, retrieve
// the saved V1-V4 states.
//
// After all four states arrive, later reconnects publish
// the current local states instead of restoring old ones.
bool startupSyncPending = true;
uint8_t startupSyncMask = 0;

unsigned long lastBlynkAttempt = 0;
unsigned long lastWiFiMessage = 0;
bool wasWiFiConnected = false;
bool wasBlynkConnected = false;

BlynkTimer timer;

// =====================================================
// BLUETOOTH SETTINGS
// =====================================================
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define COMMAND_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

volatile int pendingRelay = -1;
volatile int pendingValue = -1;
portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;

// =====================================================
// RELAY CONTROL
// =====================================================
void writeRelay(uint8_t i) {
  // Active-LOW relay:
  // LOW = ON, HIGH = OFF
  digitalWrite(relayPins[i], relayState[i] ? LOW : HIGH);
}

void setRelay(uint8_t i, bool on,
              const char *source,
              bool publish = true) {
  if (i >= 4) return;

  bool changed = (relayState[i] != on);
  relayState[i] = on;
  writeRelay(i);

  if (changed) {
    Serial.printf("[%s] Relay %u: %s\n",
                  source, i + 1,
                  on ? "ON" : "OFF");
  }

  // Publish local changes only when requested and online.
  if (publish && Blynk.connected()) {
    Blynk.virtualWrite(vPins[i], relayState[i] ? 1 : 0);
  }
}

// =====================================================
// BLYNK STARTUP STATE HANDLING
// =====================================================
void handleBlynkState(uint8_t i, int value) {
  if (i >= 4) return;

  bool on = (value == 1);

  if (startupSyncPending) {
    // Apply the saved Blynk state without sending it back.
    setRelay(i, on, "BLYNK STARTUP", false);

    startupSyncMask |= (1U << i);

    Serial.printf("[STARTUP] Received saved state for V%u: %s\n",
                  i + 1, on ? "ON" : "OFF");

    // Wait until all four saved states have arrived.
    if (startupSyncMask == 0x0F) {
      startupSyncPending = false;
      Serial.println(
        "[STARTUP] All four saved states loaded successfully."
      );
    }
  } else {
    // Normal app command after startup synchronisation.
    setRelay(i, on, "BLYNK", false);
  }
}

BLYNK_CONNECTED() {
  Serial.println("[BLYNK] Connected.");

  if (startupSyncPending) {
    Serial.println(
      "[BLYNK] Startup: requesting saved relay states V1-V4."
    );

    // Only sync saved values during startup.
    Blynk.syncVirtual(V1, V2, V3, V4);
  } else {
    Serial.println(
      "[BLYNK] Reconnected: publishing current relay states."
    );

    // Do not restore old cloud states after an ordinary
    // Wi-Fi/Blynk reconnection.
    for (uint8_t i = 0; i < 4; i++) {
      Blynk.virtualWrite(vPins[i], relayState[i] ? 1 : 0);

      Serial.printf("[BLYNK] Relay %u current state: %s\n",
                    i + 1,
                    relayState[i] ? "ON" : "OFF");
    }
  }
}

BLYNK_WRITE(V1) {
  handleBlynkState(0, param.asInt());
}

BLYNK_WRITE(V2) {
  handleBlynkState(1, param.asInt());
}

BLYNK_WRITE(V3) {
  handleBlynkState(2, param.asInt());
}

BLYNK_WRITE(V4) {
  handleBlynkState(3, param.asInt());
}

// =====================================================
// PHYSICAL SWITCHES
// =====================================================
void readSwitches() {
  unsigned long now = millis();

  for (uint8_t i = 0; i < 4; i++) {
    int raw = digitalRead(switchPins[i]);

    if (raw != lastRaw[i]) {
      lastRaw[i] = raw;
      debounceStart[i] = now;
    }

    if ((now - debounceStart[i] >= debounceMs) &&
        raw != stableState[i]) {

      stableState[i] = raw;

      // INPUT_PULLUP:
      // LOW means the button is pressed.
      if (stableState[i] == LOW) {
        Serial.printf("[SWITCH %u] ON (pressed)\n", i + 1);

        // Toggle the corresponding relay.
        setRelay(i, !relayState[i], "SWITCH", true);
      } else {
        Serial.printf("[SWITCH %u] OFF (released)\n", i + 1);
      }
    }
  }
}

// =====================================================
// BLUETOOTH COMMAND PROCESSING
// Commands: R1=1, R1=0, R2=1, R2=0, etc.
// =====================================================
class RelayCommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
  String value = characteristic->getValue();
    if (value.isEmpty()) return;

    String command = String(value.c_str());
    command.trim();
    command.toUpperCase();

    Serial.printf("[BLE] Received command: %s\n",
                  command.c_str());

    if (command.length() != 4 ||
        command.charAt(0) != 'R' ||
        command.charAt(2) != '=') {
      Serial.println("[BLE] Invalid command. Use R1=1 or R1=0.");
      return;
    }

    int relay = command.charAt(1) - '1';
    char requestedState = command.charAt(3);

    if (relay < 0 || relay > 3 ||
        (requestedState != '0' && requestedState != '1')) {
      Serial.println("[BLE] Invalid relay number or state.");
      return;
    }

    portENTER_CRITICAL(&commandMux);
    pendingRelay = relay;
    pendingValue = requestedState - '0';
    portEXIT_CRITICAL(&commandMux);
  }
};

void processBLE() {
  int relay;
  int value;

  portENTER_CRITICAL(&commandMux);
  relay = pendingRelay;
  value = pendingValue;
  pendingRelay = -1;
  pendingValue = -1;
  portEXIT_CRITICAL(&commandMux);

  if (relay >= 0 && relay < 4 && value >= 0) {
    setRelay(relay, value == 1, "BLE", true);
  }
}

void startBluetooth() {
  BLEDevice::init("TTGO-Relay-Control");

  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(SERVICE_UUID);

  BLECharacteristic *characteristic =
    service->createCharacteristic(
      COMMAND_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR
    );

  characteristic->setCallbacks(new RelayCommandCallbacks());
  characteristic->setValue("Send R1=1 or R1=0");

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();

  Serial.println("[BLE] Started: TTGO-Relay-Control");
  Serial.println("[BLE] Commands: R1=1, R1=0 ... R4=1, R4=0");
}

// =====================================================
// CONNECTION MONITOR
// =====================================================
void monitorConnections() {
  bool wifiNow = (WiFi.status() == WL_CONNECTED);
  bool blynkNow = Blynk.connected();

  if (wifiNow && !wasWiFiConnected) {
    Serial.println("[WIFI] Connected successfully.");
    Serial.print("[WIFI] IP address: ");
    Serial.println(WiFi.localIP());
    Serial.printf("[WIFI] Signal strength: %d dBm\n",
                  WiFi.RSSI());
  }

  if (!wifiNow && wasWiFiConnected) {
    Serial.println("[WIFI] Connection lost. Local controls remain active.");
  }

  if (!wifiNow && millis() - lastWiFiMessage >= 10000) {
    lastWiFiMessage = millis();
    Serial.println("[WIFI] Waiting for connection...");
  }

  if (wifiNow && !blynkNow &&
      millis() - lastBlynkAttempt >= 5000) {
    lastBlynkAttempt = millis();
    Serial.println("[BLYNK] Attempting connection...");
    Blynk.connect(1000);
  }

  if (blynkNow && !wasBlynkConnected) {
    Serial.println("[BLYNK] Online.");
  }

  if (!blynkNow && wasBlynkConnected) {
    Serial.println("[BLYNK] Offline. Physical and BLE controls remain active.");
  }

  wasWiFiConnected = (WiFi.status() == WL_CONNECTED);
  wasBlynkConnected = Blynk.connected();
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("==================================");
  Serial.println(" ESP32 FOUR-RELAY CONTROLLER");
  Serial.println(" Startup: load states from Blynk");
  Serial.println("==================================");

  // Initialise relay pins HIGH first to avoid a brief
  // active-LOW ON pulse during initialisation.
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(relayPins[i], HIGH);
    pinMode(relayPins[i], OUTPUT);

    relayState[i] = false;
    writeRelay(i);

    Serial.printf("[STARTUP] Relay %u initialised OFF pending Blynk state.\n",
                  i + 1);
  }

  // Initialise physical switches without treating a button
  // held during boot as a new press.
  for (uint8_t i = 0; i < 4; i++) {
    pinMode(switchPins[i], INPUT_PULLUP);

    int reading = digitalRead(switchPins[i]);
    lastRaw[i] = reading;
    stableState[i] = reading;
    debounceStart[i] = millis();
  }

  Serial.println("[SWITCH] Four physical inputs initialised.");

  startBluetooth();

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, pass);

  Blynk.config(BLYNK_AUTH_TOKEN);

  timer.setInterval(20L, readSwitches);
  timer.setInterval(50L, processBLE);
  timer.setInterval(1000L, monitorConnections);

  Serial.println("[STARTUP] Waiting for Wi-Fi and Blynk saved states.");
  Serial.println("[STARTUP] If Blynk is unavailable, relays remain OFF until local commands or saved states arrive.");
}

// =====================================================
// MAIN LOOP
// =====================================================
void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.run();
  }

  timer.run();
}
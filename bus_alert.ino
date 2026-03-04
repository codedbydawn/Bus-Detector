#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include <pgmspace.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "tripset.h"

// --------- WIFI ----------
const char* WIFI_SSID = "YOUR WIFI NAME";
const char* WIFI_PASS = "YOUR WIFI PASSWORD";

// --------- YOUR STOP (LON AND LAT FOUND IN LTC OPEN DATA ONLINE)----------
const double STOP_LAT = 0; // LONGITUDE OF BUSTOP
const double STOP_LON = 0; // LATITUDE OF BUSTOP

// --------- LTC VEHICLE POSITIONS (JSON) ----------
const char* VEH_POS_URL = "http://gtfs.ltconline.ca/Vehicle/VehiclePositions.json";

// --------- PINS ----------
const int LED_PIN = 25;
const int BUZZER_PIN = 26;

// --------- POLLING ----------
const unsigned long POLL_MS = 5000;

// --------- ALERT PATTERN (continuous while alertActive) ----------
const unsigned long BLINK_MS = 250;
const unsigned long BEEP_MS  = 250;

// --------- DIRECTION-AWARE ALERT THRESHOLDS ----------
const int START_METERS = 450;
const int ARRIVED_METERS = 60;            // tune 60–120 if needed
const int STOP_AFTER_LEAVE_METERS = 120;  // tune 100–200 if needed

// --------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // -1 = no reset pin

// --------- ALERT STATE ----------
bool alertActive = false;
unsigned long lastBlink = 0;
unsigned long lastBeep  = 0;
bool ledState = false;
bool buzState = false;

unsigned long lastPoll = 0;

// Distance tracking
double prevClosest = -1;
double minDuringAlert = 1e9;
bool hasArrived = false;

// Latest info for OLED
double lastClosestServing = -1;
int lastServingCount = 0;
unsigned long lastGoodUpdateMs = 0;

// ---------- Haversine distance (meters) ----------
double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double p1 = lat1 * (M_PI / 180.0);
  double p2 = lat2 * (M_PI / 180.0);
  double dlat = (lat2 - lat1) * (M_PI / 180.0);
  double dlon = (lon2 - lon1) * (M_PI / 180.0);

  double a = sin(dlat / 2) * sin(dlat / 2) +
             cos(p1) * cos(p2) * sin(dlon / 2) * sin(dlon / 2);
  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return R * c;
}

// ---------- Bloom filter helpers ----------
static inline bool bloomGetBit(uint32_t bitIndex) {
  uint32_t byteIndex = bitIndex >> 3;
  uint8_t mask = (1u << (bitIndex & 7));
  uint8_t v = pgm_read_byte(&TRIP_BLOOM[byteIndex]);
  return (v & mask) != 0;
}

// FNV-1a 32-bit
uint32_t fnv1a(const char* s) {
  uint32_t h = 2166136261u;
  for (; *s; s++) {
    h ^= (uint8_t)(*s);
    h *= 16777619u;
  }
  return h;
}

// Convert to digits-only (matches how tripset.h was built)
static void digitsOnly(const char* in, char* out, size_t outSize) {
  size_t j = 0;
  for (size_t i = 0; in && in[i] != '\0' && j + 1 < outSize; i++) {
    if (in[i] >= '0' && in[i] <= '9') out[j++] = in[i];
  }
  out[j] = '\0';
}

bool tripStopsAtOurStopDigits(const char* trip_digits) {
  if (!trip_digits || trip_digits[0] == '\0') return false;

  uint32_t h1 = fnv1a(trip_digits);
  uint32_t h2 = h1 ^ 0x9e3779b9u;

  for (uint32_t i = 0; i < BLOOM_K_HASHES; i++) {
    uint32_t bit = (h1 + i * h2) % BLOOM_M_BITS;
    if (!bloomGetBit(bit)) return false;
  }
  return true;
}

// ---------- IO ----------
void wifiConnect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to WiFi");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    if (++tries > 120) break;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi connect failed.");
  }
}

// Continuous blink/beep while shouldAlert == true
void updateAlertPattern(bool shouldAlert) {
  unsigned long now = millis();

  if (!shouldAlert) {
    ledState = false;
    buzState = false;
    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  if (now - lastBlink >= BLINK_MS) {
    lastBlink = now;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
  }

  if (now - lastBeep >= BEEP_MS) {
    lastBeep = now;
    buzState = !buzState;
    digitalWrite(BUZZER_PIN, buzState ? HIGH : LOW); // active buzzer
  }
}

// Returns closest distance among *serving* buses, or -1 if none.
// Also returns serving count.
double closestServingBusMeters(int &servingCountOut) {
  servingCountOut = 0;

  HTTPClient http;
  http.setTimeout(15000);
  http.useHTTP10(true);
  http.addHeader("Accept-Encoding", "identity");
  http.begin(VEH_POS_URL);

  int code = http.GET();
  Serial.print("HTTP code: ");
  Serial.println(code);
  if (code != 200) {
    http.end();
    return -1;
  }

  WiFiClient* stream = http.getStreamPtr();

  // Filter: position + trip_id
  StaticJsonDocument<384> filter;
  filter["entity"][0]["vehicle"]["position"]["latitude"] = true;
  filter["entity"][0]["vehicle"]["position"]["longitude"] = true;
  filter["entity"][0]["vehicle"]["trip"]["trip_id"] = true;

  DynamicJsonDocument doc(128 * 1024);
  DeserializationError err = deserializeJson(doc, *stream, DeserializationOption::Filter(filter));
  http.end();

  if (err) {
    Serial.print("JSON parse error: ");
    Serial.println(err.c_str());
    return -1;
  }

  double closest = -1;

  JsonArray entities = doc["entity"].as<JsonArray>();
  for (JsonVariant e : entities) {
    const char* trip_id = e["vehicle"]["trip"]["trip_id"] | "";
    if (!trip_id || trip_id[0] == '\0') continue;

    char tripNorm[32];
    digitsOnly(trip_id, tripNorm, sizeof(tripNorm));
    if (!tripStopsAtOurStopDigits(tripNorm)) continue; // only trips that stop at ALDEAURO

    double lat = e["vehicle"]["position"]["latitude"] | 0.0;
    double lon = e["vehicle"]["position"]["longitude"] | 0.0;
    if (lat == 0.0 && lon == 0.0) continue;

    servingCountOut++;

    double d = haversineMeters(STOP_LAT, STOP_LON, lat, lon);
    if (closest < 0 || d < closest) closest = d;
  }

  return closest;
}

void drawOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Line 1: WiFi status
  display.setCursor(0, 0);
  display.print("WiFi: ");
  display.print((WiFi.status() == WL_CONNECTED) ? "OK" : "NO");

  // Line 2: Serving buses
  display.setCursor(0, 10);
  display.print("Serving: ");
  display.print(lastServingCount);

  // Line 3: Distance
  display.setCursor(0, 20);
  display.print("Dist: ");
  if (lastClosestServing < 0) {
    display.print("--");
  } else {
    display.print((int)round(lastClosestServing));
    display.print(" m");
  }

  // Line 4: Alert state
  display.setCursor(0, 30);
  display.print("Alert: ");
  display.print(alertActive ? "ON" : "OFF");

  // Line 5: Age of last update (ms)
  display.setCursor(0, 40);
  display.print("Age: ");
  unsigned long age = (lastGoodUpdateMs == 0) ? 0 : (millis() - lastGoodUpdateMs);
  display.print(age / 1000);
  display.print("s");

  // Bottom: thresholds (optional)
  display.setCursor(0, 52);
  display.print("Start ");
  display.print(START_METERS);
  display.print(" Arr ");
  display.print(ARRIVED_METERS);
  display.print(" Stop ");
  display.print(STOP_AFTER_LEAVE_METERS);

  display.display();
}

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // I2C init (default ESP32 pins: SDA=21, SCL=22)
  Wire.begin(21, 22);

  // OLED init (0x3C is most common; if blank, try 0x3D)
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed. Try address 0x3D or check wiring.");
    // If OLED fails, keep running headless:
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Booting...");
    display.display();
  }

  wifiConnect();
  drawOLED();
}

void loop() {
  unsigned long now = millis();

  // Keep blinking/beeping when alertActive (non-blocking)
  if (alertActive) {
    updateAlertPattern(true);
  }

  // Poll feed periodically
  if (now - lastPoll >= POLL_MS) {
    lastPoll = now;

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi lost, reconnecting...");
      wifiConnect();
      // Even if reconnect fails, we still update OLED with status
    }

    int servingCount = 0;
    double closestServing = closestServingBusMeters(servingCount);

    lastServingCount = servingCount;
    lastClosestServing = closestServing;
    lastGoodUpdateMs = millis();

    Serial.print("Serving vehicles count: ");
    Serial.println(servingCount);

    Serial.print("Closest *serving* bus (m): ");
    if (closestServing < 0) Serial.println("-1");
    else Serial.println(closestServing, 1);

    // If we can't see any serving bus, stop alert
    if (closestServing < 0) {
      alertActive = false;
      hasArrived = false;
      minDuringAlert = 1e9;
      updateAlertPattern(false);
      prevClosest = closestServing;
      drawOLED();
      return;
    }

    bool gettingCloser = (prevClosest < 0) ? true : (closestServing < prevClosest);
    bool gettingFarther = (prevClosest < 0) ? false : (closestServing > prevClosest);

    // START: within START_METERS AND approaching
    if (!alertActive) {
      if (closestServing <= START_METERS && gettingCloser) {
        alertActive = true;
        hasArrived = false;
        minDuringAlert = closestServing;
        Serial.println("ALERT START (approaching within start meters)");
      }
    }

    // While alerting, track minimum distance
    if (alertActive) {
      if (closestServing < minDuringAlert) minDuringAlert = closestServing;
      if (minDuringAlert <= ARRIVED_METERS) hasArrived = true;

      // STOP: it got very close (arrived), then is leaving and past STOP_AFTER_LEAVE_METERS
      if (hasArrived && gettingFarther && closestServing >= STOP_AFTER_LEAVE_METERS) {
        alertActive = false;
        hasArrived = false;
        minDuringAlert = 1e9;
        updateAlertPattern(false);
        Serial.println("ALERT STOP (leaving past stop-after-leave meters)");
      }
    }

    // Run/stop pattern based on alertActive
    updateAlertPattern(alertActive);

    // Remember for next poll
    prevClosest = closestServing;

    // Update OLED each poll (distance changes on poll)
    drawOLED();
  }

  // Optional: refresh OLED more often even between polls (keeps age ticking smoothly)
  // (won't change distance, just updates "Age")
  static unsigned long lastOledRefresh = 0;
  if (millis() - lastOledRefresh >= 250) {
    lastOledRefresh = millis();
    drawOLED();
  }

  delay(5);
}

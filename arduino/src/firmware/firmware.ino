#include <Arduino_MKRIoTCarrier.h>
#include <WiFiNINA.h>

#include <math.h>
#include <string.h>
#include <Wire.h>

bool pressureAvailable = false;

// Capacitive soil moisture sensor v1.2 on A6.
// High ADC = dry, low ADC = wet.
// Tune DRY/WET by reading moisture_pct (or raw = 1023 - pct*1023/100) in
// each extreme: sensor in open air (DRY) and fully submerged in water (WET).
constexpr uint8_t  MOISTURE_PIN              = A5;
constexpr int16_t  MOISTURE_DRY_RAW          = 1023;  // sensor in dry air / dry soil
constexpr int16_t  MOISTURE_WET_RAW          =  800;  // sensor submerged in water
// A floating (disconnected) pin drifts between two reads; a connected sensor is stable.
constexpr int16_t  MOISTURE_DRIFT_THRESHOLD  =   50;  // ADC counts between two reads

bool i2cDevicePresent(uint8_t address)
{
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

MKRIoTCarrier carrier;
WiFiServer httpServer(80);

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

const char WIFI_SSID[] = "CERN";
const char WIFI_PASSWORD[] = "";

// Change to carrier.withCase() in setup() when using the plastic enclosure.
constexpr touchButtons NEXT_PAGE_BUTTON = TOUCH0;

constexpr uint32_t SENSOR_INTERVAL_MS      = 5000;
constexpr uint32_t PAGE_INTERVAL_MS        = 3500;
constexpr uint32_t WIFI_RETRY_MS           = 30000;
constexpr uint32_t HTTP_TIMEOUT_MS         = 500;

constexpr int16_t DISPLAY_WIDTH  = 240;
constexpr int16_t DISPLAY_HEIGHT = 240;

// -----------------------------------------------------------------------------
// Measurements
// -----------------------------------------------------------------------------

struct Measurements {
  float environmentTemperatureC;

  float pressureKPa;
  float pressureTemperatureC;

  float lightLux;

  float soilMoisturePct;  // NAN when sensor is absent (floating pin detected)

  uint32_t sampledAtMs;
};

Measurements measurements;

uint32_t samplesTotal = 0;


void initialiseMeasurements() {
  measurements.environmentTemperatureC = NAN;
  measurements.pressureKPa = NAN;
  measurements.pressureTemperatureC = NAN;

  measurements.lightLux = NAN;
  measurements.soilMoisturePct = NAN;

  measurements.sampledAtMs = 0;
}

// -----------------------------------------------------------------------------
// Display pages
// -----------------------------------------------------------------------------

struct GaugeSpec {
  const char* title;
  const char* unit;
  float value;
  float minimum;
  float maximum;
  uint8_t decimals;
};
GaugeSpec getGauge(uint8_t page);

// Returns 5 when the moisture sensor is present, 4 otherwise.
uint8_t effectivePageCount() {
  return isnan(measurements.soilMoisturePct) ? 4 : 5;
}

uint8_t currentPage = 0;
bool displayDirty = true;
float lastDisplayedValue = NAN;

uint32_t lastSensorRead = 0;
uint32_t lastPageChange = 0;

GaugeSpec getGauge(uint8_t page) {
  switch (page) {
    case 0:
      return {
        "Temp HTS221", "C",
        measurements.environmentTemperatureC,
        -10.0f, 50.0f, 1
      };

    case 1:
      return {
        "Pressure", "kPa",
        measurements.pressureKPa,
        80.0f, 120.0f, 2
      };

    case 2:
      return {
        "Temp LPS22HB", "C",
        measurements.pressureTemperatureC,
        -10.0f, 60.0f, 1
      };

    case 3:
      return {
        "Illuminance", "lux",
        measurements.lightLux,
        0.0f, 10000.0f, 0
      };

    case 4:
    default:
      return {
        "Soil Moisture", "%",
        measurements.soilMoisturePct,
        0.0f, 100.0f, 0
      };
  }
}

void printCentred(
  const char* text,
  int16_t y,
  uint8_t textSize,
  uint16_t colour) {
  int16_t x1;
  int16_t y1;
  uint16_t width;
  uint16_t height;

  carrier.display.setTextSize(textSize);
  carrier.display.setTextColor(colour);
  carrier.display.getTextBounds(text, 0, y, &x1, &y1, &width, &height);

  int16_t x = (DISPLAY_WIDTH - static_cast<int16_t>(width)) / 2;
  if (x < 0) {
    x = 0;
  }

  carrier.display.setCursor(x, y);
  carrier.display.print(text);
}

void gaugePoint(
  float angleDegrees,
  int16_t radius,
  int16_t& x,
  int16_t& y) {
  constexpr int16_t centreX = 120;
  constexpr int16_t centreY = 134;

  const float radians = angleDegrees * PI / 180.0f;

  x = centreX + static_cast<int16_t>(cosf(radians) * radius);
  y = centreY + static_cast<int16_t>(sinf(radians) * radius);
}

float clampValue(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }

  if (value > maximum) {
    return maximum;
  }

  return value;
}

void drawGauge() {
  const GaugeSpec gauge = getGauge(currentPage);

  carrier.display.fillScreen(ST77XX_BLACK);
  carrier.display.setTextWrap(false);

  printCentred(gauge.title, 30, 2, ST77XX_WHITE);

  float normalised = 0.0f;

  if (!isnan(gauge.value) && gauge.maximum > gauge.minimum) {
    const float clamped =
      clampValue(gauge.value, gauge.minimum, gauge.maximum);

    normalised =
      (clamped - gauge.minimum) / (gauge.maximum - gauge.minimum);
  }

  // Draw the 270-degree gauge arc.
  constexpr int16_t radius = 78;
  constexpr int arcSegments = 90;

  for (int thickness = 0; thickness < 4; ++thickness) {
    int16_t previousX;
    int16_t previousY;

    gaugePoint(135.0f, radius - thickness, previousX, previousY);

    for (int segment = 1; segment <= arcSegments; ++segment) {
      const float position =
        static_cast<float>(segment) / static_cast<float>(arcSegments);

      const float angle = 135.0f + position * 270.0f;

      int16_t x;
      int16_t y;
      gaugePoint(angle, radius - thickness, x, y);

      const uint16_t colour =
        position <= normalised ? ST77XX_GREEN : ST77XX_BLUE;

      carrier.display.drawLine(previousX, previousY, x, y, colour);

      previousX = x;
      previousY = y;
    }
  }

  // Major tick marks.
  for (int tick = 0; tick <= 10; ++tick) {
    const float position = static_cast<float>(tick) / 10.0f;
    const float angle = 135.0f + position * 270.0f;

    int16_t outerX;
    int16_t outerY;
    int16_t innerX;
    int16_t innerY;

    gaugePoint(angle, radius + 2, outerX, outerY);
    gaugePoint(angle, radius - 8, innerX, innerY);

    carrier.display.drawLine(
      outerX,
      outerY,
      innerX,
      innerY,
      ST77XX_WHITE);
  }

  // Needle.
  if (!isnan(gauge.value)) {
    const float needleAngle = 135.0f + normalised * 270.0f;

    int16_t needleX;
    int16_t needleY;
    gaugePoint(needleAngle, radius - 14, needleX, needleY);

    carrier.display.drawLine(
      120,
      134,
      needleX,
      needleY,
      ST77XX_RED);

    carrier.display.fillCircle(120, 134, 5, ST77XX_RED);
  }

  // Min/max labels.
  String minimumText(gauge.minimum, 0);
  String maximumText(gauge.maximum, 0);

  carrier.display.setTextSize(1);
  carrier.display.setTextColor(ST77XX_WHITE);

  carrier.display.setCursor(28, 189);
  carrier.display.print(minimumText);

  int16_t x1;
  int16_t y1;
  uint16_t width;
  uint16_t height;

  carrier.display.getTextBounds(
    maximumText.c_str(),
    0,
    189,
    &x1,
    &y1,
    &width,
    &height);

  carrier.display.setCursor(DISPLAY_WIDTH - width - 28, 189);
  carrier.display.print(maximumText);

  // Current value.
  String valueText;

  if (isnan(gauge.value)) {
    valueText = "N/A";
  } else {
    valueText = String(gauge.value, gauge.decimals);
  }

  String valueAndUnit = valueText + " " + gauge.unit;

  printCentred(
    valueAndUnit.c_str(),
    205,
    2,
    ST77XX_YELLOW);

  char pageText[16];
  snprintf(
    pageText,
    sizeof(pageText),
    "%u / %u",
    static_cast<unsigned int>(currentPage + 1),
    static_cast<unsigned int>(effectivePageCount()));

  printCentred(pageText, 231, 1, ST77XX_WHITE);

  lastDisplayedValue = gauge.value;
  displayDirty = false;
}

void advancePage(uint32_t now) {
  const uint8_t count = effectivePageCount();
  currentPage = (currentPage + 1) % count;
  lastPageChange = now;
  displayDirty = true;
}

// Call after every sensor sample to clamp page if moisture sensor disappeared.
void clampPage() {
  const uint8_t count = effectivePageCount();
  if (currentPage >= count) {
    currentPage = 0;
    displayDirty = true;
  }
}

// -----------------------------------------------------------------------------
// Sensor acquisition
// -----------------------------------------------------------------------------

// Avago APDS-9960 application-note coefficients.
float computeLux(int r, int g, int b) {
  float lux = (-0.32466f * r) + (1.57837f * g) + (-0.73581f * b);
  return lux < 0.0f ? 0.0f : lux;
}


void sampleSensors() {
  measurements.environmentTemperatureC =
    carrier.Env.readTemperature();

  if (pressureAvailable) {
    measurements.pressureKPa =
      carrier.Pressure.readPressure();

    measurements.pressureTemperatureC =
      carrier.Pressure.readTemperature();
  }

  if (carrier.Light.colorAvailable()) {
    int red, green, blue, clear;
    carrier.Light.readColor(red, green, blue, clear);
    measurements.lightLux = computeLux(red, green, blue);
  }

  {
    const int raw0 = analogRead(MOISTURE_PIN);
    delayMicroseconds(500);
    const int raw1 = analogRead(MOISTURE_PIN);
    const int raw = (raw0 + raw1) / 2;
    if (abs(raw1 - raw0) > MOISTURE_DRIFT_THRESHOLD) {
      measurements.soilMoisturePct = NAN;  // pin floating — sensor absent
    } else {
      const float pct = static_cast<float>(MOISTURE_DRY_RAW - raw)
                      * 100.0f
                      / static_cast<float>(MOISTURE_DRY_RAW - MOISTURE_WET_RAW);
      measurements.soilMoisturePct =
        pct < 0.0f ? 0.0f : (pct > 100.0f ? 100.0f : pct);
    }
  }

  measurements.sampledAtMs = millis();
  ++samplesTotal;
}

// Print one CSV row: ts and all sensor values.
static void printCsvFloat(float v, uint8_t decimals) {
  if (isnan(v)) Serial.print(F("nan"));
  else          Serial.print(v, decimals);
}

void logSample() {
  Serial.print(millis());
  Serial.print(',');
  printCsvFloat(measurements.environmentTemperatureC, 2);
  Serial.print(',');
  printCsvFloat(measurements.pressureKPa, 3);
  Serial.print(',');
  printCsvFloat(measurements.pressureTemperatureC, 2);
  Serial.print(',');
  printCsvFloat(measurements.lightLux, 1);
  Serial.print(',');
  printCsvFloat(measurements.soilMoisturePct, 1);
  Serial.print(',');
  Serial.println(samplesTotal);
}

// -----------------------------------------------------------------------------
// Wi-Fi management
// -----------------------------------------------------------------------------

bool wifiModuleAvailable = true;
bool httpServerStarted = false;
uint32_t lastWiFiAttempt = 0;

void printMetricsAddress() {
  Serial.print("Prometheus endpoint: http://");
  Serial.print(WiFi.localIP());
  Serial.println("/metrics");
}


void maintainWiFi() {
  if (!wifiModuleAvailable) {
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    if (!httpServerStarted) {
      httpServer.begin();
      httpServerStarted = true;
      printMetricsAddress();
    }

    return;
  }

  httpServerStarted = false;

  const uint32_t now = millis();

  if (
    lastWiFiAttempt != 0 && now - lastWiFiAttempt < WIFI_RETRY_MS) {
    return;
  }

  lastWiFiAttempt = now;

  Serial.print("Connecting to Wi-Fi network ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// -----------------------------------------------------------------------------
// Prometheus endpoint
// -----------------------------------------------------------------------------

void printPrometheusValue(
  WiFiClient& client,
  float value,
  uint8_t precision = 6) {
  if (isnan(value)) {
    client.print(F("NaN\n"));
  } else {
    client.print(value, precision);
    client.print('\n');
  }
}

bool readRequestLine(
  WiFiClient& client,
  char* buffer,
  size_t bufferSize) {
  size_t index = 0;
  const uint32_t startedAt = millis();

  while (
    client.connected() && millis() - startedAt < HTTP_TIMEOUT_MS) {
    while (client.available()) {
      const char c = client.read();

      if (c == '\n') {
        buffer[index] = '\0';
        return index > 0;
      }

      if (c != '\r' && index < bufferSize - 1) {
        buffer[index++] = c;
      }
    }
  }

  buffer[index] = '\0';
  return index > 0;
}

void discardRequestHeaders(WiFiClient& client) {
  bool lineIsEmpty = true;
  const uint32_t startedAt = millis();

  while (
    client.connected() && millis() - startedAt < HTTP_TIMEOUT_MS) {
    if (!client.available()) {
      continue;
    }

    const char c = client.read();

    if (c == '\n') {
      if (lineIsEmpty) {
        return;
      }

      lineIsEmpty = true;
    } else if (c != '\r') {
      lineIsEmpty = false;
    }
  }
}

void sendMetrics(WiFiClient& client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(
    F("Content-Type: text/plain; version=0.0.4; charset=utf-8"));
  client.println(F("Cache-Control: no-store"));
  // Auto-refresh in browser; gives the sensor a full minute between views.
  client.println(F("Refresh: 60"));
  client.println(F("Connection: close"));
  client.println();

  client.print(F("# HELP mkr_iot_carrier_info Static carrier information.\n"));
  client.print(F("# TYPE mkr_iot_carrier_info gauge\n"));
  client.print(F("mkr_iot_carrier_info{revision=\"1\"} 1\n"));

  client.print(F("# HELP mkr_iot_carrier_up Whether sensor acquisition has completed.\n"));
  client.print(F("# TYPE mkr_iot_carrier_up gauge\n"));
  client.print(F("mkr_iot_carrier_up "));
  client.print(measurements.sampledAtMs != 0 ? 1 : 0);
  client.print('\n');

  client.print(F("# HELP mkr_iot_carrier_temperature_celsius Temperature measured by the carrier.\n"));
  client.print(F("# TYPE mkr_iot_carrier_temperature_celsius gauge\n"));

  client.print(F("mkr_iot_carrier_temperature_celsius{sensor=\"hts221\"} "));
  printPrometheusValue(client, measurements.environmentTemperatureC, 3);

  client.print(F("mkr_iot_carrier_temperature_celsius{sensor=\"lps22hb\"} "));
  printPrometheusValue(client, measurements.pressureTemperatureC, 3);

  client.print(F("# HELP mkr_iot_carrier_pressure_kilopascals Atmospheric pressure.\n"));
  client.print(F("# TYPE mkr_iot_carrier_pressure_kilopascals gauge\n"));
  client.print(F("mkr_iot_carrier_pressure_kilopascals "));
  printPrometheusValue(client, measurements.pressureKPa, 4);

  client.print(F("# HELP mkr_iot_carrier_illuminance_lux Illuminance derived from APDS9960 RGB channels.\n"));
  client.print(F("# TYPE mkr_iot_carrier_illuminance_lux gauge\n"));
  client.print(F("mkr_iot_carrier_illuminance_lux "));
  printPrometheusValue(client, measurements.lightLux, 1);

  client.print(F("# HELP mkr_iot_carrier_soil_moisture_percent Soil moisture (A6, resistive sensor). NaN when sensor absent.\n"));
  client.print(F("# TYPE mkr_iot_carrier_soil_moisture_percent gauge\n"));
  client.print(F("mkr_iot_carrier_soil_moisture_percent "));
  printPrometheusValue(client, measurements.soilMoisturePct, 1);

  client.print(F("# HELP mkr_iot_carrier_sample_age_seconds Age of the latest sensor sample.\n"));
  client.print(F("# TYPE mkr_iot_carrier_sample_age_seconds gauge\n"));
  client.print(F("mkr_iot_carrier_sample_age_seconds "));

  if (measurements.sampledAtMs == 0) {
    client.print(F("NaN\n"));
  } else {
    const float sampleAge =
      static_cast<float>(millis() - measurements.sampledAtMs) / 1000.0f;
    client.print(sampleAge, 3);
    client.print('\n');
  }

  client.print(F("# HELP mkr_iot_carrier_samples_total Number of sensor acquisition cycles.\n"));
  client.print(F("# TYPE mkr_iot_carrier_samples_total counter\n"));
  client.print(F("mkr_iot_carrier_samples_total "));
  client.print(samplesTotal);
  client.print('\n');

  client.print(F("# HELP mkr_iot_carrier_wifi_rssi_dbm Wi-Fi received signal strength.\n"));
  client.print(F("# TYPE mkr_iot_carrier_wifi_rssi_dbm gauge\n"));
  client.print(F("mkr_iot_carrier_wifi_rssi_dbm "));
  client.print(WiFi.RSSI());
  client.print('\n');
}

void sendRootResponse(WiFiClient& client) {
  client.println(F("HTTP/1.1 200 OK"));
  client.println(F("Content-Type: text/plain; charset=utf-8"));
  client.println(F("Connection: close"));
  client.println();

  client.println(F("Arduino MKR IoT Carrier Rev1"));
  client.println(F("Prometheus metrics: /metrics"));
}

void sendNotFound(WiFiClient& client) {
  client.println(F("HTTP/1.1 404 Not Found"));
  client.println(F("Content-Type: text/plain; charset=utf-8"));
  client.println(F("Connection: close"));
  client.println();
  client.println(F("Not found"));
}

void handleHttpClient() {
  if (!httpServerStarted) {
    return;
  }

  WiFiClient client = httpServer.available();

  if (!client) {
    return;
  }

  char requestLine[96];

  if (!readRequestLine(client, requestLine, sizeof(requestLine))) {
    client.stop();
    return;
  }

  discardRequestHeaders(client);

  if (strncmp(requestLine, "GET /metrics ", 13) == 0) {
    sendMetrics(client);
  } else if (strncmp(requestLine, "GET / ", 6) == 0) {
    sendRootResponse(client);
  } else {
    sendNotFound(client);
  }

  delay(1);
  client.stop();
}

// -----------------------------------------------------------------------------
// Arduino entry points
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  const uint32_t serialStartedAt = millis();
  while (!Serial && millis() - serialStartedAt < 1500) {
    // Avoid blocking indefinitely when operating without USB.
  }

  initialiseMeasurements();

  carrier.noCase();

  // carrier.begin() aborts early if the LPS22HB pressure sensor is absent.
  // Manually start the peripherals it skips so the rest of the sketch works.
  if (!carrier.begin()) {
    Serial.println(F("Carrier: pressure sensor absent – starting remaining peripherals."));
    carrier.Env.begin();
    carrier.Light.begin();
    carrier.display.init(240, 240);
  }

  pressureAvailable = i2cDevicePresent(0x5C);
  if (!pressureAvailable) {
    Serial.println(F("Pressure sensor (LPS22HB, 0x5C) not found – readings omitted."));
  }

  carrier.display.setRotation(0);
  carrier.display.fillScreen(ST77XX_BLACK);

  sampleSensors();

  lastSensorRead = millis();
  lastPageChange = millis();

  drawGauge();

  if (WiFi.status() == WL_NO_MODULE) {
    wifiModuleAvailable = false;
    Serial.println(F("WiFiNINA module not detected."));
  } else {
    maintainWiFi();
  }

  Serial.println(F("ms,temp_hts221_c,pressure_kpa,temp_lps22hb_c,lux,moisture_pct,samples_total"));
}

void loop() {
  const uint32_t now = millis();

  // Update capacitive touch state. TOUCH0 advances immediately.
  if (
    carrier.Buttons.update() && carrier.Buttons.onTouchDown(NEXT_PAGE_BUTTON)) {
    advancePage(now);
  }

  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    sampleSensors();
    clampPage();
    logSample();
  }

  if (now - lastPageChange >= PAGE_INTERVAL_MS) {
    advancePage(now);
  }

  maintainWiFi();
  handleHttpClient();

  if (displayDirty) {
    drawGauge();
  }
}

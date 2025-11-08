#include <Adafruit_MLX90640.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoNvs.h>
#include <ArduinoOTA.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <NTPClient.h>
#include <WiFi.h>
#include <WiFiUdp.h>

AsyncWebServer server(80);

Adafruit_MLX90640 mlx;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

RTC_DATA_ATTR int lastRestartDay = -1;

#define LED_PIN 21
#define ROWS    24
#define COLS    32
#define PIXELS  (ROWS * COLS)

const float humanThreshold = 5;

uint8_t ignore_mask[PIXELS];

float current_frame[PIXELS];
float pixels[PIXELS];

String output;

const char *sensor = "MLX90640";

float getPixel(int x, int y) {
    if (x <= ROWS && x > 0 && y <= COLS && y > 0) {
        return pixels[x * COLS + y];
    }
    return 0;
}
float getIgnoreMask(int x, int y) {
    if (x <= ROWS && x > 0 && y <= COLS && y > 0) {
        return ignore_mask[x * COLS + y];
    }
    return 0;
}

char data[PIXELS * 10];
JsonDocument doc;

void getRaw() {
    uint16_t data_offset = 0;

    float min = 99;
    float max = 0;
    float avg = 0;
    uint16_t avg_pixels = 0;

    for (int i = 0; i < PIXELS; i++) {
        if (ignore_mask[i] == 0) {
            float pixel_temperature = pixels[i];

            if (pixel_temperature > max) {
                max = pixel_temperature;
            }
            if (pixel_temperature < min) {
                min = pixel_temperature;
            }

            avg += pixel_temperature;
            avg_pixels++;
        }
    }
    avg = avg / avg_pixels;

    for (int i = 0; i < PIXELS; i++) {
        float pixel_temperature = ignore_mask[i] == 0 ? pixels[i] : 0;

        uint16_t written = sprintf(data + data_offset, "%0.1f", pixel_temperature);
        data_offset += written;

        if (i < PIXELS - 1) {
            data[data_offset] = ',';
            data_offset++;
        }
    }
    data[data_offset] = 0;

    float personThreshold = humanThreshold + avg;

    bool person_detected = false;
    for (int i = 0; i < PIXELS; i++) {
        int r = i / COLS;
        int c = i % COLS;

        if (getIgnoreMask(r, c) != 0) {
            continue;
        }
        if (getPixel(r, c) > personThreshold) {
            int blobSize = 1;

            int neighbourThreshold = avg + 1.0;

            if (getPixel(r + 1, c - 1) > neighbourThreshold) {
                blobSize++;
            }
            if (getPixel(r + 1, c) > neighbourThreshold) {
                blobSize++;
            }
            if (getPixel(r + 1, c + 1) > neighbourThreshold) {
                blobSize++;
            }
            if (getPixel(r, c + 1) > neighbourThreshold) {
                blobSize++;
            }
            if (getPixel(r, c - 1) > neighbourThreshold) {
                blobSize++;
            }
            if (getPixel(r - 1, c - 1) > neighbourThreshold) {
                blobSize++;
            }
            if (getPixel(r - 1, c) > neighbourThreshold) {
                blobSize++;
            }
            if (getPixel(r - 1, c + 1) > neighbourThreshold) {
                blobSize++;
            }

            if (blobSize >= 4) {
                person_detected = true;
            }
        }
    }

    doc["sensor"] = sensor;
    doc["rows"] = ROWS;
    doc["cols"] = COLS;
    doc["data"] = data;
    doc["min"] = (int)(min * 10.0) / 10.0;
    doc["max"] = (int)(max * 10.0) / 10.0;
    doc["avg"] = (int)(avg * 10.0) / 10.0;
    doc["person_detected"] = person_detected;

    serializeJson(doc, output);
}

void printSensorInfo() {
    Serial.print("Serial number: ");
    Serial.print(mlx.serialNumber[0], HEX);
    Serial.print(mlx.serialNumber[1], HEX);
    Serial.println(mlx.serialNumber[2], HEX);

    Serial.print("Current mode: ");
    if (mlx.getMode() == MLX90640_CHESS) {
        Serial.println("Chess");
    } else {
        Serial.println("Interleave");
    }

    Serial.print("Current resolution: ");
    mlx90640_resolution_t res = mlx.getResolution();
    switch (res) {
        case MLX90640_ADC_16BIT:
            Serial.println("16 bit");
            break;
        case MLX90640_ADC_17BIT:
            Serial.println("17 bit");
            break;
        case MLX90640_ADC_18BIT:
            Serial.println("18 bit");
            break;
        case MLX90640_ADC_19BIT:
            Serial.println("19 bit");
            break;
    }

    Serial.print("Current frame rate: ");
    mlx90640_refreshrate_t rate = mlx.getRefreshRate();
    switch (rate) {
        case MLX90640_0_5_HZ:
            Serial.println("0.5 Hz");
            break;
        case MLX90640_1_HZ:
            Serial.println("1 Hz");
            break;
        case MLX90640_2_HZ:
            Serial.println("2 Hz");
            break;
        case MLX90640_4_HZ:
            Serial.println("4 Hz");
            break;
        case MLX90640_8_HZ:
            Serial.println("8 Hz");
            break;
        case MLX90640_16_HZ:
            Serial.println("16 Hz");
            break;
        case MLX90640_32_HZ:
            Serial.println("32 Hz");
            break;
        case MLX90640_64_HZ:
            Serial.println("64 Hz");
            break;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Boot");

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    NVS.begin();

    NVS.getBlob("ignore_mask", ignore_mask, sizeof(ignore_mask));

    Wire.begin(22, 19, 800000);
    if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
        Serial.println("MLX90640 not found!");
    } else {
        Serial.println("Found Adafruit MLX90640");
    }

    mlx.setMode(MLX90640_CHESS);
    mlx.setResolution(MLX90640_ADC_19BIT);
    mlx.setRefreshRate(MLX90640_16_HZ);

    printSensorInfo();

    WiFi.mode(WIFI_STA);

    WiFi.begin("<WiFi SSID>", "<WiFi Password>");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected!");
    Serial.print("Got IP: ");
    Serial.println(WiFi.localIP());

    timeClient.begin();
    timeClient.update();
    Serial.println("NTP time synchronized!");
    Serial.print("Current time: ");
    Serial.println(timeClient.getFormattedTime());

    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else  // U_SPIFFS
                type = "filesystem";

            Serial.println("Start updating " + type);
        })
        .onEnd([]() { Serial.println("\nEnd"); })
        .onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
            Serial.printf("Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR)
                Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR)
                Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR)
                Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR)
                Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR)
                Serial.println("End Failed");
        });

    ArduinoOTA.begin();

    server.on("/raw", [](AsyncWebServerRequest *request) {
        getRaw();
        request->send(200, "application/json", output);
    });

    server.on("/restart", [](AsyncWebServerRequest *request) {
        request->send(200, "application/json", output);
        ESP.restart();
    });

    server.on(
        "/ignore_mask", HTTP_POST,
        [](AsyncWebServerRequest *request) {
            if (request->_tempObject != NULL) {
                JsonDocument doc;
                deserializeJson(doc, (const char *)request->_tempObject);
                JsonArray new_ignore_mask = doc.as<JsonArray>();

                if (new_ignore_mask.size() == PIXELS) {
                    char nvs_name[10];
                    uint8_t mask_temp = 0;
                    uint8_t mask_index = 0;
                    uint8_t nvs_index = 0;

                    for (int i = 0; i < PIXELS; i++) {
                        uint8_t temp = new_ignore_mask[i];
                        ignore_mask[i] = temp == 0 ? 0 : 1;
                    }
                    NVS.eraseAll();
                    NVS.setBlob("ignore_mask", ignore_mask, sizeof(ignore_mask));
                    NVS.commit();

                    request->send(200);
                    return;
                }
            }
            request->send(400);
        },
        NULL,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            if (index == 0) {
                if (request->_tempObject == NULL) {
                    request->_tempObject = calloc(total + 1, sizeof(uint8_t));

                    if (request->_tempObject == NULL) {
                        request->abort();
                        return;
                    }
                }
            }

            if (request->_tempObject != NULL) {
                uint8_t *buffer = (uint8_t *)request->_tempObject;
                memcpy(buffer + index, data, len);
            }
        });

    server.onNotFound([](AsyncWebServerRequest *request) { request->send(404); });

    server.begin();

    digitalWrite(LED_PIN, HIGH);
}

uint32_t last_frame_update;
void loop() {
    if (millis() - last_frame_update >= 125) {
        if (mlx.getFrame(current_frame) == 0) {
            last_frame_update = millis();

            for (int i = 0; i < PIXELS; i++) {
                if (isnan(current_frame[i])) {
                    continue;
                }

                pixels[i] = (current_frame[i] + pixels[i]) / 2;
            }
        }
    }

    ArduinoOTA.handle();

    timeClient.update();

    uint32_t not_connected_start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");

        if (millis() - not_connected_start > 30000UL) {
            ESP.restart();
        }
    }
}

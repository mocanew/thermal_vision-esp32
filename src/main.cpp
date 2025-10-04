#include <Adafruit_MLX90640.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <NTPClient.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>

WebServer server(80);

Adafruit_MLX90640 mlx;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

RTC_DATA_ATTR int lastRestartDay = -1;

#define ROWS   24
#define COLS   32
#define PIXELS (ROWS * COLS)

// all uint16_t are fixed decimal (eg 35 means 3.5°C)
const uint16_t humanThreshold = 35;

float current_frame[PIXELS];
uint16_t pixels[PIXELS];

String output;

const char *sensor = "MLX90640";

uint16_t getPixel(int x, int y) {
    if (x <= ROWS && x > 0 && y <= COLS && y > 0) {
        return pixels[x * COLS + y];
    }
    return 0;
}

char data[PIXELS * 5];
JsonDocument doc;

void getRaw(uint8_t send_pixels) {
    uint16_t data_offset = 0;

    uint16_t min = pixels[0];
    uint16_t max = pixels[0];
    uint32_t avg = 0;

    for (int i = 0; i < PIXELS; i++) {
        uint16_t pixel_temperature = pixels[i];

        if (pixel_temperature > max) {
            max = pixel_temperature;
        }
        if (pixel_temperature < min) {
            min = pixel_temperature;
        }

        avg += pixel_temperature;

        if (send_pixels == 1) {
            uint16_t written = sprintf(data + data_offset, "%u", pixel_temperature);
            data_offset += written;

            if (i < PIXELS - 1) {
                data[data_offset] = ',';
                data_offset++;
            }
        }
    }
    data[data_offset] = 0;

    avg = avg / PIXELS;
    uint16_t personThreshold = humanThreshold + avg;

    bool person_detected = false;
    for (int i = 0; i < PIXELS; i++) {
        int r = i / COLS;
        int c = i % COLS;

        if (c >= COLS - 7) {
            continue;
        }
        if (getPixel(r, c) > personThreshold) {
            int blobSize = 1;

            int neighbourThreshold = avg + 10;

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
    doc["data"] = send_pixels == 1 ? data : String("0").c_str();
    doc["min"] = min;
    doc["max"] = max;
    doc["avg"] = avg;
    doc["person_detected"] = person_detected;

    String new_output;
    serializeJson(doc, new_output);
    output = new_output;
}

void sendRaw() {
    getRaw(1);
    server.send(200, "application/json", output.c_str());
}
void sendBinarySensor() {
    getRaw(0);
    server.send(200, "application/json", output.c_str());
}

void notFound() { server.send(404, "text/plain", "Not found"); }

void setup() {
    // while (!Serial)
    //   delay(10);
    Serial.begin(115200);

    Wire.begin(1, 3, 800000);
    Serial.println("Adafruit MLX90640");
    if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
        Serial.println("MLX90640 not found!");
        while (1) delay(10);
    }
    Serial.println("Found Adafruit MLX90640");

    Serial.print("Serial number: ");
    Serial.print(mlx.serialNumber[0], HEX);
    Serial.print(mlx.serialNumber[1], HEX);
    Serial.println(mlx.serialNumber[2], HEX);

    mlx.setMode(MLX90640_CHESS);
    Serial.print("Current mode: ");
    if (mlx.getMode() == MLX90640_CHESS) {
        Serial.println("Chess");
    } else {
        Serial.println("Interleave");
    }

    mlx.setResolution(MLX90640_ADC_19BIT);
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

    mlx.setRefreshRate(MLX90640_8_HZ);
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

    server.on("/binary_sensor/raw", sendBinarySensor);
    server.on("/raw", sendRaw);

    server.onNotFound(notFound);

    server.begin();
}

void loop() {
    if (mlx.getFrame(current_frame) == 0) {
        for (int i = 0; i < PIXELS; i++) {
            pixels[i] = ((uint16_t)(current_frame[i] * 10.0f)) / 2 + pixels[i] / 2;
        }
    }

    server.handleClient();
    ArduinoOTA.handle();

    timeClient.update();

    if (timeClient.getHours() == 5) {
        int currentDay = timeClient.getDay();

        if (currentDay != lastRestartDay) {
            Serial.println("Restarting ESP32...");
            lastRestartDay = currentDay;
            delay(1000);

            // 1s deep sleep instead of restart to keep lastRestartDay in memory
            esp_sleep_enable_timer_wakeup(1000000);
            esp_deep_sleep_start();
        }
    }
}

#include <Arduino.h> // Main Arduino library, required for projects that use the Arduino framework.
#include <WiFi.h> // Main library for WiFi connectivity, also used by AsyncWebServer.
#include <unordered_map> // Hashtable for storing WiFi credentials.
#include <AsyncTCP.h> // Asynchronous TCP library for the ESP32.
#include <ESPAsyncWebServer.h> // Asynchronous web server for the ESP32.
#include <AsyncElegantOTA.h> // Over the air updates for the ESP32.
#include <TFT_eSPI.h>     // Hardware-specific library
#include <TFT_eWidget.h>  // Widget library
#include "arariboat\mavlink.h" // Custom mavlink dialect for the boat generated using Mavgen tool.

// Declare a handle for each task to allow manipulation of the task from other tasks, such as sending notifications, resuming or suspending.
// The handle is initialized to nullptr to avoid the task being created before the setup() function.
// Each handle is then assigned to the task created in the setup() function.

TaskHandle_t ledBlinkerTask = nullptr;
TaskHandle_t wifiConnectionTask = nullptr;
TaskHandle_t serverTask = nullptr;
TaskHandle_t serialReaderTask = nullptr;
TaskHandle_t cockpitDisplayTask = nullptr;
TaskHandle_t highWaterMeasurerTask = nullptr;

// Array of pointers to the task handles. This allows to iterate over the array and perform operations on all tasks, such as resuming, suspending or reading free stack memory.
TaskHandle_t* taskHandles[] = { &ledBlinkerTask, &wifiConnectionTask, &serverTask, &serialReaderTask, &cockpitDisplayTask, &highWaterMeasurerTask };
constexpr auto taskHandlesSize = sizeof(taskHandles) / sizeof(taskHandles[0]); // Get the number of elements in the array.

enum BlinkRate : uint32_t {
    Slow = 1000,
    Medium = 500,
    Fast = 100,
    Pulse = 100
};

// Tasks can send notifications here to change the blink rate of the LED in order to communicate the status of the boat.
void FastBlinkPulse(int pin);
void LedBlinker(void* parameter) {

    constexpr int ledPin = 25;
    pinMode(ledPin, OUTPUT);

    uint32_t blinkRate = BlinkRate::Slow;
    uint32_t previousBlinkRate = blinkRate;
    
    while (true) {
        digitalWrite(ledPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(blinkRate));
        digitalWrite(ledPin, LOW);
        vTaskDelay(pdMS_TO_TICKS(blinkRate));
        
        // Set blink rate to the value received from the notification
        if (xTaskNotifyWait(0, 0, (uint32_t*)&blinkRate, 0) == pdTRUE) {
            Serial.printf("Received notification to change blink rate to %d\n", blinkRate);
            if (blinkRate == BlinkRate::Pulse) {
                FastBlinkPulse(ledPin);
                blinkRate = previousBlinkRate;
            }
        }
    }
}

void FastBlinkPulse(int pin) {

    for (int i = 0; i < 10; i++) {
        digitalWrite(pin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(pin, LOW);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void WifiConnectionTask(void* parameter) {
    
    // Store WiFi credentials in a hashtable.
    std::unordered_map<const char*, const char*> wifiCredentials;
    wifiCredentials["Ursula"] = "biaviad36";
    wifiCredentials["EMobil 1"] = "faraboia";
    wifiCredentials["Innorouter"] = "innomaker";
    wifiCredentials["NITEE"] = "nitee123";
    
    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.mode(WIFI_STA);
            xTaskNotify(ledBlinkerTask, BlinkRate::Fast, eSetValueWithOverwrite);
            for (auto& wifi : wifiCredentials) {
                WiFi.begin(wifi.first, wifi.second);
                Serial.printf("Trying to connect to %s\n", wifi.first);
                int i = 0;
                while (WiFi.status() != WL_CONNECTED) {
                    vTaskDelay(pdMS_TO_TICKS(500));
                    Serial.print(".");
                    i++;
                    if (i > 5) {
                        Serial.printf("Failed to connect to %s\n", wifi.first);
                        break;
                    }
                }
                if (WiFi.status() == WL_CONNECTED) {
                    Serial.printf("Connected to %s\nIP: %s\n", wifi.first, WiFi.localIP().toString().c_str());
                    xTaskNotify(ledBlinkerTask, BlinkRate::Slow, eSetValueWithOverwrite);
                    xTaskNotifyGive(serverTask);
                    break;
                }
            }          
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

void ServerTask(void* parameter) {

    // Create an async web server on port 80. This is the default port for HTTP. 
    // Async server can handle multiple requests at the same time without blocking the task.
    AsyncWebServer server(80);
    
    // Setup URL routes and attach callback methods to them. A callback method is called when a request is made to the URL.
    // The callbacks must have the signature void(AsyncWebServerRequest *request). Any function with this signature can be used.
    // Preferably, use lambda functions to keep the code in the same place.
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", "<h1>Lora32</h1><p>WiFi connected: " + WiFi.SSID() + "</p><p>IP address: " + WiFi.localIP().toString() + "</p>");
    });

    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        // log reset message
        request->send(200, "text/html", "<h1>Boat32</h1><p>Resetting...</p>");
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP.restart();
    });

    // Wait for notification from WifiConnection task that WiFi is connected in order to begin the server
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    // Attach OTA update handler to the server and initialize the server.
    AsyncElegantOTA.begin(&server); // Available at http://[esp32ip]/update or http://[esp32hostname]/update
    server.begin();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

template <std::size_t N>
void ProcessSerialMessage(const std::array<uint8_t, N> &buffer);
void SerialReaderTask(void* parameter) {
    
    std::array<uint8_t, 32> buffer = { 0 };
    static size_t bufferIndex = 0;
    while (true) {
        if (Serial.available()) {
            uint8_t receivedChar = Serial.read();
            switch (receivedChar) {
                case '\r':
                case '\n':
                    ProcessSerialMessage(buffer);
                    bufferIndex = 0;
                    buffer.fill(0);
                    break;
                default:
                    if (bufferIndex == buffer.size()) {
                        ProcessSerialMessage(buffer);
                        bufferIndex = 0;
                        buffer.fill(0);
                        break;
                    }
                    buffer[bufferIndex++] = receivedChar;
                    buffer[bufferIndex] = 0;
                    break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

template <std::size_t N>
void ProcessSerialMessage(const std::array<uint8_t, N> &buffer) {

    char command = buffer[0];
    char value = buffer[1];

    switch (command) {

        case 'B' : {
                
                static const std::unordered_map<char, BlinkRate> blinkRateMap = {
                    {'0', BlinkRate::Slow},
                    {'1', BlinkRate::Medium},
                    {'2', BlinkRate::Fast}
                };

                auto it = blinkRateMap.find(buffer[1]);
                if (it != blinkRateMap.end()) {
                    xTaskNotify(ledBlinkerTask, (uint32_t)it->second, eSetValueWithOverwrite);
                    Serial.printf("Blink rate set to %c\n", buffer[1]);
                    break;
                }
                else {
                    Serial.printf("Invalid blink rate: %c\n", buffer[1]);
                    break;
                }
            break;
        }

        case '\r':
        case '\n':
            break;

        default:
            break;
    }
}

float mapValue(float ip, float ipmin, float ipmax, float tomin, float tomax);
void CockpitDisplayTask(void* parameter) {

    //Needs Font 2 (also Font 4 if using large scale label)
    //Make sure all the display driver and pin connections are correct by
    //editing the User_Setup.h file in the TFT_eSPI library folder.

    #define LOOP_PERIOD 35 // Display updates every 35 ms
    TFT_eSPI tft_display = TFT_eSPI();      // Invoke custom library
    MeterWidget widget_battery_volts    = MeterWidget(&tft_display);
    MeterWidget widget_battery_current  = MeterWidget(&tft_display);
    MeterWidget widget_motor_current    = MeterWidget(&tft_display);
    MeterWidget widget_mppt_current     = MeterWidget(&tft_display);

    constexpr float battery_volts_full_scale = 54.0;
    constexpr float battery_volts_zero_scale = 48.0;
    constexpr float battery_amps_full_scale = 60.0;
    constexpr float battery_amps_zero_scale = 0.0;
    constexpr float motor_amps_full_scale = 60.0;
    constexpr float motor_amps_zero_scale = 0.0;
    constexpr float mppt_amps_full_scale = 40.0;
    constexpr float mppt_amps_zero_scale = 0.0;

    constexpr float widget_length = 239.0f;
    constexpr float widget_height = 126.0f;

    tft_display.init();
    tft_display.setRotation(3);
    tft_display.fillScreen(TFT_BLACK);
    tft_display.drawString("Corrente-Bateria", 240 + widget_length / 7, 2, 4);
    tft_display.drawString("Tensao-Bateria", widget_length / 7, 2, 4);
    tft_display.drawString("Corrente-Motor", widget_length / 7, 160, 4);
    tft_display.drawString("Corrente-MPPT", 240 + widget_length / 7, 160, 4);

    // Colour zones are set as a start and end percentage of full scale (0-100)
    // If start and end of a colour zone are the same then that colour is not used
    //                              -Red-   -Org-  -Yell-  -Grn-
    widget_battery_volts.setZones(0, 100, 15, 25, 0, 0, 25, 100);
    widget_battery_volts.analogMeter(0, 30, battery_volts_zero_scale, battery_volts_full_scale, "V", "48.0", "49.5", "51.0", "52.5", "54.0"); 

    //                              --Red--  -Org-   -Yell-  -Grn-
    widget_battery_current.setZones(75, 100, 50, 75, 25, 50, 0, 25); // Example here red starts at 75% and ends at 100% of full scale
    widget_battery_current.analogMeter(240, 30, battery_amps_zero_scale, battery_amps_full_scale, "A", "0", "15", "30", "45", "60"); 

    //                              -Red-   -Org-  -Yell-  -Grn-
    widget_motor_current.setZones(75, 100, 50, 75, 25, 50, 0, 25); // Example here red starts at 75% and ends at 100% of full scale
    widget_motor_current.analogMeter(0, 180, motor_amps_zero_scale, motor_amps_full_scale, "A", "0", "15", "30", "45", "60"); 

    //                           -Red-   -Org-  -Yell-  -Grn-
    widget_mppt_current.setZones(75, 100, 50, 75, 25, 50, 0, 25); // Example here red starts at 75% and ends at 100% of full scale
    widget_mppt_current.analogMeter(240, 180, mppt_amps_zero_scale, mppt_amps_full_scale, "A", "0", "10", "20", "30", "40"); 

    while (true) {
        
        static float angle = 0.0f;
        static uint32_t updateTime = 0;  
        constexpr float radians_to_degrees = 3.14159265358979323846 / 180.0;

        if (millis() - updateTime >= LOOP_PERIOD) {
            updateTime = millis();
            angle += 4; if (angle > 360) angle = 0;

            // Create a Sine wave for testing, value is in range 0 - 100
            float value = 50.0 + 50.0 * sin(angle * radians_to_degrees);

            float battery_current;
            battery_current = mapValue(value, (float)0.0, (float)100.0, battery_amps_zero_scale, battery_amps_full_scale);
            widget_battery_current.updateNeedle(battery_current, 0);

            float battery_voltage;
            battery_voltage = mapValue(value, (float)0.0, (float)100.0, battery_volts_zero_scale, battery_volts_full_scale);
            widget_battery_volts.updateNeedle(battery_voltage, 0);
            
            float motor_current;
            motor_current = mapValue(value, (float)0.0, (float)100.0, motor_amps_zero_scale, motor_amps_full_scale);
            widget_motor_current.updateNeedle(motor_current, 0);

            float mppt_current;
            mppt_current = mapValue(value, (float)0.0, (float)100.0, mppt_amps_zero_scale, mppt_amps_full_scale);
            widget_mppt_current.updateNeedle(mppt_current, 0);
        }
        vTaskDelay(pdTICKS_TO_MS(10));
    }
}

float mapValue(float ip, float ipmin, float ipmax, float tomin, float tomax) {

  return tomin + (((tomax - tomin) * (ip - ipmin))/ (ipmax - ipmin));
}


/// @brief Auxiliary task to measure free stack memory of each task and free heap of the system.
/// Useful to detect possible stack overflows on a task and allocate more stack memory for it if necessary.
/// @param parameter Unused. Just here to comply with the task function signature.
void HighWaterMeasurerTask(void* parameter) {
    while (true) {
        Serial.printf("\n");
        for (int i = 0; i < taskHandlesSize; i++) {
            Serial.printf("Task %s has %d bytes of free stack\n", pcTaskGetTaskName(*taskHandles[i]), uxTaskGetStackHighWaterMark(*taskHandles[i]));
        }
        // free heap
        Serial.printf("Free heap: %d\n", esp_get_free_heap_size());
        Serial.printf("\n");
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

void setup() {
    Serial.begin(115200);
    xTaskCreate(LedBlinker, "ledBlinker", 2048, NULL, 1, &ledBlinkerTask);
    xTaskCreate(WifiConnectionTask, "wifiConnection", 4096, NULL, 3, &wifiConnectionTask);
    xTaskCreate(ServerTask, "server", 4096, NULL, 1, &serverTask);
    xTaskCreate(SerialReaderTask, "serialReader", 4096, NULL, 1, &serialReaderTask);
    xTaskCreate(CockpitDisplayTask, "cockpitDisplay", 4096, NULL, 3, &cockpitDisplayTask);
    xTaskCreate(HighWaterMeasurerTask, "measurer", 2048, NULL, 1, NULL);  
}

void loop() {

}


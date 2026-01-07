// THIS IS A FILE FOR QUICK BENCH TESTS - CHANNEL 3 (CURRENT) & CHANNEL 2 (VOLTAGE)
// DECOUPLED ACQUISITION AND REPORTING WITH COULOMB (Ah) AND ENERGY (Wh) COUNTING

#include <Arduino.h>
#include "Adafruit_ADS1X15.h"
#include <Wire.h>

// Alias for the ADS1115 class
using ADS1115 = Adafruit_ADS1115;

// GPIO pins for I2C
constexpr gpio_num_t I2C_SCL_PIN = PIN_SCL;
constexpr gpio_num_t I2C_SDA_PIN = PIN_SDA;

// --- Configuration Structure for the Channel ---
struct ChannelConfig {
    float slope;      // Calibration slope
    float intercept;  // Calibration intercept
    adsGain_t gain;   // ADC gain setting
};

// --- Updated Channel Configuration ---
// Channel 2 (Voltage)
ChannelConfig ch2_config = { 0.002319108797f, -0.001543204122f, GAIN_ONE };
// Channel 3 (Current)
ChannelConfig ch3_config = { 0.002442646111f, -12.46604575f, GAIN_EIGHT };

// Create an instance of the ADS1115
ADS1115 adc;

// --- Shared State for Decoupling ---
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;
volatile float latest_current = 0.0f;
volatile float latest_voltage = 0.0f;
volatile float cumulative_ah = 0.0f; // Ampere-hours (Ah)
volatile float cumulative_wh = 0.0f; // Watt-hours (Wh)

void InstrumentationTask(void *parameter) {
    // Initialize the I2C bus
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // --- Scan for ADC on I2C bus ---
    uint8_t found_address = 0;
    while (found_address == 0) {
        for (uint8_t addr = 0x48; addr <= 0x4B; addr++) {
            if (adc.begin(addr)) {
                found_address = addr;
                break;
            }
        }
        if (found_address == 0) vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Configure the ADC data rate (Set to max for fast acquisition)
    adc.setDataRate(RATE_ADS1115_860SPS);

    // Variables for local processing
    int16_t raw_ch3, raw_ch2;
    float current_calc, voltage_calc;
    
    // Integration variables
    uint32_t last_micros = micros();
    float total_as = 0.0f; // Cumulative Ampere-seconds
    float total_ws = 0.0f; // Cumulative Watt-seconds

    while (true) {
        // --- High Speed Data Acquisition ---
        
        // 1. Read Channel 3 (Current)
        adc.setGain(ch3_config.gain);
        raw_ch3 = adc.readADC_SingleEnded(3);
        current_calc = (static_cast<float>(raw_ch3) * ch3_config.slope) + ch3_config.intercept;

        // 2. Read Channel 2 (Voltage)
        adc.setGain(ch2_config.gain); 
        raw_ch2 = adc.readADC_SingleEnded(2);
        voltage_calc = (static_cast<float>(raw_ch2) * ch2_config.slope) + ch2_config.intercept;

        // --- Integration Logic ---
        uint32_t now_micros = micros();
        float dt_s = (now_micros - last_micros) / 1000000.0f;
        last_micros = now_micros;

        // Charge (Coulombs): Q = I * dt
        total_as += (current_calc * dt_s);
        
        // Energy (Joules/Watt-seconds): E = V * I * dt
        total_ws += (voltage_calc * current_calc * dt_s);

        // --- Update Shared State (Thread Safe) ---
        portENTER_CRITICAL(&dataMux);
        latest_current = current_calc;
        latest_voltage = voltage_calc;
        cumulative_ah = total_as / 3600.0f;
        cumulative_wh = total_ws / 3600.0f;
        portEXIT_CRITICAL(&dataMux);

        // Minimal yield to prevent watchdog issues
        vTaskDelay(1); 
    }
}

void ReportingTask(void *parameter) {
    // Wait for the instrumentation task to initialize
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Print CSV Header
    Serial.println("Timestamp_s,Current_A,Voltage_V,Capacity_Ah,Energy_Wh");

    // Timing Setup
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // Exactly 1 second
    
    uint32_t start_time_ms = millis();

    while (true) {
        // Wait for the next 1-second interval precisely
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // --- Snapshot Shared State (Thread Safe) ---
        float current_to_print;
        float voltage_to_print;
        float ah_to_print;
        float wh_to_print;

        portENTER_CRITICAL(&dataMux);
        current_to_print = latest_current;
        voltage_to_print = latest_voltage;
        ah_to_print = cumulative_ah;
        wh_to_print = cumulative_wh;
        portEXIT_CRITICAL(&dataMux);

        // --- CSV Output ---
        float relative_timestamp_s = (millis() - start_time_ms) / 1000.0f;
        Serial.printf("%.0f,%.2f,%.2f,%.4f,%.4f\n",
            relative_timestamp_s,
            current_to_print,
            voltage_to_print,
            ah_to_print,
            wh_to_print
        );
    }
}

void setup () {
    Serial.begin(115200);
    while (!Serial) { vTaskDelay(pdMS_TO_TICKS(10)); }

    // Task 1: High-Speed Acquisition (Core 1)
    xTaskCreatePinnedToCore(
        InstrumentationTask,
        "AcqTask",
        4096,
        nullptr,
        2, 
        nullptr,
        1
    );

    // Task 2: Precise 1s Reporting (Core 0)
    xTaskCreatePinnedToCore(
        ReportingTask,
        "ReportTask",
        4096,
        nullptr,
        1, 
        nullptr,
        0
    );
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
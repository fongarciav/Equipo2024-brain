#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hardware.h"
#include "mailbox.h"
#include "motor_task.h"
#include "steer_task.h"
#include "lights_task.h"
#include "link_rx_task.h"
#include "link_tx_task.h"
#include "supervisor_task.h"
#include "web_task.h"
#include "ultrasonic_task.h"

// Mailboxes
static mailbox_t motor_mailbox;
static mailbox_t steer_mailbox;
static mailbox_t lights_mailbox;
static mailbox_t supervisor_mailbox;

// Task handles
#define STACK_SIZE_4K 4096
#define STACK_SIZE_8K 8192

void setup(void)
{
    Serial.begin(115200);
    
    // Wait for serial monitor to connect (ESP32 Serial is always available,
    // but we need time for the monitor to open)
    delay(5000);

    Serial.println("========================================");
    Serial.println("ESP32 RC Car FreeRTOS System Starting...");
    Serial.println("========================================");

    // Initialize hardware first
    hardware_init();

    // Initialize mailboxes
    mailbox_init(&motor_mailbox);
    mailbox_init(&steer_mailbox);
    mailbox_init(&lights_mailbox);
    mailbox_init(&supervisor_mailbox);

    Serial.println("[main] Mailboxes initialized");

    // Create tasks with core pinning and priorities as specified

    // LinkRxTask - Core 1, Priority 4
    link_rx_params_t link_rx_params = {
        .motor_mailbox = &motor_mailbox,
        .steer_mailbox = &steer_mailbox,
        .lights_mailbox = &lights_mailbox,
        .supervisor_mailbox = &supervisor_mailbox};
    xTaskCreatePinnedToCore(
        link_rx_task,
        "LinkRxTask",
        STACK_SIZE_8K,
        &link_rx_params,
        4, // Priority 4
        NULL,
        1 // Core 1
    );
    Serial.println("[main] LinkRxTask created on Core 1, Priority 4");

    // MotorTask - Core 0, Priority 4
    xTaskCreatePinnedToCore(
        motor_task,
        "MotorTask",
        STACK_SIZE_4K,
        &motor_mailbox,
        4, // Priority 4
        NULL,
        0 // Core 0
    );
    Serial.println("[main] MotorTask created on Core 0, Priority 4");

    // SteerTask - Core 0, Priority 3
    xTaskCreatePinnedToCore(
        steer_task,
        "SteerTask",
        STACK_SIZE_4K,
        &steer_mailbox,
        3, // Priority 3
        NULL,
        0 // Core 0
    );
    Serial.println("[main] SteerTask created on Core 0, Priority 3");

    // LightsTask - Core 1, Priority 1
    xTaskCreatePinnedToCore(
        lights_task,
        "LightsTask",
        STACK_SIZE_4K,
        &lights_mailbox,
        1, // Priority 1
        NULL,
        1 // Core 1
    );
    Serial.println("[main] LightsTask created on Core 1, Priority 1");

    // SupervisorTask - Core 1, Priority 2
    supervisor_params_t supervisor_params = {
        .supervisor_mailbox = &supervisor_mailbox,
        .motor_mailbox = &motor_mailbox,
        .steer_mailbox = &steer_mailbox};
    xTaskCreatePinnedToCore(
        supervisor_task,
        "SupervisorTask",
        STACK_SIZE_4K,
        &supervisor_params,
        2, // Priority 2
        NULL,
        1 // Core 1
    );
    Serial.println("[main] SupervisorTask created on Core 1, Priority 2");

    // LinkTxTask - Core 1, Priority 2
    xTaskCreatePinnedToCore(
        link_tx_task,
        "LinkTxTask",
        STACK_SIZE_4K,
        NULL,
        2, // Priority 2
        NULL,
        1 // Core 1
    );
    Serial.println("[main] LinkTxTask created on Core 1, Priority 2");

    // WebTask - Core 1, Priority 2
    web_task_params_t web_params = {
        .motor_mailbox = &motor_mailbox,
        .steer_mailbox = &steer_mailbox,
        .lights_mailbox = &lights_mailbox,
        .supervisor_mailbox = &supervisor_mailbox};
    xTaskCreatePinnedToCore(
        web_task,
        "WebTask",
        STACK_SIZE_8K,
        &web_params,
        2, // Priority 2
        NULL,
        1 // Core 1
    );
    Serial.println("[main] WebTask created on Core 1, Priority 2");

    // UltrasonicTask - Core 0, Priority 5 (high priority for safety)
    xTaskCreatePinnedToCore(
        ultrasonic_task,
        "UltrasonicTask",
        STACK_SIZE_4K,
        NULL,
        5, // Priority 5 (higher than motor task for safety)
        NULL,
        0 // Core 0
    );
    Serial.println("[main] UltrasonicTask created on Core 0, Priority 5");

    Serial.println("[main] All tasks created. FreeRTOS scheduler running...");
    Serial.println("[main] System ready!");
}

void loop(void)
{
    // FreeRTOS tasks handle everything, this loop should not execute
    // But we keep it to satisfy Arduino framework requirements
    vTaskDelay(pdMS_TO_TICKS(1000));
}

#include "web_task.h"
#include "hardware.h"
#include "mailbox.h"
#include "messages.h"
#include "supervisor_task.h"
#include "motor_task.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <string.h>
#include "webpage.h"

#define WIFI_AP_SSID "RC-Car-ESP32"
#define WIFI_AP_PASSWORD ""  // Open AP

static mailbox_t *motor_mb = NULL;
static mailbox_t *steer_mb = NULL;
static mailbox_t *lights_mb = NULL;
static mailbox_t *supervisor_mb = NULL;
static WebServer server(80);

void init_wifi_ap(void) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
    
    Serial.print("[WebTask] Wi-Fi AP started. SSID: ");
    Serial.print(WIFI_AP_SSID);
    Serial.print(", IP: ");
    Serial.println(WiFi.softAPIP());
    
    // Turn on built-in LED when WiFi AP is ready
    digitalWrite(GPIO_LED_BUILTIN, HIGH);
    Serial.println("[WebTask] Built-in LED turned ON (WiFi AP ready)");
}

void web_task(void *pvParameters) {
    web_task_params_t *params = (web_task_params_t *)pvParameters;
    motor_mb = params->motor_mailbox;
    steer_mb = params->steer_mailbox;
    lights_mb = params->lights_mailbox;
    supervisor_mb = params->supervisor_mailbox;
    
    // Initialize Wi-Fi AP
    init_wifi_ap();
    
    // Root handler
    server.on("/", []() {
        server.send(200, "text/html", webpage);
    });
    
    // Motor control
    server.on("/forward", []() {
        if (supervisor_get_mode() == MODE_MANUAL && supervisor_get_state() == STATE_RUNNING) {
            if (motor_mb != NULL) {
                mailbox_write(motor_mb, TOPIC_MOTOR, CMD_SET_SPEED, MOTOR_SPEED_MAX, 100);
                motor_set_direction(true);
            }
        }
        server.send(200, "text/plain", "forward");
    });
    
    server.on("/back", []() {
        if (supervisor_get_mode() == MODE_MANUAL && supervisor_get_state() == STATE_RUNNING) {
            if (motor_mb != NULL) {
                mailbox_write(motor_mb, TOPIC_MOTOR, CMD_SET_SPEED, MOTOR_SPEED_MAX, 100);
                motor_set_direction(false);
            }
        }
        server.send(200, "text/plain", "back");
    });
    
    server.on("/driveStop", []() {
        if (supervisor_get_mode() == MODE_MANUAL) {
            if (motor_mb != NULL) {
                mailbox_write(motor_mb, TOPIC_MOTOR, CMD_STOP, 0, 100);
            }
        }
        server.send(200, "text/plain", "driveStop");
    });
    
    // Steering control
    server.on("/left", []() {
        if (supervisor_get_mode() == MODE_MANUAL && supervisor_get_state() == STATE_RUNNING) {
            if (steer_mb != NULL) {
                mailbox_write(steer_mb, TOPIC_STEER, CMD_SET_STEER, SERVO_LEFT, 100);
            }
        }
        server.send(200, "text/plain", "left");
    });
    
    server.on("/right", []() {
        if (supervisor_get_mode() == MODE_MANUAL && supervisor_get_state() == STATE_RUNNING) {
            if (steer_mb != NULL) {
                mailbox_write(steer_mb, TOPIC_STEER, CMD_SET_STEER, SERVO_RIGHT, 100);
            }
        }
        server.send(200, "text/plain", "right");
    });
    
    server.on("/steerStop", []() {
        if (supervisor_get_mode() == MODE_MANUAL) {
            if (steer_mb != NULL) {
                mailbox_write(steer_mb, TOPIC_STEER, CMD_STOP, 0, 100);
            }
        }
        server.send(200, "text/plain", "steerStop");
    });
    
    // Lights control
    server.on("/LightsOn", []() {
        if (lights_mb != NULL) {
            mailbox_write(lights_mb, TOPIC_LIGHTS, CMD_LIGHTS_ON, 0, 1000);
        }
        server.send(200, "text/plain", "Luces bajas encendidas");
    });
    
    server.on("/LightsOff", []() {
        if (lights_mb != NULL) {
            mailbox_write(lights_mb, TOPIC_LIGHTS, CMD_LIGHTS_OFF, 0, 1000);
        }
        server.send(200, "text/plain", "Luces bajas apagadas");
    });
    
    server.on("/LightsAuto", []() {
        if (lights_mb != NULL) {
            mailbox_write(lights_mb, TOPIC_LIGHTS, CMD_LIGHTS_AUTO, 0, 1000);
        }
        server.send(200, "text/plain", "Luces bajas automaticas");
    });
    
    // Speed control
    server.on("/changeSpeed", []() {
        String speed_str = server.arg("speed");
        if (speed_str.length() > 0) {
            int speed = speed_str.toInt();
            if (speed >= 0 && speed <= MOTOR_SPEED_MAX) {
                if (motor_mb != NULL) {
                    mailbox_write(motor_mb, TOPIC_MOTOR, CMD_SET_SPEED, speed, 200);
                }
                server.send(200, "text/plain", "Velocidad limitada a " + speed_str);
                return;
            }
        }
        server.send(400, "text/plain", "Invalid speed");
    });
    
    // System control
    server.on("/mode", []() {
        String value_str = server.arg("value");
        if (supervisor_mb != NULL && value_str.length() > 0) {
            int32_t mode = (value_str == "AUTO") ? MODE_AUTO : MODE_MANUAL;
            mailbox_write(supervisor_mb, TOPIC_SYSTEM, CMD_SYS_MODE, mode, 5000);
        }
        server.send(200, "text/plain", "OK");
    });
    
    server.on("/arm", []() {
        if (supervisor_mb != NULL) {
            mailbox_write(supervisor_mb, TOPIC_SYSTEM, CMD_SYS_ARM, 0, 5000);
        }
        server.send(200, "text/plain", "ARMED");
    });
    
    server.on("/disarm", []() {
        if (supervisor_mb != NULL) {
            mailbox_write(supervisor_mb, TOPIC_SYSTEM, CMD_SYS_DISARM, 0, 5000);
        }
        server.send(200, "text/plain", "DISARMED");
    });
    
    server.on("/brake", []() {
        motor_task_trigger_emergency();
        server.send(200, "text/plain", "BRAKE");
    });
    
    server.on("/status", []() {
        system_mode_t mode = supervisor_get_mode();
        system_state_t state = supervisor_get_state();
        String json = "{\"mode\":\"" + String(mode == MODE_AUTO ? "AUTO" : "MANUAL") + 
                      "\",\"state\":\"" + 
                      String(state == STATE_DISARMED ? "DISARMED" :
                             state == STATE_ARMED ? "ARMED" :
                             state == STATE_RUNNING ? "RUNNING" : "FAULT") + "\"}";
        server.send(200, "application/json", json);
    });
    
    server.onNotFound([]() {
        server.send(404, "text/plain", "Not Found");
    });
    
    server.begin();
    Serial.println("[WebTask] HTTP server started");
    
    // Task loop - handle clients
    while (1) {
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay
    }
}

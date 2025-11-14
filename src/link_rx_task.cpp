#include "link_rx_task.h"
#include "hardware.h"
#include "mailbox.h"
#include "messages.h"
#include "motor_task.h"
#include "supervisor_task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

#define UART_RX_TIMEOUT_MS 100

static mailbox_t *motor_mb = NULL;
static mailbox_t *steer_mb = NULL;
static mailbox_t *lights_mb = NULL;
static mailbox_t *supervisor_mb = NULL;

// Parse UART message format: CHANNEL:COMMAND[:VALUE]
static bool parse_uart_message(const char *msg, char *channel, char *cmd, int32_t *value) {
    char *msg_copy = strdup(msg);
    if (msg_copy == NULL) {
        return false;
    }
    
    // Remove newline/carriage return
    char *p = strchr(msg_copy, '\n');
    if (p) *p = '\0';
    p = strchr(msg_copy, '\r');
    if (p) *p = '\0';
    
    // Parse channel (single character)
    if (strlen(msg_copy) < 3) {
        free(msg_copy);
        return false;
    }
    *channel = msg_copy[0];
    
    // Find command start
    char *cmd_start = strchr(msg_copy, ':');
    if (cmd_start == NULL) {
        free(msg_copy);
        return false;
    }
    cmd_start++;
    
    // Find value separator
    char *value_start = strchr(cmd_start, ':');
    if (value_start != NULL) {
        *value_start = '\0';
        value_start++;
        *value = atoi(value_start);
    } else {
        *value = 0;
    }
    
    strncpy(cmd, cmd_start, 32);
    cmd[31] = '\0';
    
    free(msg_copy);
    return true;
}

// Read a line (up to newline or carriage return) from a Serial-like stream
static int read_line_from(Stream &stream, char *buffer, int max_len) {
    int len = 0;
    while (stream.available() > 0 && len < max_len - 1) {
        buffer[len++] = stream.read();
        if (buffer[len - 1] == '\n' || buffer[len - 1] == '\r') {
            break;
        }
    }
    if (len > 0) {
        buffer[len] = '\0';
    }
    return len;
}

void link_rx_task(void *pvParameters) {
    link_rx_params_t *params = (link_rx_params_t *)pvParameters;
    motor_mb = params->motor_mailbox;
    steer_mb = params->steer_mailbox;
    lights_mb = params->lights_mailbox;
    supervisor_mb = params->supervisor_mailbox;
    
    char data[UART_BUF_SIZE];
    
    Serial.println("[LinkRxTask] LinkRx task started");
    
    while (1) {
        // Read UART data from USB Serial first (testing over single USB cable), then from Serial1
        int len = read_line_from(Serial, data, UART_BUF_SIZE);
        if (len == 0) {
            len = read_line_from(Serial1, data, UART_BUF_SIZE);
        }
        
        if (len > 0) {
            data[len] = '\0';
            
            char channel;
            char cmd[32];
            int32_t value = 0;
            
            if (parse_uart_message(data, &channel, cmd, &value)) {
                // Update heartbeat
                supervisor_update_heartbeat();
                
                // Route based on channel
                switch (channel) {
                    case CHANNEL_EMERGENCY:
                        if (strcmp(cmd, "BRAKE_NOW") == 0 || strcmp(cmd, "STOP") == 0) {
                            // Emergency: send notification to MotorTask
                            Serial.println("EVENT:CMD_RECEIVED:BRAKE_NOW");
                            Serial.flush();
                            motor_task_trigger_emergency();
                            Serial.println("[LinkRxTask] Emergency brake triggered via UART");
                        }
                        break;
                        
                    case CHANNEL_CONTROL: {
                        // Always send commands to mailboxes - tasks will validate state before execution
                        if (strcmp(cmd, "SET_SPEED") == 0) {
                            Serial.print("EVENT:CMD_RECEIVED:SET_SPEED:");
                            Serial.println(value);
                            Serial.flush();
                            if (motor_mb != NULL) {
                                mailbox_write(motor_mb, TOPIC_MOTOR, CMD_SET_SPEED, value, 200);
                            }
                        } else if (strcmp(cmd, "SET_STEER") == 0) {
                            Serial.print("EVENT:CMD_RECEIVED:SET_STEER:");
                            Serial.println(value);
                            Serial.flush();
                            if (steer_mb != NULL) {
                                mailbox_write(steer_mb, TOPIC_STEER, CMD_SET_STEER, value, 200);
                            }
                        }
                        break;
                    }
                        
                    case CHANNEL_MANAGEMENT:
                        if (strcmp(cmd, "SYS_ARM") == 0) {
                            if (supervisor_mb != NULL) {
                                Serial.println("EVENT:CMD_RECEIVED:SYS_ARM");
                                Serial.flush();
                                mailbox_write(supervisor_mb, TOPIC_SYSTEM, CMD_SYS_ARM, 0, 5000);
                                Serial.println("[LinkRxTask] SYS_ARM command");
                            }
                        } else if (strcmp(cmd, "SYS_DISARM") == 0) {
                            if (supervisor_mb != NULL) {
                                Serial.println("EVENT:CMD_RECEIVED:SYS_DISARM");
                                Serial.flush();
                                mailbox_write(supervisor_mb, TOPIC_SYSTEM, CMD_SYS_DISARM, 0, 5000);
                                Serial.println("[LinkRxTask] SYS_DISARM command");
                            }
                        } else if (strcmp(cmd, "SYS_MODE") == 0) {
                            if (supervisor_mb != NULL) {
                                // Value can be "AUTO"/"MANUAL" as string or 0/1 as integer
                                int32_t mode;
                                if (value == 1 || value == 0) {
                                    mode = (value == 1) ? MODE_AUTO : MODE_MANUAL;
                                } else {
                                    mode = MODE_AUTO; // Default to AUTO if not clear
                                }
                                Serial.print("EVENT:CMD_RECEIVED:SYS_MODE:");
                                Serial.println(mode == MODE_AUTO ? "AUTO" : "MANUAL");
                                Serial.flush();
                                mailbox_write(supervisor_mb, TOPIC_SYSTEM, CMD_SYS_MODE, mode, 5000);
                                Serial.print("[LinkRxTask] SYS_MODE: ");
                                Serial.println(mode == MODE_AUTO ? "AUTO" : "MANUAL");
                            }
                        } else if (strcmp(cmd, "LIGHTS_ON") == 0) {
                            if (lights_mb != NULL) {
                                Serial.println("EVENT:CMD_RECEIVED:LIGHTS_ON");
                                Serial.flush();
                                mailbox_write(lights_mb, TOPIC_LIGHTS, CMD_LIGHTS_ON, 0, 1000);
                                Serial.println("[LinkRxTask] LIGHTS_ON command");
                            }
                        } else if (strcmp(cmd, "LIGHTS_OFF") == 0) {
                            if (lights_mb != NULL) {
                                Serial.println("EVENT:CMD_RECEIVED:LIGHTS_OFF");
                                Serial.flush();
                                mailbox_write(lights_mb, TOPIC_LIGHTS, CMD_LIGHTS_OFF, 0, 1000);
                                Serial.println("[LinkRxTask] LIGHTS_OFF command");
                            }
                        } else if (strcmp(cmd, "LIGHTS_AUTO") == 0) {
                            if (lights_mb != NULL) {
                                Serial.println("EVENT:CMD_RECEIVED:LIGHTS_AUTO");
                                Serial.flush();
                                mailbox_write(lights_mb, TOPIC_LIGHTS, CMD_LIGHTS_AUTO, 0, 1000);
                                Serial.println("[LinkRxTask] LIGHTS_AUTO command");
                            }
                        }
                        break;
                        
                    default:
                        Serial.print("[LinkRxTask] Unknown channel: ");
                        Serial.println(channel);
                        break;
                }
            } else {
                Serial.print("[LinkRxTask] Failed to parse message: ");
                Serial.println(data);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to avoid busy waiting
    }
}

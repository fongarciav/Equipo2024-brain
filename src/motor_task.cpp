#include "motor_task.h"
#include "hardware.h"
#include "mailbox.h"
#include "messages.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <Arduino.h>

#define MOTOR_TASK_PERIOD_MS 10 // 100 Hz
#define EMERGENCY_NOTIFICATION_BIT (1 << 0)
#define DEFAULT_FORWARD_SPEED 255 // Default speed when no command received (0-255)
#define STOP_COOLDOWN_MS 5000 // 5 seconds cooldown after stop

static mailbox_t *motor_mailbox = NULL;
static TaskHandle_t motor_task_handle = NULL;

void motor_task(void *pvParameters)
{
    motor_mailbox = (mailbox_t *)pvParameters;
    motor_task_handle = xTaskGetCurrentTaskHandle();

    uint8_t current_speed = 0;
    bool motor_direction = true; // forward
    bool has_valid_command = false;
    uint32_t last_stop_timestamp = 0;
    bool in_cooldown = false;

    Serial.println("[MotorTask] Motor task started");

    while (1)
    {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // Check if cooldown period has elapsed
        if (last_stop_timestamp > 0)
        {
            uint32_t elapsed = current_time - last_stop_timestamp;
            if (elapsed >= STOP_COOLDOWN_MS)
            {
                in_cooldown = false;
                last_stop_timestamp = 0; // Reset
                Serial.println("[MotorTask] Stop cooldown expired, motor can move again");
            }
            else
            {
                in_cooldown = true;
            }
        }
        else
        {
            in_cooldown = false;
        }

        // Check for emergency notifications first (<1ms response)
        uint32_t notification_value = ulTaskNotifyTake(pdTRUE, 0);
        if (notification_value & EMERGENCY_NOTIFICATION_BIT)
        {
            Serial.println("[MotorTask] Emergency brake triggered!");
            motor_stop();
            lights_set_reverse(false);
            current_speed = 0;
            has_valid_command = false;
            last_stop_timestamp = current_time;
            in_cooldown = true;
            Serial.println("[MotorTask] 5 second cooldown started");
            vTaskDelay(pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS));
            continue;
        }

        // Read mailbox for motor commands
        topic_t topic;
        command_type_t cmd;
        int32_t value;
        uint32_t ts_ms;
        bool expired;

        has_valid_command = false;

        if (mailbox_read(motor_mailbox, &topic, &cmd, &value, &ts_ms, &expired))
        {
            if (!expired)
            {
                has_valid_command = true;
                switch (cmd)
                {
                case CMD_SET_SPEED:
                    // Only allow speed commands if not in cooldown
                    if (in_cooldown)
                    {
                        Serial.println("[MotorTask] Speed command ignored (in cooldown)");
                        has_valid_command = false; // Treat as no command
                    }
                    else
                    {
                        current_speed = (uint8_t)value;
                        if (current_speed > MOTOR_SPEED_MAX)
                        {
                            current_speed = MOTOR_SPEED_MAX;
                        }
                        // Ensure forward direction when setting speed
                        motor_set_direction(true);
                        motor_set_speed(current_speed);
                        motor_direction = true;
                        lights_set_reverse(false);
                    }
                    break;

                case CMD_BRAKE_NOW:
                case CMD_STOP:
                    motor_stop();
                    lights_set_reverse(false);
                    current_speed = 0;
                    motor_direction = true;
                    last_stop_timestamp = current_time;
                    in_cooldown = true;
                    Serial.println("[MotorTask] Motor stopped (brake/stop command), 5 second cooldown started");
                    break;

                default:
                    break;
                }
            }
        }

        // Default behavior: if no valid command received and not in cooldown, move forward at default speed
        if (!has_valid_command && !in_cooldown)
        {
            current_speed = DEFAULT_FORWARD_SPEED;
            motor_set_direction(true);
            motor_set_speed(current_speed);
            motor_direction = true;
            lights_set_reverse(false);
        }
        else if (in_cooldown)
        {
            // Ensure motor stays stopped during cooldown
            motor_stop();
            current_speed = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(MOTOR_TASK_PERIOD_MS));
    }
}

void motor_task_trigger_emergency(void)
{
    if (motor_task_handle != NULL)
    {
        xTaskNotify(motor_task_handle, EMERGENCY_NOTIFICATION_BIT, eSetBits);
    }
}

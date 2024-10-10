#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "mpu6050.h"
#include <Fusion.h>
#include <math.h>

#define UART_ID uart0
#define BAUD_RATE 115200
#define UART_TX_PIN 0
#define UART_RX_PIN 1
#define SAMPLE_PERIOD (0.01f)

#define MAX_OUTPUT 250
#define MAX_ROLL_YAW_ANGLE 90.0f
#define DEADZONE_THRESHOLD 1.0f
#define CLICK_THRESHOLD 500.0f
#define ALPHA 0.75f

#define MPU_ADDRESS 0x68
#define I2C_SDA_GPIO 4
#define I2C_SCL_GPIO 5

typedef struct {
    uint8_t axis;
    int16_t value;
} MouseData;

QueueHandle_t xQueueMouse;
FusionVector gyro_offset = {0.0f, 0.0f, 0.0f};

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    reg = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }
}

int16_t scale_value(float value, float max_angle) {
    if (value > -DEADZONE_THRESHOLD && value < DEADZONE_THRESHOLD) {
        return 0;
    }
    float scaled_value = (value / max_angle) * MAX_OUTPUT;
    if (scaled_value > MAX_OUTPUT) scaled_value = MAX_OUTPUT;
    if (scaled_value < -MAX_OUTPUT) scaled_value = -MAX_OUTPUT;
    return (int16_t)scaled_value;
}

float smooth_value(float current_value, float previous_value) {
    return (ALPHA * previous_value) + ((1.0f - ALPHA) * current_value);
}

float calculate_acceleration_magnitude(const int16_t accel[3]) {
    return sqrtf(accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]);
}

void send_uart_data(MouseData data) {
    uint8_t msb = (data.value >> 8) & 0xFF;
    uint8_t lsb = data.value & 0xFF;
    uart_putc(UART_ID, data.axis);
    uart_putc(UART_ID, msb);
    uart_putc(UART_ID, lsb);
    uart_putc(UART_ID, 0xFF);
}

void calibrate_gyro(FusionVector *gyro_offset) {
    int16_t acceleration[3], gyro[3], temp;
    int samples = 500;
    FusionVector sum = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < samples; i++) {
        mpu6050_read_raw(acceleration, gyro, &temp);
        sum.axis.x += gyro[0] / 131.0f;
        sum.axis.y += gyro[1] / 131.0f;
        sum.axis.z += gyro[2] / 131.0f;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    gyro_offset->axis.x = sum.axis.x / samples;
    gyro_offset->axis.y = sum.axis.y / samples;
    gyro_offset->axis.z = sum.axis.z / samples;
}

void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    mpu6050_reset();

    int16_t acceleration[3], gyro[3], temp;
    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);
    calibrate_gyro(&gyro_offset);

    int click_status = 0;

    while (1) {
        mpu6050_read_raw(acceleration, gyro, &temp);
        FusionVector gyroscope = {
            .axis.x = (gyro[0] / 131.0f) - gyro_offset.axis.x,
            .axis.y = (gyro[1] / 131.0f) - gyro_offset.axis.y,
            .axis.z = (gyro[2] / 131.0f) - gyro_offset.axis.z,
        };
        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f,
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        float roll_smooth = euler.angle.roll;
        float yaw_smooth = euler.angle.yaw;

        MouseData rollData = {0, scale_value(roll_smooth, MAX_ROLL_YAW_ANGLE)};
        MouseData yawData = {1, scale_value(yaw_smooth, MAX_ROLL_YAW_ANGLE)};
        xQueueSend(xQueueMouse, &rollData, portMAX_DELAY);
        xQueueSend(xQueueMouse, &yawData, portMAX_DELAY);

        float accel_magnitude = calculate_acceleration_magnitude(acceleration);
        if (accel_magnitude > CLICK_THRESHOLD && click_status == 0) {
            MouseData clickData = {2, 1};
            xQueueSend(xQueueMouse, &clickData, portMAX_DELAY);
            click_status = 1;
        } else if (accel_magnitude <= CLICK_THRESHOLD && click_status == 1) {
            MouseData clickData = {2, 0};
            xQueueSend(xQueueMouse, &clickData, portMAX_DELAY);
            click_status = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void uart_task(void *p) {
    MouseData data;

    while (1) {
        if (xQueueReceive(xQueueMouse, &data, portMAX_DELAY)) {
            send_uart_data(data);
        }
    }
}

int main() {
    stdio_init_all();

    xQueueMouse = xQueueCreate(10, sizeof(MouseData));
    xTaskCreate(mpu6050_task, "MPU6050 Task", 4096, NULL, 1, NULL);
    xTaskCreate(uart_task, "UART Task", 4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true) {
        tight_loop_contents();
    }
}

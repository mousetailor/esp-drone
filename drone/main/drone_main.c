/*
 * ESP32 Drone Flight Controller Firmware (ESP-IDF 5.5.2)
 * =======================================================
 * This ESP32 sits ON the drone. It receives control commands from the
 * controller ESP32 via ESP-NOW, stabilizes flight using an MPU6050 IMU,
 * drives 4 motors via ESCs, and sends telemetry back.
 *
 * DATA FLOW:
 *   Controller ESP32 ──ESP-NOW──▶ this ESP32 ──PWM──▶ ESCs ──▶ Motors
 *   Controller ESP32 ◀──ESP-NOW── this ESP32 ◀──I2C── MPU6050
 *                                             ◀──ADC── Battery
 *
 * QUADCOPTER X-CONFIGURATION (top view):
 *
 *         FRONT
 *     M3(FL,CCW)  M1(FR,CW)
 *          \\      //
 *           \\    //
 *            ====
 *           //    \\
 *          //      \\
 *     M4(BL,CW)   M2(BR,CCW)
 *          BACK
 *
 * MOTOR MIXING:
 *   M1 (FR, CW ) = throttle - pitch + roll - yaw
 *   M2 (BR, CCW) = throttle + pitch + roll + yaw
 *   M3 (FL, CCW) = throttle - pitch - roll + yaw
 *   M4 (BL, CW ) = throttle + pitch - roll - yaw
 *
 * BEFORE FLASHING:
 *   1. Flash this firmware, run `idf.py monitor`, note the MAC address
 *   2. Put that MAC into controller firmware's `drone_mac[]`
 *   3. Put the controller's MAC into `controller_mac[]` below
 *
 * GPIO ASSIGNMENTS (change in CONFIGURATION section):
 *   Motor 1 (FR): GPIO 25
 *   Motor 2 (BR): GPIO 26
 *   Motor 3 (FL): GPIO 27
 *   Motor 4 (BL): GPIO 14
 *   MPU6050 SDA:  GPIO 21
 *   MPU6050 SCL:  GPIO 22
 *   Battery ADC:  GPIO 34 (ADC1_CH6, input-only)
 */

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <math.h>
#include <string.h>

static const char *TAG = "DRONE";

/* =========================================================
 *  CONFIGURATION — edit these to match your hardware
 * ========================================================= */

/* Wi-Fi channel — MUST match the controller */
#define ESPNOW_CHANNEL          1

/* Set to 1 for Espressif Long-Range mode. MUST match controller. */
#define USE_LONG_RANGE          1

/* Motor GPIO pins */
#define MOTOR1_GPIO             25   /* Front-Right (CW)  */
#define MOTOR2_GPIO             26   /* Back-Right  (CCW) */
#define MOTOR3_GPIO             27   /* Front-Left  (CCW) */
#define MOTOR4_GPIO             14   /* Back-Left   (CW)  */

/* MPU6050 I2C */
#define I2C_SDA_GPIO            21
#define I2C_SCL_GPIO            22
#define I2C_FREQ_HZ             400000
#define MPU6050_ADDR            0x68

/* Battery voltage ADC */
#define BATT_ADC_CHANNEL        ADC_CHANNEL_6   /* GPIO 34 */
/* Voltage divider ratio: if using 10k+10k divider, ratio = 2.0
 * Adjust to your actual divider. Set to 1.0 if no divider. */
#define BATT_DIVIDER_RATIO      2.0f

/* Failsafe: if no control packet received for this long, disarm */
#define FAILSAFE_TIMEOUT_MS     500

/* PID loop frequency target */
#define PID_LOOP_FREQ_HZ        250
#define PID_LOOP_INTERVAL_US    (1000000 / PID_LOOP_FREQ_HZ)

/* Telemetry send rate */
#define TELEMETRY_INTERVAL_MS   200

/* ESC PWM settings */
#define ESC_PWM_FREQ_HZ         50     /* 50 Hz = standard servo/ESC */
#define ESC_LEDC_RESOLUTION     LEDC_TIMER_16_BIT
#define ESC_DUTY_MIN            3277   /* ~1000 µs at 50 Hz, 16-bit */
#define ESC_DUTY_MAX            6554   /* ~2000 µs at 50 Hz, 16-bit */

/* Motor output limits (in 1000-2000 range) */
#define MOTOR_MIN               1000
#define MOTOR_MAX               2000
#define MOTOR_IDLE              1050   /* slight spin when armed */

/* !!!  CHANGE THIS to your controller ESP32's MAC address  !!!
 * Flash the controller, run its monitor, note the MAC.
 * FF:FF:FF:FF:FF:FF = accept from ANY sender (broadcast). */
static uint8_t controller_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

/* =========================================================
 *  PID GAINS — tune these for your specific drone
 *  Start with small values and increase gradually!
 * ========================================================= */
#define ROLL_KP     1.5f
#define ROLL_KI     0.02f
#define ROLL_KD     0.8f

#define PITCH_KP    1.5f
#define PITCH_KI    0.02f
#define PITCH_KD    0.8f

#define YAW_KP      2.0f
#define YAW_KI      0.01f
#define YAW_KD      0.0f

/* PID integral windup limits */
#define PID_I_MAX   200.0f

/* =========================================================
 *  PACKET DEFINITIONS  (must match controller firmware)
 * ========================================================= */

#define MAGIC_CTRL  0xAB
#define MAGIC_TELE  0xCD

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint16_t throttle;   /* 1000 … 2000     */
    int16_t  roll;       /* -500 … +500     */
    int16_t  pitch;      /* -500 … +500     */
    int16_t  yaw;        /* -500 … +500     */
    uint8_t  armed;      /* 0 or 1          */
    uint8_t  checksum;
} control_packet_t;      /* 11 bytes        */

typedef struct __attribute__((packed)) {
    uint8_t  magic;
    uint16_t voltage_mv;
    int16_t  roll_deg;   /* angle × 10      */
    int16_t  pitch_deg;
    int16_t  yaw_deg;
    uint8_t  flags;      /* bit0=armed, bit1=failsafe */
    uint8_t  checksum;
} telemetry_packet_t;    /* 11 bytes        */

/* =========================================================
 *  PID CONTROLLER STRUCTURE
 * ========================================================= */
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output;
} pid_t;

/* =========================================================
 *  GLOBAL STATE
 * ========================================================= */

/* Latest control input (protected by mutex) */
static SemaphoreHandle_t ctrl_mutex;
static uint16_t ctrl_throttle = 1000;
static int16_t  ctrl_roll     = 0;
static int16_t  ctrl_pitch    = 0;
static int16_t  ctrl_yaw     = 0;
static uint8_t  ctrl_armed    = 0;

/* Timestamp of last received control packet */
static volatile int64_t last_ctrl_time_us = 0;

/* Failsafe flag */
static volatile bool failsafe_active = false;

/* IMU data (written by IMU task, read by PID task) */
static SemaphoreHandle_t imu_mutex;
static float imu_roll  = 0.0f;  /* degrees */
static float imu_pitch = 0.0f;
static float imu_yaw   = 0.0f;
static float gyro_roll_rate  = 0.0f;  /* deg/s */
static float gyro_pitch_rate = 0.0f;
static float gyro_yaw_rate   = 0.0f;

/* Battery voltage */
static volatile uint16_t battery_mv = 0;

/* Motor outputs (1000-2000 range) */
static uint16_t motor_out[4] = {1000, 1000, 1000, 1000};

/* I2C master bus handle */
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t mpu_dev = NULL;

/* ADC handle */
static adc_oneshot_unit_handle_t adc_handle = NULL;

/* =========================================================
 *  CHECKSUM
 * ========================================================= */
static uint8_t compute_checksum(const uint8_t *data, size_t len)
{
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++)
        cs ^= data[i];
    return cs;
}

/* =========================================================
 *  PID HELPERS
 * ========================================================= */
static void pid_init(pid_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

static void pid_reset(pid_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
    pid->output     = 0.0f;
}

static float pid_compute(pid_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    /* Proportional */
    float p_term = pid->kp * error;

    /* Integral with anti-windup */
    pid->integral += error * dt;
    if (pid->integral > PID_I_MAX) pid->integral = PID_I_MAX;
    if (pid->integral < -PID_I_MAX) pid->integral = -PID_I_MAX;
    float i_term = pid->ki * pid->integral;

    /* Derivative (on error; consider derivative-on-measurement
     * if you see derivative kick on setpoint changes) */
    float derivative = (dt > 0.0001f) ? (error - pid->prev_error) / dt : 0.0f;
    float d_term = pid->kd * derivative;
    pid->prev_error = error;

    pid->output = p_term + i_term + d_term;
    return pid->output;
}

/* =========================================================
 *  MOTOR PWM (LEDC)
 * ========================================================= */
static const int motor_gpios[4] = {
    MOTOR1_GPIO, MOTOR2_GPIO, MOTOR3_GPIO, MOTOR4_GPIO
};

static void motors_init(void)
{
    /* One shared timer for all 4 motor channels */
    ledc_timer_config_t timer_conf = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = ESC_LEDC_RESOLUTION,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = ESC_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ch_conf = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel    = (ledc_channel_t)i,
            .timer_sel  = LEDC_TIMER_0,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = motor_gpios[i],
            .duty       = ESC_DUTY_MIN,  /* start at minimum (1000 µs) */
            .hpoint     = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));
    }

    ESP_LOGI(TAG, "Motors initialized on GPIOs %d, %d, %d, %d",
             MOTOR1_GPIO, MOTOR2_GPIO, MOTOR3_GPIO, MOTOR4_GPIO);
}

/* Convert 1000-2000 range to LEDC duty value */
static uint32_t throttle_to_duty(uint16_t throttle_us)
{
    if (throttle_us < MOTOR_MIN) throttle_us = MOTOR_MIN;
    if (throttle_us > MOTOR_MAX) throttle_us = MOTOR_MAX;

    /* Linear interpolation: 1000 -> ESC_DUTY_MIN, 2000 -> ESC_DUTY_MAX */
    return ESC_DUTY_MIN +
           (uint32_t)((throttle_us - MOTOR_MIN) *
                      (ESC_DUTY_MAX - ESC_DUTY_MIN)) / (MOTOR_MAX - MOTOR_MIN);
}

static void motors_set(const uint16_t outputs[4])
{
    for (int i = 0; i < 4; i++) {
        uint32_t duty = throttle_to_duty(outputs[i]);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
    }
}

static void motors_stop(void)
{
    uint16_t idle[4] = {MOTOR_MIN, MOTOR_MIN, MOTOR_MIN, MOTOR_MIN};
    motors_set(idle);
}

/* =========================================================
 *  MPU6050 I2C DRIVER
 * ========================================================= */

/* MPU6050 register addresses */
#define MPU_REG_PWR_MGMT_1     0x6B
#define MPU_REG_SMPLRT_DIV     0x19
#define MPU_REG_CONFIG         0x1A
#define MPU_REG_GYRO_CONFIG    0x1B
#define MPU_REG_ACCEL_CONFIG   0x1C
#define MPU_REG_INT_ENABLE     0x38
#define MPU_REG_ACCEL_XOUT_H   0x3B
#define MPU_REG_WHO_AM_I       0x75

/* Sensitivity scale factors */
#define ACCEL_SCALE_2G         16384.0f    /* LSB/g at ±2g   */
#define GYRO_SCALE_250DPS      131.0f      /* LSB/(°/s) at ±250°/s */

/* Complementary filter coefficient (0-1, higher = trust gyro more) */
#define COMP_FILTER_ALPHA      0.98f

static esp_err_t mpu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(mpu_dev, buf, 2, 100);
}

static esp_err_t mpu_read_regs(uint8_t start_reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(mpu_dev, &start_reg, 1, data, len, 100);
}

static bool mpu6050_init(void)
{
    /* I2C bus configuration using new driver API (IDF 5.x) */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port   = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(ret));
        return false;
    }

    /* Add MPU6050 device on the bus */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MPU6050_ADDR,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    ret = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &mpu_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 device add failed: %s", esp_err_to_name(ret));
        return false;
    }

    /* Check WHO_AM_I register (should be 0x68) */
    uint8_t who = 0;
    ret = mpu_read_regs(MPU_REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK || (who != 0x68 && who != 0x72 && who != 0x70)) {
        ESP_LOGE(TAG, "MPU6050 not found (WHO_AM_I=0x%02X, err=%s)",
                 who, esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "MPU6050 detected (WHO_AM_I=0x%02X)", who);

    /* Wake up (clear sleep bit) and use internal 8 MHz oscillator */
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Use PLL with X-axis gyro as clock source (more stable) */
    mpu_write_reg(MPU_REG_PWR_MGMT_1, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Sample rate = 1 kHz / (1 + divider) = 1000 / (1+3) = 250 Hz */
    mpu_write_reg(MPU_REG_SMPLRT_DIV, 3);

    /* Digital Low Pass Filter: ~44 Hz bandwidth (config = 3)
     * Reduces noise but adds ~4.9 ms delay */
    mpu_write_reg(MPU_REG_CONFIG, 0x03);

    /* Gyroscope: ±250 °/s (most sensitive, best for small quads) */
    mpu_write_reg(MPU_REG_GYRO_CONFIG, 0x00);

    /* Accelerometer: ±2 g */
    mpu_write_reg(MPU_REG_ACCEL_CONFIG, 0x00);

    /* Disable all interrupts (we poll instead) */
    mpu_write_reg(MPU_REG_INT_ENABLE, 0x00);

    ESP_LOGI(TAG, "MPU6050 configured: ±250°/s, ±2g, 250 Hz, DLPF=44Hz");
    return true;
}

/* Read 14 bytes: accel(6) + temp(2) + gyro(6) */
static bool mpu6050_read_raw(int16_t *ax, int16_t *ay, int16_t *az,
                              int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[14];
    esp_err_t ret = mpu_read_regs(MPU_REG_ACCEL_XOUT_H, buf, 14);
    if (ret != ESP_OK)
        return false;

    /* MPU6050 data is big-endian: high byte first */
    *ax = (int16_t)((buf[0]  << 8) | buf[1]);
    *ay = (int16_t)((buf[2]  << 8) | buf[3]);
    *az = (int16_t)((buf[4]  << 8) | buf[5]);
    /* buf[6..7] = temperature, skip */
    *gx = (int16_t)((buf[8]  << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);

    return true;
}

/* =========================================================
 *  BATTERY ADC
 * ========================================================= */
static void battery_adc_init(void)
{
    adc_oneshot_unit_init_cfg_t adc_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,  /* 0-3.3V range (approximately) */
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, BATT_ADC_CHANNEL,
                                                &chan_cfg));
    ESP_LOGI(TAG, "Battery ADC initialized on channel %d", BATT_ADC_CHANNEL);
}

static uint16_t battery_read_mv(void)
{
    int raw = 0;
    esp_err_t ret = adc_oneshot_read(adc_handle, BATT_ADC_CHANNEL, &raw);
    if (ret != ESP_OK)
        return 0;

    /* Convert 12-bit raw (0-4095) to millivolts.
     * At 12dB attenuation, full scale ≈ 3100 mV (ESP32 specific).
     * Then multiply by divider ratio. */
    float mv = (raw / 4095.0f) * 3100.0f * BATT_DIVIDER_RATIO;
    return (uint16_t)mv;
}

/* =========================================================
 *  ESP-NOW CALLBACKS
 * ========================================================= */

static void on_data_sent(const esp_now_send_info_t *tx_info,
                         esp_now_send_status_t status)
{
    /* Telemetry send status — just for debugging */
    (void)tx_info;
    (void)status;
}

static void on_data_recv(const esp_now_recv_info_t *info,
                         const uint8_t *data, int data_len)
{
    if (data_len != sizeof(control_packet_t))
        return;

    const control_packet_t *pkt = (const control_packet_t *)data;

    if (pkt->magic != MAGIC_CTRL)
        return;

    /* Verify checksum */
    uint8_t expected = compute_checksum(data, data_len - 1);
    if (pkt->checksum != expected)
        return;

    /* Sanity-check values */
    if (pkt->throttle < 900 || pkt->throttle > 2100)
        return;
    if (pkt->armed > 1)
        return;

    /* Update control state (protected by mutex) */
    if (xSemaphoreTake(ctrl_mutex, 0) == pdTRUE) {
        ctrl_throttle = pkt->throttle;
        ctrl_roll     = pkt->roll;
        ctrl_pitch    = pkt->pitch;
        ctrl_yaw      = pkt->yaw;
        ctrl_armed    = pkt->armed;
        xSemaphoreGive(ctrl_mutex);
    }

    /* Update last-received timestamp (atomic on ESP32) */
    last_ctrl_time_us = esp_timer_get_time();

    /* Clear failsafe if we were in it */
    if (failsafe_active) {
        failsafe_active = false;
        ESP_LOGW(TAG, "Signal restored — failsafe cleared");
    }
}

/* =========================================================
 *  IMU TASK — reads MPU6050 and computes attitude
 * ========================================================= */
static void imu_task(void *arg)
{
    int16_t ax, ay, az, gx, gy, gz;
    float roll_acc, pitch_acc;
    float roll_est = 0.0f, pitch_est = 0.0f, yaw_est = 0.0f;
    int64_t prev_time = esp_timer_get_time();
    bool first_reading = true;

    /* Gyro calibration: average readings at startup (drone must be still!) */
    ESP_LOGI(TAG, "Calibrating gyro — keep drone STILL for 2 seconds...");
    float gx_offset = 0, gy_offset = 0, gz_offset = 0;
    const int cal_samples = 500;
    for (int i = 0; i < cal_samples; i++) {
        if (mpu6050_read_raw(&ax, &ay, &az, &gx, &gy, &gz)) {
            gx_offset += gx;
            gy_offset += gy;
            gz_offset += gz;
        }
        vTaskDelay(pdMS_TO_TICKS(4));  /* ~250 Hz */
    }
    gx_offset /= cal_samples;
    gy_offset /= cal_samples;
    gz_offset /= cal_samples;
    ESP_LOGI(TAG, "Gyro cal done: offsets = (%.1f, %.1f, %.1f)",
             gx_offset, gy_offset, gz_offset);

    while (1) {
        if (!mpu6050_read_raw(&ax, &ay, &az, &gx, &gy, &gz)) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        int64_t now = esp_timer_get_time();
        float dt = (now - prev_time) / 1000000.0f;  /* seconds */
        prev_time = now;

        /* Clamp dt to avoid huge jumps */
        if (dt > 0.05f) dt = 0.05f;
        if (dt < 0.0001f) dt = 0.0001f;

        /* Convert gyro to °/s, applying calibration offset */
        float gx_dps = (gx - gx_offset) / GYRO_SCALE_250DPS;
        float gy_dps = (gy - gy_offset) / GYRO_SCALE_250DPS;
        float gz_dps = (gz - gz_offset) / GYRO_SCALE_250DPS;

        /* Convert accelerometer to g */
        float ax_g = ax / ACCEL_SCALE_2G;
        float ay_g = ay / ACCEL_SCALE_2G;
        float az_g = az / ACCEL_SCALE_2G;

        /* Compute roll and pitch from accelerometer (only valid when not accelerating) */
        roll_acc  = atan2f(ay_g, az_g) * (180.0f / M_PI);
        pitch_acc = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * (180.0f / M_PI);

        if (first_reading) {
            /* Initialize with accelerometer angles */
            roll_est  = roll_acc;
            pitch_est = pitch_acc;
            yaw_est   = 0.0f;
            first_reading = false;
        } else {
            /* Complementary filter:
             * - Trust gyro for fast changes (98%)
             * - Trust accelerometer for long-term drift correction (2%)
             * This gives good response without gyro drift. */
            roll_est  = COMP_FILTER_ALPHA * (roll_est  + gx_dps * dt) +
                        (1.0f - COMP_FILTER_ALPHA) * roll_acc;
            pitch_est = COMP_FILTER_ALPHA * (pitch_est + gy_dps * dt) +
                        (1.0f - COMP_FILTER_ALPHA) * pitch_acc;
        }

        /* Yaw: gyro integration only (accelerometer can't measure yaw).
         * This WILL drift over time — a magnetometer would fix this. */
        yaw_est += gz_dps * dt;
        /* Wrap yaw to -180..+180 */
        if (yaw_est > 180.0f)  yaw_est -= 360.0f;
        if (yaw_est < -180.0f) yaw_est += 360.0f;

        /* Publish IMU data */
        if (xSemaphoreTake(imu_mutex, 0) == pdTRUE) {
            imu_roll  = roll_est;
            imu_pitch = pitch_est;
            imu_yaw   = yaw_est;
            gyro_roll_rate  = gx_dps;
            gyro_pitch_rate = gy_dps;
            gyro_yaw_rate   = gz_dps;
            xSemaphoreGive(imu_mutex);
        }

        /* Aim for PID_LOOP_FREQ_HZ; the read + compute takes some time,
         * so delay only the remaining time. Minimum 1 ms delay. */
        int64_t elapsed_us = esp_timer_get_time() - now;
        int32_t delay_us = PID_LOOP_INTERVAL_US - (int32_t)elapsed_us;
        if (delay_us > 1000) {
            vTaskDelay(pdMS_TO_TICKS(delay_us / 1000));
        } else {
            vTaskDelay(1);
        }
    }
}

/* =========================================================
 *  FLIGHT CONTROL TASK — PID + motor mixing
 * ========================================================= */

static pid_t pid_roll, pid_pitch, pid_yaw;

static void flight_control_task(void *arg)
{
    pid_init(&pid_roll,  ROLL_KP,  ROLL_KI,  ROLL_KD);
    pid_init(&pid_pitch, PITCH_KP, PITCH_KI, PITCH_KD);
    pid_init(&pid_yaw,   YAW_KP,   YAW_KI,   YAW_KD);

    int64_t prev_time = esp_timer_get_time();

    ESP_LOGI(TAG, "Flight control task started (PID @ %d Hz)", PID_LOOP_FREQ_HZ);

    while (1) {
        int64_t now = esp_timer_get_time();
        float dt = (now - prev_time) / 1000000.0f;
        prev_time = now;
        if (dt > 0.05f) dt = 0.05f;
        if (dt < 0.0001f) dt = 0.0001f;

        /* --- Check failsafe --- */
        int64_t since_last_ctrl = now - last_ctrl_time_us;
        if (last_ctrl_time_us > 0 &&
            since_last_ctrl > (int64_t)FAILSAFE_TIMEOUT_MS * 1000) {
            if (!failsafe_active) {
                failsafe_active = true;
                ESP_LOGW(TAG, "FAILSAFE — no signal for %d ms, disarming!",
                         FAILSAFE_TIMEOUT_MS);
            }
        }

        /* --- Read inputs --- */
        uint16_t thr;
        int16_t  cmd_roll, cmd_pitch, cmd_yaw;
        uint8_t  armed;

        if (xSemaphoreTake(ctrl_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            thr       = ctrl_throttle;
            cmd_roll  = ctrl_roll;
            cmd_pitch = ctrl_pitch;
            cmd_yaw   = ctrl_yaw;
            armed     = ctrl_armed;
            xSemaphoreGive(ctrl_mutex);
        } else {
            /* Mutex timeout — use safe defaults */
            thr = 1000; cmd_roll = 0; cmd_pitch = 0; cmd_yaw = 0; armed = 0;
        }

        /* Failsafe overrides arming */
        if (failsafe_active) {
            armed = 0;
        }

        /* --- Read IMU --- */
        float cur_roll, cur_pitch, cur_yaw_rate;

        if (xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(2)) == pdTRUE) {
            cur_roll     = imu_roll;
            cur_pitch    = imu_pitch;
            cur_yaw_rate = gyro_yaw_rate;
            xSemaphoreGive(imu_mutex);
        } else {
            cur_roll = 0; cur_pitch = 0; cur_yaw_rate = 0;
        }

        if (!armed || thr <= MOTOR_MIN) {
            /* DISARMED or zero throttle — motors off, reset PIDs */
            motors_stop();
            pid_reset(&pid_roll);
            pid_reset(&pid_pitch);
            pid_reset(&pid_yaw);
            motor_out[0] = motor_out[1] = motor_out[2] = motor_out[3] = MOTOR_MIN;
        } else {
            /* --- Compute setpoints ---
             * Control sticks (-500..+500) map to desired angle (roll/pitch)
             * or desired rate (yaw). Scale factor determines max angle. */

            /* Max tilt angle = 30° at full stick */
            float sp_roll  = (cmd_roll  / 500.0f) * 30.0f;   /* degrees */
            float sp_pitch = (cmd_pitch / 500.0f) * 30.0f;
            /* Yaw: rate control. Max yaw rate = 180°/s at full stick */
            float sp_yaw_rate = (cmd_yaw / 500.0f) * 180.0f; /* deg/s  */

            /* --- PID computation --- */
            float pid_out_roll  = pid_compute(&pid_roll,  sp_roll,     cur_roll,     dt);
            float pid_out_pitch = pid_compute(&pid_pitch, sp_pitch,    cur_pitch,    dt);
            float pid_out_yaw   = pid_compute(&pid_yaw,   sp_yaw_rate, cur_yaw_rate, dt);

            /* --- Motor mixing (X-quad) ---
             * M1 (FR, CW ) = throttle - pitch + roll - yaw
             * M2 (BR, CCW) = throttle + pitch + roll + yaw
             * M3 (FL, CCW) = throttle - pitch - roll + yaw
             * M4 (BL, CW ) = throttle + pitch - roll - yaw   */

            float m1 = thr - pid_out_pitch + pid_out_roll - pid_out_yaw;
            float m2 = thr + pid_out_pitch + pid_out_roll + pid_out_yaw;
            float m3 = thr - pid_out_pitch - pid_out_roll + pid_out_yaw;
            float m4 = thr + pid_out_pitch - pid_out_roll - pid_out_yaw;

            /* Clamp to valid range, ensure minimum idle when armed */
            #define CLAMP_MOTOR(m) do { \
                if ((m) < MOTOR_IDLE) (m) = MOTOR_IDLE; \
                if ((m) > MOTOR_MAX)  (m) = MOTOR_MAX;  \
            } while(0)

            CLAMP_MOTOR(m1);
            CLAMP_MOTOR(m2);
            CLAMP_MOTOR(m3);
            CLAMP_MOTOR(m4);

            motor_out[0] = (uint16_t)m1;
            motor_out[1] = (uint16_t)m2;
            motor_out[2] = (uint16_t)m3;
            motor_out[3] = (uint16_t)m4;

            motors_set(motor_out);
        }

        /* Delay for loop timing */
        int64_t elapsed_us = esp_timer_get_time() - now;
        int32_t delay_us = PID_LOOP_INTERVAL_US - (int32_t)elapsed_us;
        if (delay_us > 1000) {
            vTaskDelay(pdMS_TO_TICKS(delay_us / 1000));
        } else {
            vTaskDelay(1);
        }
    }
}

/* =========================================================
 *  TELEMETRY + BATTERY TASK
 * ========================================================= */
static void telemetry_task(void *arg)
{
    ESP_LOGI(TAG, "Telemetry task started (every %d ms)", TELEMETRY_INTERVAL_MS);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_INTERVAL_MS));

        /* Read battery */
        battery_mv = battery_read_mv();

        /* Read IMU angles */
        float r, p, y;
        if (xSemaphoreTake(imu_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            r = imu_roll;
            p = imu_pitch;
            y = imu_yaw;
            xSemaphoreGive(imu_mutex);
        } else {
            r = p = y = 0.0f;
        }

        /* Build telemetry packet */
        telemetry_packet_t tele;
        tele.magic      = MAGIC_TELE;
        tele.voltage_mv = battery_mv;
        tele.roll_deg   = (int16_t)(r * 10.0f);
        tele.pitch_deg  = (int16_t)(p * 10.0f);
        tele.yaw_deg    = (int16_t)(y * 10.0f);
        tele.flags      = 0;
        if (ctrl_armed)      tele.flags |= 0x01;
        if (failsafe_active) tele.flags |= 0x02;

        /* Compute checksum over all bytes except checksum field */
        tele.checksum = compute_checksum((const uint8_t *)&tele,
                                          sizeof(tele) - 1);

        /* Send via ESP-NOW back to controller */
        esp_now_send(controller_mac, (const uint8_t *)&tele, sizeof(tele));

        /* Log periodically for serial monitor debugging */
        ESP_LOGD(TAG, "Tele: batt=%umV roll=%.1f° pitch=%.1f° yaw=%.1f° "
                 "M=[%u,%u,%u,%u] %s%s",
                 battery_mv, r, p, y,
                 motor_out[0], motor_out[1], motor_out[2], motor_out[3],
                 ctrl_armed ? "ARMED" : "safe",
                 failsafe_active ? " FAILSAFE" : "");
    }
}

/* =========================================================
 *  WI-FI + ESP-NOW INIT
 * ========================================================= */
static void wifi_and_espnow_init(void)
{
    /* NVS */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

#if USE_LONG_RANGE
    ESP_ERROR_CHECK(
        esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR));
    ESP_LOGI(TAG, "Long-Range (LR) mode enabled");
#endif

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(
        esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));

    /* Print our MAC address — user needs this for the controller firmware */
    uint8_t my_mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, my_mac);
    ESP_LOGI(TAG, "╔════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  Drone MAC: %02X:%02X:%02X:%02X:%02X:%02X              ║",
             my_mac[0], my_mac[1], my_mac[2],
             my_mac[3], my_mac[4], my_mac[5]);
    ESP_LOGI(TAG, "║  Put this in controller's drone_mac[]      ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════╝");

    /* ESP-NOW */
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    /* Register controller as a peer (for sending telemetry back) */
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, controller_mac, 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    ESP_LOGI(TAG, "ESP-NOW ready on channel %d", ESPNOW_CHANNEL);
}

/* =========================================================
 *  app_main — ESP-IDF entry point
 * ========================================================= */
void app_main(void)
{
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   ESP32 Drone Flight Controller      ║");
    ESP_LOGI(TAG, "║   IDF 5.5.2                          ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");

    /* Create mutexes */
    ctrl_mutex = xSemaphoreCreateMutex();
    imu_mutex  = xSemaphoreCreateMutex();
    if (!ctrl_mutex || !imu_mutex) {
        ESP_LOGE(TAG, "FATAL: failed to create mutexes");
        return;
    }

    /* Initialize subsystems */
    wifi_and_espnow_init();
    motors_init();
    battery_adc_init();

    /* Motors to minimum (important: ESCs need to see low throttle at boot) */
    motors_stop();
    ESP_LOGI(TAG, "Motors set to minimum — safe to power ESCs now");
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* Initialize IMU */
    bool imu_ok = mpu6050_init();
    if (!imu_ok) {
        ESP_LOGE(TAG, "╔══════════════════════════════════════════════════╗");
        ESP_LOGE(TAG, "║  MPU6050 not found! Check wiring:               ║");
        ESP_LOGE(TAG, "║    SDA = GPIO %2d    SCL = GPIO %2d               ║",
                 I2C_SDA_GPIO, I2C_SCL_GPIO);
        ESP_LOGE(TAG, "║    VCC = 3.3V       GND = GND                   ║");
        ESP_LOGE(TAG, "║    AD0 = GND (for address 0x68)                 ║");
        ESP_LOGE(TAG, "║  Flight will NOT be stabilized without IMU!     ║");
        ESP_LOGE(TAG, "╚══════════════════════════════════════════════════╝");
        /* Continue without IMU — motors will still respond to throttle
         * but there will be no stabilization. */
    }

    /* Launch tasks
     * Priority guide: higher number = higher priority
     *   IMU:     6 (highest — timing-critical sensor reads)
     *   Flight:  5 (PID loop — needs fresh IMU data)
     *   Telem:   3 (lower priority — not time-critical)
     * Stack: 4096 bytes each is plenty for these tasks */

    if (imu_ok) {
        xTaskCreate(imu_task,            "imu",    4096, NULL, 6, NULL);
        xTaskCreate(flight_control_task, "flight", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGW(TAG, "IMU tasks skipped — running in passthrough mode");
        /* TODO: could create a simplified passthrough task here */
    }

    xTaskCreate(telemetry_task, "telem", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║  Drone ready — waiting for commands  ║");
    ESP_LOGI(TAG, "║  Failsafe timeout: %4d ms            ║", FAILSAFE_TIMEOUT_MS);
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
}
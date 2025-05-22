#include "mpu6050.h"

#define MPU6050_ADDR 0x68

#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_MOT_THR      0x1F
#define MPU6050_MOT_DUR      0x20
#define MPU6050_INT_ENABLE   0x38
#define MPU6050_INT_STATUS   0x3A

static int write_register(imu_c config, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_write_blocking(config.i2c, MPU6050_ADDR, buf, 2, false) == 2;
}

static int read_registers(imu_c config, uint8_t reg, uint8_t *buf, size_t len) {
    if (i2c_write_blocking(config.i2c, MPU6050_ADDR, &reg, 1, true) != 1) return 0;
    return i2c_read_blocking(config.i2c, MPU6050_ADDR, buf, len, false) == len;
}

void mpu6050_set_config(imu_c *config, i2c_inst_t *i2c, int pin_sda, int pin_scl, int acc_scale) {
    config->i2c = i2c;
    config->pin_sda = pin_sda;
    config->pin_scl = pin_scl;
    config->acc_scale = acc_scale;
}

int mpu6050_init(imu_c config) {
    i2c_init(config.i2c, 400000);
    gpio_set_function(config.pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(config.pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(config.pin_sda);
    gpio_pull_up(config.pin_scl);
    sleep_ms(100);
    return write_register(config, MPU6050_PWR_MGMT_1, 0x00);  // wake up
}

int mpu6050_reset(imu_c config) {
    return write_register(config, MPU6050_PWR_MGMT_1, 0x80); // reset bit
}

int mpu6050_read_acc(imu_c config, int16_t accel[3]) {
    uint8_t buf[6];
    if (!read_registers(config, MPU6050_ACCEL_XOUT_H, buf, 6)) return 0;
    for (int i = 0; i < 3; i++)
        accel[i] = (buf[i * 2] << 8) | buf[i * 2 + 1];
    return 1;
}

int mpu6050_read_gyro(imu_c config, int16_t gyro[3]) {
    uint8_t buf[6];
    if (!read_registers(config, MPU6050_GYRO_XOUT_H, buf, 6)) return 0;
    for (int i = 0; i < 3; i++)
        gyro[i] = (buf[i * 2] << 8) | buf[i * 2 + 1];
    return 1;
}

int mpu6050_read_temp(imu_c config, int16_t *temp) {
    uint8_t buf[2];
    if (!read_registers(config, MPU6050_TEMP_OUT_H, buf, 2)) return 0;
    *temp = (buf[0] << 8) | buf[1];
    return 1;
}

// Motion Detection (opcional)
int mpu6050_set_motion_detection(imu_c config, int enable) {
    return write_register(config, MPU6050_INT_ENABLE, enable ? 0x40 : 0x00);
}

int mpu6050_get_motion_interrupt_status(imu_c config) {
    uint8_t status;
    return read_registers(config, MPU6050_INT_STATUS, &status, 1) ? status & 0x40 : 0;
}

int mpu6050_set_motion_detection_threshold(imu_c config, uint8_t thr) {
    return write_register(config, MPU6050_MOT_THR, thr);
}

int mpu6050_set_motion_detection_duration(imu_c config, uint8_t dur) {
    return write_register(config, MPU6050_MOT_DUR, dur);
}

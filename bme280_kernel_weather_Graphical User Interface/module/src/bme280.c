/**
 * @brief Modified Bosch Sensortec's BME280 pressure sensor driver for
 * linux kernel
 *
 * @author Nguyen Nhan
 * @version 1.0
 */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>  // Cho extern gpioLED

#include <bme280.h>

#define BME280_TEMP_MIN -4000
#define BME280_TEMP_MAX 8500
#define BME280_PRESS_MIN 30000
#define BME280_PRESS_MAX 110000
#define BME280_HUM_MAX 102400
#define OVERSAMPLING_SETTINGS 0x07
#define FILTER_STANDBY_SETTINGS 0x18

extern unsigned int gpioLED;  // Extern từ module.c

/***************************** Common Functions *******************************/

static inline ssize_t null_ptr_check(const struct bme280 *self)
{
    return self == NULL ? BME280_E_NULL_PTR : BME280_OK;
}

/******************** Device Calibration Data Functions ***********************/

static void parse_temp_press_calib_data(struct bme280 *self, const u8 *reg_data)
{
    struct bme280_calib_data *calib_data = &self->calib_data;

    calib_data->dig_T1 = bme280_concat_bytes(reg_data[1], reg_data[0]);
    calib_data->dig_T2 = (s16)bme280_concat_bytes(reg_data[3], reg_data[2]);
    calib_data->dig_T3 = (s16)bme280_concat_bytes(reg_data[5], reg_data[4]);
    calib_data->dig_P1 = bme280_concat_bytes(reg_data[7], reg_data[6]);
    calib_data->dig_P2 = (s16)bme280_concat_bytes(reg_data[9], reg_data[8]);
    calib_data->dig_P3 = (s16)bme280_concat_bytes(reg_data[11], reg_data[10]);
    calib_data->dig_P4 = (s16)bme280_concat_bytes(reg_data[13], reg_data[12]);
    calib_data->dig_P5 = (s16)bme280_concat_bytes(reg_data[15], reg_data[14]);
    calib_data->dig_P6 = (s16)bme280_concat_bytes(reg_data[17], reg_data[16]);
    calib_data->dig_P7 = (s16)bme280_concat_bytes(reg_data[19], reg_data[18]);
    calib_data->dig_P8 = (s16)bme280_concat_bytes(reg_data[21], reg_data[20]);
    calib_data->dig_P9 = (s16)bme280_concat_bytes(reg_data[23], reg_data[22]);
    calib_data->dig_H1 = reg_data[25];
}

static void parse_humidity_calib_data(struct bme280 *self, const u8 *reg_data)
{
    struct bme280_calib_data *calib_data = &self->calib_data;
    s16 dig_H4_msb, dig_H4_lsb, dig_H5_msb, dig_H5_lsb;

    calib_data->dig_H2 = (s16)bme280_concat_bytes(reg_data[1], reg_data[0]);
    calib_data->dig_H3 = reg_data[2];
    dig_H4_msb = (s16)(s8)reg_data[3] * 16;
    dig_H4_lsb = (s16)(reg_data[4] & 0x0F);
    calib_data->dig_H4 = dig_H4_msb | dig_H4_lsb;
    dig_H5_msb = (s16)(s8)reg_data[5] * 16;
    dig_H5_lsb = (s16)(reg_data[4] >> 4);
    calib_data->dig_H5 = dig_H5_msb | dig_H5_lsb;
    calib_data->dig_H6 = (s8)reg_data[6];
}

static ssize_t get_calib_data(struct bme280 *self)
{
    ssize_t ret;
    u8 calib_data[BME280_TEMP_PRESS_CALIB_DATA_LEN] = {0};
    u8 reg_addr = BME280_TEMP_PRESS_CALIB_DATA_ADDR;

    ret = bme280_get_regs(self, reg_addr, calib_data, BME280_TEMP_PRESS_CALIB_DATA_LEN);
    if (ret != BME280_OK) {
        return ret;
    }
    parse_temp_press_calib_data(self, calib_data);

    reg_addr = BME280_HUM_CALIB_DATA_ADDR;
    ret = bme280_get_regs(self, reg_addr, calib_data, BME280_HUM_CALIB_DATA_LEN);
    if (ret != BME280_OK) {
        return ret;
    }
    parse_humidity_calib_data(self, calib_data);

    return BME280_OK;
}

/***************************** Initialization *********************************/

ssize_t bme280_init(struct bme280 *self, struct i2c_client *client)
{
    ssize_t ret;
    u8 chip_id;

    ret = null_ptr_check(self);
    if (ret != BME280_OK) {
        return ret;
    }

    self->client = client;
    ret = bme280_get_regs(self, BME280_CHIP_ID_ADDR, &chip_id, 1);
    if (ret != BME280_OK) {
        return ret;
    }

    if (chip_id != BME280_CHIP_ID) {
        pr_err(THIS_MODULE_NAME ": chip ID mismatch: 0x%x\n", chip_id);
        return -ENODEV;
    }
    self->chip_id = chip_id;

    ret = get_calib_data(self);
    if (ret != BME280_OK) {
        return ret;
    }

    return BME280_OK;
}

/***************************** Register Access ********************************/

ssize_t bme280_get_regs(const struct bme280 *self, u8 reg_addr, u8 *reg_data, u8 len)
{
    ssize_t ret;

    ret = null_ptr_check(self);
    if (ret != BME280_OK) {
        return ret;
    }

    if (len == 0) {
        return BME280_E_INVALID_LEN;
    }

    ret = i2c_smbus_read_i2c_block_data(self->client, reg_addr, len, reg_data);
    if (ret < 0) {
        pr_err(THIS_MODULE_NAME ": failed to read registers at 0x%x\n", reg_addr);
        return BME280_E_COMM_FAIL;
    }

    return BME280_OK;
}

ssize_t bme280_set_regs(const struct bme280 *self, const u8 *reg_addr, const u8 *reg_data, u8 len)
{
    ssize_t ret;
    u8 tmp[32];
    int i;

    ret = null_ptr_check(self);
    if (ret != BME280_OK) {
        return ret;
    }

    if (len == 0 || len > 31) {
        return BME280_E_INVALID_LEN;
    }

    for (i = 0; i < len; i++) {
        tmp[i * 2] = reg_addr[i];
        tmp[i * 2 + 1] = reg_data[i];
    }

    ret = i2c_master_send(self->client, tmp, len * 2);
    if (ret < 0) {
        pr_err(THIS_MODULE_NAME ": failed to write registers at 0x%x\n", reg_addr[0]);
        return BME280_E_COMM_FAIL;
    }

    return BME280_OK;
}

/***************************** Sensor Settings ********************************/

ssize_t bme280_get_sensor_settings(struct bme280 *self)
{
    ssize_t ret;
    u8 reg_data[3];

    ret = null_ptr_check(self);
    if (ret != BME280_OK) {
        return ret;
    }

    ret = bme280_get_regs(self, BME280_CTRL_HUM_ADDR, reg_data, 3);
    if (ret != BME280_OK) {
        return ret;
    }

    self->settings.osrs_h = reg_data[0] & 0x07;
    self->settings.osrs_t = (reg_data[1] >> 5) & 0x07;
    self->settings.osrs_p = (reg_data[1] >> 2) & 0x07;
    self->settings.filter = (reg_data[2] >> 2) & 0x07;
    self->settings.standby_time = (reg_data[2] >> 5) & 0x07;

    return BME280_OK;
}

ssize_t bme280_set_sensor_settings(const struct bme280 *self, u8 desired_settings)
{
    ssize_t ret;
    union bme280_ctrl_hum ctrl_hum;
    union bme280_ctrl_meas ctrl_meas;
    union bme280_config config;

    ret = null_ptr_check(self);
    if (ret != BME280_OK) {
        return ret;
    }

    if (desired_settings & BME280_OSRS_HUM_SEL) {
        ctrl_hum.bits.osrs_h = self->settings.osrs_h;
        ctrl_hum.bits.reserved = 0;
        ret = bme280_set_regs(self, &(u8){BME280_CTRL_HUM_ADDR}, &ctrl_hum.reg, 1);
        if (ret != BME280_OK) {
            return ret;
        }
    }

    if (desired_settings & (BME280_OSRS_PRESS_SEL | BME280_OSRS_TEMP_SEL)) {
        ret = bme280_get_regs(self, BME280_CTRL_MEAS_ADDR, &ctrl_meas.reg, 1);
        if (ret != BME280_OK) {
            return ret;
        }
        if (desired_settings & BME280_OSRS_PRESS_SEL) {
            ctrl_meas.bits.osrs_p = self->settings.osrs_p;
        }
        if (desired_settings & BME280_OSRS_TEMP_SEL) {
            ctrl_meas.bits.osrs_t = self->settings.osrs_t;
        }
        ret = bme280_set_regs(self, &(u8){BME280_CTRL_MEAS_ADDR}, &ctrl_meas.reg, 1);
        if (ret != BME280_OK) {
            return ret;
        }
    }

    if (desired_settings & (BME280_FILTER_SEL | BME280_STANDBY_TIME_SEL)) {
        ret = bme280_get_regs(self, BME280_CONFIG_ADDR, &config.reg, 1);
        if (ret != BME280_OK) {
            return ret;
        }
        if (desired_settings & BME280_FILTER_SEL) {
            config.bits.filter = self->settings.filter;
        }
        if (desired_settings & BME280_STANDBY_TIME_SEL) {
            config.bits.t_sb = self->settings.standby_time;
        }
        config.bits.spi3w_en = 0;
        config.bits.reserved1 = 0;
        ret = bme280_set_regs(self, &(u8){BME280_CONFIG_ADDR}, &config.reg, 1);
        if (ret != BME280_OK) {
            return ret;
        }
    }

    return BME280_OK;
}

/***************************** Sensor Mode ************************************/

ssize_t bme280_get_sensor_mode(const struct bme280 *self, u8 *sensor_mode)
{
    ssize_t ret;
    union bme280_ctrl_meas ctrl_meas;

    ret = null_ptr_check(self);
    if (ret != BME280_OK) {
        return ret;
    }

    ret = bme280_get_regs(self, BME280_CTRL_MEAS_ADDR, &ctrl_meas.reg, 1);
    if (ret != BME280_OK) {
        return ret;
    }

    *sensor_mode = ctrl_meas.bits.mode;
    return BME280_OK;
}

ssize_t bme280_set_sensor_mode(const struct bme280 *self, u8 sensor_mode)
{
    ssize_t ret;
    union bme280_ctrl_meas ctrl_meas;

    ret = null_ptr_check(self);
    if (ret != BME280_OK) {
        return ret;
    }

    ret = bme280_get_regs(self, BME280_CTRL_MEAS_ADDR, &ctrl_meas.reg, 1);
    if (ret != BME280_OK) {
        return ret;
    }

    ctrl_meas.bits.mode = sensor_mode;
    ret = bme280_set_regs(self, &(u8){BME280_CTRL_MEAS_ADDR}, &ctrl_meas.reg, 1);
    if (ret != BME280_OK) {
        return ret;
    }

    return BME280_OK;
}

/***************************** Soft Reset *************************************/

ssize_t bme280_soft_reset(const struct bme280 *self)
{
    ssize_t ret;

    ret = null_ptr_check(self);
    if (ret != BME280_OK) {
        return ret;
    }

    ret = bme280_set_regs(self, &(u8){BME280_RESET_ADDR}, &(u8){BME280_SOFT_RESET_COMMAND}, 1);
    if (ret != BME280_OK) {
        return ret;
    }

    msleep(2);  // Wait for reset to complete
    return BME280_OK;
}

/***************************** Sensor Data ************************************/

void bme280_parse_sensor_data(const u8 *reg_data, struct bme280_uncomp_data *uncomp_data)
{
    uncomp_data->pressure = (u32)((reg_data[0] << 12) | (reg_data[1] << 4) | (reg_data[2] >> 4));
    uncomp_data->temperature = (u32)((reg_data[3] << 12) | (reg_data[4] << 4) | (reg_data[5] >> 4));
    uncomp_data->humidity = (u32)bme280_concat_bytes(reg_data[6], reg_data[7]);
}

ssize_t bme280_compensate_data(u8 sensor_comp, const struct bme280_uncomp_data *uncomp_data,
                               struct bme280_data *comp_data, struct bme280_calib_data *calib_data)
{
    s32 t_fine;
    s64 var1, var2, p;

    if (sensor_comp & BME280_TEMP) {
        // Temperature compensation
        var1 = (((s32)uncomp_data->temperature) >> 3) - ((s32)calib_data->dig_T1 << 1);
        var1 = (var1 * ((s32)calib_data->dig_T2)) >> 11;
        var2 = (((((s32)uncomp_data->temperature) >> 4) - ((s32)calib_data->dig_T1)) *
                (((s32)uncomp_data->temperature >> 4) - ((s32)calib_data->dig_T1))) >> 12;
        var2 = (var2 * ((s32)calib_data->dig_T3)) >> 14;
        t_fine = var1 + var2;
        comp_data->temperature = (t_fine * 5 + 128) >> 8;
        calib_data->t_fine = t_fine;
    }

    if (sensor_comp & BME280_PRESS) {
        // Pressure compensation
        var1 = (((s64)t_fine) >> 1) - (s64)64000;
        var2 = var1 * var1 * ((s64)calib_data->dig_P6) >> 21;
        var2 = var2 + ((var1 * ((s64)calib_data->dig_P5)) << 1);
        var2 = (var2 >> 2) + (((s64)calib_data->dig_P4) << 35);
        var1 = ((var1 * var1 * ((s64)calib_data->dig_P3) >> 8) +
                ((var1 * ((s64)calib_data->dig_P2)) << 12)) >> 13;
        var1 = (((((s64)1) << 47) + var1)) * ((s64)calib_data->dig_P1) >> 33;
        if (var1 == 0) {
            comp_data->pressure = 0;
            return BME280_E_NULL_PTR;
        }
        p = 1048576 - uncomp_data->pressure;
        p = (((p << 31) - var2) * 3125) / var1;
        var1 = (((s64)calib_data->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2 = (((s64)calib_data->dig_P8) * p) >> 19;
        p = ((p + var1 + var2) >> 8) + (((s64)calib_data->dig_P7) << 4);
        comp_data->pressure = (u32)p;
    }

    if (sensor_comp & BME280_HUM) {
        // Humidity compensation
        if (calib_data->t_fine == 0) {
            return BME280_E_NULL_PTR;  // Temperature compensation required first
        }
        s32 v_x1_u32r;
        v_x1_u32r = (calib_data->t_fine - ((s32)76800));
        v_x1_u32r = (((((uncomp_data->humidity << 14) - (((s32)calib_data->dig_H4) << 20) -
                        (((s32)calib_data->dig_H5) * v_x1_u32r)) +
                       ((s32)16384)) >> 15) *
                     (((((((v_x1_u32r * ((s32)calib_data->dig_H6)) >> 10) *
                         (((v_x1_u32r * ((s32)calib_data->dig_H3)) >> 11) + ((s32)32768))) >> 10) +
                        ((s32)2097152)) * ((s32)calib_data->dig_H2) + 8192) >> 14));
        v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                                  ((s32)calib_data->dig_H1)) >> 4));
        v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
        v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
        comp_data->humidity = (u32)(v_x1_u32r >> 12);
    }

    return BME280_OK;
}

ssize_t bme280_get_sensor_data(struct bme280 *self, u8 sensor_comp, struct bme280_data *comp_data)
{
    ssize_t ret;
    u8 reg_data[BME280_PRESS_TEMP_HUM_DATA_LEN];
    struct bme280_uncomp_data uncomp_data;

    ret = null_ptr_check(self);
    if (ret != BME280_OK) {
        return ret;
    }

    if (sensor_comp == 0) {
        return BME280_E_INVALID_LEN;
    }

    ret = bme280_get_regs(self, BME280_DATA_ADDR, reg_data, BME280_PRESS_TEMP_HUM_DATA_LEN);
    if (ret != BME280_OK) {
        return ret;
    }

    bme280_parse_sensor_data(reg_data, &uncomp_data);
    ret = bme280_compensate_data(sensor_comp, &uncomp_data, comp_data, &self->calib_data);
    if (ret != BME280_OK) {
        return ret;
    }

    return BME280_OK;
}

ssize_t bme280_get_sensor_data_forced(struct bme280 *self, u8 sensor_comp, struct bme280_data *comp_data)
{
    ssize_t ret;
    union bme280_status status;

    ret = null_ptr_check(self);
    if (ret != BME280_OK) {
        return ret;
    }

    // Bật LED trước khi đọc
    gpio_set_value(gpioLED, 1);

    ret = bme280_set_sensor_mode(self, BME280_FORCED_MODE);
    if (ret != BME280_OK) {
        goto err;
    }

    ret = bme280_get_regs(self, BME280_STATUS_ADDR, &status.reg, 1);
    if (ret != BME280_OK) {
        goto err;
    }

    while (status.bits.measuring) {
        ret = bme280_get_regs(self, BME280_STATUS_ADDR, &status.reg, 1);
        if (ret != BME280_OK) {
            goto err;
        }
        msleep(1);  // Wait for measurement to complete
    }

    ret = bme280_get_sensor_data(self, sensor_comp, comp_data);
    if (ret != BME280_OK) {
        goto err;
    }

    ret = bme280_set_sensor_mode(self, BME280_SLEEP_MODE);
    if (ret != BME280_OK) {
        goto err;
    }

err:
    // Tắt LED sau khi đọc
    gpio_set_value(gpioLED, 0);
    return ret;
}
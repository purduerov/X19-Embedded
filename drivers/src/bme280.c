/**
 * @file bme280.c
 * @brief Bosch BME280 Environmental Sensor Driver Implementation.
 * @organization Purdue ROV
 */

#include "bme280.h"
#include <stdbool.h>
#include <string.h>

// BME280 map 
#define BME280_CHIP_ID       0x60u

#define BME280_REG_CALIB00   0x88u 
#define BME280_REG_H1        0xA1u
#define BME280_REG_CHIP_ID   0xD0u
#define BME280_REG_CTRL_HUM  0xF2u
#define BME280_REG_CTRL_MEAS 0xF4u
#define BME280_REG_CONFIG    0xF5u
#define BME280_REG_DATA      0xF7u
#define BME280_REG_CALIB26   0xE1u

#define BME280_I2C_ADDR_PRIMARY   0x76u
#define BME280_I2C_ADDR_SECONDARY 0x77u

static rov_status_t bme280_check_id(bme280_dev_t *dev){
    uint8_t id = 0; 

    rov_status_t status = bsp_i2c_mem_read(
        dev->i2c_addr, 
        BME280_REG_CHIP_ID, 
        &id, 
        1
    ); 

    if(status != ROV_OK){
        return status; 
    }

    if(id != BME280_CHIP_ID){
        return ROV_ERROR; 
    }

    return ROV_OK; 
}

static uint16_t read_u16_le(const uint8_t *p) {
    return (uint16_t)p[0] |
           ((uint16_t)p[1] << 8);
}

static int16_t read_s16_le(const uint8_t *p) {
    return (int16_t)read_u16_le(p);
}

static rov_status_t bme280_read_calibration(bme280_dev_t *dev){
    uint8_t calib1[26]; 
    uint8_t calib2[7]; 

    rov_status_t status; 

    status = bsp_i2c_mem_read(
        dev->i2c_addr, 
        BME280_REG_CALIB00, 
        calib1, 
        sizeof(calib1)
    ); 

    if(status != ROV_OK){
        return status; 
    }

    dev->calib.dig_T1 = read_u16_le(&calib1[0]);
    dev->calib.dig_T2 = read_s16_le(&calib1[2]);
    dev->calib.dig_T3 = read_s16_le(&calib1[4]);

    dev->calib.dig_P1 = read_u16_le(&calib1[6]);
    dev->calib.dig_P2 = read_s16_le(&calib1[8]);
    dev->calib.dig_P3 = read_s16_le(&calib1[10]);
    dev->calib.dig_P4 = read_s16_le(&calib1[12]);
    dev->calib.dig_P5 = read_s16_le(&calib1[14]);
    dev->calib.dig_P6 = read_s16_le(&calib1[16]);
    dev->calib.dig_P7 = read_s16_le(&calib1[18]);
    dev->calib.dig_P8 = read_s16_le(&calib1[20]);
    dev->calib.dig_P9 = read_s16_le(&calib1[22]);

    dev->calib.dig_H1 = calib1[25];

    status = bsp_i2c_mem_read(
        dev->i2c_addr, 
        BME280_REG_CALIB26, 
        calib2, 
        sizeof(calib2)
    ); 

    if(status != ROV_OK){
        return status; 
    }

    dev->calib.dig_H2 = read_s16_le(&calib2[0]); 
    dev->calib.dig_H3 = calib2[2]; 

    // Bosch packs them into 12 bits? 
    dev->calib.dig_H4 = (int16_t)(((int16_t)(uint8_t)calib2[3] << 4) | (calib2[4] & 0x0fu)); 
    dev->calib.dig_H5 = (int16_t)(((int16_t)(uint8_t)calib2[5] << 4) | (calib2[4] >> 4)); 
    
    dev->calib.dig_H6 = (uint8_t)calib2[6]; 

    //validate the coefficients 
    if(dev->calib.dig_T1 == 0u || dev->calib.dig_P1 == 0u){
        return ROV_ERROR; 
    }

    return ROV_OK; 
}

static rov_status_t bme280_configure(bme280_dev_t *dev){
    uint8_t value; 
    rov_status_t status; 

    value = 0x01u; // humidity oversampling x1 
    status = bsp_i2c_mem_write(
        dev->i2c_addr, 
        BME280_REG_CTRL_HUM, 
        &value, 
        1
    ); 

    if(status != ROV_OK){
        return status; 
    }

    value = 0x10u; // standby 0.5ms, IIR coefficient 16 
    status = bsp_i2c_mem_write(
        dev->i2c_addr, 
        BME280_REG_CONFIG, 
        &value, 
        1
    ); 

    if(status != ROV_OK){
        return status; 
    }

    value = 0x57u; // temperature x2, pressure x16, normal mode 
    status = bsp_i2c_mem_write(
        dev->i2c_addr, 
        BME280_REG_CTRL_MEAS, 
        &value, 
        1
    ); 

    if(status != ROV_OK){
        return status; 
    }

    return ROV_OK; 
}

rov_status_t bme280_init(bme280_dev_t *dev) {
    rov_status_t status;

    if (dev == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));

    dev->i2c_addr = BME280_I2C_ADDR_PRIMARY;

    status = bme280_check_id(dev);

    if (status != ROV_OK) {
        dev->i2c_addr = BME280_I2C_ADDR_SECONDARY;

        status = bme280_check_id(dev);

        if (status != ROV_OK) {
            return status;
        }
    }

    status = bme280_read_calibration(dev);
    if (status != ROV_OK) {
        return status;
    }

    status = bme280_configure(dev);
    if (status != ROV_OK) {
        return status;
    }

    dev->initialized = true;

    return ROV_OK;
}

// Bosch's integer compensation algorithm.
static int32_t bme280_compensate_temperature(bme280_dev_t *dev, int32_t adc_t){
    int32_t var1; 
    int32_t var2; 

    var1 = (((adc_t >> 3) - ((uint32_t)dev->calib.dig_T1 << 1)) * 
        ((uint32_t)dev->calib.dig_T2)) >> 11; 
    var2 = (((((adc_t >> 4) - ((int32_t)dev->calib.dig_T1)) * 
        ((adc_t >> 4) - ((uint32_t)dev->calib.dig_T1))) >> 12) * 
        ((uint32_t)dev->calib.dig_T3)) >> 14; 
    dev->t_fine = var1 + var2; 
    
    return (dev->t_fine * 5 + 128) >> 8; 
}

static uint32_t bme280_compensate_pressure(bme280_dev_t *dev, int32_t adc_p){
    int64_t var1; // 64 for large intermediate calculation 
    int64_t var2; 
    int64_t p; 

    var1 = ((int64_t)dev->t_fine) - 128000;

    var2 = var1 * var1 * (int64_t)dev->calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->calib.dig_P4) << 35);

    var1 = ((var1 * var1 * (int64_t)dev->calib.dig_P3) >> 8) +
           ((var1 * (int64_t)dev->calib.dig_P2) << 12);

    var1 =
        (((((int64_t)1) << 47) + var1) *
         (int64_t)dev->calib.dig_P1) >>
        33;

    /* Protect against division by zero. */
    if (var1 == 0) {
        return 0;
    }

    p = 1048576 - adc_p;

    p = (((p << 31) - var2) * 3125) / var1;

    var1 =
        (((int64_t)dev->calib.dig_P9) *
         (p >> 13) *
         (p >> 13)) >>
        25;

    var2 =
        (((int64_t)dev->calib.dig_P8) * p) >>
        19;

    p =
        ((p + var1 + var2) >> 8) +
        (((int64_t)dev->calib.dig_P7) << 4);

    return (uint32_t)p;
}

static uint32_t bme280_compensate_humidity(bme280_dev_t *dev, int32_t adc_h) {
    int32_t v_x1_u32r;

    v_x1_u32r = dev->t_fine - ((int32_t)76800);

    v_x1_u32r =
        (((((adc_h << 14) -
            (((int32_t)dev->calib.dig_H4) << 20) -
            (((int32_t)dev->calib.dig_H5) * v_x1_u32r)) +
           ((int32_t)16384)) >>
          15) *
         (((((((v_x1_u32r *
                ((int32_t)dev->calib.dig_H6)) >>
               10) *
              (((v_x1_u32r *
                  ((int32_t)dev->calib.dig_H3)) >>
                 11) +
                ((int32_t)32768))) >>
             10) +
            ((int32_t)2097152)) *
               ((int32_t)dev->calib.dig_H2) +
           8192) >>
          14));

    v_x1_u32r =
        v_x1_u32r -
        (((((v_x1_u32r >> 15) *
            (v_x1_u32r >> 15)) >>
           7) *
          ((int32_t)dev->calib.dig_H1)) >>
         4);

    /* Clamp humidity to the valid range. */
    if (v_x1_u32r < 0) {
        v_x1_u32r = 0;
    }

    if (v_x1_u32r > 419430400) {
        v_x1_u32r = 419430400;
    }

    return (uint32_t)(v_x1_u32r >> 12);
}

rov_status_t bme280_read_all(bme280_dev_t *dev) {
    if (dev == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    if (!dev->initialized) {
        return ROV_ERROR;
    }

    uint8_t data[8];

    rov_status_t status = bsp_i2c_mem_read(
        dev->i2c_addr,
        BME280_REG_DATA,
        data,
        sizeof(data)
    );

    if (status != ROV_OK) {
        return status;
    }

    int32_t adc_p =
        ((int32_t)data[0] << 12) |
        ((int32_t)data[1] << 4) |
        ((int32_t)data[2] >> 4);

    int32_t adc_t =
        ((int32_t)data[3] << 12) |
        ((int32_t)data[4] << 4) |
        ((int32_t)data[5] >> 4);

    int32_t adc_h =
        ((int32_t)data[6] << 8) |
        (int32_t)data[7];

    int32_t temp_x100 =
        bme280_compensate_temperature(dev, adc_t);

    uint32_t pressure_q24_8 =
        bme280_compensate_pressure(dev, adc_p);

    uint32_t humidity_q22_10 =
        bme280_compensate_humidity(dev, adc_h);

    dev->temperature_c =
        (float)temp_x100 / 100.0f;

    dev->pressure_hpa =
        (float)pressure_q24_8 / 25600.0f;

    dev->humidity_pct =
        (float)humidity_q22_10 / 1024.0f;

    return ROV_OK;
}
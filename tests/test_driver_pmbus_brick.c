/**
 * @file test_driver_pmbus_brick.c
 * @brief Unit tests for Power Slab PMBus DC-DC Converter Brick Driver.
 * @organization Purdue ROV
 */

#include "pmbus_brick.h"

#include "mocks/mock_sensors.h"

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define FLOAT_TOLERANCE 0.001f

static bool float_near(float actual, float expected) {
    float difference = actual - expected;

    if (difference < 0.0f) {
        difference = -difference;
    }

    return difference <= FLOAT_TOLERANCE;
}

/**
 * @brief Verify basic PMBus brick initialization and mocked telemetry.
 */
static void test_pmbus_brick_driver(void) {
    pmbus_brick_dev_t dev;

    /* Negative test: NULL pointer */
    assert(pmbus_brick_init(NULL, PMBUS_BRICK_ADDR_12V_0) == ROV_ERR_INVALID_ARG);

    assert(pmbus_brick_read_telemetry(NULL) == ROV_ERR_INVALID_ARG);

    /* Valid initialization */
    assert(pmbus_brick_init(&dev, PMBUS_BRICK_ADDR_12V_1) == ROV_OK);

    assert(dev.pmbus_addr == PMBUS_BRICK_ADDR_12V_1);
    assert(float_near(dev.output_voltage_v, 12.0f));

    /*
     * Test mock telemetry injection:
     * 48.0 V in, 12.05 V out @ 8.5 A, 41.5 C
     */
    mock_sensors_reset();

    mock_sensors_set_tps25990(1U, 48.0f, 12.05f, 8.5f, 41.5f, 0x0000U);

    assert(pmbus_brick_read_telemetry(&dev) == ROV_OK);

    assert(float_near(dev.input_voltage_v, 48.0f));
    assert(float_near(dev.output_voltage_v, 12.05f));
    assert(float_near(dev.output_current_a, 8.5f));
    assert(float_near(dev.temperature_c, 41.5f));

    printf("[PASS] test_pmbus_brick_driver\n");
}

/**
 * @brief Verify known positive and negative LINEAR11 values.
 */
static void test_pmbus_linear11_conversion(void) {
    /*
     * LINEAR11:
     *
     * value = mantissa * 2^exponent
     *
     * 48.0  =  768 * 2^-4
     * 12.0  =  768 * 2^-6
     * 0.5   =  512 * 2^-10
     * -2.5  = -640 * 2^-8
     * 45.0  =  720 * 2^-4
     */

    assert(float_near(pmbus_linear11_to_float(0xE300U), 48.0f));

    assert(float_near(pmbus_linear11_to_float(0xD300U), 12.0f));

    assert(float_near(pmbus_linear11_to_float(0xB200U), 0.5f));

    assert(float_near(pmbus_linear11_to_float(0xC580U), -2.5f));

    assert(float_near(pmbus_linear11_to_float(0xE2D0U), 45.0f));

    printf("[PASS] test_pmbus_linear11_conversion\n");
}

/**
 * @brief Verify all five Power Slab PMBus addresses.
 */
static void test_pmbus_address_validation(void) {
    assert(pmbus_brick_address_valid(PMBUS_BRICK_ADDR_12V_0));

    assert(pmbus_brick_address_valid(PMBUS_BRICK_ADDR_12V_1));

    assert(pmbus_brick_address_valid(PMBUS_BRICK_ADDR_12V_2));

    assert(pmbus_brick_address_valid(PMBUS_BRICK_ADDR_12V_3));

    assert(pmbus_brick_address_valid(PMBUS_BRICK_ADDR_5V2));

    assert(!pmbus_brick_address_valid(0x3FU));
    assert(!pmbus_brick_address_valid(0x45U));

    printf("[PASS] test_pmbus_address_validation\n");
}

/**
 * @brief Verify correct converter voltage initialization.
 */
static void test_pmbus_brick_voltage_mapping(void) {
    pmbus_brick_dev_t dev;

    assert(pmbus_brick_init(&dev, PMBUS_BRICK_ADDR_12V_0) == ROV_OK);

    assert(float_near(dev.output_voltage_v, 12.0f));

    assert(pmbus_brick_init(&dev, PMBUS_BRICK_ADDR_12V_1) == ROV_OK);

    assert(float_near(dev.output_voltage_v, 12.0f));

    assert(pmbus_brick_init(&dev, PMBUS_BRICK_ADDR_12V_2) == ROV_OK);

    assert(float_near(dev.output_voltage_v, 12.0f));

    assert(pmbus_brick_init(&dev, PMBUS_BRICK_ADDR_12V_3) == ROV_OK);

    assert(float_near(dev.output_voltage_v, 12.0f));

    assert(pmbus_brick_init(&dev, PMBUS_BRICK_ADDR_5V2) == ROV_OK);

    assert(float_near(dev.output_voltage_v, 5.2f));

    /* Invalid address */
    assert(pmbus_brick_init(&dev, 0x45U) == ROV_ERR_INVALID_ARG);

    printf("[PASS] test_pmbus_brick_voltage_mapping\n");
}

/**
 * @brief Verify round-robin polling order across all five bricks.
 */
static void test_pmbus_round_robin(void) {
    uint8_t address = PMBUS_BRICK_ADDR_12V_0;

    address = pmbus_brick_next_address(address);
    assert(address == PMBUS_BRICK_ADDR_12V_1);

    address = pmbus_brick_next_address(address);
    assert(address == PMBUS_BRICK_ADDR_12V_2);

    address = pmbus_brick_next_address(address);
    assert(address == PMBUS_BRICK_ADDR_12V_3);

    address = pmbus_brick_next_address(address);
    assert(address == PMBUS_BRICK_ADDR_5V2);

    address = pmbus_brick_next_address(address);
    assert(address == PMBUS_BRICK_ADDR_12V_0);

    printf("[PASS] test_pmbus_round_robin\n");
}

/**
 * @brief Verify PMBus STATUS_WORD fault-bit decoding.
 */
static void test_pmbus_status_word_faults(void) {
    uint16_t status = 0U;

    /*
     * No faults.
     */
    assert(!pmbus_status_has_overcurrent(status));
    assert(!pmbus_status_has_overvoltage(status));
    assert(!pmbus_status_has_uvlo(status));
    assert(!pmbus_status_has_thermal_fault(status));

    /*
     * Individual faults.
     */
    status = PMBUS_STATUS_IOUT_OC;
    assert(pmbus_status_has_overcurrent(status));

    status = PMBUS_STATUS_VOUT_OV;
    assert(pmbus_status_has_overvoltage(status));

    status = PMBUS_STATUS_VIN_UV;
    assert(pmbus_status_has_uvlo(status));

    status = PMBUS_STATUS_TEMP;
    assert(pmbus_status_has_thermal_fault(status));

    /*
     * Multiple fault bits asserted simultaneously.
     */
    status = PMBUS_STATUS_IOUT_OC | PMBUS_STATUS_VOUT_OV | PMBUS_STATUS_VIN_UV | PMBUS_STATUS_TEMP;

    assert(pmbus_status_has_overcurrent(status));
    assert(pmbus_status_has_overvoltage(status));
    assert(pmbus_status_has_uvlo(status));
    assert(pmbus_status_has_thermal_fault(status));

    printf("[PASS] test_pmbus_status_word_faults\n");
}

int main(void) {
    printf("Running PMBus Brick Driver Unit Tests...\n");

    test_pmbus_brick_driver();
    test_pmbus_linear11_conversion();
    test_pmbus_address_validation();
    test_pmbus_brick_voltage_mapping();
    test_pmbus_round_robin();
    test_pmbus_status_word_faults();

    printf("All PMBus Brick Driver Tests Passed Successfully!\n");

    return 0;
}

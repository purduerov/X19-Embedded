/**
 * @file test_mock_i2c_bus.c
 * @brief Tests for the host-side injected I2C bus over the mock register file.
 * @organization Purdue ROV
 */

#include "mock_bsp.h"
#include "rov_i2c.h"
#include <assert.h>
#include <stdio.h>

static void test_mem_read_and_write_round_trip(void) {
    const rov_i2c_bus_t *bus = bsp_i2c_bus();
    uint8_t value = 0x5Au;
    uint8_t readback = 0u;

    mock_bsp_reset();

    assert(rov_i2c_mem_write(bus, 0x29u, 0x02u, &value, 1U) == ROV_OK);
    assert(rov_i2c_mem_read(bus, 0x29u, 0x02u, &readback, 1U) == ROV_OK);
    assert(readback == 0x5Au);
    assert(mock_bsp_i2c_get_reg(0x29u, 0x02u) == 0x5Au);
}

static void test_multi_byte_read_spans_registers(void) {
    const rov_i2c_bus_t *bus = bsp_i2c_bus();
    uint8_t seed[3] = {0x11u, 0x22u, 0x33u};
    uint8_t out[3] = {0u};

    mock_bsp_reset();
    mock_bsp_i2c_set_regs(0x76u, 0x10u, seed, sizeof(seed));

    assert(rov_i2c_mem_read(bus, 0x76u, 0x10u, out, 3U) == ROV_OK);
    assert(out[0] == 0x11u);
    assert(out[1] == 0x22u);
    assert(out[2] == 0x33u);
}

static void test_fault_is_returned_and_cleared_on_reset(void) {
    const rov_i2c_bus_t *bus = bsp_i2c_bus();
    uint8_t out[1] = {0u};

    mock_bsp_reset();
    mock_bsp_i2c_set_fault(ROV_TIMEOUT);

    assert(rov_i2c_mem_read(bus, 0x76u, 0x10u, out, 1U) == ROV_TIMEOUT);

    mock_bsp_reset();
    assert(rov_i2c_mem_read(bus, 0x76u, 0x10u, out, 1U) == ROV_OK);
}

static void test_probe_reports_presence(void) {
    const rov_i2c_bus_t *bus = bsp_i2c_bus();

    mock_bsp_reset();
    assert(bus->probe(bus->ctx, 0x76u) == false);

    mock_bsp_i2c_set_present(0x76u, true);
    assert(bus->probe(bus->ctx, 0x76u) == true);
}

static void test_word_register_round_trip(void) {
    const rov_i2c_bus_t *bus = bsp_i2c_bus();
    uint8_t readback[2] = {0u, 0u};
    uint8_t writeback[2] = {0xABu, 0xCDu};

    mock_bsp_reset();
    mock_bsp_i2c_set_word(0x40u, 0x05u, 0x1234u);

    /* A two-byte read at register 0x05 returns the word, MSB first. */
    assert(rov_i2c_mem_read(bus, 0x40u, 0x05u, readback, 2U) == ROV_OK);
    assert(readback[0] == 0x12u);
    assert(readback[1] == 0x34u);

    /*
     * The word store must not touch the byte register at the same index, or
     * register 0x05 (VBUS) and register 0x06 (DIETEMP) would alias.
     */
    assert(mock_bsp_i2c_get_reg(0x40u, 0x05u) == 0x00u);
    assert(mock_bsp_i2c_get_reg(0x40u, 0x06u) == 0x00u);

    /* A two-byte write through the bus updates the word file. */
    assert(rov_i2c_mem_write(bus, 0x40u, 0x05u, writeback, 2U) == ROV_OK);
    assert(mock_bsp_i2c_get_word(0x40u, 0x05u) == 0xABCDu);

    /* A byte-addressed address is unaffected by the word path. */
    mock_bsp_i2c_set_regs(0x76u, 0x10u, writeback, 2U);
    assert(mock_bsp_i2c_get_reg(0x76u, 0x10u) == 0xABu);
    assert(rov_i2c_mem_read(bus, 0x76u, 0x10u, readback, 2U) == ROV_OK);
    assert(readback[0] == 0xABu);
    assert(readback[1] == 0xCDu);
}

int main(void) {
    printf("Running mock I2C bus tests...\n");

    test_mem_read_and_write_round_trip();
    test_multi_byte_read_spans_registers();
    test_fault_is_returned_and_cleared_on_reset();
    test_probe_reports_presence();
    test_word_register_round_trip();

    printf("[PASS] test_mock_i2c_bus\n");
    return 0;
}

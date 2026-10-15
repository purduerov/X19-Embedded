/**
 * @file test_rov_i2c.c
 * @brief Unit tests for the injected I2C bus contract helpers.
 * @organization Purdue ROV
 */

#include "rov_i2c.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    uint8_t regs[256];
    uint8_t ptr;
    int write_calls;
    int read_calls;
    rov_status_t write_status;
} fake_bus_t;

static rov_status_t fake_write(void *ctx, uint8_t addr, const uint8_t *data, uint16_t len) {
    fake_bus_t *b = (fake_bus_t *)ctx;
    (void)addr;

    if (len == 0U || data == NULL) {
        return ROV_ERR_INVALID_ARG;
    }

    b->write_calls++;

    if (b->write_status != ROV_OK) {
        return b->write_status;
    }

    b->ptr = data[0];

    for (uint16_t i = 1U; i < len; i++) {
        b->regs[(uint8_t)(b->ptr + (i - 1U))] = data[i];
    }

    return ROV_OK;
}

static rov_status_t fake_read(void *ctx, uint8_t addr, uint8_t *data, uint16_t len) {
    fake_bus_t *b = (fake_bus_t *)ctx;
    (void)addr;

    b->read_calls++;

    for (uint16_t i = 0U; i < len; i++) {
        data[i] = b->regs[(uint8_t)(b->ptr + i)];
    }

    return ROV_OK;
}

static bool fake_probe(void *ctx, uint8_t addr) {
    (void)ctx;
    (void)addr;
    return true;
}

static void test_mem_read_issues_pointer_write_then_read(void) {
    fake_bus_t b;
    uint8_t out[2] = {0};
    rov_i2c_bus_t bus = {&b, fake_write, fake_read, fake_probe};

    memset(&b, 0, sizeof(b));
    b.regs[0x10] = 0xABu;
    b.regs[0x11] = 0xCDu;

    assert(rov_i2c_mem_read(&bus, 0x76u, 0x10u, out, 2U) == ROV_OK);
    assert(out[0] == 0xABu);
    assert(out[1] == 0xCDu);
    assert(b.write_calls == 1);
    assert(b.read_calls == 1);
}

static void test_mem_write_sends_pointer_then_data(void) {
    fake_bus_t b;
    uint8_t value = 0x57u;
    rov_i2c_bus_t bus = {&b, fake_write, fake_read, fake_probe};

    memset(&b, 0, sizeof(b));

    assert(rov_i2c_mem_write(&bus, 0x76u, 0xF4u, &value, 1U) == ROV_OK);
    assert(b.regs[0xF4] == 0x57u);
    assert(b.write_calls == 1);
}

static void test_mem_write_rejects_over_long_payload(void) {
    fake_bus_t b;
    uint8_t value[16] = {0};
    rov_i2c_bus_t bus = {&b, fake_write, fake_read, fake_probe};

    memset(&b, 0, sizeof(b));

    assert(rov_i2c_mem_write(&bus, 0x76u, 0x00u, value, sizeof(value)) == ROV_ERR_INVALID_ARG);
    assert(b.write_calls == 0);
}

static void test_zero_length_transfers_are_rejected(void) {
    fake_bus_t b;
    uint8_t out[1] = {0};
    rov_i2c_bus_t bus = {&b, fake_write, fake_read, fake_probe};

    memset(&b, 0, sizeof(b));

    assert(rov_i2c_mem_read(&bus, 0x76u, 0x10u, out, 0U) == ROV_ERR_INVALID_ARG);
    assert(rov_i2c_mem_write(&bus, 0x76u, 0x10u, out, 0U) == ROV_ERR_INVALID_ARG);
    assert(b.write_calls == 0);
    assert(b.read_calls == 0);
}

static void test_null_bus_is_rejected(void) {
    uint8_t out[1] = {0};
    assert(rov_i2c_mem_read(NULL, 0x76u, 0x00u, out, 1U) == ROV_ERR_INVALID_ARG);
    assert(rov_i2c_mem_write(NULL, 0x76u, 0x00u, out, 1U) == ROV_ERR_INVALID_ARG);
}

static void test_write_error_propagates_before_read(void) {
    fake_bus_t b;
    uint8_t out[1] = {0};
    rov_i2c_bus_t bus = {&b, fake_write, fake_read, fake_probe};

    memset(&b, 0, sizeof(b));
    b.write_status = ROV_TIMEOUT;

    assert(rov_i2c_mem_read(&bus, 0x76u, 0x10u, out, 1U) == ROV_TIMEOUT);
    assert(b.read_calls == 0);
}

int main(void) {
    printf("Running injected I2C bus contract tests...\n");

    test_mem_read_issues_pointer_write_then_read();
    test_mem_write_sends_pointer_then_data();
    test_mem_write_rejects_over_long_payload();
    test_zero_length_transfers_are_rejected();
    test_null_bus_is_rejected();
    test_write_error_propagates_before_read();

    printf("[PASS] test_rov_i2c\n");
    return 0;
}

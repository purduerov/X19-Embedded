/**
 * @file app.c
 * @brief Application Layer Logic for Testing & R&D Node.
 * Contains 100% pure application logic with ZERO vendor ST HAL calls.
 */

#include "app.h"
#include "bench_config.h"
#include "bme280.h"
#include "bsp.h"
#include "can_interface.h"
#include <stdio.h>

#if defined(BENCH_APP_NODE1_PI_SHIELD)
#include "../../node1_pi_shield/Core/Inc/app.h"
#elif defined(BENCH_APP_NODE2_CONTROL)
#include "../../node2_control_board/Core/Inc/app.h"
#elif defined(BENCH_APP_NODE3_POWER)
#include "../../node3_power_slab/Core/Inc/app.h"
#endif

void app_main(void) {
    /* Initialize Board Support Package. */
    bsp_init();

#if defined(BENCH_APP_NODE1_PI_SHIELD)

    printf("\r\n======================================================\r\n");
    printf("  Bench Running: Node 1 (Pi Shield) on NUCLEO-F411RE \r\n");
    printf("======================================================\r\n");

    bsp_i2c_init();
    node1_app_init();

    while (1) {
        node1_app_step();
    }

#elif defined(BENCH_APP_NODE2_CONTROL)

    printf("\r\n======================================================\r\n");
    printf("  Bench Running: Node 2 (Control Board) on NUCLEO-F411\r\n");
    printf("======================================================\r\n");

    node2_app_init();

    while (1) {
        node2_app_step();
    }

#elif defined(BENCH_APP_NODE3_POWER)

    printf("\r\n======================================================\r\n");
    printf("  Bench Running: Node 3 (Power Slab) on NUCLEO-F411   \r\n");
    printf("======================================================\r\n");

    node3_app_init();

    while (1) {
        node3_app_step();
    }

#else /* BENCH_APP_RND_SCANNER */

    printf("\r\n======================================================\r\n");
    printf("  Bench Running: R&D BME280 Hardware Test             \r\n");
    printf("======================================================\r\n");

    /*
     * PB8 = SCL
     * PB9 = SDA
     */
    bsp_i2c_init();

    /*
     * First prove that something is physically responding
     * on the I2C bus.
     */
    bsp_i2c_scan();

    /*
     * Now initialize the real BME280 driver.
     *
     * bme280_init() will:
     *   1. Try address 0x76
     *   2. Try address 0x77 if necessary
     *   3. Read chip-ID register 0xD0
     *   4. Verify that it contains 0x60
     *   5. Read factory calibration coefficients
     *   6. Configure the sensor
     */
    bme280_dev_t bme280;

    rov_status_t status = bme280_init(&bme280);

    if (status != ROV_OK) {
        printf("\r\nBME280 initialization FAILED.\r\n");
        printf("Status = %d\r\n", (int)status);

        while (1) {
            delay_ms(1000);
        }
    }

    printf("\r\nBME280 initialized successfully!\r\n");
    printf("BME280 I2C address: 0x%02X\r\n", bme280.i2c_addr);
    printf("Reading live sensor data...\r\n\r\n");

    uint32_t last_bme280_time = 0;

    while (1) {
        /*
         * Read the physical BME280 once every second.
         */
        if (time_get_ms() - last_bme280_time >= 1000U) {
            last_bme280_time = time_get_ms();

            status = bme280_read_all(&bme280);

            if (status == ROV_OK) {
                printf("BME280 | Temp: %.2f F | Pressure: %.2f hPa | Humidity: %.2f %%\r\n",
                       (double)((bme280.temperature_c) * (float)(9.0 / 5) + (float)32.0), (double)bme280.pressure_hpa, (double)bme280.humidity_pct);
            } else {
                printf("BME280 read FAILED. Status = %d\r\n", (int)status);
            }
        }

        delay_ms(5);
    }

#endif
}

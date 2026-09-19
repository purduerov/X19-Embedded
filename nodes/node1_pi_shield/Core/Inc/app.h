/**
 * @file app.h
 * @brief Node 1 (Pi Shield) Application Entry Point & Stepping Interface.
 * @organization Purdue ROV
 */

#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the Node 1 application and hardware abstraction layer.
 */
void node1_app_init(void);

/**
 * @brief Execute one iteration of the Node 1 application.
 */
void node1_app_step(void);

/**
 * @brief Immediate floor leak interrupt application handler.
 *
 * Called by the hardware EXTI/BSP layer when either physical floor leak
 * probe detects water.
 *
 * This function immediately trips the local emergency brake and
 * broadcasts the signed Emergency Break frame on CAN ID 0x001.
 */
void node1_leak_irq_handler(void);

/**
 * @brief Node 1 application main loop.
 */
void app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */

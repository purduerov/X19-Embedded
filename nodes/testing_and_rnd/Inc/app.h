/**
 * @file app.h
 * @brief Application Layer Entry Point.
 */

#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Main application loop. Invoked by main.c after low-level boot.
 * Contains pure application logic with zero direct vendor ST HAL calls.
 */
void app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */

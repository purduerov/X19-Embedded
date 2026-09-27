/**
 * @file app.h
 * @brief Node 2 (Control Board) Application Entry Point & Stepping Interface.
 * @organization Purdue ROV
 */

#ifndef X19_NODE2_APP_H
#define X19_NODE2_APP_H

#ifdef __cplusplus
extern "C" {
#endif

void node2_app_init(void);
void node2_app_step(void);
void app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* X19_NODE2_APP_H */

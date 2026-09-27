/**
 * @file app.h
 * @brief Node 3 (Power Slab) Application Entry Point & Stepping Interface.
 * @organization Purdue ROV
 */

#ifndef X19_NODE3_APP_H
#define X19_NODE3_APP_H

#ifdef __cplusplus
extern "C" {
#endif

void node3_app_init(void);
void node3_app_step(void);
void app_main(void);

#ifdef __cplusplus
}
#endif

#endif /* X19_NODE3_APP_H */

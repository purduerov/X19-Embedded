#ifndef X19_TEST_FAKE_MAIN_H
#define X19_TEST_FAKE_MAIN_H

/*
 * ============================================================================
 * THIS HEADER IS NOT A SOURCE OF TRUTH FOR PINS. DO NOT DERIVE BSP CODE,
 * A .ioc FILE, OR A CUBEMX CONFIGURATION FROM IT.
 * ============================================================================
 *
 * Every pin macro below is a hand-written guess. None of it came from a
 * generated CubeMX `main.h` or from a schematic netlist. Passing the
 * `target_bsp_*` contract tests proves that the BSP logic is internally
 * correct and calls the HAL in the expected order. It does NOT prove that
 * firmware drives real hardware.
 *
 * Verified against the KiCad schematics on 2026-09-28, this header is
 * actively wrong:
 *
 *   - `EMERGENCY_CUTOFF` is on `GPIOC PIN_10`. The STM32C542CCT6 LQFP-48 that
 *     all three boards use has NO PC0-PC12 pins. PC13 is its only PC pin.
 *     This pin cannot physically be bonded out.
 *   - The Pi Shield has no leak-probe net and no emergency-cutoff net at all.
 *     PA4 and PA5 are unconnected on that board.
 *   - The Power Slab has no brick-enable net. Its four Murata E48SC12030
 *     converters have `ON/OFF` hard-tied to `VIN(-)`, so as drawn they are all
 *     held off. It also contains no LM74700.
 *   - The Control Board has no `SOL_0..SOL_9`. Its ten solenoid MOSFET gates are
 *     netted `GATE1..GATE10` on PA0-PA3, PA7, PB0-PB2, PB10 and VCAP.
 *   - `LED_STATUS` and `LED_HEARTBEAT` are both defined on `GPIOC PIN_13`, so
 *     they collide with each other. No board has a GPIO LED at all; the three
 *     Control Board LEDs are rail indicators.
 *
 * A green `target_bsp_node*` test is a statement about logic, never about
 * silicon. See docs/pinout_mappings.md and docs/board_findings.md.
 */

#include <stdbool.h>
#include <stdint.h>

#define STM32G4 1

#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
#define GPIO_MODE_INPUT 0x01U
#define GPIO_MODE_OUTPUT_PP 0x02U
#define GPIO_PULLUP 0x01U
#define GPIO_NOPULL 0x00U
#define GPIO_SPEED_FREQ_LOW 0x00U
#define HAL_OK 0U
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 1U
#define TIM_CHANNEL_3 2U
#define TIM_CHANNEL_4 3U

typedef struct {
    uint8_t id;
} GPIO_TypeDef;

typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Pull;
    uint32_t Speed;
} GPIO_InitTypeDef;

typedef struct {
    uint16_t compare[4];
    uint8_t pwm_started_mask;
} TIM_HandleTypeDef;

extern GPIO_TypeDef fake_gpio_a;
extern GPIO_TypeDef fake_gpio_b;
extern GPIO_TypeDef fake_gpio_c;
extern TIM_HandleTypeDef fake_htim1;
extern TIM_HandleTypeDef fake_htim8;

#define GPIOA (&fake_gpio_a)
#define GPIOB (&fake_gpio_b)
#define GPIOC (&fake_gpio_c)

#define htim1 fake_htim1
#define htim8 fake_htim8

#define PIN_0 (1U << 0U)
#define PIN_1 (1U << 1U)
#define PIN_2 (1U << 2U)
#define PIN_3 (1U << 3U)
#define PIN_4 (1U << 4U)
#define PIN_5 (1U << 5U)
#define PIN_6 (1U << 6U)
#define PIN_7 (1U << 7U)
#define PIN_8 (1U << 8U)
#define PIN_9 (1U << 9U)
#define PIN_10 (1U << 10U)
#define PIN_13 (1U << 13U)

#define LEAK_PROBE0_GPIO_Port GPIOA
#define LEAK_PROBE0_Pin PIN_4
#define LEAK_PROBE1_GPIO_Port GPIOA
#define LEAK_PROBE1_Pin PIN_5
#define EMERGENCY_CUTOFF_GPIO_Port GPIOC
#define EMERGENCY_CUTOFF_Pin PIN_10
#define LED_STATUS_GPIO_Port GPIOC
#define LED_STATUS_Pin PIN_13
#define LED_HEARTBEAT_GPIO_Port GPIOC
#define LED_HEARTBEAT_Pin PIN_13

#define SOL_0_GPIO_Port GPIOB
#define SOL_0_Pin PIN_0
#define SOL_1_GPIO_Port GPIOB
#define SOL_1_Pin PIN_1
#define SOL_2_GPIO_Port GPIOB
#define SOL_2_Pin PIN_2
#define SOL_3_GPIO_Port GPIOB
#define SOL_3_Pin PIN_3
#define SOL_4_GPIO_Port GPIOB
#define SOL_4_Pin PIN_4
#define SOL_5_GPIO_Port GPIOB
#define SOL_5_Pin PIN_5
#define SOL_6_GPIO_Port GPIOB
#define SOL_6_Pin PIN_6
#define SOL_7_GPIO_Port GPIOB
#define SOL_7_Pin PIN_7
#define SOL_8_GPIO_Port GPIOB
#define SOL_8_Pin PIN_8
#define SOL_9_GPIO_Port GPIOB
#define SOL_9_Pin PIN_9

#define BRICK1_EN_GPIO_Port GPIOB
#define BRICK1_EN_Pin PIN_0
#define BRICK2_EN_GPIO_Port GPIOB
#define BRICK2_EN_Pin PIN_1
#define BRICK3_EN_GPIO_Port GPIOB
#define BRICK3_EN_Pin PIN_2
#define BRICK4_EN_GPIO_Port GPIOB
#define BRICK4_EN_Pin PIN_4
#define LM74700_STAT_GPIO_Port GPIOA
#define LM74700_STAT_Pin PIN_9

#define __HAL_TIM_SET_COMPARE(timer, channel, value) fake_hal_set_compare((timer), (channel), (value))

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init);
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint32_t state);
uint32_t HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin);
void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin);
uint32_t HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel);
void fake_hal_set_compare(TIM_HandleTypeDef *timer, uint32_t channel, uint16_t value);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t milliseconds);
void HAL_Init(void);
void fake_hal_reset(void);
void fake_hal_set_input(GPIO_TypeDef *port, uint16_t pin, uint32_t state);
uint32_t fake_hal_get_output(GPIO_TypeDef *port, uint16_t pin);
bool fake_hal_gpio_was_configured(GPIO_TypeDef *port, uint16_t pin, uint32_t mode, uint32_t pull);
uint8_t fake_hal_can_init_count(void);
bool fake_hal_can_init(void);
void fake_hal_set_ina237(float voltage_v, float current_a, bool available);
void fake_hal_set_tmp1075(float temperature_c, bool available);

/* Startup calls are instrumented by target-startup acceptance tests. */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_FDCAN1_Init(void);
void MX_I2C1_Init(void);
void MX_TIM1_Init(void);
void MX_TIM8_Init(void);

#endif

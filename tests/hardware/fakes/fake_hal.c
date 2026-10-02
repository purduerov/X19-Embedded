#include "main.h"
#include "can_interface.h"
#include <string.h>

GPIO_TypeDef fake_gpio_a = {0};
GPIO_TypeDef fake_gpio_b = {1};
GPIO_TypeDef fake_gpio_c = {2};
TIM_HandleTypeDef fake_htim1;
TIM_HandleTypeDef fake_htim8;

static uint16_t s_gpio_levels[3];
static bool s_gpio_configured[3][16];
static uint8_t s_gpio_modes[3][16];
static uint8_t s_gpio_pulls[3][16];
static uint32_t s_tick_ms;
static uint8_t s_can_init_count;
static float s_ina_voltage_v;
static float s_ina_current_a;
static float s_tmp1075_temperature_c;
static bool s_ina_available;
static bool s_tmp1075_available;

void fake_hal_reset(void) {
    memset(s_gpio_levels, 0, sizeof(s_gpio_levels));
    memset(s_gpio_configured, 0, sizeof(s_gpio_configured));
    memset(s_gpio_modes, 0, sizeof(s_gpio_modes));
    memset(s_gpio_pulls, 0, sizeof(s_gpio_pulls));
    memset(&fake_htim1, 0, sizeof(fake_htim1));
    memset(&fake_htim8, 0, sizeof(fake_htim8));
    s_tick_ms = 0;
    s_can_init_count = 0;
    s_ina_voltage_v = 0.0f;
    s_ina_current_a = 0.0f;
    s_tmp1075_temperature_c = 0.0f;
    s_ina_available = false;
    s_tmp1075_available = false;
}

void fake_hal_set_input(GPIO_TypeDef *port, uint16_t pin, uint32_t state) {
    if (port != NULL && port->id < 3U) {
        if (state == GPIO_PIN_SET) {
            s_gpio_levels[port->id] |= pin;
        } else {
            s_gpio_levels[port->id] &= (uint16_t)~pin;
        }
    }
}

uint32_t fake_hal_get_output(GPIO_TypeDef *port, uint16_t pin) {
    return (port != NULL && port->id < 3U && (s_gpio_levels[port->id] & pin) != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}

void HAL_GPIO_Init(GPIO_TypeDef *port, GPIO_InitTypeDef *init) {
    if (port == NULL || init == NULL || port->id >= 3U) {
        return;
    }
    for (uint8_t bit = 0; bit < 16U; bit++) {
        uint16_t pin = (uint16_t)(1U << bit);
        if ((init->Pin & pin) != 0U) {
            s_gpio_configured[port->id][bit] = true;
            s_gpio_modes[port->id][bit] = (uint8_t)init->Mode;
            s_gpio_pulls[port->id][bit] = (uint8_t)init->Pull;
        }
    }
}

bool fake_hal_gpio_was_configured(GPIO_TypeDef *port, uint16_t pin, uint32_t mode, uint32_t pull) {
    if (port == NULL || port->id >= 3U || pin == 0U || (pin & (uint16_t)(pin - 1U)) != 0U) {
        return false;
    }
    uint8_t bit = 0;
    while ((pin & (uint16_t)(1U << bit)) == 0U) {
        bit++;
    }
    return s_gpio_configured[port->id][bit] && s_gpio_modes[port->id][bit] == mode &&
           s_gpio_pulls[port->id][bit] == pull;
}

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, uint32_t state) {
    fake_hal_set_input(port, pin, state);
}

uint32_t HAL_GPIO_ReadPin(GPIO_TypeDef *port, uint16_t pin) {
    return fake_hal_get_output(port, pin);
}

void HAL_GPIO_TogglePin(GPIO_TypeDef *port, uint16_t pin) {
    if (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    }
}

uint32_t HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel) {
    if (timer == NULL || channel >= 4U) {
        return 1U;
    }
    timer->pwm_started_mask |= (uint8_t)(1U << channel);
    return HAL_OK;
}

void fake_hal_set_compare(TIM_HandleTypeDef *timer, uint32_t channel, uint16_t value) {
    if (timer != NULL && channel < 4U) {
        timer->compare[channel] = value;
    }
}

uint32_t HAL_GetTick(void) {
    return s_tick_ms;
}

void HAL_Delay(uint32_t milliseconds) {
    s_tick_ms += milliseconds;
}

bool can_init(void) {
    s_can_init_count++;
    return fake_hal_can_init();
}

bool fake_hal_can_init(void) {
    return true;
}

uint8_t fake_hal_can_init_count(void) {
    return s_can_init_count;
}

void fake_hal_set_ina237(float voltage_v, float current_a, bool available) {
    s_ina_voltage_v = voltage_v;
    s_ina_current_a = current_a;
    s_ina_available = available;
}

void fake_hal_set_tmp1075(float temperature_c, bool available) {
    s_tmp1075_temperature_c = temperature_c;
    s_tmp1075_available = available;
}

bool mock_sensors_get_ina226(float *voltage_v, float *current_a) {
    if (!s_ina_available || voltage_v == NULL || current_a == NULL) {
        return false;
    }
    *voltage_v = s_ina_voltage_v;
    *current_a = s_ina_current_a;
    return true;
}

bool mock_sensors_get_tmp1075(float *temperature_c) {
    if (!s_tmp1075_available || temperature_c == NULL) {
        return false;
    }
    *temperature_c = s_tmp1075_temperature_c;
    return true;
}

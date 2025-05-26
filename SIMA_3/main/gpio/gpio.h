#ifndef GPIO_H
#define GPIO_H

#include "driver/gpio.h"

//#define DEBUG_GPIO
#define HIGH_DEBUG_GPIO_LEVEL 1
#define LOW_DEBUG_GPIO_LEVEL 0

#if defined(DEBUG_GPIO)
    #include "esp_log.h"

    #define GPIO_TAG "GPIO"
#endif

#if !defined(DEBUG_GPIO_LEVEL)
    #define DEBUG_GPIO_LEVEL HIGH_DEBUG_GPIO_LEVEL
#endif

/***
 * @brief Function used to initialize GPIO pin
 * 
 * @param mode      GPIO mode in which pin should run
 * @param intr      GPIO interrupt for pin
 * @param pin       GPIO pin number on device
 * @param pulldown  Pull-down enable/disable
 * @param pullup    Pull-up enable/disable
 * 
 * @retval None
 */
void init_gpio(gpio_mode_t mode, gpio_int_type_t intr, gpio_num_t pin, gpio_pulldown_t pulldown, gpio_pullup_t pullup);

/***
 * @brief Function used to set pin (set it to HIGH)
 * 
 * @param pin GPIO pin number
 * 
 * @retval None
 */
void gpio_set(gpio_num_t pin);

/***
 * @brief Function used to reset pin (set it to LOW)
 * 
 * @param pin GPIO pin number
 * 
 * @retval None
 */
void gpio_reset(gpio_num_t pin);

#endif // GPIO_H
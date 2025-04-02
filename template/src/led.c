#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_hal_gpio.h"
#include "am_util_delay.h"

// Define available LED pins
uint32_t led_pins[] = {
    AM_BSP_GPIO_LED0,
    AM_BSP_GPIO_LED1,
    AM_BSP_GPIO_LED2
};
#define NUM_LEDS (sizeof(led_pins) / sizeof(led_pins[0]))

void configure_leds(void)
{
    for (int i = 0; i < NUM_LEDS; i++)
    {
        am_hal_gpio_pinconfig(led_pins[i], am_hal_gpio_pincfg_output);
        am_hal_gpio_output_clear(led_pins[i]); // Turn off initially
        am_hal_gpio_output_toggle(led_pins[0]);
        am_hal_gpio_output_toggle(led_pins[2]);
    }
}

void toggle_leds(void)
{
    for (int i = 0; i < NUM_LEDS; i++)
    {
        am_hal_gpio_output_toggle(led_pins[i]);
    }
}

int main(void)
{
    configure_leds(); // Initialize LEDs

    while (1)
    {
        toggle_leds(); // Toggle all LEDs
        am_util_delay_ms(500); // Delay for blinking
    }
}

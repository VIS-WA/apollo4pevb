#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include <string.h>

#include "uart.h"



// Main
int main(void)
{
    // LEDs for testing
    uint32_t led_pin = AM_BSP_GPIO_LED0;
    uint32_t led_pin1 = AM_BSP_GPIO_LED1;
    am_hal_gpio_pinconfig(led_pin, am_hal_gpio_pincfg_output);
    am_hal_gpio_pinconfig(led_pin1, am_hal_gpio_pincfg_output);
    am_hal_gpio_output_clear(led_pin);
    am_hal_gpio_output_clear(led_pin1);
    am_hal_gpio_output_toggle(led_pin1);

    // Low power default
    am_hal_pwrctrl_low_power_init();

    
    // am_bsp_itm_printf_enable();
    uart_init();
    am_util_stdio_printf("Reading ADC every 4s:\n");

    while (1)
    {
        am_hal_gpio_output_toggle(led_pin);
        am_hal_gpio_output_toggle(led_pin1);
        am_util_stdio_printf("Reading ADC...\n");
        am_util_delay_ms(2000);
    }
}

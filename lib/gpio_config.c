#include "gpio_config.h"

LOG_MODULE_REGISTER(GPIO_CONFIG, LOG_LEVEL_DBG);

int gpio_init(void)
{
    int ret;

    ret = gpio_pin_configure_dt(&lora_reset_pin, (GPIO_OUTPUT));

    if (ret)
    {
        LOG_ERR("Unable to configure %d pin", (uint8_t)(*(&lora_reset_pin.pin)));
        return -1;
    }

    // ret = gpio_pin_configure_dt(&busy_pin, (GPIO_INPUT | GPIO_PULL_UP));
    // if (ret)
    // {
    //     LOG_ERR("Unable to configure %d pin", (uint8_t)(*(&busy_pin.pin)));
    //     return -1;
    // }

    ret = gpio_pin_interrupt_configure_dt(&busy_pin, (GPIO_INPUT | GPIO_PULL_DOWN | GPIO_INT_EDGE_BOTH) != 0);
    if (ret)
    {
        printk("Error: failed to configure interrupt on %s pin %d\n", busy_pin.port->name, busy_pin.pin);
        return;
    }

    ret = gpio_pin_configure_dt(&tx_en_pin, (GPIO_OUTPUT_INACTIVE));
    if (ret)
    {
        LOG_ERR("Unable to configure %d pin", (uint8_t)(*(&tx_en_pin.pin)));
        return -1;
    }

    ret = gpio_pin_configure_dt(&rx_en_pin, (GPIO_OUTPUT_INACTIVE));
    if (ret)
    {
        LOG_ERR("Unable to configure %d pin", (uint8_t)(*(&rx_en_pin.pin)));
        return -1;
    }

    LOG_DBG("GPIOS configured successfully!");

    return 0;
}
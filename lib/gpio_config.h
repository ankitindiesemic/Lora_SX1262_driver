#ifndef __GPIO_CONFIG_H__
#define __GPIO_CONFIG_H__

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#define LOW 0
#define HIGH 1


static const struct gpio_dt_spec lora_reset_pin = GPIO_DT_SPEC_GET(DT_ALIAS(resetpin),gpios);
static const struct gpio_dt_spec busy_pin = GPIO_DT_SPEC_GET(DT_ALIAS(busypin), gpios);
static const struct gpio_dt_spec tx_en_pin = GPIO_DT_SPEC_GET(DT_ALIAS(tx_en_pin), gpios);
static const struct gpio_dt_spec rx_en_pin = GPIO_DT_SPEC_GET(DT_ALIAS(rx_en_pin), gpios);

int gpio_init(void);

#endif /* __GPIO_CONFIG_H__ */
/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_TPS65_H_
#define ZEPHYR_DRIVERS_SENSOR_TPS65_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

/* TPS65 with IQS550 controller register definitions */
#define TPS65_REG_DEVICE_INFO       0x00
#define TPS65_REG_SYS_INFO_0        0x01
#define TPS65_REG_SYS_INFO_1        0x02
#define TPS65_REG_VERSION_INFO      0x03
#define TPS65_REG_XY_INFO_0         0x10
#define TPS65_REG_XY_INFO_1         0x11
#define TPS65_REG_TOUCH_STRENGTH    0x12
#define TPS65_REG_TOUCH_AREA        0x13
#define TPS65_REG_COORDINATES_X     0x14
#define TPS65_REG_COORDINATES_Y     0x16
#define TPS65_REG_PROX_STATUS       0x20
#define TPS65_REG_TOUCH_STATUS      0x21
#define TPS65_REG_COUNTS            0x22
#define TPS65_REG_LTA               0x23
#define TPS65_REG_DELTAS            0x24
#define TPS65_REG_MULTIPLIERS       0x25
#define TPS65_REG_COMPENSATION      0x26
#define TPS65_REG_PROX_SETTINGS     0x40
#define TPS65_REG_THRESHOLDS        0x41
#define TPS65_REG_TIMINGS_0         0x42
#define TPS65_REG_TIMINGS_1         0x43
#define TPS65_REG_GESTURE_TIMERS    0x44
#define TPS65_REG_ACTIVE_CHANNELS   0x45

/* IQS550 specific values */
#define TPS65_MAX_X                 3072
#define TPS65_MAX_Y                 2048
#define TPS65_MAX_TOUCH_POINTS      5

/* Device state flags */
#define TPS65_SHOW_RESET            BIT(7)
#define TPS65_TOUCH_EVENT           BIT(0)
#define TPS65_PROX_EVENT            BIT(1)

struct tps65_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec ready_gpio;
	struct gpio_dt_spec reset_gpio;
	uint16_t max_x;
	uint16_t max_y;
    uint8_t max_touch_points;
};

struct tps65_data {
	const struct device *dev;
	struct k_work work;
	struct k_timer poll_timer;
	struct k_sem sem;
	bool ready;

	/* For ready GPIO callback */
	struct gpio_callback ready_cb;

	/* Touch data */
	uint8_t num_touches;
	uint16_t x[TPS65_MAX_TOUCH_POINTS];
	uint16_t y[TPS65_MAX_TOUCH_POINTS];
	uint8_t touch_strength[TPS65_MAX_TOUCH_POINTS];
	uint8_t touch_area[TPS65_MAX_TOUCH_POINTS];
	uint8_t gesture;
};

int tps65_init(const struct device *dev);
int tps65_sample_fetch(const struct device *dev, enum sensor_channel chan);
int tps65_channel_get(const struct device *dev, enum sensor_channel chan,
		      struct sensor_value *val);

#endif /* ZEPHYR_DRIVERS_SENSOR_TPS65_H_ */ 
/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_TPS43_H_
#define ZEPHYR_DRIVERS_SENSOR_TPS43_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

/* TPS43 with IQS572 controller register definitions */
#define TPS43_REG_DEVICE_INFO       0x00
#define TPS43_REG_SYS_INFO_0        0x01
#define TPS43_REG_SYS_INFO_1        0x02
#define TPS43_REG_VERSION_INFO      0x03
#define TPS43_REG_XY_INFO_0         0x10
#define TPS43_REG_XY_INFO_1         0x11
#define TPS43_REG_TOUCH_STRENGTH    0x12
#define TPS43_REG_TOUCH_AREA        0x13
#define TPS43_REG_COORDINATES_X     0x14
#define TPS43_REG_COORDINATES_Y     0x16
#define TPS43_REG_PROX_STATUS       0x20
#define TPS43_REG_TOUCH_STATUS      0x21
#define TPS43_REG_COUNTS            0x22
#define TPS43_REG_LTA               0x23
#define TPS43_REG_DELTAS            0x24
#define TPS43_REG_MULTIPLIERS       0x25
#define TPS43_REG_COMPENSATION      0x26
#define TPS43_REG_PROX_SETTINGS     0x40
#define TPS43_REG_THRESHOLDS        0x41
#define TPS43_REG_TIMINGS_0         0x42
#define TPS43_REG_TIMINGS_1         0x43
#define TPS43_REG_GESTURE_TIMERS    0x44
#define TPS43_REG_ACTIVE_CHANNELS   0x45

/* IQS572 specific values for TPS43 */
#define TPS43_MAX_X                 2048
#define TPS43_MAX_Y                 1792
#define TPS43_MAX_TOUCH_POINTS      5

/* Device state flags */
#define TPS43_SHOW_RESET            BIT(7)
#define TPS43_TOUCH_EVENT           BIT(0)
#define TPS43_PROX_EVENT            BIT(1)

struct tps43_config {
	struct i2c_dt_spec i2c;
	struct gpio_dt_spec ready_gpio;
	struct gpio_dt_spec reset_gpio;
	uint16_t max_x;
	uint16_t max_y;
};

struct tps43_data {
	const struct device *dev;
	struct k_work work;
	struct k_timer poll_timer;
	struct k_sem sem;
	bool ready;

	/* For ready GPIO callback */
	struct gpio_callback ready_cb;

	/* Touch data */
	uint8_t num_touches;
	uint16_t x[TPS43_MAX_TOUCH_POINTS];
	uint16_t y[TPS43_MAX_TOUCH_POINTS];
	uint8_t touch_strength[TPS43_MAX_TOUCH_POINTS];
	uint8_t touch_area[TPS43_MAX_TOUCH_POINTS];
	uint8_t gesture;
};

int tps43_init(const struct device *dev);
int tps43_sample_fetch(const struct device *dev, enum sensor_channel chan);
int tps43_channel_get(const struct device *dev, enum sensor_channel chan,
		      struct sensor_value *val);

#endif /* ZEPHYR_DRIVERS_SENSOR_TPS43_H_ */ 
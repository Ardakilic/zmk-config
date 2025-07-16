/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT azoteq_tps65

#include "tps65.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(tps65, CONFIG_SENSOR_LOG_LEVEL);

#if CONFIG_ZMK_SENSOR_TPS65_GESTURE_SUPPORT
// Gesture-related code would go here
#endif

static void tps65_poll_handler(struct k_timer *timer)
{
    struct tps65_data *data = CONTAINER_OF(timer, struct tps65_data, poll_timer);
    k_work_submit(&data->work);
}

static void tps65_work_handler(struct k_work *work)
{
	struct tps65_data *data = CONTAINER_OF(work, struct tps65_data, work);
	const struct device *dev = data->dev;
	const struct tps65_config *config = dev->config;
	uint8_t status;
	int ret;

	/* Read status register */
	ret = i2c_reg_read_byte_dt(&config->i2c, TPS65_REG_TOUCH_STATUS, &status);
	if (ret < 0) {
		LOG_ERR("Failed to read status register: %d", ret);
		return;
	}

	if (status & TPS65_TOUCH_EVENT) {
		/* Read touch data */
		uint8_t xy_info[2];
		ret = i2c_burst_read_dt(&config->i2c, TPS65_REG_XY_INFO_0, xy_info, sizeof(xy_info));
		if (ret < 0) {
			LOG_ERR("Failed to read XY info: %d", ret);
			return;
		}

		data->num_touches = xy_info[0] & 0x0F;
		if (data->num_touches > CONFIG_ZMK_SENSOR_TPS65_MAX_TOUCHES) {
			data->num_touches = CONFIG_ZMK_SENSOR_TPS65_MAX_TOUCHES;
		}

		/* Read coordinate data for each touch */
		for (int i = 0; i < data->num_touches; i++) {
			uint8_t coord_data[6];
			ret = i2c_burst_read_dt(&config->i2c, 
				TPS65_REG_COORDINATES_X + (i * 6), 
				coord_data, sizeof(coord_data));
			if (ret < 0) {
				LOG_ERR("Failed to read coordinates for touch %d: %d", i, ret);
				continue;
			}

			/* Parse coordinates (little endian) */
			data->x[i] = sys_get_le16(&coord_data[0]);
			data->y[i] = sys_get_le16(&coord_data[2]);
			data->touch_strength[i] = coord_data[4];
			data->touch_area[i] = coord_data[5];

			/* Clamp to maximum values */
			if (data->x[i] > config->max_x) data->x[i] = config->max_x;
			if (data->y[i] > config->max_y) data->y[i] = config->max_y;
		}

		LOG_DBG("Touch event: %d touches", data->num_touches);
		for (int i = 0; i < data->num_touches; i++) {
			LOG_DBG("Touch %d: (%d, %d) strength=%d area=%d", 
				i, data->x[i], data->y[i], 
				data->touch_strength[i], data->touch_area[i]);
		}
	}

	k_sem_give(&data->sem);
}

static void tps65_ready_callback(const struct device *gpio_dev,
				 struct gpio_callback *cb, uint32_t pins)
{
	struct tps65_data *data = CONTAINER_OF(cb, struct tps65_data, ready_cb);

	k_work_submit(&data->work);
}

static int tps65_reset_device(const struct device *dev)
{
	const struct tps65_config *config = dev->config;
	int ret;

	if (!gpio_is_ready_dt(&config->reset_gpio)) {
		LOG_ERR("Reset GPIO not ready");
		return -ENODEV;
	}

	/* Reset sequence: Low -> High */
	ret = gpio_pin_set_dt(&config->reset_gpio, 0);
	if (ret < 0) {
		return ret;
	}

	k_msleep(10);

	ret = gpio_pin_set_dt(&config->reset_gpio, 1);
	if (ret < 0) {
		return ret;
	}

	k_msleep(100); /* Wait for reset to complete */

	return 0;
}

static int tps65_configure_device(const struct device *dev)
{
	const struct tps65_config *config = dev->config;
	int ret;

	/* Basic configuration for IQS550 */
	uint8_t prox_settings[] = {0x01, 0x00}; /* Enable proximity */
	ret = i2c_burst_write_dt(&config->i2c, TPS65_REG_PROX_SETTINGS, 
				 prox_settings, sizeof(prox_settings));
	if (ret < 0) {
		LOG_ERR("Failed to configure proximity settings: %d", ret);
		return ret;
	}

	/* Configure touch thresholds */
	uint8_t thresholds[] = {CONFIG_ZMK_SENSOR_TPS65_SENSITIVITY, 0x08}; /* Touch threshold, proximity threshold */
	ret = i2c_burst_write_dt(&config->i2c, TPS65_REG_THRESHOLDS, 
				 thresholds, sizeof(thresholds));
	if (ret < 0) {
		LOG_ERR("Failed to configure thresholds: %d", ret);
		return ret;
	}

	return 0;
}

int tps65_init(const struct device *dev)
{
	const struct tps65_config *config = dev->config;
	struct tps65_data *data = dev->data;
	int ret;

	LOG_INF("Initializing TPS65 trackpad");

	data->dev = dev;
	k_sem_init(&data->sem, 0, 1);
	k_work_init(&data->work, tps65_work_handler);
    k_timer_init(&data->poll_timer, tps65_poll_handler, NULL);

	/* Check I2C bus */
	if (!i2c_is_ready_dt(&config->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Reset device */
	ret = tps65_reset_device(dev);
	if (ret < 0) {
		LOG_ERR("Failed to reset device: %d", ret);
		return ret;
	}

	/* Verify device presence */
	uint8_t device_info;
	ret = i2c_reg_read_byte_dt(&config->i2c, TPS65_REG_DEVICE_INFO, &device_info);
	if (ret < 0) {
		LOG_ERR("Failed to read device info: %d", ret);
		return ret;
	}

	LOG_INF("TPS65 device info: 0x%02x", device_info);

	/* Configure device */
	ret = tps65_configure_device(dev);
	if (ret < 0) {
		LOG_ERR("Failed to configure device: %d", ret);
		return ret;
	}

	/* Setup ready GPIO interrupt */
	if (gpio_is_ready_dt(&config->ready_gpio)) {
		ret = gpio_pin_configure_dt(&config->ready_gpio, GPIO_INPUT);
		if (ret < 0) {
			LOG_ERR("Failed to configure ready GPIO: %d", ret);
			return ret;
		}

		ret = gpio_pin_interrupt_configure_dt(&config->ready_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Failed to configure ready interrupt: %d", ret);
			return ret;
		}

		gpio_init_callback(&data->ready_cb, tps65_ready_callback, 
				   BIT(config->ready_gpio.pin));
		
		ret = gpio_add_callback(config->ready_gpio.port, &data->ready_cb);
		if (ret < 0) {
			LOG_ERR("Failed to add ready callback: %d", ret);
			return ret;
		}
	} else {
        k_timer_start(&data->poll_timer, K_MSEC(CONFIG_ZMK_SENSOR_TPS65_POLL_RATE_MS), K_MSEC(CONFIG_ZMK_SENSOR_TPS65_POLL_RATE_MS));
    }

	LOG_INF("TPS65 initialized successfully");
	return 0;
}

int tps65_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct tps65_data *data = dev->data;

	if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_POS_XY) {
		return -ENOTSUP;
	}

	return k_sem_take(&data->sem, K_MSEC(CONFIG_ZMK_SENSOR_TPS65_POLL_RATE_MS));
}

int tps65_channel_get(const struct device *dev, enum sensor_channel chan,
		      struct sensor_value *val)
{
	struct tps65_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_POS_X:
		if (data->num_touches > 0) {
			val->val1 = data->x[0];
			val->val2 = 0;
		} else {
			val->val1 = 0;
			val->val2 = 0;
		}
		break;

	case SENSOR_CHAN_POS_Y:
		if (data->num_touches > 0) {
			val->val1 = data->y[0];
			val->val2 = 0;
		} else {
			val->val1 = 0;
			val->val2 = 0;
		}
		break;

	case SENSOR_CHAN_PRESS:
		val->val1 = data->num_touches;
		val->val2 = 0;
		break;

	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api tps65_driver_api = {
	.sample_fetch = tps65_sample_fetch,
	.channel_get = tps65_channel_get,
};

#define TPS65_INIT(inst)						\
	static struct tps65_data tps65_data_##inst;			\
									\
	static const struct tps65_config tps65_config_##inst = {	\
		.i2c = I2C_DT_SPEC_INST_GET(inst),			\
		.ready_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, rdy_gpios, {}), \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, rst_gpios, {}), \
		.max_x = DT_INST_PROP_OR(inst, max_x, TPS65_MAX_X),	\
		.max_y = DT_INST_PROP_OR(inst, max_y, TPS65_MAX_Y),	\
		.max_touch_points = DT_INST_PROP_OR(inst, max_touch_points, TPS65_MAX_TOUCH_POINTS), \
	};								\
									\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, tps65_init, NULL,		\
				      &tps65_data_##inst,		\
				      &tps65_config_##inst,		\
				      POST_KERNEL,			\
				      CONFIG_SENSOR_INIT_PRIORITY,	\
				      &tps65_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TPS65_INIT) 
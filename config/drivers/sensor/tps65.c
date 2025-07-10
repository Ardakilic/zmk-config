/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT azoteq_tps65

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "tps65.h"

LOG_MODULE_REGISTER(tps65, CONFIG_SENSOR_LOG_LEVEL);

/* I2C Communication Functions */
int tps65_read_reg(const struct device *dev, uint16_t reg, uint8_t *data, size_t len)
{
    const struct tps65_config *config = dev->config;
    uint8_t reg_buf[2];
    int ret;

    /* IQS5xx uses 16-bit addressing, MSB first */
    reg_buf[0] = (reg >> 8) & 0xFF;
    reg_buf[1] = reg & 0xFF;

    ret = i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), data, len);
    if (ret < 0) {
        LOG_ERR("Failed to read register 0x%04X: %d", reg, ret);
        return ret;
    }

    return 0;
}

int tps65_write_reg(const struct device *dev, uint16_t reg, uint8_t *data, size_t len)
{
    const struct tps65_config *config = dev->config;
    uint8_t buf[len + 2];
    int ret;

    /* IQS5xx uses 16-bit addressing, MSB first */
    buf[0] = (reg >> 8) & 0xFF;
    buf[1] = reg & 0xFF;
    memcpy(&buf[2], data, len);

    ret = i2c_write_dt(&config->i2c, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to write register 0x%04X: %d", reg, ret);
        return ret;
    }

    return 0;
}

int tps65_write_reg_8(const struct device *dev, uint16_t reg, uint8_t value)
{
    return tps65_write_reg(dev, reg, &value, 1);
}

int tps65_write_reg_16(const struct device *dev, uint16_t reg, uint16_t value)
{
    uint8_t data[2];
    
    /* IQS5xx uses little-endian 16-bit values */
    data[0] = value & 0xFF;
    data[1] = (value >> 8) & 0xFF;
    
    return tps65_write_reg(dev, reg, data, 2);
}

int tps65_end_communication(const struct device *dev)
{
    const struct tps65_config *config = dev->config;
    uint8_t end_cmd[2] = {0xEE, 0xEE};
    int ret;

    ret = i2c_write_dt(&config->i2c, end_cmd, sizeof(end_cmd));
    if (ret < 0) {
        LOG_ERR("Failed to end communication: %d", ret);
        return ret;
    }

    return 0;
}

/* Device Management Functions */
int tps65_reset_device(const struct device *dev)
{
    const struct tps65_config *config = dev->config;
    struct tps65_data *data = dev->data;
    int ret;

    /* Hardware reset if RST GPIO is configured */
    if (config->rst_gpio.port != NULL) {
        ret = gpio_pin_set_dt(&config->rst_gpio, 0);
        if (ret < 0) {
            LOG_ERR("Failed to assert reset: %d", ret);
            return ret;
        }
        
        k_msleep(10);
        
        ret = gpio_pin_set_dt(&config->rst_gpio, 1);
        if (ret < 0) {
            LOG_ERR("Failed to deassert reset: %d", ret);
            return ret;
        }
        
        k_msleep(100);
    } else {
        /* Software reset */
        ret = tps65_write_reg_8(dev, TPS65_REG_SYSTEM_CONTROL_1, TPS65_SYS_CTRL_RESET);
        if (ret < 0) {
            return ret;
        }
        
        k_msleep(100);
        
        /* Acknowledge reset */
        ret = tps65_write_reg_8(dev, TPS65_REG_SYSTEM_CONTROL_1, TPS65_SYS_CTRL_ACK_RESET);
        if (ret < 0) {
            return ret;
        }
    }

    data->device_ready = false;
    data->ati_complete = false;
    
    return tps65_end_communication(dev);
}

int tps65_run_ati(const struct device *dev)
{
    struct tps65_data *data = dev->data;
    uint8_t sys_info;
    int ret;
    int retry_count = 0;

    LOG_INF("Running ATI (Automatic Tuning Implementation)");

    /* Start ATI */
    ret = tps65_write_reg_8(dev, TPS65_REG_SYSTEM_CONTROL_1, TPS65_SYS_CTRL_AUTO_ATI);
    if (ret < 0) {
        return ret;
    }

    ret = tps65_end_communication(dev);
    if (ret < 0) {
        return ret;
    }

    /* Wait for ATI completion (up to 2 seconds) */
    while (retry_count < 200) {
        k_msleep(10);
        
        ret = tps65_read_reg(dev, TPS65_REG_SYSTEM_INFO_1, &sys_info, 1);
        if (ret < 0) {
            return ret;
        }
        
        ret = tps65_end_communication(dev);
        if (ret < 0) {
            return ret;
        }
        
        /* Check if ATI is complete */
        if (!(sys_info & BIT(2))) {  /* ATI_BUSY bit cleared */
            data->ati_complete = true;
            LOG_INF("ATI completed successfully");
            return 0;
        }
        
        retry_count++;
    }

    LOG_ERR("ATI timeout");
    return -ETIMEDOUT;
}

int tps65_init_device(const struct device *dev)
{
    const struct tps65_config *config = dev->config;
    struct tps65_data *data = dev->data;
    uint8_t product_num[2];
    uint8_t sys_config;
    int ret;

    LOG_INF("Initializing TPS65 trackpad (%s variant)", config->variant);

    /* Reset device */
    ret = tps65_reset_device(dev);
    if (ret < 0) {
        return ret;
    }

    /* Read product number to verify communication */
    ret = tps65_read_reg(dev, TPS65_REG_PRODUCT_NUM_L, product_num, 2);
    if (ret < 0) {
        LOG_ERR("Failed to read product number");
        return ret;
    }
    
    ret = tps65_end_communication(dev);
    if (ret < 0) {
        return ret;
    }

    uint16_t product = sys_get_le16(product_num);
    LOG_INF("Product number: 0x%04X", product);

    /* Configure resolution */
    ret = tps65_write_reg_16(dev, TPS65_REG_X_RESOLUTION_L, config->resolution_x);
    if (ret < 0) {
        return ret;
    }

    ret = tps65_write_reg_16(dev, TPS65_REG_Y_RESOLUTION_L, config->resolution_y);
    if (ret < 0) {
        return ret;
    }

    /* Configure coordinate transformations */
    uint8_t xy_config = 0;
    if (config->invert_x) {
        xy_config |= TPS65_XY_CFG_FLIP_X;
    }
    if (config->invert_y) {
        xy_config |= TPS65_XY_CFG_FLIP_Y;
    }
    if (config->swap_axes) {
        xy_config |= TPS65_XY_CFG_SWITCH_XY;
    }
    
    ret = tps65_write_reg_8(dev, TPS65_REG_XY_CONFIG_0, xy_config);
    if (ret < 0) {
        return ret;
    }

    /* Configure maximum touches */
    ret = tps65_write_reg_8(dev, TPS65_REG_MAX_MULTITOUCHES, config->max_touches);
    if (ret < 0) {
        return ret;
    }

    /* Configure touch threshold */
    ret = tps65_write_reg_8(dev, TPS65_REG_GLOBAL_TOUCH_MULT_SET, config->touch_threshold);
    if (ret < 0) {
        return ret;
    }

    /* Configure gestures */
    if (config->gesture_enable) {
        /* Enable single finger gestures */
        ret = tps65_write_reg_8(dev, TPS65_REG_SINGLE_FINGER_GESTURES, 0x3F);
        if (ret < 0) {
            return ret;
        }
        
        /* Enable multi finger gestures */
        ret = tps65_write_reg_8(dev, TPS65_REG_MULTI_FINGER_GESTURES, 0x07);
        if (ret < 0) {
            return ret;
        }
    } else {
        /* Disable gestures */
        ret = tps65_write_reg_8(dev, TPS65_REG_SINGLE_FINGER_GESTURES, 0x00);
        if (ret < 0) {
            return ret;
        }
        
        ret = tps65_write_reg_8(dev, TPS65_REG_MULTI_FINGER_GESTURES, 0x00);
        if (ret < 0) {
            return ret;
        }
    }

    /* Setup coordinate scaling for variant */
    if (strcmp(config->variant, "43mm") == 0) {
        /* 43mm variant: 43x40mm physical size */
        data->scale_x = (43 * 100) / config->resolution_x;  /* Scale to 0.01mm units */
        data->scale_y = (40 * 100) / config->resolution_y;
        data->offset_x = 0;
        data->offset_y = 0;
    } else {
        /* 65mm variant: 65x49mm physical size */
        data->scale_x = (65 * 100) / config->resolution_x;
        data->scale_y = (49 * 100) / config->resolution_y;
        data->offset_x = 0;
        data->offset_y = 0;
    }

    /* Enable event mode and complete setup */
    ret = tps65_read_reg(dev, TPS65_REG_SYSTEM_CONFIG_0, &sys_config, 1);
    if (ret < 0) {
        return ret;
    }
    
    sys_config |= TPS65_SYS_CFG_EVENT_MODE | TPS65_SYS_CFG_SETUP_COMPLETE;
    
    ret = tps65_write_reg_8(dev, TPS65_REG_SYSTEM_CONFIG_0, sys_config);
    if (ret < 0) {
        return ret;
    }

    ret = tps65_end_communication(dev);
    if (ret < 0) {
        return ret;
    }

    /* Run ATI */
    ret = tps65_run_ati(dev);
    if (ret < 0) {
        return ret;
    }

    data->device_ready = true;
    LOG_INF("TPS65 initialization complete");

    return 0;
}

/* Data Processing Functions */
void tps65_process_coordinates(const struct device *dev, struct tps65_touch_point *point)
{
    const struct tps65_config *config = dev->config;
    struct tps65_data *data = dev->data;

    /* Apply coordinate transformations if needed */
    if (config->invert_x) {
        point->x = config->resolution_x - point->x;
    }
    if (config->invert_y) {
        point->y = config->resolution_y - point->y;
    }
    if (config->swap_axes) {
        uint16_t temp = point->x;
        point->x = point->y;
        point->y = temp;
    }

    /* Apply scaling for physical coordinates */
    point->x = (point->x * data->scale_x) / 100 + data->offset_x;
    point->y = (point->y * data->scale_y) / 100 + data->offset_y;
}

int tps65_read_touch_data(const struct device *dev)
{
    struct tps65_data *data = dev->data;
    uint8_t num_fingers;
    uint8_t touch_data[TPS65_FINGER_DATA_SIZE];
    int ret;

    /* Read number of active fingers */
    ret = tps65_read_reg(dev, TPS65_REG_NUM_FINGERS, &num_fingers, 1);
    if (ret < 0) {
        return ret;
    }

    /* Limit to maximum supported touches */
    if (num_fingers > TPS65_MAX_TOUCHES) {
        num_fingers = TPS65_MAX_TOUCHES;
    }

    data->num_touches = num_fingers;

    /* Clear all touch points */
    memset(data->touches, 0, sizeof(data->touches));

    /* Read touch data for each finger */
    for (int i = 0; i < num_fingers; i++) {
        uint16_t reg_addr = TPS65_REG_ABS_X_L + (i * TPS65_FINGER_DATA_SIZE);
        
        ret = tps65_read_reg(dev, reg_addr, touch_data, TPS65_FINGER_DATA_SIZE);
        if (ret < 0) {
            return ret;
        }

        /* Parse touch data */
        data->touches[i].x = sys_get_le16(&touch_data[0]);
        data->touches[i].y = sys_get_le16(&touch_data[2]);
        data->touches[i].strength = sys_get_le16(&touch_data[4]);
        data->touches[i].area = touch_data[6];
        data->touches[i].active = true;

        /* Process coordinates */
        tps65_process_coordinates(dev, &data->touches[i]);
    }

    return tps65_end_communication(dev);
}

int tps65_read_gesture_data(const struct device *dev)
{
    struct tps65_data *data = dev->data;
    uint8_t gesture_buf[4];
    int ret;

    /* Read gesture events and relative coordinates */
    ret = tps65_read_reg(dev, TPS65_REG_GESTURE_EVENTS_0, gesture_buf, 4);
    if (ret < 0) {
        return ret;
    }

    data->gesture.events_0 = gesture_buf[0];
    data->gesture.events_1 = gesture_buf[1];
    
    /* Read relative coordinates */
    ret = tps65_read_reg(dev, TPS65_REG_REL_X_L, gesture_buf, 4);
    if (ret < 0) {
        return ret;
    }

    data->gesture.rel_x = sys_get_le16(&gesture_buf[0]);
    data->gesture.rel_y = sys_get_le16(&gesture_buf[2]);

    return tps65_end_communication(dev);
}

/* Interrupt and Work Functions */
static void tps65_work_handler(struct k_work *work)
{
    struct tps65_data *data = CONTAINER_OF(work, struct tps65_data, work);
    const struct device *dev = DEVICE_DT_GET(DT_DRV_INST(0));

    if (!data->device_ready) {
        return;
    }

    /* Read touch and gesture data */
    int ret = tps65_read_touch_data(dev);
    if (ret < 0) {
        LOG_ERR("Failed to read touch data: %d", ret);
        return;
    }

    ret = tps65_read_gesture_data(dev);
    if (ret < 0) {
        LOG_ERR("Failed to read gesture data: %d", ret);
        return;
    }

    /* Log touch events for debugging */
    if (data->num_touches > 0) {
        LOG_DBG("Touches: %d", data->num_touches);
        for (int i = 0; i < data->num_touches; i++) {
            LOG_DBG("  [%d]: (%d,%d) strength=%d area=%d", i,
                    data->touches[i].x, data->touches[i].y,
                    data->touches[i].strength, data->touches[i].area);
        }
    }

    /* Log gesture events */
    if (data->gesture.events_0 || data->gesture.events_1) {
        LOG_DBG("Gestures: 0x%02X 0x%02X, rel=(%d,%d)",
                data->gesture.events_0, data->gesture.events_1,
                data->gesture.rel_x, data->gesture.rel_y);
    }
}

static void tps65_rdy_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct tps65_data *data = CONTAINER_OF(cb, struct tps65_data, rdy_cb);
    
    /* Schedule work to read data */
    k_work_submit(&data->work);
}

static void tps65_poll_timer_handler(struct k_timer *timer)
{
    struct tps65_data *data = CONTAINER_OF(timer, struct tps65_data, poll_timer);
    
    /* Schedule work to read data */
    k_work_submit(&data->work);
}

/* ZMK Sensor API Implementation */
static int tps65_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct tps65_data *data = dev->data;
    int ret;

    if (!data->device_ready) {
        return -ENODEV;
    }

    if (chan != SENSOR_CHAN_ALL && chan != SENSOR_CHAN_POS_XY) {
        return -ENOTSUP;
    }

    /* Read current touch and gesture data */
    ret = tps65_read_touch_data(dev);
    if (ret < 0) {
        return ret;
    }

    return tps65_read_gesture_data(dev);
}

static int tps65_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct tps65_data *data = dev->data;

    if (!data->device_ready) {
        return -ENODEV;
    }

    switch (chan) {
    case SENSOR_CHAN_POS_X:
        if (data->num_touches > 0) {
            val->val1 = data->touches[0].x;
            val->val2 = 0;
        } else {
            val->val1 = 0;
            val->val2 = 0;
        }
        break;

    case SENSOR_CHAN_POS_Y:
        if (data->num_touches > 0) {
            val->val1 = data->touches[0].y;
            val->val2 = 0;
        } else {
            val->val1 = 0;
            val->val2 = 0;
        }
        break;

    case SENSOR_CHAN_POS_XY:
        if (data->num_touches > 0) {
            val[0].val1 = data->touches[0].x;
            val[0].val2 = 0;
            val[1].val1 = data->touches[0].y;
            val[1].val2 = 0;
        } else {
            val[0].val1 = 0;
            val[0].val2 = 0;
            val[1].val1 = 0;
            val[1].val2 = 0;
        }
        break;

    default:
        return -ENOTSUP;
    }

    return 0;
}

static int tps65_attr_set(const struct device *dev, enum sensor_channel chan,
                         enum sensor_attribute attr, const struct sensor_value *val)
{
    /* Attributes like sensitivity could be implemented here */
    return -ENOTSUP;
}

static int tps65_attr_get(const struct device *dev, enum sensor_channel chan,
                         enum sensor_attribute attr, struct sensor_value *val)
{
    /* Attributes like sensitivity could be implemented here */
    return -ENOTSUP;
}

static const struct sensor_driver_api tps65_driver_api = {
    .sample_fetch = tps65_sample_fetch,
    .channel_get = tps65_channel_get,
    .attr_set = tps65_attr_set,
    .attr_get = tps65_attr_get,
};

/* Device Initialization */
static int tps65_init(const struct device *dev)
{
    const struct tps65_config *config = dev->config;
    struct tps65_data *data = dev->data;
    int ret;

    LOG_INF("Initializing TPS65 driver");

    /* Verify I2C bus is ready */
    if (!device_is_ready(config->i2c.bus)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    /* Initialize work queue */
    k_work_init(&data->work, tps65_work_handler);

    /* Configure GPIO pins */
    if (config->rst_gpio.port != NULL) {
        if (!device_is_ready(config->rst_gpio.port)) {
            LOG_ERR("Reset GPIO not ready");
            return -ENODEV;
        }
        
        ret = gpio_pin_configure_dt(&config->rst_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure reset GPIO: %d", ret);
            return ret;
        }
    }

    if (config->rdy_gpio.port != NULL) {
        if (!device_is_ready(config->rdy_gpio.port)) {
            LOG_ERR("Ready GPIO not ready");
            return -ENODEV;
        }
        
        ret = gpio_pin_configure_dt(&config->rdy_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure ready GPIO: %d", ret);
            return ret;
        }

        /* Setup interrupt */
        gpio_init_callback(&data->rdy_cb, tps65_rdy_callback, BIT(config->rdy_gpio.pin));
        ret = gpio_add_callback(config->rdy_gpio.port, &data->rdy_cb);
        if (ret < 0) {
            LOG_ERR("Failed to add GPIO callback: %d", ret);
            return ret;
        }

        ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure GPIO interrupt: %d", ret);
            return ret;
        }

        LOG_INF("Using interrupt-driven mode");
    } else {
        /* Setup polling timer */
        k_timer_init(&data->poll_timer, tps65_poll_timer_handler, NULL);
        k_timer_start(&data->poll_timer, K_MSEC(CONFIG_TPS65_POLL_RATE_MS), 
                      K_MSEC(CONFIG_TPS65_POLL_RATE_MS));
        LOG_INF("Using polling mode (%d ms)", CONFIG_TPS65_POLL_RATE_MS);
    }

    /* Initialize device */
    ret = tps65_init_device(dev);
    if (ret < 0) {
        LOG_ERR("Failed to initialize device: %d", ret);
        return ret;
    }

    LOG_INF("TPS65 driver initialized successfully");
    return 0;
}

/* Device Tree Macros */
#define TPS65_CONFIG(inst)                                                  \
    {                                                                       \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                \
        .rdy_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, rdy_gpios, {0}),       \
        .rst_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, rst_gpios, {0}),       \
        .variant = DT_INST_PROP_OR(inst, variant, "65mm"),                \
        .resolution_x = DT_INST_PROP_OR(inst, resolution_x, 4096),        \
        .resolution_y = DT_INST_PROP_OR(inst, resolution_y, 4096),        \
        .invert_x = DT_INST_PROP_OR(inst, invert_x, false),               \
        .invert_y = DT_INST_PROP_OR(inst, invert_y, false),               \
        .swap_axes = DT_INST_PROP_OR(inst, swap_axes, false),             \
        .touch_threshold = DT_INST_PROP_OR(inst, touch_threshold, 40),    \
        .gesture_enable = DT_INST_PROP_OR(inst, gesture_enable, true),    \
        .max_touches = DT_INST_PROP_OR(inst, max_touches, 5),             \
        .palm_reject_threshold = DT_INST_PROP_OR(inst, palm_reject_threshold, 100), \
        .i2c_timeout_ms = DT_INST_PROP_OR(inst, i2c_timeout_ms, 10),      \
    }

#define TPS65_DEFINE(inst)                                                  \
    static struct tps65_data tps65_data_##inst;                           \
    static const struct tps65_config tps65_config_##inst =                \
        TPS65_CONFIG(inst);                                                \
    DEVICE_DT_INST_DEFINE(inst, tps65_init, NULL,                         \
                          &tps65_data_##inst, &tps65_config_##inst,       \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,        \
                          &tps65_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TPS65_DEFINE) 
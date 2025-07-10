/*
 * Copyright (c) 2025 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_TPS65_H_
#define ZEPHYR_DRIVERS_SENSOR_TPS65_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

/* IQS5xx-B000 Default I2C Address */
#define TPS65_I2C_ADDR 0x74

/* Memory Map Registers */
#define TPS65_REG_PRODUCT_NUM_L         0x0000
#define TPS65_REG_PRODUCT_NUM_H         0x0001
#define TPS65_REG_PROJECT_NUM_L         0x0002
#define TPS65_REG_PROJECT_NUM_H         0x0003
#define TPS65_REG_MAJOR_VERSION         0x0004
#define TPS65_REG_MINOR_VERSION         0x0005
#define TPS65_REG_BOOTLOADER_STATUS     0x0006

/* System Info and Events */
#define TPS65_REG_MAX_TOUCH             0x000B
#define TPS65_REG_CYCLE_TIME            0x000C
#define TPS65_REG_GESTURE_EVENTS_0      0x000D
#define TPS65_REG_GESTURE_EVENTS_1      0x000E
#define TPS65_REG_SYSTEM_INFO_0         0x000F
#define TPS65_REG_SYSTEM_INFO_1         0x0010
#define TPS65_REG_NUM_FINGERS           0x0011

/* Touch Data */
#define TPS65_REG_REL_X_L               0x0012
#define TPS65_REG_REL_X_H               0x0013
#define TPS65_REG_REL_Y_L               0x0014
#define TPS65_REG_REL_Y_H               0x0015

/* Absolute coordinates for finger 1 */
#define TPS65_REG_ABS_X_L               0x0016
#define TPS65_REG_ABS_X_H               0x0017
#define TPS65_REG_ABS_Y_L               0x0018
#define TPS65_REG_ABS_Y_H               0x0019
#define TPS65_REG_TOUCH_STRENGTH_L      0x001A
#define TPS65_REG_TOUCH_STRENGTH_H      0x001B
#define TPS65_REG_TOUCH_AREA            0x001C

/* Additional fingers (2-5) start at 0x001D with same layout */
#define TPS65_FINGER_DATA_SIZE          8

/* Status registers */
#define TPS65_REG_PROX_STATUS_START     0x0039
#define TPS65_REG_TOUCH_STATUS_START    0x0059
#define TPS65_REG_SNAP_STATUS_START     0x0077

/* Control registers */
#define TPS65_REG_SYSTEM_CONTROL_0      0x0431
#define TPS65_REG_SYSTEM_CONTROL_1      0x0432

/* Configuration registers */
#define TPS65_REG_SYSTEM_CONFIG_0       0x058E
#define TPS65_REG_SYSTEM_CONFIG_1       0x058F

/* Trackpad configuration */
#define TPS65_REG_TOTAL_RX              0x063D
#define TPS65_REG_TOTAL_TX              0x063E
#define TPS65_REG_XY_CONFIG_0           0x0669
#define TPS65_REG_MAX_MULTITOUCHES      0x066A
#define TPS65_REG_X_RESOLUTION_L        0x066E
#define TPS65_REG_X_RESOLUTION_H        0x066F
#define TPS65_REG_Y_RESOLUTION_L        0x0670
#define TPS65_REG_Y_RESOLUTION_H        0x0671

/* Gesture configuration */
#define TPS65_REG_SINGLE_FINGER_GESTURES 0x06B7
#define TPS65_REG_MULTI_FINGER_GESTURES  0x06B8

/* ATI and Thresholds */
#define TPS65_REG_ATI_TARGET_L          0x056D
#define TPS65_REG_ATI_TARGET_H          0x056E
#define TPS65_REG_GLOBAL_TOUCH_MULT_SET 0x0596
#define TPS65_REG_GLOBAL_TOUCH_MULT_CLR 0x0597

/* Communication end */
#define TPS65_END_COMM_WINDOW           0xEEEE

/* Gesture event bits */
#define TPS65_GESTURE_SINGLE_TAP        BIT(0)
#define TPS65_GESTURE_PRESS_HOLD        BIT(1)
#define TPS65_GESTURE_SWIPE_X_NEG       BIT(2)
#define TPS65_GESTURE_SWIPE_X_POS       BIT(3)
#define TPS65_GESTURE_SWIPE_Y_POS       BIT(4)
#define TPS65_GESTURE_SWIPE_Y_NEG       BIT(5)

#define TPS65_GESTURE_TWO_FINGER_TAP    BIT(0)
#define TPS65_GESTURE_SCROLL            BIT(1)
#define TPS65_GESTURE_ZOOM              BIT(2)

/* System control bits */
#define TPS65_SYS_CTRL_ACK_RESET        BIT(7)
#define TPS65_SYS_CTRL_AUTO_ATI         BIT(5)
#define TPS65_SYS_CTRL_ALP_RESEED       BIT(4)
#define TPS65_SYS_CTRL_RESEED           BIT(3)

#define TPS65_SYS_CTRL_RESET            BIT(1)
#define TPS65_SYS_CTRL_SUSPEND          BIT(0)

/* System config bits */
#define TPS65_SYS_CFG_MANUAL_CONTROL    BIT(7)
#define TPS65_SYS_CFG_SETUP_COMPLETE    BIT(6)
#define TPS65_SYS_CFG_EVENT_MODE        BIT(0)

/* XY Config bits */
#define TPS65_XY_CFG_PALM_REJECT        BIT(3)
#define TPS65_XY_CFG_SWITCH_XY          BIT(2)
#define TPS65_XY_CFG_FLIP_Y             BIT(1)
#define TPS65_XY_CFG_FLIP_X             BIT(0)

/* Maximum values */
#define TPS65_MAX_TOUCHES               5
#define TPS65_MAX_COORD                 0x7FFF

/* Touch point structure */
struct tps65_touch_point {
    uint16_t x;
    uint16_t y;
    uint16_t strength;
    uint8_t area;
    bool active;
};

/* Gesture data structure */
struct tps65_gesture_data {
    uint8_t events_0;
    uint8_t events_1;
    int16_t rel_x;
    int16_t rel_y;
};

/* Device configuration */
struct tps65_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec rdy_gpio;
    struct gpio_dt_spec rst_gpio;
    
    /* Configuration from device tree */
    const char *variant;
    uint16_t resolution_x;
    uint16_t resolution_y;
    bool invert_x;
    bool invert_y;
    bool swap_axes;
    uint8_t touch_threshold;
    bool gesture_enable;
    uint8_t max_touches;
    uint8_t palm_reject_threshold;
    uint16_t i2c_timeout_ms;
};

/* Device runtime data */
struct tps65_data {
    /* Current touch state */
    struct tps65_touch_point touches[TPS65_MAX_TOUCHES];
    uint8_t num_touches;
    
    /* Gesture state */
    struct tps65_gesture_data gesture;
    
    /* Device status */
    bool device_ready;
    bool ati_complete;
    
    /* Interrupt handling */
    struct gpio_callback rdy_cb;
    struct k_work work;
    
    /* Polling mode */
    struct k_timer poll_timer;
    
    /* Coordinate scaling */
    uint16_t scale_x;
    uint16_t scale_y;
    uint16_t offset_x;
    uint16_t offset_y;
};

/* Function prototypes */
int tps65_read_reg(const struct device *dev, uint16_t reg, uint8_t *data, size_t len);
int tps65_write_reg(const struct device *dev, uint16_t reg, uint8_t *data, size_t len);
int tps65_write_reg_8(const struct device *dev, uint16_t reg, uint8_t value);
int tps65_write_reg_16(const struct device *dev, uint16_t reg, uint16_t value);
int tps65_end_communication(const struct device *dev);

int tps65_init_device(const struct device *dev);
int tps65_reset_device(const struct device *dev);
int tps65_setup_trackpad(const struct device *dev);
int tps65_run_ati(const struct device *dev);

int tps65_read_touch_data(const struct device *dev);
int tps65_read_gesture_data(const struct device *dev);
void tps65_process_coordinates(const struct device *dev, struct tps65_touch_point *point);

/* ZMK sensor API */
int tps65_attr_set(const struct device *dev, enum sensor_channel chan,
                   enum sensor_attribute attr, const struct sensor_value *val);
int tps65_attr_get(const struct device *dev, enum sensor_channel chan,
                   enum sensor_attribute attr, struct sensor_value *val);
int tps65_sample_fetch(const struct device *dev, enum sensor_channel chan);
int tps65_channel_get(const struct device *dev, enum sensor_channel chan,
                      struct sensor_value *val);

#endif /* ZEPHYR_DRIVERS_SENSOR_TPS65_H_ */ 
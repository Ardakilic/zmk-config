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
#define TPS65_REG_DEVICE_INFO        0x00
#define TPS65_REG_SYS_INFO_0         0x01
#define TPS65_REG_SYS_INFO_1         0x02
#define TPS65_REG_VERSION_INFO       0x03
#define TPS65_REG_XY_INFO_0          0x10
#define TPS65_REG_XY_INFO_1          0x11
#define TPS65_REG_TOUCH_STRENGTH     0x12
#define TPS65_REG_TOUCH_AREA         0x13
#define TPS65_REG_COORDINATES_X      0x14
#define TPS65_REG_COORDINATES_Y      0x16
#define TPS65_REG_PROX_STATUS        0x20
#define TPS65_REG_TOUCH_STATUS       0x21
#define TPS65_REG_COUNTS             0x22
#define TPS65_REG_LTA                0x23
#define TPS65_REG_DELTAS             0x24
#define TPS65_REG_MULTIPLIERS        0x25
#define TPS65_REG_COMPENSATION       0x26
#define TPS65_REG_PROX_SETTINGS      0x40
#define TPS65_REG_THRESHOLDS         0x41
#define TPS65_REG_TIMINGS_0          0x42
#define TPS65_REG_TIMINGS_1          0x43
#define TPS65_REG_GESTURE_TIMERS     0x44
#define TPS65_REG_ACTIVE_CHANNELS    0x45

/* IQS550 specific configuration registers used by the driver */
#define TPS65_REG_SYS_CONFIG_0       0x50
#define TPS65_REG_FILTER_SETTINGS    0x51
#define TPS65_REG_POWER_MODE         0x52
/* Resolution register pair (X then Y, 2 bytes each) */
#define TPS65_REG_X_RESOLUTION       0x60

/* IQS550 limits */
#define TPS65_MAX_X                  3072
#define TPS65_MAX_Y                  2048
#define TPS65_MAX_TOUCH_POINTS       5

/* IQS550 system configuration flags */
#define TPS65_SYS_CFG_FLIP_X        BIT(0)
#define TPS65_SYS_CFG_FLIP_Y        BIT(1)
#define TPS65_SYS_CFG_SWITCH_XY     BIT(2)
#define TPS65_SYS_CFG_TP_EVENT      BIT(3)
#define TPS65_SYS_CFG_PROX_EVENT    BIT(4)

/* Touch detection flags */
#define TPS65_XY_INFO_TOUCH_MASK     0x01

/* IQS550 expected product ID */
#define TPS65_EXPECTED_PRODUCT_ID    0x0040

/* Error recovery thresholds */
#define TPS65_MAX_ERROR_COUNT       5

/* Device state flags */
#define TPS65_SHOW_RESET            BIT(7)
#define TPS65_TOUCH_EVENT           BIT(0)
#define TPS65_PROX_EVENT            BIT(1)

/* Communication timeouts and retry counts */
#define TPS65_I2C_TIMEOUT_MS        100
#define TPS65_INIT_TIMEOUT_MS       500
#define TPS65_MAX_RETRIES           3

/* Opaque forward declarations; layout is private to the driver C file */
struct tps65_data;
struct tps65_config;

#endif /* ZEPHYR_DRIVERS_SENSOR_TPS65_H_ */ 
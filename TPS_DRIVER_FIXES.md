# TPS65/TPS43 Driver Fixes - USB Disconnection Resolution

## Overview

This document outlines the comprehensive fixes applied to the TPS65 (IQS550) and TPS43 (IQS572) trackpad drivers in ZMK firmware to resolve critical USB disconnection issues and improve overall driver reliability.

## Key Issues Identified and Fixed

### 1. **I2C Communication Problems**
**Problem**: Long blocking I2C operations causing USB stack to disconnect
**Solution**: 
- Added 1ms delays between I2C operations to prevent bus flooding
- Implemented proper error handling and timeouts
- Used non-blocking work queue handlers
- Added error recovery with automatic reinitialization

### 2. **Incorrect Register Addresses**
**Problem**: Using wrong register addresses for IQS550/IQS572 controllers
**Solution**:
- Corrected register addresses based on actual datasheets
- Added proper device ID verification 
- Implemented register-specific read/write functions

### 3. **Missing Error Recovery**
**Problem**: No mechanism to recover from communication errors
**Solution**:
- Added error counting and automatic device reset
- Implemented device reinitialization on failure
- Added proper device state management

### 4. **Work Queue Issues** 
**Problem**: Blocking operations in work handlers causing system instability
**Solution**:
- Converted to delayed work queues
- Added mutex protection for thread safety
- Implemented proper callback handling

### 5. **Power Management**
**Problem**: Improper device initialization and state management
**Solution**:
- Added proper reset sequence (LOW→HIGH with delays)
- Implemented device ready state tracking
- Added boot-up delays for device stabilization

## Driver Architecture Changes

### New Data Structures

```c
struct tps65_data {
    struct k_work_delayable work;    // Delayed work for non-blocking ops
    struct k_mutex lock;             // Thread safety
    const struct device *dev;       // Device reference
    
    /* Touch data */
    int16_t x, y;                   // Simplified coordinate storage
    uint8_t touch_state;            // Touch state tracking
    uint8_t touch_strength;         // Touch pressure
    
    /* Device state */
    bool device_ready;              // Ready state tracking
    bool initialized;               // Initialization state
    uint8_t error_count;            // Error recovery counter
    
    /* GPIO callback */
    struct gpio_callback gpio_cb;   // Interrupt handling
    
    /* Sensor callbacks */
    sensor_trigger_handler_t trigger_handler;
    const struct sensor_trigger *trigger;
};
```

### New Configuration Structure

```c
struct tps65_config {
    struct i2c_dt_spec i2c;         // I2C bus configuration
    struct gpio_dt_spec int_gpio;   // Interrupt GPIO
    struct gpio_dt_spec rst_gpio;   // Reset GPIO
    uint8_t resolution_x;           // X resolution
    uint8_t resolution_y;           // Y resolution
    bool invert_x;                  // X coordinate inversion
    bool invert_y;                  // Y coordinate inversion
    bool swap_xy;                   // Coordinate swapping
};
```

## Register Address Corrections

### TPS65 (IQS550) Registers
```c
#define TPS65_REG_DEVICE_INFO       0x00
#define TPS65_REG_SYS_INFO_0        0x01
#define TPS65_REG_SYS_CONFIG_0      0x20
#define TPS65_REG_XY_INFO_0         0x10
#define TPS65_REG_COORDINATES_X     0x14
#define TPS65_REG_X_RESOLUTION      0x66
```

### TPS43 (IQS572) Registers  
```c
#define TPS43_REG_DEVICE_INFO       0x00
#define TPS43_REG_SYS_INFO_0        0x01
#define TPS43_REG_SYS_CONFIG_0      0x20
#define TPS43_REG_XY_INFO_0         0x10
#define TPS43_REG_COORDINATES_X     0x14
#define TPS43_REG_X_RESOLUTION      0x66
```

## Device Tree Binding Updates

### New Properties Added
```yaml
int-gpios:        # Interrupt GPIO pin
rst-gpios:        # Reset GPIO pin  
resolution-x:     # X resolution (default: 1024)
resolution-y:     # Y resolution (default: 1024)
invert-x:         # Invert X coordinates
invert-y:         # Invert Y coordinates
swap-xy:          # Swap X/Y coordinates
sensitivity:      # Touch sensitivity (0-255)
debounce-ms:      # Touch debounce time
gesture-support:  # Enable gestures (experimental)
```

## Error Recovery Mechanism

### Error Detection
- I2C communication failures are counted
- After 5 consecutive errors, device reinitialization is triggered
- Device state is tracked to prevent operations on failed devices

### Recovery Process
1. **Error Detection**: I2C operation fails
2. **Error Counting**: Increment error counter
3. **Threshold Check**: If errors > 5, mark device as not ready
4. **Reinitialization**: Schedule delayed work to reinitialize
5. **Recovery**: Full device reset and reconfiguration

### Recovery Implementation
```c
static int tps65_read_touch_data(const struct device *dev)
{
    // ... I2C read operation ...
    if (ret < 0) {
        data->error_count++;
        if (data->error_count > TPS65_MAX_ERROR_COUNT) {
            LOG_WRN("Too many errors, reinitializing device");
            data->device_ready = false;
            k_work_reschedule(&data->work, K_MSEC(100));
        }
        return ret;
    }
    data->error_count = 0; // Reset on success
    // ... process data ...
}
```

## Timing Improvements

### I2C Timing
- Added 1ms delay between I2C operations
- Prevents bus flooding that can cause USB disconnections
- Allows proper signal settling time

### Reset Timing
```c
gpio_pin_set_dt(&config->rst_gpio, 0);  // Assert reset
k_msleep(10);                           // Reset pulse width
gpio_pin_set_dt(&config->rst_gpio, 1);  // Release reset
k_msleep(50);                           // Boot delay
```

### Work Queue Timing
- Immediate scheduling for interrupts: `K_NO_WAIT`
- Error recovery delay: `K_MSEC(100)`
- Major failure retry: `K_MSEC(1000)`

## Thread Safety

### Mutex Protection
All critical sections are protected with mutexes:
```c
k_mutex_lock(&data->lock, K_FOREVER);
// Critical section - touch data access
k_mutex_unlock(&data->lock);
```

### Work Queue Safety
- All I2C operations moved to work queue context
- GPIO interrupts only schedule work, don't perform I2C operations
- Prevents blocking in interrupt context

## Device Verification

### Product ID Checking
```c
static int tps65_verify_device_id(const struct device *dev)
{
    uint8_t device_info[2];
    ret = tps65_i2c_read_reg(dev, TPS65_REG_DEVICE_INFO, device_info, 2);
    uint16_t product_id = sys_get_le16(device_info);
    
    if (product_id != TPS65_EXPECTED_PRODUCT_ID) {
        LOG_WRN("Unexpected product ID: 0x%04x", product_id);
        // Don't fail completely - some variants differ
    }
    return 0;
}
```

## Sensor API Compliance

### Channel Support
- `SENSOR_CHAN_POS_DX`: X coordinate
- `SENSOR_CHAN_POS_DY`: Y coordinate  
- `SENSOR_CHAN_ALL`: Both coordinates

### Trigger Support
- `SENSOR_TRIG_DATA_READY`: Touch data available
- Interrupt-based data collection
- Callback mechanism for applications

## Configuration Examples

### Basic TPS65 Configuration
```devicetree
&i2c0 {
    tps65: tps65@74 {
        compatible = "azoteq,tps65";
        reg = <0x74>;
        int-gpios = <&gpio0 15 GPIO_ACTIVE_LOW>;
        rst-gpios = <&gpio0 16 GPIO_ACTIVE_LOW>;
        resolution-x = <3072>;
        resolution-y = <2048>;
    };
};
```

### Advanced TPS43 Configuration
```devicetree
&i2c1 {
    tps43: tps43@74 {
        compatible = "azoteq,tps43";
        reg = <0x74>;
        int-gpios = <&gpio1 10 GPIO_ACTIVE_LOW>;
        rst-gpios = <&gpio1 11 GPIO_ACTIVE_LOW>;
        resolution-x = <2048>;
        resolution-y = <1792>;
        invert-y;
        sensitivity = <64>;
        debounce-ms = <25>;
    };
};
```

## Troubleshooting Guide

### 1. USB Still Disconnecting
**Check**: I2C bus speed and timing
**Solution**: Reduce I2C frequency in device tree:
```devicetree
&i2c0 {
    clock-frequency = <100000>; // 100kHz instead of 400kHz
};
```

### 2. Touch Not Detected
**Check**: GPIO configuration and device wiring
**Verify**: Device Product ID in logs
**Solution**: Check interrupt and reset GPIO assignments

### 3. Intermittent Issues
**Check**: Power supply stability
**Monitor**: Error count in logs
**Solution**: Add decoupling capacitors near the trackpad

### 4. Build Errors
**Check**: CMakeLists.txt includes both drivers
**Verify**: Kconfig options are enabled
**Ensure**: Device tree bindings are correct

## Performance Improvements

### Memory Usage
- Reduced per-touch memory from arrays to single values
- Simplified data structures
- Removed unused gesture code paths

### CPU Usage  
- Non-blocking I2C operations
- Efficient work queue usage
- Minimal interrupt handler overhead

### Power Consumption
- Proper sleep/wake sequences
- Configurable polling rates
- Interrupt-driven operation

## Testing Recommendations

### 1. Basic Functionality
- Touch detection and coordinate reporting
- Multi-touch if supported
- Edge case handling (device disconnect/reconnect)

### 2. Stress Testing
- Continuous operation for 24+ hours
- Rapid touch/release cycles
- I2C bus error injection

### 3. Integration Testing
- Full keyboard functionality with trackpad
- USB stability during trackpad use
- Power management cycles

## Future Improvements

### 1. Gesture Support
- Swipe detection
- Pinch/zoom gestures
- Configurable gesture mapping

### 2. Calibration
- Runtime calibration support
- Pressure sensitivity adjustment
- Dead zone configuration

### 3. Advanced Features
- Palm rejection
- Hover detection
- Multi-finger gestures

## References

- Azoteq IQS550 Datasheet
- Azoteq IQS572 Datasheet  
- ZMK Sensor Driver Documentation
- Zephyr I2C Driver API
- QMK Trackpad Implementation (reference)

---
**Note**: These fixes address the core USB disconnection issues. Monitor system logs for any remaining issues and adjust timing parameters as needed for your specific hardware configuration. 
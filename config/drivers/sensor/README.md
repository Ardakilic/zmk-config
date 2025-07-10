# TPS65 Trackpad Driver for ZMK

This driver provides support for the Azoteq TPS-65 trackpad module (based on IQS5xx-B000 series) in ZMK keyboards.

## Features

- **Multi-touch support**: Up to 5 simultaneous touch points
- **Gesture recognition**: Single and multi-finger gestures (taps, swipes, zoom, scroll)
- **Dual variant support**: 43mm (43x40mm) and 65mm (65x49mm) trackpad sizes
- **Flexible interface**: I2C communication with optional interrupt-driven mode
- **Coordinate transformations**: Support for axis inversion and swapping
- **Configurable sensitivity**: Adjustable touch thresholds and palm rejection
- **ZMK integration**: Full sensor API implementation for position data

## Hardware Requirements

### TPS-65 Module Specifications
- **Interface**: I2C (address 0x74)
- **Supply voltage**: 3.3V
- **Operating current**: <10mA active, <1mA standby
- **Touch resolution**: Up to 4096x4096 coordinates
- **Update rate**: Up to 100Hz
- **Operating temperature**: -20°C to +70°C

### Pin Connections
| TPS-65 Pin | Function | Required | Notes |
|------------|----------|----------|-------|
| VCC | Power (3.3V) | Yes | Connect to 3V3 rail |
| GND | Ground | Yes | Connect to ground |
| SDA | I2C Data | Yes | Connect to MCU I2C SDA with pullup |
| SCL | I2C Clock | Yes | Connect to MCU I2C SCL with pullup |
| RDY | Ready/Interrupt | Optional | Connect to GPIO for interrupt mode |
| NRST | Reset | Optional | Connect to GPIO for hardware reset |

### I2C Pullup Resistors
- **Required**: 4.7kΩ pullup resistors on SDA and SCL lines
- **Location**: Can be on MCU board or trackpad module
- **Voltage**: Pull up to 3.3V

## Configuration

### Device Tree Integration

Add the trackpad to your keyboard's device tree overlay:

```dts
&i2c0 {
    status = "okay";
    
    tps65: trackpad@74 {
        compatible = "azoteq,tps65";
        reg = <0x74>;
        
        /* Basic configuration */
        variant = "65mm";              // "43mm" or "65mm"
        resolution-x = <4096>;
        resolution-y = <4096>;
        
        /* GPIO connections (optional) */
        rdy-gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>;  // For interrupt mode
        rst-gpios = <&gpio0 16 GPIO_ACTIVE_HIGH>;  // For hardware reset
        
        /* Touch behavior */
        touch-threshold = <40>;        // 1-255 (lower = more sensitive)
        max-touches = <5>;             // 1-5 simultaneous touches
        palm-reject-threshold = <100>; // Ignore large touches
        
        /* Optional features */
        gesture-enable;                // Enable gesture recognition
        // invert-x;                   // Flip X coordinates
        // invert-y;                   // Flip Y coordinates  
        // swap-axes;                  // Swap X and Y axes
    };
};
```

### Kconfig Options

Add to your keyboard's `.conf` file:

```kconfig
# Required
CONFIG_TPS65=y

# Optional features
CONFIG_TPS65_GESTURE_SUPPORT=y    # Enable gestures
CONFIG_TPS65_POLL_RATE_MS=20      # Polling rate (no RDY GPIO)
CONFIG_TPS65_MAX_TOUCHES=5        # Max simultaneous touches
CONFIG_TPS65_SENSITIVITY=40       # Touch sensitivity

# Dependencies (usually already enabled)
CONFIG_I2C=y
CONFIG_GPIO=y
CONFIG_SENSOR=y
```

## Variant Differences

### 65mm Variant (TPS65-201A-S)
- **Physical size**: 65x49mm active area
- **Resolution**: Up to 4096x4096 coordinates
- **Best for**: Larger keyboards, desktop use
- **Coordinate scaling**: Automatic scaling to physical dimensions

### 43mm Variant (TPS43-201A-S) 
- **Physical size**: 43x40mm active area
- **Resolution**: Up to 4096x4096 coordinates  
- **Best for**: Compact keyboards, portable devices
- **Coordinate scaling**: Automatic scaling to physical dimensions

## Usage in ZMK

### Reading Touch Data

```c
#include <zephyr/drivers/sensor.h>

const struct device *trackpad = DEVICE_DT_GET(DT_NODELABEL(tps65));

// Fetch current touch data
sensor_sample_fetch(trackpad, SENSOR_CHAN_ALL);

// Get X coordinate of first touch
struct sensor_value x_val;
sensor_channel_get(trackpad, SENSOR_CHAN_POS_X, &x_val);

// Get Y coordinate of first touch  
struct sensor_value y_val;
sensor_channel_get(trackpad, SENSOR_CHAN_POS_Y, &y_val);

// Get both coordinates at once
struct sensor_value xy_val[2];
sensor_channel_get(trackpad, SENSOR_CHAN_POS_XY, xy_val);
```

### Coordinate System

- **Origin**: Top-left corner (0,0)
- **X-axis**: Increases left to right
- **Y-axis**: Increases top to bottom  
- **Range**: 0 to resolution (default 4096)
- **Units**: Raw coordinates or scaled to physical dimensions

### Gesture Events

The driver recognizes these gestures:

**Single Finger:**
- Single tap
- Press and hold
- Swipe left/right/up/down

**Multi Finger:**
- Two-finger tap
- Scroll (two-finger drag)
- Zoom (pinch/spread)

## Troubleshooting

### Common Issues

1. **Device not detected**
   - Check I2C connections and pullup resistors
   - Verify power supply (3.3V)
   - Confirm I2C address (0x74)

2. **Erratic touch behavior**
   - Adjust `touch-threshold` (try 30-60 range)
   - Enable palm rejection
   - Check for electrical interference

3. **No interrupts (polling mode)**
   - Normal if RDY GPIO not connected
   - Adjust `CONFIG_TPS65_POLL_RATE_MS`
   - Consider adding RDY connection for better performance

4. **Coordinate issues**
   - Use `invert-x`, `invert-y`, or `swap-axes` options
   - Verify variant setting matches hardware
   - Check resolution settings

### Debug Logging

Enable detailed logging:

```kconfig
CONFIG_SENSOR_LOG_LEVEL_DBG=y
CONFIG_LOG=y
```

### ATI (Automatic Tuning Implementation)

The driver automatically runs ATI calibration on startup:
- Takes 1-2 seconds to complete
- Adapts to overlay material and thickness
- Failure indicates hardware or connection issues

## Hardware Integration Examples

### Example 1: Interrupt-Driven Setup
```dts
tps65: trackpad@74 {
    compatible = "azoteq,tps65";
    reg = <0x74>;
    variant = "65mm";
    rdy-gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>;
    rst-gpios = <&gpio0 16 GPIO_ACTIVE_HIGH>;
    touch-threshold = <35>;
    gesture-enable;
};
```

### Example 2: Polling-Only Setup  
```dts
tps65: trackpad@74 {
    compatible = "azoteq,tps65";
    reg = <0x74>;
    variant = "43mm";
    resolution-x = <2048>;
    resolution-y = <2048>;
    touch-threshold = <50>;
    max-touches = <2>;
};
```

## Technical Details

### I2C Communication
- **Address**: 0x74 (7-bit)
- **Speed**: Up to 400kHz (Fast Mode)
- **Addressing**: 16-bit register addresses
- **End sequence**: 0xEEEE command required after each transaction

### Register Map
The driver implements the complete IQS5xx-B000 register map including:
- System configuration and control
- Touch data and coordinates  
- Gesture events and relative motion
- ATI settings and thresholds
- Resolution and coordinate scaling

### Performance
- **Interrupt mode**: <1ms response time
- **Polling mode**: Response time = poll rate
- **Power consumption**: <10mA active, <1mA standby
- **Update rate**: Up to 100Hz

## License

This driver is licensed under the MIT License, same as ZMK.

## Support

For issues and questions:
1. Check this documentation
2. Review ZMK sensor documentation
3. Check hardware connections and configuration
4. Enable debug logging for detailed diagnostics 
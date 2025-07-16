# TPS43 Trackpad Integration Guide

This document provides a comprehensive guide for integrating the Azoteq TPS43 trackpad with IQS572 controller into ZMK firmware.

## Overview

The TPS43 is a compact 43mm × 40mm trackpad that uses the IQS572 touch controller from Azoteq. It provides multi-touch capability with gesture recognition support, making it ideal for compact keyboard builds.

### Key Features

- **Resolution**: 2048 × 1792 (optimized for the 43mm × 40mm surface)
- **Touch Controller**: IQS572
- **Multi-touch**: Up to 5 simultaneous touches
- **Gesture Support**: Swipe, tap, press & hold, pinch & zoom, scroll
- **Communication**: I2C interface (up to 400kHz)
- **Power**: 1.65V to 3.6V supply voltage
- **Size**: 43mm × 40mm (compact form factor)

## Hardware Requirements

### Connections

| TPS43 Pin | Function | Controller Pin | Notes |
|-----------|----------|----------------|-------|
| VCC | Power | 3.3V | 1.65V to 3.6V |
| GND | Ground | GND | Common ground |
| SDA | I2C Data | I2C SDA | Pull-up required |
| SCL | I2C Clock | I2C SCL | Pull-up required |
| RDY | Ready/Interrupt | GPIO (optional) | Interrupt mode |
| RST | Reset | GPIO (recommended) | Hardware reset |

### I2C Address

The TPS43 uses I2C address `0x74` by default.

## Software Integration

### 1. Driver Files

The TPS43 driver consists of several files:

```
config/drivers/sensor/
├── tps43.c          # Main driver implementation
├── tps43.h          # Driver header file
└── CMakeLists.txt   # Build configuration

config/dts/bindings/sensor/
└── azoteq,tps43.yaml # Device tree binding
```

### 2. Configuration Options

Add these options to your keyboard's `.conf` file:

```cmake
# Enable TPS43 sensor driver
CONFIG_ZMK_SENSOR_TPS43=y

# TPS43 specific settings
CONFIG_ZMK_SENSOR_TPS43_POLL_RATE_MS=10       # Polling rate (10ms = 100Hz)
CONFIG_ZMK_SENSOR_TPS43_MAX_TOUCHES=5         # Maximum simultaneous touches
CONFIG_ZMK_SENSOR_TPS43_SENSITIVITY=40        # Touch sensitivity (lower = more sensitive)
CONFIG_ZMK_SENSOR_TPS43_GESTURE_SUPPORT=y     # Enable gesture recognition

# Required dependencies
CONFIG_I2C=y          # Enable I2C subsystem
CONFIG_SENSOR=y       # Enable sensor subsystem
```

### 3. Device Tree Configuration

Add the TPS43 to your keyboard's overlay file:

```c
&pro_micro_i2c {
    status = "okay";
    
    tps43: trackpad@74 {
        compatible = "azoteq,tps43";
        reg = <0x74>;
        
        /* TPS43 specifications */
        variant = "43mm";
        resolution-x = <2048>;
        resolution-y = <1792>;
        
        /* GPIO connections */
        rdy-gpios = <&pro_micro 8 GPIO_ACTIVE_HIGH>;  /* Interrupt pin */
        rst-gpios = <&pro_micro 9 GPIO_ACTIVE_HIGH>;  /* Reset pin */
        
        /* Touch settings */
        touch-threshold = <40>;
        max-touches = <5>;
        palm-reject-threshold = <100>;
        
        /* Enable features */
        gesture-enable;
        
        /* Communication */
        i2c-timeout-ms = <10>;
    };
};
```

## Comparison: TPS43 vs TPS65

| Feature | TPS43 | TPS65 |
|---------|--------|--------|
| **Size** | 43mm × 40mm | 65mm × 49mm |
| **Controller** | IQS572 | IQS550 |
| **Resolution** | 2048 × 1792 | 3072 × 2048 |
| **Max Touches** | 5 | 5 |
| **I2C Address** | 0x74 | 0x74 |
| **Power** | 1.65V - 3.6V | 1.65V - 3.6V |
| **Features** | Multi-touch, Gestures | Multi-touch, Gestures |

### When to Use TPS43

- **Compact builds**: When space is limited
- **Portable keyboards**: Better for travel keyboards
- **Cost-effective**: Smaller sensor, potentially lower cost
- **Sufficient resolution**: 2048×1792 is adequate for most use cases

### When to Use TPS65

- **Larger keyboards**: When you have more space
- **High precision**: Need maximum resolution
- **Extensive gestures**: Larger surface for complex gestures

## Example: Corne TPS43 Shield

The repository includes a complete Corne TPS43 shield implementation:

```
config/boards/shields/corne_tps43/
├── corne_tps43.dtsi              # Common device tree
├── corne_tps43_left.overlay      # Left half configuration
├── corne_tps43_right.overlay     # Right half with TPS43
├── corne_tps43_left.conf         # Left half config
├── corne_tps43_right.conf        # Right half config
├── Kconfig.shield                # Shield Kconfig
├── Kconfig.defconfig             # Default configuration
└── corne_tps43.yml               # Shield metadata
```

### Building Corne TPS43

```bash
# For Nice!Nano v2
west build -p -b nice_nano_v2 -- -DSHIELD=corne_tps43_left
west build -p -b nice_nano_v2 -- -DSHIELD=corne_tps43_right

# For Pro Micro
west build -p -b pro_micro -- -DSHIELD=corne_tps43_left
west build -p -b pro_micro -- -DSHIELD=corne_tps43_right
```

## Troubleshooting

### Common Issues

1. **Device not detected**
   - Check I2C wiring (SDA/SCL)
   - Verify I2C pull-up resistors
   - Confirm power supply voltage
   - Check I2C address (should be 0x74)

2. **Touch not responsive**
   - Adjust `touch-threshold` (lower = more sensitive)
   - Check ground connection
   - Verify trackpad surface is clean

3. **Erratic behavior**
   - Check for electrical interference
   - Adjust `palm-reject-threshold`
   - Ensure stable power supply

### Debug Configuration

Enable debugging with these config options:

```cmake
CONFIG_ZMK_USB_LOGGING=y
CONFIG_LOG=y
CONFIG_SENSOR_LOG_LEVEL_DBG=y
CONFIG_I2C_LOG_LEVEL_DBG=y
```

### I2C Scan

Use an I2C scanner to verify the device is detected:

```bash
# Enable I2C tools in your build
CONFIG_I2C_SHELL=y
```

## Advanced Configuration

### Polling vs Interrupt Mode

**Interrupt Mode** (recommended):
- Uses RDY pin for efficient operation
- Lower power consumption
- Faster response time

**Polling Mode**:
- No RDY pin required
- Uses timer-based polling
- Simpler wiring

### Gesture Configuration

The TPS43 supports various gestures when `gesture-enable` is set:

- Single tap
- Double tap
- Press and hold
- Swipe (up/down/left/right)
- Pinch and zoom
- Two-finger scroll

### Power Management

Configure power saving features:

```cmake
CONFIG_ZMK_SLEEP=y
CONFIG_ZMK_IDLE_SLEEP_TIMEOUT=900000  # 15 minutes
```

## Sample Files

The repository includes sample files for quick setup:

- `config/samples/tps43_sample.conf` - Sample configuration
- `config/samples/tps43_sample.overlay` - Sample device tree overlay

## Support

For issues or questions:

1. Check the troubleshooting section above
2. Review the TPS43 datasheet from Azoteq
3. Compare with the working TPS65 implementation
4. Enable debug logging to diagnose issues

## License

This implementation is licensed under the MIT License, consistent with the ZMK project. 
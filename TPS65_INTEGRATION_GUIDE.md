# TPS65 Trackpad Integration Guide

This document provides a comprehensive guide for integrating the Azoteq TPS65 trackpad with IQS550 controller into ZMK firmware.

## Overview

The TPS65 is a large 65mm × 49mm trackpad that uses the IQS550 touch controller from Azoteq. It provides high-resolution multi-touch capability with advanced gesture recognition support, making it ideal for premium keyboard builds that require precise cursor control.

### Key Features

- **Resolution**: 3072 × 2048 (optimized for the 65mm × 49mm surface)
- **Touch Controller**: IQS550
- **Multi-touch**: Up to 5 simultaneous touches
- **Gesture Support**: Swipe, tap, press & hold, pinch & zoom, scroll
- **Communication**: I2C interface (up to 400kHz)
- **Power**: 1.65V to 3.6V supply voltage
- **Size**: 65mm × 49mm (large form factor for precision)

## Hardware Requirements

### Connections

| TPS65 Pin | Function | Controller Pin | Notes |
|-----------|----------|----------------|-------|
| VCC | Power | 3.3V | 1.65V to 3.6V |
| GND | Ground | GND | Common ground |
| SDA | I2C Data | I2C SDA | Pull-up required |
| SCL | I2C Clock | I2C SCL | Pull-up required |
| RDY | Ready/Interrupt | GPIO (optional) | Interrupt mode |
| RST | Reset | GPIO (recommended) | Hardware reset |

### I2C Address

The TPS65 uses I2C address `0x74` by default.

## Software Integration

### 1. Driver Files

The TPS65 driver consists of several files:

```
config/drivers/sensor/
├── tps65.c          # Main driver implementation
├── tps65.h          # Driver header file
└── CMakeLists.txt   # Build configuration

config/dts/bindings/sensor/
└── azoteq,tps65.yaml # Device tree binding
```

### 2. Configuration Options

Add these options to your keyboard's `.conf` file:

```cmake
# Enable TPS65 sensor driver
CONFIG_ZMK_SENSOR_TPS65=y

# TPS65 specific settings
CONFIG_ZMK_SENSOR_TPS65_POLL_RATE_MS=10       # Polling rate (10ms = 100Hz)
CONFIG_ZMK_SENSOR_TPS65_MAX_TOUCHES=5         # Maximum simultaneous touches
CONFIG_ZMK_SENSOR_TPS65_SENSITIVITY=40        # Touch sensitivity (lower = more sensitive)
CONFIG_ZMK_SENSOR_TPS65_GESTURE_SUPPORT=y     # Enable gesture recognition

# Required dependencies
CONFIG_I2C=y          # Enable I2C subsystem
CONFIG_SENSOR=y       # Enable sensor subsystem
```

### 3. Device Tree Configuration

Add the TPS65 to your keyboard's overlay file:

```c
&pro_micro_i2c {
    status = "okay";
    
    tps65: trackpad@74 {
        compatible = "azoteq,tps65";
        reg = <0x74>;
        
        /* TPS65 specifications */
        variant = "65mm";
        resolution-x = <3072>;
        resolution-y = <2048>;
        
        /* GPIO connections */
        rdy-gpios = <&pro_micro 8 GPIO_ACTIVE_HIGH>;  /* Interrupt pin */
        rst-gpios = <&pro_micro 9 GPIO_ACTIVE_HIGH>;  /* Reset pin */
        
        /* Touch settings */
        touch-threshold = <40>;
        max-touches = <5>;
        palm-reject-threshold = <100>;
        
        /* Enable features */
        gesture-enable;
        
        /* Optional coordinate transformations */
        // invert-x;        /* Invert X-axis if needed */
        // invert-y;        /* Invert Y-axis if needed */
        // swap-axes;       /* Swap X and Y axes if needed */
        
        /* Communication */
        i2c-timeout-ms = <10>;
    };
};
```

## Comparison: TPS65 vs TPS43

| Feature | TPS65 | TPS43 |
|---------|--------|--------|
| **Size** | 65mm × 49mm | 43mm × 40mm |
| **Controller** | IQS550 | IQS572 |
| **Resolution** | 3072 × 2048 | 2048 × 1792 |
| **Max Touches** | 5 | 5 |
| **I2C Address** | 0x74 | 0x74 |
| **Power** | 1.65V - 3.6V | 1.65V - 3.6V |
| **Features** | Multi-touch, Gestures | Multi-touch, Gestures |

### When to Use TPS65

- **Large keyboards**: When you have ample space for a larger trackpad
- **High precision**: Maximum resolution for detailed cursor control
- **Professional use**: CAD, design work, or precision applications
- **Extensive gestures**: Larger surface for complex multi-finger gestures
- **Premium builds**: High-end keyboards where trackpad size matters

### When to Use TPS43

- **Compact builds**: When space is limited
- **Portable keyboards**: Better for travel keyboards
- **Cost-effective**: Smaller sensor, potentially lower cost
- **Sufficient resolution**: 2048×1792 is adequate for most use cases

## Example: Corne TPS65 Shield

The repository includes a complete Corne TPS65 shield implementation:

```
config/boards/shields/corne_tps65/
├── corne_tps65.dtsi              # Common device tree
├── corne_tps65_left.overlay      # Left half configuration
├── corne_tps65_right.overlay     # Right half with TPS65
├── corne_tps65_left.conf         # Left half config
├── corne_tps65_right.conf        # Right half config
├── Kconfig.shield                # Shield Kconfig
├── Kconfig.defconfig             # Default configuration
└── corne_tps65.yml               # Shield metadata
```

### Building Corne TPS65

```bash
# For Nice!Nano v2
west build -p -b nice_nano_v2 -- -DSHIELD=corne_tps65_left
west build -p -b nice_nano_v2 -- -DSHIELD=corne_tps65_right

# For Pro Micro
west build -p -b pro_micro -- -DSHIELD=corne_tps65_left
west build -p -b pro_micro -- -DSHIELD=corne_tps65_right
```

## Advanced Configuration

### Coordinate Transformations

The TPS65 supports coordinate transformations for different mounting orientations:

```c
tps65: trackpad@74 {
    compatible = "azoteq,tps65";
    reg = <0x74>;
    
    /* Mirror coordinates horizontally */
    invert-x;
    
    /* Mirror coordinates vertically */
    invert-y;
    
    /* Rotate trackpad 90 degrees */
    swap-axes;
};
```

### High-Resolution Mode

For maximum precision, configure the TPS65 for full resolution:

```c
tps65: trackpad@74 {
    compatible = "azoteq,tps65";
    reg = <0x74>;
    
    /* Full TPS65 resolution */
    resolution-x = <3072>;
    resolution-y = <2048>;
    max-x = <3072>;
    max-y = <2048>;
    
    /* Lower sensitivity for precision */
    touch-threshold = <30>;
    
    /* Strict palm rejection */
    palm-reject-threshold = <80>;
};
```

### Polling vs Interrupt Mode

**Interrupt Mode** (recommended):
- Uses RDY pin for efficient operation
- Lower power consumption
- Faster response time
- Better gesture detection

**Polling Mode**:
- No RDY pin required
- Uses timer-based polling
- Simpler wiring
- Slightly higher power consumption

```c
/* Interrupt mode configuration */
tps65: trackpad@74 {
    compatible = "azoteq,tps65";
    reg = <0x74>;
    rdy-gpios = <&pro_micro 8 GPIO_ACTIVE_HIGH>;
    rst-gpios = <&pro_micro 9 GPIO_ACTIVE_HIGH>;
};

/* Polling mode configuration */
tps65: trackpad@74 {
    compatible = "azoteq,tps65";
    reg = <0x74>;
    /* Only reset pin, no ready pin */
    rst-gpios = <&pro_micro 9 GPIO_ACTIVE_HIGH>;
};
```

### Gesture Configuration

The TPS65 supports advanced gestures when `gesture-enable` is set:

- **Single touch gestures**:
  - Single tap
  - Double tap
  - Press and hold
  - Swipe (up/down/left/right)

- **Multi-touch gestures**:
  - Pinch and zoom
  - Two-finger scroll
  - Three-finger swipe
  - Rotate gesture

### Power Management

Configure power saving features for battery-powered keyboards:

```cmake
CONFIG_ZMK_SLEEP=y
CONFIG_ZMK_IDLE_SLEEP_TIMEOUT=900000  # 15 minutes

# TPS65 low power mode
CONFIG_ZMK_SENSOR_TPS65_LOW_POWER=y
```

## Troubleshooting

### Common Issues

1. **Device not detected**
   - Check I2C wiring (SDA/SCL)
   - Verify I2C pull-up resistors (typically 4.7kΩ)
   - Confirm power supply voltage (3.3V recommended)
   - Check I2C address (should be 0x74)
   - Test with I2C scanner tool

2. **Touch not responsive**
   - Adjust `touch-threshold` (lower = more sensitive)
   - Check ground connection quality
   - Verify trackpad surface is clean and dry
   - Ensure no air gaps between trackpad and overlay

3. **Erratic behavior**
   - Check for electrical interference from nearby components
   - Adjust `palm-reject-threshold`
   - Ensure stable power supply with proper decoupling
   - Verify I2C signal integrity

4. **Poor gesture recognition**
   - Increase polling rate (`CONFIG_ZMK_SENSOR_TPS65_POLL_RATE_MS=5`)
   - Lower touch threshold for better sensitivity
   - Ensure trackpad surface is properly calibrated

### Debug Configuration

Enable comprehensive debugging:

```cmake
CONFIG_ZMK_USB_LOGGING=y
CONFIG_LOG=y
CONFIG_SENSOR_LOG_LEVEL_DBG=y
CONFIG_I2C_LOG_LEVEL_DBG=y

# TPS65 specific debugging
CONFIG_ZMK_SENSOR_TPS65_DEBUG=y
```

### I2C Bus Analysis

Use I2C tools to verify communication:

```bash
# Enable I2C shell commands
CONFIG_I2C_SHELL=y

# In ZMK shell:
i2c scan I2C_0    # Should show device at 0x74
```

### Performance Optimization

For optimal performance:

1. **High-frequency polling**: Set to 5-10ms for responsive gestures
2. **Interrupt mode**: Always use RDY pin when available
3. **Proper grounding**: Ensure solid ground plane
4. **Short I2C traces**: Keep SDA/SCL traces as short as possible

## Sample Files

The repository includes comprehensive sample files:

- `config/samples/tps65_sample.conf` - Complete configuration example
- `config/samples/tps65_sample.overlay` - Device tree overlay with all options

## Hardware Integration Tips

### PCB Design

- **Ground plane**: Use solid ground plane under trackpad
- **I2C routing**: Keep SDA/SCL traces short and matched
- **Power decoupling**: Add 100nF ceramic capacitor near TPS65
- **ESD protection**: Consider ESD protection on I2C lines

### Mechanical Integration

- **Mounting**: Secure trackpad to prevent movement
- **Overlay**: Use appropriate overlay thickness (optimized for 1mm)
- **Alignment**: Ensure proper alignment with keyboard layout

### Testing

1. **Basic connectivity**: Verify I2C communication
2. **Touch response**: Test single touch across entire surface
3. **Multi-touch**: Verify simultaneous touch detection
4. **Gestures**: Test all enabled gesture types
5. **Power consumption**: Measure current in active and sleep modes

## Support

For issues or questions:

1. Check the troubleshooting section above
2. Review the TPS65 datasheet from Azoteq
3. Compare with the working TPS43 implementation
4. Enable debug logging to diagnose communication issues
5. Test with minimal configuration first

## Additional Resources

- [Azoteq TPS65 Datasheet](https://www.azoteq.com/images/stories/pdf/proxsense_i2c_trackpad_datasheet.pdf)
- [IQS550 Controller Documentation](https://www.azoteq.com/products/trackpads/iqs550)
- [ZMK Sensor Documentation](https://zmk.dev/docs/features/sensors)

## License

This implementation is licensed under the MIT License, consistent with the ZMK project. 
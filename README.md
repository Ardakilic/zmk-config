# ZMK Keyboard Firmware Configurations

This repository contains ZMK firmware configurations for various keyboards, automatically built using GitHub Actions.

## Overview

This repository houses the configuration files for all my ZMK-powered keyboards. Each keyboard has its own keymap and configuration files tailored for use with a Turkish keyboard layout.

## Keyboards in this Repository

### Pre-configured Shields
- **Corne**: Split ergonomic keyboard with 42 keys
- **Reviung41**: Unibody 41-key keyboard with RGB underglow support
- **Contra**: 40% ortholinear keyboard (4×12 layout)

### Custom Boards/Shields
- **Plonck**: Custom 40% ortholinear keyboard based on the nRF52840 chip
- **Istanbul**: Custom split ergonomic keyboard with encoder support

## Features

All keyboard configurations include:
- Turkish keyboard layout mappings
- Multi-layer key configurations
- Bluetooth profile management
- Special character shortcuts specific to Turkish layout
- Common media and system control keys

## Custom Board Details

### Plonck

A custom 40% ortholinear keyboard powered by the nRF52840 SoC. Features include:
- 4×12 key matrix
- Custom board definition
- Power management with sleep mode
- Battery voltage monitoring
- Multiple Bluetooth profiles

Configuration files:
- Keymap
- Board Configuration

### Istanbul

A custom split ergonomic keyboard with:
- Columnar staggered layout
- Visual layout definition
- Rotary encoder support
- ZMK Studio integration
- Specialized behaviors for Turkish characters
- Mouse key support

Configuration files:
- Keymap
- Hardware Configuration

## Building and Flashing

The firmware is automatically built using GitHub Actions whenever changes are pushed to the repository:

1. The build process is defined in [`build.yaml`](build.yaml)
2. GitHub Actions runs the official ZMK build workflow from [`.github/workflows/build.yml`](.github/workflows/build.yml)
3. When the build completes, firmware artifacts can be downloaded from the Actions tab
4. Flash the firmware to your keyboard using the appropriate method for your controller

## Getting Started

To use these configurations with your own keyboard:

1. Fork this repository
2. Modify the keymap files in the [`config`](config) directory to suit your needs
3. Update [`build.yaml`](build.yaml) to include your board/shield combination
4. Commit and push your changes
5. Download the firmware artifacts from the Actions tab

## ZMK Dependencies

This repository requires [ZMK Firmware](https://github.com/zmkfirmware/zmk) as specified in the west.yml file.

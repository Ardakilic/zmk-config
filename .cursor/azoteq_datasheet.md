# PROXSENSE® STANDARD TRACKPAD MODULE DATASHEET

**ProxSense® Capacitive Trackpads with XY Coordinate, Gesture Recognition & Patented Snap / Push Button Detection**

The ProxSense® series of capacitive trackpads offer best in class sensitivity, signal to noise ratio and power consumption. Automatic tuning for sense electrodes guarantees optimal operation over production and environmental changes.

---

## Main Features

- Trackpad with on chip XY coordinate calculation
- 3072 x 2048 resolution (TPS65)
- 100Hz typical report rate (TPS65)
- Adjustable sensitivity
- Proximity wake up from low power
- Automatic recalibration for environmental changes
- 1 & 2 Finger Gesture Detection
  - Swipe
  - Tap
  - Press & Hold
  - Pinch & Zoom
  - Scroll Gestures
- Up to Fast I2C (400kHz) Interface
- Optional Snap Overlay
- Low Power, suitable for battery applications
- Supply voltage: 1.65V to 3.6V
- <40µA active sensing LP mode
- I2C interface to BlueTooth SoC

**Applications**
- Micro Projectors
- Remote Controls
- Printers & White Goods
- Mechanical Push Button Replacement

---

## Contents

1. Hardware Description  
    1.1 PCB Specifications  
    1.2 Adhesive Specification  
    1.3 Stack-Up A Thickness  
    1.4 Stack-Up B Thickness  
    1.5 Stack-Up C Thickness  
    1.6 Stack-Up D Thickness  
    1.7 Stack-Up E Thickness  
    1.8 Compatible Overlay Thickness  
    1.9 Finger Sizes  
2. TPS43  
3. TPS65  
4. Gestures and Implementation  
    4.1 Swipe Gestures  
    4.2 Tap Gesture  
    4.3 Press and Hold Gesture  
    4.4 Pinch & Zoom  
    4.5 Scroll Gestures  
5. Specifications  
    5.1 Absolute Maximum Specifications  
    5.2 Application Level Tests  
    5.3 Power Consumption  
6. Ordering Information  

---

## 1 Hardware Description

The trackpad modules are constructed on RoHS2 and REACH compliant FR4 PCB material. The module PCBs are 1mm thick and have an ENIG finish with a hotbar footprint and ZIF (zero insertion force) connector. The standard modules are not Halogen free.

### Table 1.1: Summary of Trackpad Offerings

| Module Name | Shape       | Size          | Touch IC | Resolution     |
|-------------|-------------|---------------|----------|----------------|
| TPS43       | Rectangular | 43mm x 40mm   | IQS572   | 2048 x 1792    |
| TPS65       | Rectangular | 65mm x 49mm   | IQS550   | 3072 x 2048    |

### Table 1.2: Summary of Trackpad Overlay Offerings

| Overlay Option | Description                                                 | Stack-Up |
|----------------|-------------------------------------------------------------|----------|
| Adhesive       | 3M Adhesive supplied with liner and pull tab               | A        |
| Mylar Overlay  | 0.2mm Mylar adhered to module with 3M double sided adhesive | B        |
| 4mm Metal Dome | For TPS43 only. Metal Dome sheet added on top of isolation | C        |
| Printed Rubber | For TPS43 only. Black Overlay with Snap Keys               | D        |
| Glass Overlay  | 0.7mm Glass adhered to module with 3M double sided adhesive | E        |

---

### 1.1 PCB Specifications

- Material: 2-layer, FR4 PCB (not Halogen free material)  
- Conductor: 35µm Copper (1oz. Cu)  
- Finish: ENIG  
- Size: Module Specific  
- PCB Final Thickness = 1.0mm ± 10%  
- Outline: Precision DIE-CUT Profile  

### 1.2 Adhesive Specification

- Type: 3M 468 200MP  
- Thickness: 0.13mm  
- Liner: Polycoated Kraft Paper  
- Liner with Pull-Tab (No glue on Pull-Tab)  
- Adhesive sized to fit entire tracking area (module specific)  

---

### 1.3 Stack-Up A Thickness

- PCB + 3M double sided adhesive  

### 1.4 Stack-Up B Thickness

- PCB + 3M double sided adhesive + Mylar overlay  

### 1.5 Stack-Up C Thickness

- PCB + isolation + metal dome sheet  

### 1.6 Stack-Up D Thickness

- PCB + isolation + metal dome sheet + 0.2mm printed rubber key mat  

### 1.7 Stack-Up E Thickness

- PCB + 0.7mm Glass Overlay (Total Thickness: 2.73mm ±10%)  

### 1.8 Compatible Overlay Thickness

TPS65 and TPS43 support up to 3mm overlays, optimized for 1mm.

---

### 1.9 Finger Sizes

| Module | Min Finger | Min Finger Separation |
|--------|------------|------------------------|
| TPS43  | 6.5 mm     | 12 mm                  |
| TPS65  | 7.0 mm     | 12.9 mm                |

---

## 2 TPS43

The TPS43 is a 43mm x 40mm rectangular trackpad with rounded corners.

### Table 2.1: FPC connector pin out for TPS43

| J1 | Connection |
|----|------------|
| 1  | RDY        |
| 2  | NRST       |
| 3  | GND        |
| 4  | VDDHI      |
| 5  | SCL        |
| 6  | SDA        |

---

## 3 TPS65

The TPS65 is a 65mm x 49mm rectangular trackpad with rounded corners.

### Table 3.1: FPC connector pin out for TPS65

| J1 | Connection |
|----|------------|
| 1  | RDY        |
| 2  | NRST       |
| 3  | GND        |
| 4  | VDDHI      |
| 5  | SCL        |
| 6  | SDA        |

---

## 4 Gestures and Implementation

Supports up to 5 fingers and includes:

### 4.1 Swipe Gestures

Recognises 1-finger swipe gestures with interrupt events.

### 4.2 Tap Gesture

Recognises tap at any point with interrupt events.

### 4.3 Press and Hold Gesture

Recognises press & hold with interrupt events.

### 4.4 Pinch & Zoom

- Pinch = two touches move closer  
- Zoom = two touches move apart  

### 4.5 Scroll Gestures

Recognises 2-finger scroll gestures with interrupt events.

---

## 5 Specifications

### 5.1 Absolute Maximum Specifications

- Operating temperature: -40℃ to 85℃  
- Supply Voltage (VDDHI – GND): 3.6V  
- Minimum power-on slope: 100 V/s  
- ESD protection: ±2kV (Human body model)  

### 5.2 Application Level Tests

- 16kV IEC air discharge  
- 1Vp-p Conducted Immunity  

### 5.3 Power Consumption

#### Table 5.1: General Operating Conditions

| DESCRIPTION            | MIN  | TYP  | MAX  | UNIT |
|------------------------|------|------|------|------|
| Supply Voltage         | 1.65 | 3.3  | 3.6  | V    |
| Tracking Mode Current  | 1.89 | 2.23 | 2.56 | mA   |
| Low Power Current      | —    | 23   | —    | µA   |

Conditions:

- No finger on trackpad  
- Event Mode Comms selected  
- LP2 Report Rate set to 160ms  

#### Table 5.2: Start-up and Shut-down

| DESCRIPTION         | PARAMETER | MIN  | MAX  | UNIT |
|---------------------|-----------|------|------|------|
| Power On Reset      | VPOR      | 1.44 | 1.65 | V    |
| Power Down Reset    | VPDR      | 1.30 | 1.60 | V    |

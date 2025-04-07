
# Blood Pressure Analysis System

This project is a blood pressure analysis system produced for a Digital Signal Processing course, using the LilyGO T-Display-S3 ESP32 board. It actuates a small air pump and air valve solenoid via MOSFET drivers and microcontroller control. The system features a real-time graph of pressure values displayed on the onboard TFT, as well as compile-time options to include serial debugging and a special "hi-speed" mode which disables the TFT update while logging.

**Note: This is an academic project and is not intended to replace real medical equipment. This is not a professionally developed or built device and should not be used to make any assessment of health or diagnose medical conditions.**

---

## Features
- **Pressure Monitoring**: Reads pressure values from a Sparkfun Qwiic Micropressure module using I²C protocol.
- **Pressure Control**: Uses a solenoid valve to open and close valves and pressurizes an arm cuff to 200 mmHg.
- **Real-Time Graphing**: Displays pressure readings over time on the TFT display.
- **Status Indicators**: Shows current solenoid status (ON/OFF) and pressure value, as well as timestamp and loop duration (sample time).
- **Self Calibrating**: A robust calibration algorithm adjusts to atmospheric pressure, and can be recalibrated any time.

---

## Hardware Requirements
### Recommended Components
1. **Microcontroller**:
   - [LilyGO T-Display-S3 (ESP32-S3)](https://github.com/Xinyuan-LilyGO/T-Display-S3)
2. **pH Sensor**:
   - [SparkFun Qwiic MicroPressure Sensor](https://www.sparkfun.com/sparkfun-qwiic-micropressure-sensor.html)
3. **DC Air Pump**:
   - Any 4.5V 1-3 LPM DC air pump (e.g., [Adafruit Air Pump And Vacuum Dc Motor - 4.5 V And 2.5 Lpm - Zr370-02pm](https://www.electromaker.io/shop/product/air-pump-and-vacuum-dc-motor-45-v-and-25-lpm-zr370-02pm)).
3. **Air Solenoid Valve**:
   - Any 3-6V solenoid valve (e.g., [DFRobot DFR0866](https://www.digikey.com/en/products/detail/dfrobot/DFR0866/15283079)).
4. **Power Supply**:
   - 5V USB power supply for the LilyGO board.
5. **Other Components**:
   - Level shifter or MOSFET (e.g., IRLB3813, IRLZ44N) for solenoid control.

---

## Wiring
| Component         | LilyGO Pin      | Notes                              |
|--------------------|-----------------|------------------------------------|
| Pressure Sensor Signal (I²C)   | GPIO44 (SCL) | Connect sensor SCL to SCL pin. |
| Pressure Sensor Signal (I²C)   | GPIO43 (SDA) | Connect sensor SDA to SDA pin. |
| Solenoid Control   | GPIO21          | Use a MOSFET to drive the solenoid. |
| Pump Control   | GPIO16          | Use a MOSFET to drive the pump. |

---

## Software Setup
### 1. **Install PlatformIO**
- Install [PlatformIO](https://platformio.org/) in your shell of choice, or use the VS Code extension.

### 2. **Clone This Repository**
```bash
git clone https://github.com/Jbsco/esp-bp
cd esp-bp
```

### 3. **Set the Debug Mode**
- In `src/main.cpp` edit compile-time defines to use different pins or select option debug or hi-speed modes:
```
// Enable/disable serial debug output
#define SERIAL_DEBUG_OUTPUT 1
#define HI_SPEED_DEBUG 1

#define PIN_BAT_VOLT 4

// Solenoid & pump pins
// To gates of IRLB3813PbF FETs or similar
#define SOLENOID_PIN 21 // GPIO pin to control pressure solenoid
#define DRIVER_PIN 16 // GPIO pin to control pump driver
```


### 4. **Compile and Upload**
Connect the board and run the following in PlatformIO:
```
pio run --target upload
```

---

## Usage
1. Connect all hardware as per the wiring table.
2. Power on the system.
3. Monitor the real-time graph and solenoid status on the TFT display.
4. Use the serial monitor (baud: 115200) for debugging.
5. Use the MATLAB script to perform analysis directly.
6. Adjust debug modes as needed.

---

## Notes on Pressure Calibration
- Calibrate the pressure sensor with the programmed procedure.
- Simply press button 2. The system will release pressure and wait for the difference between consectutive measurements to be less than 0.01 mmHg 50 times. After this, 100 measurements are averaged. All pressure readings will have this value subtracted, and readings will be relative to atmospheric pressure.

---

## Display Output & Sampling Performance
TODO

---

## Results
TODO
MATLAB:
![log_9_hs](https://github.com/user-attachments/assets/7f5b74f6-9abb-4c8a-9af0-1fabcf2bfbc7)

---

## Future Enhancements
- Add photos/graphics of device wiring, setup examples, & results to README.
- Extend to any other possible control outputs.

---

## References
1.https://github.com/sparkfun/SparkFun_MicroPressure_Arduino_Library
2. https://www.sparkfun.com/sparkfun-qwiic-micropressure-sensor.html
3. https://registry.platformio.org/libraries/sparkfun/SparkFun%20MicroPressure%20Library

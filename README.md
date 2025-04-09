
# Blood Pressure Analysis System

This project is a blood pressure analysis system produced for a Digital Signal Processing course, using the LilyGO T-Display-S3 ESP32 board. It actuates a small air pump and air valve solenoid via MOSFET drivers and microcontroller control. The system features a real-time graph of pressure values displayed on the onboard TFT, as well as compile-time options to include serial debugging and a special "hi-speed" mode which disables the TFT update while logging.

**Note: This is an academic project and is not intended to replace real medical equipment. This is not a professionally developed or built device and should not be used to make any assessment of health or otherwise be used to diagnose medical conditions.**

---

## Features
- **Pressure Monitoring**: Reads pressure values from a Sparkfun Qwiic Micropressure module using I²C protocol.
- **Pressure Control**: Uses a solenoid valve to open and close valves and pressurizes an arm cuff to 200 mmHg.
- **Real-Time Graphing**: Displays pressure readings over time on the TFT display.
- **Status Indicators**: Shows current solenoid status (ON/OFF) and pressure value, as well as timestamp and loop duration (sample time).
- **Self Calibrating**: A robust calibration algorithm adjusts to atmospheric pressure, and can be recalibrated any time.

![PXL_20250408_184445834](https://github.com/user-attachments/assets/b2bf1973-43b4-4620-ab3b-306a7eac417f)


---
## Background
Blood pressure is an important biometric widely used in the healthcare and medical device industry. Systolic pressure is a measure of the force of blood flow as it is pumped out of the heart. Diastolic pressure is a measure of the blood pressure in between heartbeats, while the heart is filling with blood. Measurement of these values is typically measured by ear using a sphygmomanometer. The typical procedure is to pressurize the subject's arm cuff to a high pressure, typically 180-200 mmHg for the average adult, and then to slowly release pressure until the pulse is detectable (systolic), noting this value, and then continuing to release pressure until the pulse is no longer detectable (diastolic). Many devices exist which automatically perform this procedure, some of which measure these values during the pressurization stage. This procedure is an ideal project to develop for applications in digital signal processing.

## Hardware Requirements
### Recommended Components
1. **Microcontroller**:
   - [LilyGO T-Display-S3 (ESP32-S3)](https://github.com/Xinyuan-LilyGO/T-Display-S3)
2. **Pressure Sensor**:
   - [SparkFun Qwiic MicroPressure Sensor](https://www.sparkfun.com/sparkfun-qwiic-micropressure-sensor.html)
3. **DC Air Pump**:
   - Any 4.5V 1-3 LPM DC air pump (e.g., [Adafruit Air Pump And Vacuum Dc Motor - 4.5 V And 2.5 Lpm - Zr370-02pm](https://www.electromaker.io/shop/product/air-pump-and-vacuum-dc-motor-45-v-and-25-lpm-zr370-02pm)).
3. **Air Solenoid Valve**:
   - Any 3-6V solenoid valve (e.g., [DFRobot DFR0866](https://www.digikey.com/en/products/detail/dfrobot/DFR0866/15283079)).
4. **Power Supply**:
   - 5V USB power supply for the LilyGO board.
5. **Other Components**:
   - Level shifter or MOSFET (e.g., IRLB3813, IRLZ44N) for solenoid control.
   - A blood-pressure cuff and related fittings, as necessary (this will depend primarily on selected components).

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

https://github.com/user-attachments/assets/655ae936-46af-4a66-a9a8-09aed62cbf73

---

## Results
MATLAB:
The included MATLAB script can begin pressurization as well as process data over serial. This is most effective when using the hi-speed debug mode, as sample rates nearly double when skipping TFT updates. The loop time is around 6ms in this mode, so sampling is fast and data is improved. The script plots the pressure run, performs an FFT, applies a band-pass filter and adds another plot/FFT, then applies a narrower band-pass and ands a final plot/FFT. The Q-factor of the second FFT is also determined and used as a factor to relax thresholding, which is used to determine systolic and diastolic pressure output to the console. In the majority of cases, the subject's pulse is clearly visible after filtering, and the output values are close to results from a reference device. Future improvements will include improvement to the dynamic thresholding pipeline to improve accuracy of systolic and diastolic pressure output.
![log_9_hs](https://github.com/user-attachments/assets/d8d6c571-79cc-472f-9f94-d6884d8efff7)

Some Linux installations of MATLAB may encounter a lock file error for ttyACM0. A shell script is provided to temporarily resolve this issue by creating the required file with permissions to allow MATLAB to proceed. It runs the following commands:

```
sudo touch /run/lock/LCK..ttyACM0
sudo chgrp $USER /run/lock/LCK..ttyACM0
sudo chown $USER /run/lock/LCK..ttyACM0
```
The script uses ttyACM0 as the default argument, but can take other ports as the argument instead.

---

## Output & Sampling Performance Comparison
The system was compared with a typical battery-operated system to check accuracy. It should be noted that automated devices may not be accurate themselves. Generally, the performance is expected to be similar, within 10mmHg or so.

Run 1:

![run_1_sys](https://github.com/user-attachments/assets/b6e999df-b0a4-45bc-a8e3-482f335e7b90)![run_1_ref](https://github.com/user-attachments/assets/6364d8b9-189a-4050-b98f-bf82d5a7f899)

Output:
```
Diastolic: 79.084
Systolic: 138.463
```

Run 2:

![run_2_sys](https://github.com/user-attachments/assets/f39aca99-e9b8-4639-aa6c-feaf17c553e4)![run_2_ref](https://github.com/user-attachments/assets/2f8da617-1245-49b5-91b0-3402e124c931)

Output:
```
Diastolic: 82.020
Systolic: 133.168
```

Run 3:

![run_3_sys](https://github.com/user-attachments/assets/76ffffbd-6f63-4977-8b15-242111560419)![run_3_ref](https://github.com/user-attachments/assets/420ea4b1-55eb-4875-a5f9-4043666529b9)

Output:
```
Diastolic: 82.768
Systolic: 120.599
```

Not all values were observed to be in agreement. Run #3 systolic pressure was the furthest from what was observed on the reference device. Further testing on subjects with a wider range of resting heart rate is needed before further adjustment to the calculation and processing.

---

## Future Enhancements
- Add photos/graphics of device wiring, setup examples, & results to README.
- Extend to any other possible control outputs.

---

## References
1. https://github.com/sparkfun/SparkFun_MicroPressure_Arduino_Library
2. https://www.sparkfun.com/sparkfun-qwiic-micropressure-sensor.html
3. https://registry.platformio.org/libraries/sparkfun/SparkFun%20MicroPressure%20Library

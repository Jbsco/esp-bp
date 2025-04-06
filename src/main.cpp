#include <Arduino.h>
#include <PID_v1.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <SparkFun_MicroPressure.h>
#include <cmath>

#define SERIAL_DEBUG_OUTPUT 0 // Enable/disable serial debug output

#define PIN_BAT_VOLT 4

// Buttons
#define PIN_BUTTON_1 0
#define PIN_BUTTON_2 14
int startFlag = 0, calibrateFlag = 0;

// Using the Sparkfun Qwiic Micropressure sensor, info available at:
// https://learn.sparkfun.com/tutorials/sparkfun-qwiic-micropressure-hookup-guide
// I2C
#define I2C_ADDRESS 0x18
#define SDA_PIN 43
#define SCL_PIN 44
// When pressure sensor is ready to read this pin is high:
#define MEASURE_RDY 18

/* Initialize Constructor
 * Optional parameters:
 *  - EOC_PIN: End Of Conversion (defualt: -1)
 *  - RST_PIN: Reset (defualt: -1)
 *  - MIN_PSI: Minimum Pressure (default: 0 PSI)
 *  - MAX_PSI: Maximum Pressure (default: 25 PSI)
 */
//SparkFun_MicroPressure mpr(EOC_PIN, RST_PIN, MIN_PSI, MAX_PSI);
SparkFun_MicroPressure mpr(MEASURE_RDY, -1, 0, 4); // Use EOC, no reset pin, max pressure 4 PSI
double atmos = 0; // atmospheric pressure to calibrate against

double loopTime; // Timestamp
double dT = 0; // Timestep
String response;

#define SOLENOID_PIN 21 // GPIO pin to control pressure solenoid
#define DRIVER_PIN 16 // GPIO pin to control pump driver
bool solenoidState = 0;
bool driverState = 0;

// PID parameters
double setpoint = 100; // Reference pressure
double input, output; // Input from pressure sensor, output to solenoid control
double kp = 25.0, ki = 1.0, kd = 5.0; // PID coefficients

// Create PID instance
PID myPID(&input, &output, &setpoint, kp, ki, kd, REVERSE);

// TFT display
TFT_eSPI tft = TFT_eSPI();
#define GRAPH_HEIGHT 70
#define GRAPH_WIDTH 320
double graphData[GRAPH_WIDTH]; // Array to store pH values for plotting

void calibratePressureSensor(){
    tft.setCursor(0, GRAPH_HEIGHT + 10);
    tft.setTextColor(TFT_PURPLE, TFT_BLACK);
    tft.printf("Atmos: %19s", "RELEASING");
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    // Get atmospheric pressure for use later
    #if SERIAL_DEBUG_OUTPUT
        Serial.println("Calibrating pressure sensor.");
    #endif
    digitalWrite(SOLENOID_PIN, LOW);
    digitalWrite(DRIVER_PIN, LOW);
    double check1 = 0, check2 = 0;
    int min = 50;
    tft.fillRect(0, GRAPH_HEIGHT + 70, GRAPH_WIDTH, GRAPH_HEIGHT + 85, TFT_BLACK);
    while(min > 0){
        check1 = mpr.readPressure(TORR);
        // Display pressure value
        tft.setCursor(0, GRAPH_HEIGHT + 30);
        tft.printf("Current Pressure: %8.3f", check1);
        if (abs(check1 - check2) <= 0.01) min--;
        // Fill percentage bar
        tft.fillRect(0, GRAPH_HEIGHT + 70, (50 - min) * (GRAPH_WIDTH / 50.0), GRAPH_HEIGHT + 85, TFT_PURPLE);
        delay(5);
        check2 = mpr.readPressure(TORR);
        // Display pressure value
        tft.setCursor(0, GRAPH_HEIGHT + 50);
        tft.printf("Current Pressure: %8.3f", check2);
        if (abs(check1 - check2) <= 0.01) min--;
        // Fill percentage bar
        tft.fillRect(0, GRAPH_HEIGHT + 70, (50 - min) * (GRAPH_WIDTH / 50.0), GRAPH_HEIGHT + 85, TFT_PURPLE);
        delay(5);
    }
    tft.setCursor(0, GRAPH_HEIGHT + 10);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.printf("Atmos: %19s", "CALIBRATING");
    atmos = 0;
    tft.fillRect(0, GRAPH_HEIGHT + 70, GRAPH_WIDTH, GRAPH_HEIGHT + 85, TFT_BLACK);
    for(int i = 0; i < 100; ++i){
        atmos += mpr.readPressure(TORR);
        // Display pressure value
        tft.setCursor(0, GRAPH_HEIGHT + 30);
        tft.printf("Total Atmos: % 13f", atmos);
        delay(10);
        tft.setCursor(0, GRAPH_HEIGHT + 50);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.printf("Percent: % 16i%%", i);
        // Fill percentage bar
        tft.fillRect(0, GRAPH_HEIGHT + 70, i * (GRAPH_WIDTH / 100.0), GRAPH_HEIGHT + 85, TFT_CYAN);
    }
    atmos /= 100;
    tft.setCursor(0, GRAPH_HEIGHT + 30);
    tft.printf("Current Pressure: %8.3f", atmos);

    tft.fillRect(0, GRAPH_HEIGHT + 70, GRAPH_WIDTH, GRAPH_HEIGHT + 90, TFT_BLACK);
    #if SERIAL_DEBUG_OUTPUT
        Serial.printf("Calibrated pressure sensor at %.4f.\n", atmos);
    #endif
    calibrateFlag = 0;
}

void IRAM_ATTR buttonISR1(){
    #if SERIAL_DEBUG_OUTPUT
        Serial.println("Button 1 click.");
    #endif
    if (not calibrateFlag && not startFlag){
        #if SERIAL_DEBUG_OUTPUT
            Serial.println("Starting pressurize!");
        #endif
        startFlag = 1;
    }
}

void IRAM_ATTR buttonISR2(){
    #if SERIAL_DEBUG_OUTPUT
        Serial.println("Button 2 click.");
    #endif
    startFlag = 0;
    calibrateFlag = 1;
}

void setup() {
    Serial.begin(115200);

    // INPUT INITIALIZATION
    Wire.begin(SDA_PIN, SCL_PIN);
    if(!mpr.begin(I2C_ADDRESS, Wire)){
        #if SERIAL_DEBUG_OUTPUT
            Serial.println("Cannot connect to MicroPressure sensor.");
        #endif
        while(1);
    }

    pinMode(PIN_BUTTON_1, INPUT_PULLUP);
    pinMode(PIN_BUTTON_2, INPUT_PULLUP);
    // link the button 1 functions.
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_1), buttonISR1, RISING);
    // link the button 2 functions.
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_2), buttonISR2, RISING);

    pinMode(SOLENOID_PIN, OUTPUT);
    pinMode(DRIVER_PIN, OUTPUT);
    digitalWrite(SOLENOID_PIN, LOW); // Ensure solenoid starts released
    digitalWrite(DRIVER_PIN, LOW); // Ensure pump starts off

    // Initialize PID controller
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 1); // Output range for solenoid (0 = OFF, 1 = ON)

    // Initialize TFT
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(2);
    tft.setTextDatum(TL_DATUM);
    tft.setCursor(0, 5);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("Programmed by: %11s", "Jacob Seman");
    tft.setCursor(0, 25);
    tft.printf("%26s", "ECEN4532 DSP Laboratory");
    tft.setCursor(0, 45);
    tft.printf("%26s", "Spring 2025");

    // Clear graph data
    memset(graphData, 0, sizeof(graphData));

    calibratePressureSensor();
    #if SERIAL_DEBUG_OUTPUT
        Serial.println("Pressure Control System Initialized!");
    #endif
}

void loop(){
    dT = millis() / 1000.0 - loopTime;
    loopTime = millis() / 1000.0;

    if(calibrateFlag) calibratePressureSensor();

    // Read and process pressure sensor value (torr/mmHg)
    input = mpr.readPressure(TORR) - atmos; // Subtract atmospheric calibration

    // Run PID computation
    //myPID.Compute(); // TODO: keep increase constant linear

    // Control solenoid
    if (input >= 200.0) startFlag = 0;
    digitalWrite(SOLENOID_PIN, (startFlag && not calibrateFlag) ? HIGH : LOW);
    digitalWrite(DRIVER_PIN, (startFlag && not calibrateFlag) ? HIGH : LOW);

    // Update graph data
    memmove(graphData, graphData + 1, (GRAPH_WIDTH - 1) * sizeof(double));
    graphData[GRAPH_WIDTH - 1] = GRAPH_HEIGHT - map(input, 0, 200, 0, GRAPH_HEIGHT);
    // Clear and redraw graph area
    tft.fillRect(0, 0, GRAPH_WIDTH, GRAPH_HEIGHT + 1, TFT_BLACK);
    // Draw dashed red line for setpoint
    double setpointY = GRAPH_HEIGHT - map(setpoint, 0, 200, 0, GRAPH_HEIGHT); // Map setpoint to graph Y range
    for (int x = 10; x < 310; x += 10) // Dashed pattern: line every 10px
        tft.drawLine(x, setpointY, x + 5, setpointY, TFT_RED); // Short dashes
    // Draw data
    for (int i = 1; i < GRAPH_WIDTH; i++)
        tft.drawPixel(i - 1, graphData[i], graphData[i] > setpointY ? TFT_GREEN : TFT_YELLOW);

    // Display solenoid status
    tft.setTextSize(2);
    tft.setCursor(0, GRAPH_HEIGHT + 10);
    tft.setTextColor(solenoidState ? TFT_GREEN : TFT_RED, TFT_BLACK);
    tft.printf("Solenoid: %16s",solenoidState ? "PRESSURIZE" : "RELEASE");

    // Display pressure value
    tft.setCursor(0, GRAPH_HEIGHT + 30);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("Current Pressure: %8.3f", input);

    // Display timestamp and timestep
    tft.setCursor(0, GRAPH_HEIGHT + 50);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("Timestamp: %15.3f", loopTime);
    tft.setCursor(0, GRAPH_HEIGHT + 70);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("Timestep: %16.3f", dT);

    #if SERIAL_DEBUG_OUTPUT
        // Debug output
        // TODO: instead of debug log analysis perform onboard filtering, FFT, and HR systolic/diastolic
        Serial.printf("Loop Step: %.4f | Loop Time: %.2f | Pressure: %.2f | Solenoid: %s\n", dT, loopTime, input, solenoidState ? "PRESSURIZE" : "RELEASE");
    #endif

    // delay(10); // Update interval
}

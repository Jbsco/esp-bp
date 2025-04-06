#include <Arduino.h>
#include <PID_v1.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <SparkFun_MicroPressure.h>
#include <cmath>

// Enable/disable serial debug output
#define SERIAL_DEBUG_OUTPUT 1
#define HI_SPEED_DEBUG 0

#define PIN_BAT_VOLT 4

// Solenoid & pump pins
// To gates of IRLB3813PbF FETs or similar
#define SOLENOID_PIN 21 // GPIO pin to control pressure solenoid
#define DRIVER_PIN 16 // GPIO pin to control pump driver

// Button pins
#define PIN_BUTTON_1 0
#define PIN_BUTTON_2 14
int startFlag = 0, calibrateFlag = 0, drawOnce = 0;

// Using the Sparkfun Qwiic Micropressure sensor, info available at:
// https://learn.sparkfun.com/tutorials/sparkfun-qwiic-micropressure-hookup-guide
// I2C pins
#define I2C_ADDRESS 0x18
#define SDA_PIN 43
#define SCL_PIN 44
#define MEASURE_RDY 18

// SparkFun_MicroPressure mpr(EOC_PIN, RST_PIN, MIN_PSI, MAX_PSI);
SparkFun_MicroPressure mpr(MEASURE_RDY, -1, 0, 25); // Use EOC, no reset pin, max pressure 4 PSI

// Global variables
double atmos = 0; // Calibrated atmospheric pressure
double loopTime; // Timestamp
double dT = 0; // Timestep


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
double graphData[GRAPH_WIDTH]; // Array to store pressure values for plotting
int graphIndex = 0;

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
        if(abs(check1 - check2) <= 0.01) min--;
        // Fill percentage bar
        tft.fillRect(0, GRAPH_HEIGHT + 70, (50 - min) * (GRAPH_WIDTH / 50.0), GRAPH_HEIGHT + 85, TFT_PURPLE);
        delay(5);
        check2 = mpr.readPressure(TORR);
        // Display pressure value
        tft.setCursor(0, GRAPH_HEIGHT + 50);
        tft.printf("Current Pressure: %8.3f", check2);
        if(abs(check1 - check2) <= 0.01) min--;
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
    // Clear and redraw graph area
    tft.fillRect(0, 0, GRAPH_WIDTH, GRAPH_HEIGHT + 1, TFT_BLACK);
    tft.setCursor(0, GRAPH_HEIGHT + 30);
    tft.printf("Current Pressure: %8.3f", atmos);

    tft.fillRect(0, GRAPH_HEIGHT + 70, GRAPH_WIDTH, GRAPH_HEIGHT + 90, TFT_BLACK);
    #if SERIAL_DEBUG_OUTPUT
        Serial.printf("Calibrated pressure sensor at %.4f.\n", atmos);
    #endif
    calibrateFlag = 0;
    drawOnce = 1;
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
        drawOnce = 1;
    }
}

void IRAM_ATTR buttonISR2(){
    #if SERIAL_DEBUG_OUTPUT
        Serial.println("Button 2 click.");
    #endif
    startFlag = 0;
    calibrateFlag = 1;
    drawOnce = 1;
}

void setup() {
    #if SERIAL_DEBUG_OUTPUT
        Serial.begin(115200);
    #endif

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
    // "It's me, ya boi"
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
    unsigned long now = millis();
    dT = now * 0.001 - loopTime;
    loopTime = now * 0.001;

    if(calibrateFlag) calibratePressureSensor();

    // Read and process pressure sensor value (torr/mmHg)
    input = mpr.readPressure(TORR) - atmos; // Subtract atmospheric calibration

    // Run PID computation
    //myPID.Compute(); // TODO: keep increase constant linear

    // Control solenoids
    if (input >= 200) startFlag = 0, drawOnce = 1;
    digitalWrite(SOLENOID_PIN, startFlag ? HIGH : LOW);
    digitalWrite(DRIVER_PIN, startFlag ? HIGH : LOW);

    #if HI_SPEED_DEBUG
        Serial.printf("Loop Step: %.4f | Loop Time: %.4f | Pressure: %.8f\n", dT, loopTime, input);
    #else
        for(int i = 1, j = graphIndex; i < GRAPH_WIDTH; i++)
            // Clear and redraw graph area
            tft.drawPixel(i - 1, GRAPH_HEIGHT - ((int)(graphData[i] * 0.35)), TFT_BLACK);
        // Update graph data
        memmove(graphData, graphData + 1, (GRAPH_WIDTH - 1) * sizeof(double));
        graphData[GRAPH_WIDTH - 1] = input;
        // Draw data
        for(int i = 1; i < GRAPH_WIDTH; i++)
            tft.drawPixel(i - 1, GRAPH_HEIGHT - ((int)(graphData[i] * 0.35)), TFT_GREEN);

        tft.setTextSize(2);
        if(drawOnce){
            // Display solenoid status
            tft.setCursor(0, GRAPH_HEIGHT + 10);
            tft.setTextColor(startFlag ? TFT_GREEN : TFT_RED, TFT_BLACK);
            tft.printf("Solenoid: %16s",startFlag ? "PRESSURIZE" : "RELEASE");
            // Display pressure value
            tft.setCursor(0, GRAPH_HEIGHT + 30);
            tft.setTextColor(TFT_WHITE, TFT_BLACK);
            tft.printf("Current Pressure: ");
            // Display timestamp and timestep
            tft.setCursor(0, GRAPH_HEIGHT + 50);
            tft.print("Timestamp: ");
            tft.setCursor(0, GRAPH_HEIGHT + 70);
            tft.print("Timestep: ");
            drawOnce = 0;
        }
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        // Display pressure value
        tft.setCursor(216, GRAPH_HEIGHT + 30);
        tft.printf("%8.3f", input);
        // Display timestamp and timestep
        tft.setCursor(132, GRAPH_HEIGHT + 50);
        tft.printf("%15.3f", loopTime);
        tft.setCursor(120, GRAPH_HEIGHT + 70);
        tft.printf("%16.3f", dT);

        #if SERIAL_DEBUG_OUTPUT
            // Debug output
            // TODO: instead of debug log analysis perform onboard filtering, FFT, and HR systolic/diastolic
            Serial.printf("Loop Step: %.4f | Loop Time: %.4f | Pressure: %.8f | Solenoid: %s\n", dT, loopTime, input, startFlag ? "PRESSURIZE" : "RELEASE");
        #endif
    #endif
    // delay(10); // Update interval
}

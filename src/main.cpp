#include <Arduino.h>
#include <PID_v1.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <SparkFun_MicroPressure.h>

#define SERIAL_DEBUG_OUTPUT 1 // Enable/disable serial debug output

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
SparkFun_MicroPressure mpr(MEASURE_RDY, -1, 0, 25); // Use default values with reset and EOC pins unused
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
    // Get atmospeheric pressure for use later
    #if SERIAL_DEBUG_OUTPUT
        Serial.println("Calibrating pressure sensor.");
    #endif
    for(int _ = 0; _ < 100; ++_){
       atmos += mpr.readPressure(TORR);
    }
    atmos /= 100;
    #if SERIAL_DEBUG_OUTPUT
        Serial.printf("Calibrated pressure sensor at %.4f.\n", atmos);
    #endif

    pinMode(SOLENOID_PIN, OUTPUT);
    pinMode(DRIVER_PIN, OUTPUT);
    digitalWrite(SOLENOID_PIN, LOW); // Ensure solenoid starts closed
    digitalWrite(DRIVER_PIN, LOW); // Ensure solenoid starts closed

    // Initialize PID controller
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(0, 1); // Output range for solenoid (0 = OFF, 1 = ON)

    // Initialize TFT
    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextDatum(TL_DATUM);

    // Clear graph data
    memset(graphData, 0, sizeof(graphData));

    #if SERIAL_DEBUG_OUTPUT
        Serial.println("Pressure Control System Initialized!");
    #endif
}

void loop() {
    dT = millis() / 1000.0 - loopTime;
    loopTime = millis() / 1000.0;

    // Read and process pressure sensor value (torr/mmHg)
    input = mpr.readPressure(TORR) - atmos; // Subtract atmospheric calibration

    // Run PID computation
    //myPID.Compute(); // TODO: keep increase constant linear

    // Control solenoid
    solenoidState = (input > 200 ? 0 : 1); // TODO: release & shut off pump after reaching 200
    driverState = (loopTime > 5 ? 1 : 0); // TODO: start with button press
    digitalWrite(SOLENOID_PIN, solenoidState ? HIGH : LOW);
    digitalWrite(DRIVER_PIN, driverState ? HIGH : LOW);

    // Update graph data
    memmove(graphData, graphData + 1, (GRAPH_WIDTH - 1) * sizeof(double));
    graphData[GRAPH_WIDTH - 1] = GRAPH_HEIGHT - map(input, 0, 200, 0, GRAPH_HEIGHT);
    // Clear and redraw graph area
    tft.fillRect(0, 0, GRAPH_WIDTH, GRAPH_HEIGHT, TFT_BLACK);
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
    tft.print(solenoidState ? "Solenoid: PRESSURIZE" : "Solenoid: RELEASE   ");

    // Display pressure value
    tft.setCursor(0, GRAPH_HEIGHT + 30);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("Current Pressure: %.4f", input);

    // Display timestamp and timestep
    tft.setCursor(0, GRAPH_HEIGHT + 50);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("Timestamp: %.4f", loopTime);
    tft.setCursor(0, GRAPH_HEIGHT + 70);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.printf("Timestep: %.4f", dT);

    #if SERIAL_DEBUG_OUTPUT
        // Debug output
        // TODO: instead of debug log analysis perform onboard filtering, FFT, and HR systolic/diastolic
        Serial.printf("Loop Step: %.4f | Loop Time: %.2f | Pressure: %.2f | Solenoid: %s\n", dT, loopTime, input, solenoidState ? "PRESSURIZE" : "RELEASE");
    #endif

    // delay(10); // Update interval
}

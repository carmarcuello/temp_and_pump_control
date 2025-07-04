/* 
  Carlos Marcuello López
  CONTROL - PUMP AND PELTIER
  Microcontroller M5Stack Core 2
*/

//  *********************************************************
//  INCLUDES, PROGRAM DEFINITIONS & CONSTRUCTORS
//  *********************************************************

#include "M5Core2.h"
#include <BluetoothSerial.h>
#include <Wire.h>
#include <MCP342x.h>

//  *********************************************************
//  STRUCTS
//  *********************************************************

// Struct to generate rectangles with 4 parameters
struct Rectangle {
  int x;
  int y;
  int width;
  int height;
};

// Struct to generate circles with 4 parameters
struct Circle{
  int x;
  int y;
  int radius;
  int color;
};

// Struct to generate text coordinates to display text on screen
struct TextCoordinates{
  int x;
  int y;
};

//  *********************************************************
//  PROGRAM DEFINITIONS & CONSTRUCTORS
//  *********************************************************

// Pin definitions for I2C communication
#define I2C_SDATA 32  // I2C Data line
#define I2C_SCLOCK 33  // I2C Clock line

// Event types for touch screen
#define NUM_EVENTS  8
#define TE_TOUCH    0x0001
#define TE_RELEASE  0x0002
#define TE_MOVE     0x0004
#define TE_GESTURE  0x0008
#define TE_TAP      0x0010
#define TE_DBLTAP   0x0020
#define TE_DRAGGED  0x0040
#define TE_PRESSED  0x0080
#define TE_ALL      0x0FFF
#define TE_BTNONLY  0x1000

// Definition for task running at core0 of the microcontroller
TaskHandle_t Task_CORE0;
 
// Bluetooth CONSTRUCTOR 
BluetoothSerial SerialBT;

// ADC MCP3422 CONSTRUCTOR
uint8_t address = 0x68; // 0x68 default address for all MCP342x devices
MCP342x adc = MCP342x(address);


//  *********************************************************
//  VARIABLE DECLARATIONS  
//  *********************************************************

// Rectangles' dimensionss to display on screen
Rectangle titleLineRectangle = {0, 22, 320, 02};  // x, y, width, height
Rectangle temperatureRectangle = {0, 27, 320, 100};  // x, y, width, height
Rectangle controlRectangle = {0, 131, 158, 60};  //  x, y, width, height
Rectangle bombaRectangle = {162, 131, 159, 60};  // x, y, width, height


// Circle for showing that the microcontroller is operating
Circle blinkingCircle = {310, 8, 5, GREEN};

// Text coordinates for displaying data
TextCoordinates textInitial = {15, 45}; // x, y
TextCoordinates textInitialBT = {15, 85}; // x, y
TextCoordinates textInitialVersion = {15, 125}; // x, y
TextCoordinates textTitle = {7, 2}; // x, y
TextCoordinates textPV = {0, 33}; // x, y
TextCoordinates textSP = {0, 65}; // x, y
TextCoordinates textPower = {0, 97}; // x, y
TextCoordinates textBomba = {162, 137}; // x, y
TextCoordinates textControl = {0, 137}; // x, y
TextCoordinates textButtonA = {0, 210}; // x, y
TextCoordinates textButtonB = {145, 210}; // x, y
TextCoordinates textButtonC = {250, 210}; // x, y

// Touch screen button for the controller switch button
Button bombaButton(bombaRectangle.x, bombaRectangle.y, bombaRectangle.width, bombaRectangle.height, "Bomba ON/OFF button"); 
Button controlButton(controlRectangle.x, controlRectangle.y, controlRectangle.width, controlRectangle.height, "Control ON/OFF button"); 

// Time to read the initial message for the user
int timeToReadInitialMessage = 1800; // Value in milliseconds

// Variables for minimum and maximum temperatures
float minTempSP = 20.0;
float maxTempSP = 60.0;

// Variables for the temperature set point
float tempSP = 37.0; // The variable that stores the temperature SP

float smallStep = 0.02;
float mediumStep = 0.06;
float largeStep = 0.3;

// Screen button state variables
bool bombaON = false; // Assign "1" for selecting BOMBA ON and "0" for BOMBA OFF
bool controlON = false; // Assign "true" for selecting PID CONTROL ON and "false" for PID CONTROL OFF

// Sensed temperature value variable
float sensedTemperature = 0.0;

// Variables for the temperature error
float previousTempError = 0.0;
float currentTempError = 0.0;

// Previous time variables for PID controller
unsigned long previousTimeValue = 0; // New time value in milliseconds

// Variable to store the value of the integral of the error curve
double integralOfError = 0;

// Define gains for PID controller
float Kp = 220.0;
float Ki = 10.0;
float Kd = 0.0;

//Min and max gain values
float minGainValue = 0.0;
float maxGainValue = 10000.0;

//PIDs output temperature variable (PWM value from 0 to 4095)
double outputSignalPID = 0;

// Variable for power percentage of the PWM value
float powerPercentage = 0.0;

// PWM Variables
int frequency = 1000;        // PWM frequency of 1 KHz
int resolution = 12;         // 12-bit resolution, 4096 possible values
int maxPWMOutputValue = pow(2, resolution) - 1;  // Calculate the maximum PWM output value

int pwmHeatChannel = 0;     // Selects channel for PWM heating
int pwmCoolChannel = 1;     // Selects channel for PWM cooling
int pwmHeatPin = 19;        // Pin GPIO M5Stack for writing the PID output for heating
int pwmCoolPin = 26;        // Pin GPIO M5Stack for writing the PID output for cooling

// BOMBA variable
int bombaPin = 27;          // Pin G27 GPIO M5Stack for activating/deactivating the BOMBA

// Variables to store the colors for BOMBA and the temperature sensor
int temperatureDisplayAreaColor = BLUE;
int bombaStateColor = RED;
String bombaStateText = "";
int sensedTemperatureTextColor = WHITE;

// Variables for the CONTROL ON/OFF info display
int controlStateColor = RED;
String controlStateText = "";

// Variables to store the text color and the display background color
int textColor = WHITE;
int displayBackgroundColor = BLACK;

// String variable for Serial communication USB and Bluetooth
String receivedCommand = "";

// Variable for showing that the M5Stack is working
bool circleVisible = true;  // Initial state of the circle (visible)
unsigned long previousMillisCircle = 0;  // Previous millis in milisegundos
const int blinkingCircleInterval = 1000;  // Time interval for blinking the cicle (1 second)

// Variables for the MCP342x ADC Converter
float voltAccuracyMCP3422 = 2.048; // Volts
float resolution18bitsMCP3422 = 131072.0; // 32768 para 16 bits (2^16), 131072 para 18 bits (2^18)
int errores = 0;
float sensorValueChannel1 = 0.0; // SENSOR CHANNEL 1 OF THE MCP3422
float sensorValueChannel2 = 0.0; // SENSOR CHANNEL 2 OF THE MCP3422
float Kfilter = 0.1;          // FILTER WEIGTH


//  *********************************************************
//  DECLARACIONES DE SUBRUTINAS  DECLARACIONES DE SUBRUTINAS  
//  *********************************************************

void displayInitialMessage();
void updateControlStateAndColor();
void updateBombaStateAndColor();

void readButtonStates(); // Function to read the button states

float readAdcChannel(int theAdcChannel);

float convertVoltValueToTemperature(float voltageValueFromSensor);  // Function to read temperature sensed with the Neoptix Reflex fiber optic device

float calculateTempError(float tempSetPoint, float sensedTemp);
double computePIDOutputSignal(float sensedTemp, float currTempError, float prevTempError, unsigned long &prevTimeValue, double &integral_OfError);
void applyPWMValue();

void displayData();

void blinkCircle();

// Function declarations for USB and bluetooth communication
void implementSerialUSBCommunication();
void implementSerialBTCommunication();

void wasControlButtonReleased(Event& selectedEvent);
void wasBombaButtonReleased(Event& selectedEvent);

// **************************************************************
// LOOP_CORE0 FUNCTION (TO RUN ON PARALLEL TO THE LOOP FUNCTION)
// **************************************************************

void loop_core0(void *parameter){
  for(;;){
    
    // Read button states for sensor and Set point selection
    readButtonStates();

    // Print data into the display
    displayData();

    // Blink circle to show that the M5Stack is working correctly
    blinkCircle();

    // Call the communication functions
    implementSerialUSBCommunication();  // Call the USB communication function
    implementSerialBTCommunication();  // Call the Bluetooth communication function
  
    //delay(100);
  }
  vTaskDelay(10);
}

//  ****************************************************************
//  SETUP FUNCTION
//  ****************************************************************

void setup() {
  Serial.begin(115200);  // Define the baud rate for communication
  delay(10);

  M5.begin();    // INIT M5Core2 (Initialization of external I2C is also included)
  Wire.begin();

  // Initialize bluetooth for communication and check if it's ready
  if (!SerialBT.begin("M5Stack_Core2")) {
    Serial.println("Starting bluetooth failed!");
    //while (1);  //Stop program
  } 
  else {
    Serial.println("Bluetooth: M5Stack_Core2 OK!");
    Serial.println("The device started, now you can pair it with bluetooth!");
  }
  SerialBT.setTimeout(100);
  delay(100);  

  MCP342x::generalCallReset();              // Reset i2c devices
  delay(10);                                // MC342x 300us min. to settle
  Wire.requestFrom(address, (uint8_t)1);    // Check device present
  if (!Wire.available()) {
    // M5.Lcd.print(" MCP3428 ERROR");
    // while (1);                              // Stop programa
  }  
  Serial.println("ADC Ready");

  // Set the pin for outputting the PWM signal and the channel for generating the PWM signal (HEAT MODE)
  ledcSetup(pwmHeatChannel, frequency, resolution); // Configure channel with the chosen frequency and resolution
  ledcAttachPin(pwmHeatPin, pwmHeatChannel);        // Assign the PWM channel to the PWM pin 

  // Set the pin for outputting the PWM signal and the channel for generating the PWM signal (COOL MODE)
  ledcSetup(pwmCoolChannel, frequency, resolution); // Configure channel with the chosen frequency and resolution
  ledcAttachPin(pwmCoolPin, pwmCoolChannel);        // Assign the PWM channel to the PWM pin 

  // Set the bombaPin as output
  pinMode(bombaPin, OUTPUT);

  // Initialize bomba pin output to LOW
  digitalWrite(bombaPin, LOW);

  // Call function to display the initial message
  displayInitialMessage();

  // Call the functions to display on screen the CONTROL and BOMBA state 
  updateControlStateAndColor();
  updateBombaStateAndColor();

  // Start to run parallel task on core 0
  xTaskCreatePinnedToCore(loop_core0, "Task_CORE0", 10000, NULL, 1, &Task_CORE0, 1);
  
  // Function for the screen touch buttons
  bombaButton.addHandler(wasBombaButtonReleased, TE_RELEASE);
  controlButton.addHandler(wasControlButtonReleased, TE_RELEASE);
}


//  ************************************************************
//  LOOP MAIN FUNCTION
//  ************************************************************

void loop() {

  // Read the voltage value from the Neoptix Reflex fiber optic temperature sensor, voltage value read from sensor 1 (0-2.048V)
  float voltageValueSensor = readAdcChannel(1); // The argument of the function readAdcChannel() is the channel to read from (select channel "1" or "2")

  // Convert the voltage value to temperature data
  sensedTemperature = convertVoltValueToTemperature(voltageValueSensor);
  
  // Check if controller is ON or OFF
  if(controlON == true){ //If controller is ON, turn on the PID controller

    // Store the previous error value as the last error value
    previousTempError = currentTempError;

    // Calculate the current (new) error to input it into the PID controller
    currentTempError = calculateTempError(tempSP, sensedTemperature);

    // Calculate the PID output signal
    outputSignalPID = computePIDOutputSignal(sensedTemperature, currentTempError, previousTempError, previousTimeValue, integralOfError);
    
  }else{ // If controller is OFF, turn off the PID controller

    // Assign 0 to the outputSignalPID and reset completely the PID controller to its default original values
    outputSignalPID = 0; // Set 0 for PID OUTPUT

    //Reset PID controller to default original values
    integralOfError = 0;
    previousTimeValue = millis();
    currentTempError = 0.0;
    previousTempError = 0.0;
  }

  // Apply the PID output as PWM value to the right pins
  applyPWMValue(); 

  // Calculate the PWM output signal as a power percentage
  powerPercentage = (outputSignalPID / maxPWMOutputValue) * 100;
  
} // End loop() function


// **************************************************************************************
// ******************************     Custom functions     ******************************
// **************************************************************************************

void displayInitialMessage() {   // Display dimensions x:320 y:240
           
  M5.Lcd.setBrightness(100);                              
  M5.Lcd.fillScreen(displayBackgroundColor);
  M5.Lcd.setTextColor(textColor, displayBackgroundColor);
  M5.Lcd.setRotation(1);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(textInitial.x, textInitial.y);                       
  M5.Lcd.print("CONTROL TEMP. AND PUMP\n");        
  M5.Lcd.setCursor(textInitialBT.x, textInitialBT.y);             
  M5.Lcd.print("Bluetooth: M5Stack_Core2\n");   // Show Bluetooth name
  M5.Lcd.setCursor(textInitialVersion.x, textInitialVersion.y);             
  M5.Lcd.print("Version 1.0");                
  delay(timeToReadInitialMessage);  // Hold the initial message for some time so that the user can read it
  
  M5.Lcd.clear();  // Clear the contents displayed on the screen

  // Display title for the information screen
  int titleTextColor = WHITE;
  M5.Lcd.setTextColor(titleTextColor, displayBackgroundColor);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(textTitle.x, textTitle.y);
  M5.Lcd.print("CONTROL DE TEMP. Y BOMBA");
  
  // Draw white line under the title
  M5.Lcd.fillRect(titleLineRectangle.x, titleLineRectangle.y, titleLineRectangle.width, titleLineRectangle.height, titleTextColor); // Draw a filled rectangle at (X, Y) position with width, height and color
  
  // Draw color rectangle in the background for the temperature display area
  M5.Lcd.fillRect(temperatureRectangle.x, temperatureRectangle.y, temperatureRectangle.width, temperatureRectangle.height, temperatureDisplayAreaColor); // Draw a filled rectangle at (X, Y) position with width, height and color for background color

} //End the displayInitialMessage() function

void readButtonStates(){
  M5.update();  // Update the state of A, B and C buttons

  if(M5.BtnB.pressedFor(6000)){ // If button B is pressed, then decrease set point temperature value
    tempSP = tempSP - largeStep;
  }else if(M5.BtnB.pressedFor(3000)){
    tempSP = tempSP - mediumStep;
  }else if(M5.BtnB.pressedFor(1)){
    tempSP = tempSP - smallStep;
  }
  else if(M5.BtnB.isPressed()){
    if(controlON == true){
      controlON = false; // Turn controller off
      updateControlStateAndColor();
    }
  }

  if(M5.BtnC.pressedFor(6000)){ // If button C is pressed, then decrease set point temperature value
    tempSP = tempSP + largeStep;
  }else if(M5.BtnC.pressedFor(3000)){
    tempSP = tempSP + mediumStep;
  }else if(M5.BtnC.pressedFor(1)){
    tempSP = tempSP + smallStep;
  }
  else if(M5.BtnC.isPressed()){
    if(controlON == true){
      controlON = false; // Turn controller off
      updateControlStateAndColor();
    }
  }

  tempSP = constrain(tempSP, minTempSP, maxTempSP);

} // End the readButtonStates() function

float readAdcChannel(int theAdcChannel) {

  float averagedDigitalValue, voltageValueReadFromChannel, filteredVoltageValue;
  long valueRead = 0; // var to store the digital value read
  long sumOfTheValuesRead = 0; // Accumulator of values 
  int numberOfReadings = 4;  // Number of averages (Num. promedios)
  int error;

  // Loop to read values from the ADC multiple times
  for (int index = 0; index < numberOfReadings; index++)
  {

    MCP342x::Config status;   

    if(theAdcChannel == 1){
      // Initiate conversion and read ADC value from speficied channel: convertAndRead() will wait until it can be read
      error = adc.convertAndRead(MCP342x::channel1, MCP342x::oneShot, MCP342x::resolution18, MCP342x::gain1, 1000000, valueRead, status);

    }else if(theAdcChannel == 2){
      // Initiate conversion and read ADC value from speficied channel: convertAndRead() will wait until it can be read
      error = adc.convertAndRead(MCP342x::channel2, MCP342x::oneShot, MCP342x::resolution18, MCP342x::gain1, 1000000, valueRead, status);

    }
    
    sumOfTheValuesRead = sumOfTheValuesRead + valueRead;
    delay(10); // Small delay between readings

    if (error) errores = errores + 1; // Increment error counter if any errors occurred
  } 

  // Average the readings
  averagedDigitalValue = (float)sumOfTheValuesRead / (float)numberOfReadings; // Averaged digital value read from the specified channel

  // Convert the averaged digital value to voltage
  voltageValueReadFromChannel = (averagedDigitalValue / resolution18bitsMCP3422) * voltAccuracyMCP3422; // 131072.0 for res=18 bits, 2.048 for voltAccuracy  
  
  // The filtered digital voltage value
  // filteredVoltageValue = filteredVoltageValue * Kfilter + voltageValueReadFromChannel * (1.0 - Kfilter);

  // Return the voltage value read from the specified channel
  return voltageValueReadFromChannel; 

} // End of the readAdcChannel() function


float convertVoltValueToTemperature(float voltageValueFromSensor){
  // Neoptix Reflex fiber optic temperature sensor characteristics
  float tempFromSensor1 = -100.0; // Celsius degrees
  float tempFromSensor2 = 21.7;  // Celsius degrees
  float voltValue1 = 0.0; // Volts
  float voltValue2 = 0.644; // Volts

  // Mapping voltage values to temperature values linearly
  float m = (tempFromSensor2 - tempFromSensor1) / (voltValue2 - voltValue1); //  Slope of the line
  float n = tempFromSensor2 -  (m * voltValue2); // Term "n" of the equation of a line
  float tempValue = m * voltageValueFromSensor + n; // Equation of a line y = m*x + n
  
  return tempValue;

} // End of the convertVoltValueToTemperature() function

float calculateTempError(float tempSetPoint, float sensedTemp){

  float errorTemp = tempSetPoint - sensedTemp;
  return errorTemp;

} // End of the calculateTempError() function

double computePIDOutputSignal(float sensedTemp, float currTempError, float prevTempError, unsigned long &prevTimeValue, double &integral_OfError){

  // Calculate the temperature error increment
  float tempErrorIncrement = currentTempError - previousTempError;

  // Update current and previous time values
  unsigned long currentTimeValue = millis(); // milliseconds
  float timeIncrement = (currentTimeValue - previousTimeValue) / 1000.0;  // This variable will be the time elapsed between currentTimeValue and previousTimeValue (dt) in seconds
  previousTimeValue = currentTimeValue;

  // Calculate the slope of the error curve (derivative)
  double derivativeOfError = tempErrorIncrement / timeIncrement;

  // Calculate the integral of the error curve  
  if(Kp != 0){
    if(abs(currentTempError) <= (maxPWMOutputValue / Kp)){
      integral_OfError = integral_OfError + ((currentTempError + previousTempError)/2) * timeIncrement;
    }
  }
  
  // Clamp the integral value of the error so that it does not keep adding infinitely
  if(Ki != 0){
    integral_OfError = constrain(integral_OfError, -(maxPWMOutputValue / Ki), (maxPWMOutputValue / Ki));
  }
  
  // Calculate the PID Proportional term
  double P_Term = Kp * currentTempError;
  
  // Calculate the PID Integral term
  double I_Term = Ki * integral_OfError;

  // Calculate the PID Derivative term
  double D_Term = Kd * derivativeOfError;

  // Compute the PID output temperature and return its absolute value
  double outputPID = P_Term + I_Term + D_Term;

  // Define the limits for the PID signal output 
  outputPID = constrain(outputPID, -maxPWMOutputValue, maxPWMOutputValue);

  /*
  Serial.println(P_Term);
  Serial.println(I_Term);
  Serial.println(D_Term);
  Serial.println(outputPID);
  Serial.println(" ");
  delay(500);
  */

  // Return the value for the PID output signal
  return outputPID;

} // End of the computePIDOutputSignal() function

void applyPWMValue(){

  // Apply the PID output as a PWM signal to the right channel
  if(outputSignalPID > (maxPWMOutputValue / 100 * 3)){ // If PID output is positive (heating is required), then pwmHeatpin = ON and pwmCoolpin = 0

  // Send the PWM signal to the corresponding heating pin
  ledcWrite(pwmHeatChannel, abs(outputSignalPID)); 

  // Turn off the cooling pin by applying a zero to it
  ledcWrite(pwmCoolChannel, 0);

  }else if (outputSignalPID < -(maxPWMOutputValue / 100 * 3)){ // If PID output is negative (cooling is required), then pwmCoolpin = ON and pwmHeatpin = 0
  
  // Send the PWM signal to the corresponding cooling pin
  ledcWrite(pwmCoolChannel, abs(outputSignalPID)); 

  // Turn off the heating pin by applying a zero to it
  ledcWrite(pwmHeatChannel, 0);

  }else{ // if outputSignalPID is between (-maxPWMOutputValue / 100 * 3) and (maxPWMOutputValue / 100 * 3)

  // Send the PWM signal to the corresponding cooling pin
  ledcWrite(pwmCoolChannel, 0); 

  // Turn off the heating pin by applying a zero to it
  ledcWrite(pwmHeatChannel, 0);
  }

} // End of the applyPWMValue() function

// Functions for the screen touch buttons

void wasControlButtonReleased(Event& selectedEvent) {

  //Serial.println("--- CONTROL BUTTON WAS RELEASED ---");

  // Switch bool state from "CONTROL ON" to "CONTROL OFF" and viceversa
  if (controlON == false){ 
    controlON = true; // Change the bool state to true (CONTROL ON)
  }else{
    controlON = false; // Change the bool state to false (CONTROL OFF)
  }
  updateControlStateAndColor();

} // End of wasBombaButtonReleased() function

void wasBombaButtonReleased(Event& selectedEvent) {

  //Serial.println("--- BOMBA BUTTON WAS RELEASED ---");

  // Switch bool state from "BOMBA ON" to "BOMBA OFF" and viceversa
  if (bombaON == false){ 
    bombaON = true; // Change the bool state to true (BOMBA ON)
  }else{
    bombaON = false; // Change the bool state to false (BOMBA OFF)
  }
  updateBombaStateAndColor();

} // End of wasBombaButtonReleased() function

void updateControlStateAndColor(){

  if(controlON == true){
    // Define display color and text for the PID CONTROL
    controlStateColor = DARKGREEN;
    controlStateText = "   ON ";

  }else{
    // Define display color and text for the PID CONTROL
    controlStateColor = RED;
    controlStateText = "   OFF ";
  }
  
  // Draw color rectangle in the background for BOMBA activation mode
  M5.Lcd.fillRect(controlRectangle.x, controlRectangle.y, controlRectangle.width, controlRectangle.height, controlStateColor); // Draw a filled rectangle for the background color for Controller

} // End of the updateControlStateAndColor() function

void updateBombaStateAndColor(){

  if(bombaON == true){
    // Define display color and text for the BOMBA
    bombaStateColor = DARKGREEN;
    bombaStateText = "   ON ";

    //Send a 1 to activate the BOMBA
    digitalWrite(bombaPin, HIGH);

  }else{
    // Define display color and text for the BOMBA
    bombaStateColor = RED;
    bombaStateText = "   OFF ";

    //Send a 0 to deactivate the BOMBA
    digitalWrite(bombaPin, LOW);
    
  }
  
  // Draw color rectangle in the background for BOMBA activation mode
  M5.Lcd.fillRect(bombaRectangle.x, bombaRectangle.y, bombaRectangle.width, bombaRectangle.height, bombaStateColor); // Draw a filled rectangle for the background color for BOMBA

} // End of the updateBombaStateAndColor() function

void displayData() {

  // Text for temperature data
  M5.Lcd.setTextColor(sensedTemperatureTextColor, temperatureDisplayAreaColor);
  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(textPV.x, textPV.y);
  M5.Lcd.printf(" PV [C]: %3.1f ", sensedTemperature);
  
  M5.Lcd.setTextColor(textColor, temperatureDisplayAreaColor);
  M5.Lcd.setCursor(textSP.x, textSP.y);
  M5.Lcd.printf(" SP [C]: %3.1f ", tempSP);

  M5.Lcd.setTextColor(textColor, temperatureDisplayAreaColor);
  M5.Lcd.setCursor(textPower.x, textPower.y);
  M5.Lcd.print(" POWER [%]");
  M5.Lcd.printf(": %3.0f ", powerPercentage); // Display PID output as a power consumption percentage from 0 to 100%
  
  // Text for CONTROL state
  int yCursorOffset = 27;

  M5.Lcd.setTextColor(textColor, controlStateColor);
  M5.Lcd.setCursor(textControl.x, textControl.y);
  M5.Lcd.print(" CONTROL");
  M5.Lcd.setCursor(textControl.x, textControl.y + yCursorOffset);
  M5.Lcd.print(controlStateText);

  // Text for BOMBA state
  M5.Lcd.setTextColor(textColor, bombaStateColor);
  M5.Lcd.setCursor(textBomba.x, textBomba.y);
  M5.Lcd.print("  BOMBA");
  M5.Lcd.setCursor(textBomba.x, textBomba.y + yCursorOffset);
  M5.Lcd.print(bombaStateText);

  M5.Lcd.setTextSize(3);
  M5.Lcd.setCursor(textButtonB.x, textButtonB.y);
  M5.Lcd.setTextColor(textColor, displayBackgroundColor);
  M5.Lcd.print("SP-");

  M5.Lcd.setCursor(textButtonC.x, textButtonC.y);
  M5.Lcd.setTextColor(textColor, displayBackgroundColor);
  M5.Lcd.print("SP+");

} // End of the displayData() function

void blinkCircle(){
  // Obtain the current time in milliseconds
  unsigned long currentMillisCircle = millis();

  // Verify if the blinking time interval is completed 
  if (currentMillisCircle - previousMillisCircle >= blinkingCircleInterval) {
    // Update the previous millis to the current millis
    previousMillisCircle = currentMillisCircle;
    
    // Change the boolean state of the circle visibility
    circleVisible = !circleVisible;

    // If the bool state of the circle is "visible", draw the circle with the specified color
    if (circleVisible == true) {
      M5.Lcd.fillCircle(blinkingCircle.x, blinkingCircle.y, blinkingCircle.radius, blinkingCircle.color);  // Coordinates X,Y, radius, color
    } 
    else { // If the bool state of the circle is "not visible", draw the circle with background color
      M5.Lcd.fillCircle(blinkingCircle.x, blinkingCircle.y, blinkingCircle.radius, displayBackgroundColor);  // Coordinates X,Y, radius, color
    }
  }

} // End of the blinkCircle() function

void implementSerialUSBCommunication(){
  
  while(Serial.available() > 0){
    char c = Serial.read(); // Read the incoming character

    if (c == '\r') { // If a newline character is received, it means that the user has finished entering an input string
      
      if (receivedCommand.substring(0,1) == "P") {  // If "P" is the first character, return the control data for the PID controller
        Serial.printf("Kp:%.1f ", Kp);
        Serial.printf("Ki:%.1f ", Ki);
        Serial.printf("Kd:%.1f ", Kd);
        Serial.print("\r");
      }

      if (receivedCommand.substring(0,1) == "D") {  // If "D" is the first character, return the control data for the PID controller
        Serial.printf("SP:%3.1f ", tempSP);   
        Serial.printf("PV:%3.1f ", sensedTemperature);
        Serial.printf("PWR:%3.0f ", powerPercentage);
        Serial.printf("CONTROL: %d ", controlON);
        Serial.printf("BOMBA: %d ", bombaON);
        Serial.print("\r");
      }
      
      if (receivedCommand.substring(0,1) == "s") { // If "s" is the first character, adjust Set Point
        if (receivedCommand.length() >= 2) { // Assuming the format is "sx" where x is the new Set Point
          float newSetPoint = receivedCommand.substring(1).toFloat(); // Extract the new Set Point from the received string
          newSetPoint = constrain(newSetPoint, minTempSP, maxTempSP);
          tempSP = newSetPoint; // Update Set Point to the new received value
        }
      }

      if (receivedCommand.substring(0,1) == "p") { // If "p" is the first character, adjust Kp
        if (receivedCommand.length() >= 2) { // Assuming the format is "px" where x is the new Kp
          float newKp = receivedCommand.substring(1).toFloat(); // Extract the new Kp from the received string
          newKp = constrain(newKp, minGainValue, maxGainValue);
          Kp = newKp; // Update Kp to the new received value
        }
      }

      if (receivedCommand.substring(0,1) == "i") { // If "i" is the first character, adjust Ki
        if (receivedCommand.length() >= 2) { // Assuming the format is "ix" where x is the new Ki
          float newKi = receivedCommand.substring(1).toFloat(); // Extract the new Ki from the received string
          newKi = constrain(newKi, minGainValue, maxGainValue);
          Ki = newKi; // Update Ki to the new received value
        }
      }

      if (receivedCommand.substring(0,1) == "d") { // If "d" is the first character, adjust Kd
        if (receivedCommand.length() >= 2) { // Assuming the format is "dx" where x is the new Kd
          float newKd = receivedCommand.substring(1).toFloat(); // Extract the new Kd from the received string
          newKd = constrain(newKd, minGainValue, maxGainValue);
          Kd = newKd; // Update Kd to the new received value
        }
      }

      if (receivedCommand.substring(0,1) == "c") { // If "c" is the first character, then
        if (receivedCommand.length() >= 2) { // Assuming the format is "cx" where x is the new control state

          int newControlState = receivedCommand.substring(1).toInt(); // Extract the new control state from the received string
          
          if(newControlState == 0){ // if c0 received, turn control OFF
            controlON = false;
          }else if(newControlState == 1){ // if c1 received, turn control ON
            controlON = true;
          }

          updateControlStateAndColor();
        }
      }

      if (receivedCommand.substring(0,1) == "b") { // If "b" is the first character, then
        if (receivedCommand.length() >= 2) { // Assuming the format is "bx" where x is the new bomba state

          int newBombaState = receivedCommand.substring(1).toInt(); // Extract the new bomba state from the received string
          
          if(newBombaState == 0){ // if b0 received, turn bomba OFF
            bombaON = false;
          }else if(newBombaState == 1){ // if b1 received, turn bomba ON
            bombaON = true;
          }

          updateBombaStateAndColor();
        }
      }
      
      receivedCommand = ""; // Reset the received command string to an empty string

    }else { // The command sent by the user is not finished yet, so 
      receivedCommand += c; // Append the character to the "receivedCommand" string var
    }
  } 

} // End of the implementSerialUSBCommunication() function

void implementSerialBTCommunication() {
    
  while(SerialBT.available() > 0){
    char c = SerialBT.read(); // Read the incoming character

    if (c == '\r') { // If a newline character is received, it means that the user has finished entering an input string
      
      if (receivedCommand.substring(0,1) == "P") {  // If "P" is the first character, return the control data for the PID controller
        SerialBT.printf("Kp:%.1f ", Kp);
        SerialBT.printf("Ki:%.1f ", Ki);
        SerialBT.printf("Kd:%.1f ", Kd);
        SerialBT.print("\r");
      }

      if (receivedCommand.substring(0,1) == "D") {  // If "D" is the first character, return the control data for the PID controller
        SerialBT.printf("SP:%3.1f ", tempSP);   
        SerialBT.printf("PV:%3.1f ", sensedTemperature);
        SerialBT.printf("PWR:%3.0f ", powerPercentage);
        SerialBT.printf("CONTROL: %d ", controlON);
        SerialBT.printf("BOMBA: %d ", bombaON);
        SerialBT.print("\r");
      }
      
      if (receivedCommand.substring(0,1) == "s") { // If "s" is the first character, adjust Set Point
        if (receivedCommand.length() >= 2) { // Assuming the format is "sx" where x is the new Set Point
          float newSetPoint = receivedCommand.substring(1).toFloat(); // Extract the new Set Point from the received string
          newSetPoint = constrain(newSetPoint, minTempSP, maxTempSP);
          tempSP = newSetPoint; // Update Set Point to the new received value
        }
      }

      if (receivedCommand.substring(0,1) == "p") { // If "p" is the first character, adjust Kp
        if (receivedCommand.length() >= 2) { // Assuming the format is "px" where x is the new Kp
          float newKp = receivedCommand.substring(1).toFloat(); // Extract the new Kp from the received string
          newKp = constrain(newKp, minGainValue, maxGainValue);
          Kp = newKp; // Update Kp to the new received value
        }
      }

      if (receivedCommand.substring(0,1) == "i") { // If "i" is the first character, adjust Ki
        if (receivedCommand.length() >= 2) { // Assuming the format is "ix" where x is the new Ki
          float newKi = receivedCommand.substring(1).toFloat(); // Extract the new Ki from the received string
          newKi = constrain(newKi, minGainValue, maxGainValue);
          Ki = newKi; // Update Ki to the new received value
        }
      }

      if (receivedCommand.substring(0,1) == "d") { // If "d" is the first character, adjust Kd
        if (receivedCommand.length() >= 2) { // Assuming the format is "dx" where x is the new Kd
          float newKd = receivedCommand.substring(1).toFloat(); // Extract the new Kd from the received string
          newKd = constrain(newKd, minGainValue, maxGainValue);
          Kd = newKd; // Update Kd to the new received value
        }
      }

      if (receivedCommand.substring(0,1) == "c") { // If "c" is the first character, then
        if (receivedCommand.length() >= 2) { // Assuming the format is "cx" where x is the new control state

          int newControlState = receivedCommand.substring(1).toInt(); // Extract the new control state from the received string
          
          if(newControlState == 0){ // if c0 received, turn control OFF
            controlON = false;
          }else if(newControlState == 1){ // if c1 received, turn control ON
            controlON = true;
          }

          updateControlStateAndColor();
        }
      }

      if (receivedCommand.substring(0,1) == "b") { // If "b" is the first character, then
        if (receivedCommand.length() >= 2) { // Assuming the format is "bx" where x is the new bomba state

          int newBombaState = receivedCommand.substring(1).toInt(); // Extract the new bomba state from the received string
          
          if(newBombaState == 0){ // if b0 received, turn bomba OFF
            bombaON = false;
          }else if(newBombaState == 1){ // if b1 received, turn bomba ON
            bombaON = true;
          }

           updateBombaStateAndColor();
        }
      }

      receivedCommand = ""; // Reset the received command string to an empty string

    }else { // The command sent by the user is not finished yet, so 
      receivedCommand += c; // Append the character to the "receivedCommand" string var
    }
  } 

} // End of the implementSerialBTCommunication() function

const int solarVoltagePin  = A0;
const int solarCurrentPin = A1;

const int sensorPin = A2; //Defines the pin that the anemometer output is connected to
int sensorValue = 0; //Variable stores the value direct from the analog pin
float sensorVoltage = 0; //Variable that stores the voltage (in Volts) from the anemometer being sent to the analog pin
float windSpeed = 0; // Wind speed in meters per second (m/s)
 
float voltageConversionConstant = .004882814; //This constant maps the value provided from the analog read function, which ranges from 0 to 1023, to actual voltage, which ranges from 0V to 5V
int sensorDelay = 15*1000; //Delay between sensor readings, measured in milliseconds (ms)
 
//Anemometer Technical Variables
//The following variables correspond to the anemometer sold by Adafruit, but could be modified to fit other anemometers.
 
float voltageMin = .4; // Mininum output voltage from anemometer in mV.
float windSpeedMin = 0; // Wind speed in meters/sec corresponding to minimum voltage
 
float voltageMax = 2.0; // Maximum output voltage from anemometer in mV.
float windSpeedMax = 32; // Wind speed in meters/sec corresponding to maximum voltage

const float voltageDividerRatio = 2.0; // 10kΩ and 10kΩ resistors (1:1 ratio)

// ACS712 calibration values
const float ACS712_Zero_Current_Voltage = 2.5; // Typical zero current output voltage of ACS712 (2.5V)
const float ACS712_Sensitivity = 0.066; // Sensitivity for ACS712-30A (66mV/A)

void setup() {
  Serial.begin(9600); 
}

  void loop() {
  // Read the solar panel voltage
  //int solarValue = analogRead(solarVoltagePin);
  sensorValue = analogRead(sensorPin);
  //float panelVoltage = solarValue * (5.0 / 1023.0); // Convert the analog reading to voltage

  int voltageSensorValue = analogRead(solarVoltagePin);
  float measuredVoltage = voltageSensorValue * (5.0 / 1023.0); // Convert the analog reading to voltage
  float panelVoltage = measuredVoltage * voltageDividerRatio; // Adjust for the voltage divider

  int currentSensorValue = analogRead(solarCurrentPin);
  float measuredCurrentVoltage = currentSensorValue * (5.0 / 1023.0); // Convert the analog reading to voltage
  float panelCurrent = (measuredCurrentVoltage - ACS712_Zero_Current_Voltage) / ACS712_Sensitivity;
  
  
  if (panelCurrent < 0) {
    panelCurrent = -panelCurrent; // Correct the current direction
  }
  

  sensorVoltage = sensorValue * voltageConversionConstant; //Convert sensor value to actual voltage
  //Convert voltage value to wind speed using range of max and min voltages and wind speed for the anemometer
  if (sensorVoltage <= voltageMin){
    windSpeed = 0; //Check if voltage is below minimum value. If so, set wind speed to zero.
  }else {
    windSpeed = (sensorVoltage - voltageMin)*windSpeedMax/(voltageMax - voltageMin); //For voltages above minimum value, use the linear relationship to calculate wind speed.
  }

  Serial.print("Solar Voltage: ");
  Serial.print(panelVoltage);
  Serial.println(" V");

  // Print the current to the serial monitor
  Serial.print("Solar Current: ");
  Serial.print(panelCurrent);
  Serial.println(" A");

   //Print voltage and windspeed to serial
  //Serial.print("Voltage: ");
  Serial.print(sensorVoltage);
  Serial.print(",");
  Serial.println(windSpeed); 
  
  delay(1000); // Delay for 1 second
}
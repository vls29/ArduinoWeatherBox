#include <SPI.h>
#include <Ethernet.h>

///////// CHANGEABLE VALUES /////////

const int humiditySensorPin = A0;

const int temperatureSensorPin = A1;

const int pressureSensorPin = A2;

const bool trace = false;
const bool debug = true;

const char serverAddress[] = "home-monitoring.scaleys.co.uk";
const int serverPort = 80;
const int httpRequestDelay = 15;

const char serviceEndpoint[] = "/weather";

const double temperatureMultiplier = 1.0;
const double temperatureCalculationOffset = 1.0;
//const double temperatureOffset = -7.5;
const double temperatureOffset = -2.5;
// 21-05-09 - Room temp roughly 21.5C. temperature offset required roughly -7.5

///////// CHANGEABLE VALUES ABOVE /////////

// Ethernet
EthernetClient ethernetClient;
const byte mac[] = {0x90, 0xA2, 0xDA, 0x0F, 0x86, 0xF1};

// calibrated to the exact voltage on the ethernet arduino I'm using
const double FIVE_VOLTS = 5.006;
const double ANALOG_PIN_RANGE = 1024.0;
const long READING_INCREMENT = 1L;
const long ZERO_LONG = 0L;
const double ZERO_DOUBLE = 0.0;

// pressure constants
const double KPA_TO_MILLIBARS = 10.0;
const double pressureOffset = 10.3;

// temperature constatnts
const double offset = 0.5;
const double milliVolts = 100.0;

// counters
long loopCounter = ZERO_LONG;

unsigned long temperatureReadings = 0L;
double humidityReadings = ZERO_DOUBLE;
double pressureReadings = ZERO_DOUBLE;

// timings
const double minutesBetweenCalls = 1.0;

const unsigned long millisecondsPerMinute = 60000;
const unsigned long minutesInHour = 60;
const unsigned long timeBetweenCalls = minutesBetweenCalls * millisecondsPerMinute;

unsigned long lastTimeUploaded = millis();
unsigned long previousTime = 0UL;

void setup() {
  Serial.begin(9600);
  connectToEthernet();
}

void connectToEthernet() {
  // start the Ethernet connection:
  bool connectedToNetwork = false;
  while(!connectedToNetwork) {
    Serial.println("Attempting to connect to network...");

    if (Ethernet.begin(mac) == 0) {
        Serial.println("Failed to connect, trying again...");
    } else {
        Serial.println("Connected successfully");
        connectedToNetwork = true;
    }
  }

  // give the Ethernet a second to initialize:
  delay(1000);
  Serial.println("connecting...");

  Serial.print("Connected to the network IP: ");
  Serial.println(Ethernet.localIP());
}

void loop() {
  readInstantTemperatureSensorValue();
  readInstantHumiditySensorValue();
  readInstantPressureSensorValue();
  
  incrementLoopCounter();

  if (isTimeToUpload()) {
    if(debug) {
      Serial.println("******************************************************");
    }
    double averageTemperature = calculateAverageTemperature();
    double averageTemperatureCompensatedHumidity = calculateAverageTemperatureCompensatedHumidity(averageTemperature);
    double averageTemperatureCompensatedPressure = calculateAverageTemperatureCompensatedPressure(averageTemperature);
    if(debug) {
      Serial.println("******************************************************");
    }

    sendResultsToServer(averageTemperature, averageTemperatureCompensatedHumidity, averageTemperatureCompensatedPressure);
    resetAverageTemperature();
    resetAverageHumidity();
    resetAveragePressure();

    resetLoopCounter();
  }
}

boolean isTimeToUpload() {
  unsigned long currentTime = millis();

  if (currentTime < previousTime)  {
    lastTimeUploaded = currentTime;
  }

  previousTime = currentTime;

  if ( (currentTime - lastTimeUploaded) >= timeBetweenCalls) {
    Serial.println("Time to upload");
    lastTimeUploaded = currentTime;
    return true;
  }
  return false;
}

/*
 * Read from a 5V pin and multiply it up to it's true value.
 */
double read5VAnalogPin(const int pin) {
  double sensorVal = analogRead(pin);

  if(trace) {
    Serial.print("Raw pin '");
    Serial.print(pin);
    Serial.print("' sensor value: ");
    Serial.println(sensorVal);
  }
  
  sensorVal = (sensorVal / ANALOG_PIN_RANGE) * FIVE_VOLTS;
  
  if(trace) {
    Serial.print("Pin '");
    Serial.print(pin);
    Serial.print("' true value: ");
    Serial.println(sensorVal);
  }

  return sensorVal;
}

void incrementLoopCounter() {
  loopCounter = loopCounter + READING_INCREMENT;
}

void resetLoopCounter() {
  if(debug) {
    Serial.print("Loops completed ");
    Serial.println(loopCounter);
  }
  
  loopCounter = ZERO_LONG;
}

///////////////////////////////////////////////////////
////////////////// Humidity functions /////////////////
///////////////////////////////////////////////////////

/*
 * For efficiency reasons, the stored value is the relative uncompensated humidity.
 * Difference in number of readings is staggering if too many calculations done on the arduino.
 */
void readInstantHumiditySensorValue() {
  double humidity = read5VAnalogPin(humiditySensorPin);
  if(trace) {
    serialPrintln("Humidity sensor value: ", humidity);
  }

  humidityReadings = humidityReadings + humidity;
}

/**
 * Calculates the average non-temperature compensated humidity first and then uses the value in the average temperature compensated humidity calculation.
 * All the calculations are done here for efficiency.
 */
double calculateAverageTemperatureCompensatedHumidity(double averageTemperature) {
  double averageNonCompensatedHumidity = (double)humidityReadings/(double)loopCounter;

  // From Honeywell: Vout = (Vsupply)(0.00636(Sensor RH) + 0.1515), typical at 25 deg. C
  // Or written to calculate Relative Humidity: Relative Humidity = ((Vout / Vsupply) - 0.1515) / 0.00636, typical at 25 deg. C
  double averageRelativeHumidity = ((averageNonCompensatedHumidity / FIVE_VOLTS) - 0.1515) / 0.00636;

  // Temperature Compensated Relative Humidity = (Relative Humidity)/(1.0546 - 0.00216T), T in deg. C
  double averageTemperatureCompensatedHumidity = (averageRelativeHumidity / (1.0546 - (0.00216 * averageTemperature)));

  if(debug) {
    serialPrintln("Average humidity sensor value: ", averageNonCompensatedHumidity);
    serialPrintln("Average non-compensated humidity: ", averageRelativeHumidity);
    serialPrintln("Temperature compensated humidity: ", averageTemperatureCompensatedHumidity);
  }

  return averageTemperatureCompensatedHumidity;
}

void resetAverageHumidity() {
  humidityReadings = ZERO_DOUBLE;
}

///////////////////////////////////////////////////////
//////////////// Temperature functions ////////////////
///////////////////////////////////////////////////////

void readInstantTemperatureSensorValue() {
  int sensorVal = analogRead(temperatureSensorPin);
  
  if(trace) {
    serialPrintln("temperature: ", sensorVal);
  }
  
  temperatureReadings = temperatureReadings + sensorVal;
}

double calculateAverageTemperature() {
  double averageTemperatureSensorValue = (double)temperatureReadings/(double)loopCounter;
  double voltageAverage = (averageTemperatureSensorValue / ANALOG_PIN_RANGE) * FIVE_VOLTS;
  serialPrintln("temperatureReadings: ", temperatureReadings);
  serialPrintln("averageTemperatureSensorValue: ", averageTemperatureSensorValue);
  serialPrintln("Temperature Average Voltage: ", voltageAverage);

  double averageTemperature = (voltageAverage - offset) * milliVolts;
  serialPrintln("averageTemperature before more complex calc...: ", averageTemperature);
  averageTemperature = ((temperatureMultiplier * (averageTemperature + temperatureOffset)) + temperatureCalculationOffset);


  if(debug) {
    serialPrintln("Average Temperature: ", averageTemperature);
  }

  return averageTemperature;
}

void resetAverageTemperature() {
  temperatureReadings = 0L;
}


///////////////////////////////////////////////////////
///////////////// Pressure functions //////////////////
///////////////////////////////////////////////////////

void readInstantPressureSensorValue() {
  double uncompensatedPressure = read5VAnalogPin(pressureSensorPin);

  if(trace) {
    serialPrintln("Uncompensated pressure: ", uncompensatedPressure);
  }
  
  pressureReadings = pressureReadings + uncompensatedPressure;
}

/**
 * From the pressure sensor document:
 * Vout = Vsupply * ((Pressure * 0.004) - 0.04) +- (Pressure Error * Temperature Factor * 0.004 x Vsupply)
 *
 *
 * BECOMES
 *
 * Vout
 * ____    = ((Pressure * 0.004) - 0.04)
 * 
 * Vsupply
 *
 *
 * BECOMES
 *
 *    Vout
 * (  ____   ) + 0.04 = (Pressure * 0.004
 * 
 *   Vsupply
 *
 *
 * BECOMES
 *
 *     Vout
 * ( (  ____   ) + 0.04) / 0.004 = Pressure
 * 
 *    Vsupply
 *
 * E.g. https://www.mathpapa.com/quadratic-formula/?q=2x%5E2-5x-3%3D0
 *
 * IGNORING the pressure compensation +- as it merely gives the upper and lower limit of the epossible pressure reading.
 *
 */
double calculateAverageTemperatureCompensatedPressure(double averageTemperature) {
  double averagePressure = (double)pressureReadings/(double)loopCounter;

  double calculatedPressure = ((averagePressure / FIVE_VOLTS) + 0.04) / 0.004;
  calculatedPressure = (calculatedPressure * KPA_TO_MILLIBARS) + pressureOffset;

  if(debug) {
    serialPrintln("Average pressure sensor reading: ", averagePressure);
    serialPrintln("Calculated pressure: ", calculatedPressure);
  }

  return calculatedPressure;
}

void resetAveragePressure() {
  pressureReadings = ZERO_DOUBLE;
}

void serialPrintln(String message, double value) {
  Serial.print(message);
  Serial.println(value);
}

void sendResultsToServer(double averageTemperature, double averageTemperatureCompensatedHumidity, double averageTemperatureCompensatedPressure) {
  Serial.println("sendResultsToServer");

  String postData = getPostData(averageTemperature, averageTemperatureCompensatedHumidity, averageTemperatureCompensatedPressure);
  Serial.println("post data: " + postData);

  if (ethernetClient.connect(serverAddress, serverPort)) {
    Serial.println("connected to server");
    // Make a HTTP request:
    ethernetClient.println("POST " + String(serviceEndpoint) + " HTTP/1.1");
    ethernetClient.println("Host: " + String(serverAddress) + ":" + serverPort);
    ethernetClient.println("Content-Type: application/json");
    ethernetClient.println("Content-Length: " + String(postData.length()));
    ethernetClient.println("Pragma: no-cache");
    ethernetClient.println("Cache-Control: no-cache");
    ethernetClient.println("Connection: close");
    ethernetClient.println();

    ethernetClient.println(postData);
    ethernetClient.println();

    delay(httpRequestDelay);
    ethernetClient.stop();
    ethernetClient.flush();
    Serial.println("Called server");
  }
}

String getPostData(double averageTemperature, double averageTemperatureCompensatedHumidity, double averageTemperatureCompensatedPressure)
{
  char tempChar[10];
  dtostrf(averageTemperature, 3, 2, tempChar);

  char humidityChar[10];
  dtostrf(averageTemperatureCompensatedHumidity, 3, 2, humidityChar);

  char pressureChar[10];
  dtostrf(averageTemperatureCompensatedPressure, 4, 2, pressureChar);

  return "{\"temperature\":" + String(tempChar) + ",\"humidity\":" + String(humidityChar) + ",\"pressure\":" + String(pressureChar) + "}";
}

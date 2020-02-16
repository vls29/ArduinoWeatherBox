

///////// CHANGEABLE VALUES /////////

const int humiditySensorPin = A0;

const int temperatureSensorPin = A1;

const int pressureSensorPin = A2;

const bool trace = false;
const bool debug = true;

///////// CHANGEABLE VALUES ABOVE /////////

// calibrated to the exact voltage on the ethernet arduino I'm using
const double FIVE_VOLTS = 5.006;
const double ANALOG_PIN_RANGE = 1024.0;
const long READING_INCREMENT = 1L;
const long ZERO_LONG = 0L;
const double ZERO_DOUBLE = 0.0;

// pressure constants
const double PRESSURE_TEMPERATURE_ERROR_FACTOR_NORMAL = 1.0;

const double KPA_TO_MILLIBARS = 10.0;

// temperature constatnts
const double temperatureOffset = 0.0;

const double temperatureMultiplier = 1.252;
const double temperatureCalculationOffset = 1.188;

// counters
long loopCounter = ZERO_LONG;

double temperatureReadings = ZERO_DOUBLE;
double humidityReadings = ZERO_DOUBLE;
double pressureReadings = ZERO_DOUBLE;

// timings
const double minutesBetweenCalls = 0.5;

const unsigned long millisecondsPerMinute = 15000;
const unsigned long minutesInHour = 60;
const unsigned long timeBetweenCalls = minutesBetweenCalls * millisecondsPerMinute;

unsigned long lastTimeUploaded = millis();

void setup() {
  Serial.begin(9600);
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

    resetAverageTemperature();
    resetAverageHumidity();
    resetAveragePressure();

    resetLoopCounter();
  }
}

boolean isTimeToUpload() {
  unsigned long time = millis();

  if( (time - lastTimeUploaded) >= timeBetweenCalls) {
    Serial.println("Time to upload");
    lastTimeUploaded = time;
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
  double temperature = calculateInstantTemperature();
  
  if(trace) {
    serialPrintln("temperature: ", temperature);
  }
  
  temperatureReadings = temperatureReadings + temperature;
}

/**
 * From the MCP9700 documentation, at 0C, 500mV with 10mV per 1C, so take away 500mV (0.5) to give the degrees C above zero then divide by 10mV per 1C to give temperature
 * Further reading: https://starter-kit.nettigo.eu/2010/how-to-measure-temperature-with-arduino-and-mcp9700/
 */
double calculateInstantTemperature() {
  double voltage = read5VAnalogPin(temperatureSensorPin);

  double temperature = ((voltage - 0.5) / 0.01);

  return temperature;
}

double calculateAverageTemperature() {
  double averageTemperature = temperatureReadings/(double)loopCounter;

  if(debug) {
    serialPrintln("Average Temperature: ", averageTemperature);
  }

  return averageTemperature;
}

void resetAverageTemperature() {
  temperatureReadings = ZERO_DOUBLE;
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
  calculatedPressure = calculatedPressure * KPA_TO_MILLIBARS;

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

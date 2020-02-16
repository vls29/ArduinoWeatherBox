

///////// CHANGEABLE VALUES /////////

const int humiditySensorPin = A0;

const int temperatureSensorPin = A1;

const int pressureSensorPin = A2;

const bool trace = false;
const bool debug = false;

///////// CHANGEABLE VALUES ABOVE /////////

// calibrated to the exact voltage on the ethernet arduino I'm using
const double FIVE_VOLTS = 5.006;
const double ANALOG_PIN_RANGE = 1023.0;
const long READING_INCREMENT = 1L;
const long ZERO_LONG = 0L;
const double ZERO_DOUBLE = 0.0;

// pressure constants
const double PRESSURE_TEMPERATURE_ERROR_FACTOR_NORMAL = 1.0;

const double KPA_TO_MILLIBARS = 10.0;

// counters
long loopCounter = 0L;

double temperatureReadings = 0.0;
double humidityReadings = 0.0;
double pressureReadings = 0.0;

// timings
const double minutesBetweenCalls = 0.5;

const unsigned long millisecondsPerMinute = 60000;
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
  
  sensorVal = sensorVal * (FIVE_VOLTS / ANALOG_PIN_RANGE);
  
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
    Serial.print("Humidity sensor value: ");
    Serial.println(humidity);
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
    Serial.print("Average humidity sensor value: ");
    Serial.println(averageNonCompensatedHumidity);
    Serial.print("Average non-compensated humidity: ");
    Serial.println(averageRelativeHumidity);
    Serial.print("Temperature compensated humidity: ");
    Serial.println(averageTemperatureCompensatedHumidity);
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
    Serial.print("temperature: ");
    Serial.println(temperature);
  }
  
  temperatureReadings = temperatureReadings + temperature;
}

double calculateInstantTemperature() {
  double voltage = read5VAnalogPin(temperatureSensorPin);
  double temperature = ((voltage - 0.5) * 100.0); // NEEDS CHECKING AS CANNOT FIND ANY DOCUMENTATION ON THIS FIGURE...


  return temperature;
}

double calculateAverageTemperature() {
  double averageTemperature = temperatureReadings/(double)loopCounter;

  if(debug) {
    Serial.print("Average Temperature: ");
    Serial.println(averageTemperature);
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
    Serial.print("Uncompensated pressure: ");
    Serial.println(uncompensatedPressure);
  }
  
  pressureReadings = pressureReadings + uncompensatedPressure;
}

/**
 * From the pressure sensor document:
 * Vout = Vsupply * ((Pressure * 0.004) - 0.04) +- (Pressure Error * Temperature Factor * 0.004 x Vsupply)
 *
 * Calculate: Vout = Vsupply * ((Pressure * 0.004) - 0.04) using getErrorFactor(averageTemperature) for the error factor portion of the equation.
 *
 *
 *
 */
double calculateAverageTemperatureCompensatedPressure(double averageTemperature) {
  double averagePressure = (double)pressureReadings/(double)loopCounter;

  double calculatedPressure = ((averagePressure / FIVE_VOLTS) + 0.04) / 0.004;
  //pressure = (pressure - getErrorFactor(instantTemperature)) * kpaToMillibarsMultiplier;
  calculatedPressure = calculatedPressure * KPA_TO_MILLIBARS;
  
  if(debug) {
    Serial.print("Average pressure sensor reading: ");
    Serial.println(averagePressure);
    Serial.print("Calculated pressure: ");
    Serial.println(calculatedPressure);
  }

  return calculatedPressure;
}

void resetAveragePressure() {
  pressureReadings = ZERO_DOUBLE;
}

/**
 * NOT IN USE
 *
 * Because the document is too vague on the +- values, cannot reliably use this error factor...
 *
 *
 * From the pressure sensor document:
 * Vout = Vsupply * ((Pressure * 0.004) - 0.04) +- (Pressure Error * Temperature Factor * 0.004 x Vsupply)
 *
 * This method calculates the Error Factor, which is: (Pressure Error * Temperature Factor * 0.004 x Vsupply).
 *
 * From the manual, the Pressure Error is a fixed +-3.45 kPa between 20 and 250kPa
 */
double getErrorFactor(double instantTemperature) {
    double temperatureCompensation = PRESSURE_TEMPERATURE_ERROR_FACTOR_NORMAL;
    if( instantTemperature < 0.0 || instantTemperature > 85.0)
    {
    double difference = 0.0;
      if(instantTemperature > 0.0)  {
          difference = instantTemperature - 85.0;
      } else {
      // invert the negative number
          difference = instantTemperature - (instantTemperature * 2.0);
      }
    
      temperatureCompensation = (difference * (2.0/40.0)) + 1.0;
    }
  
    return (3.45 * temperatureCompensation * 0.004 * FIVE_VOLTS);
}

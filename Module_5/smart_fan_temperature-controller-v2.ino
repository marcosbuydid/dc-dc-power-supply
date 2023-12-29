/*
   Created on March 2022
   by Marcos A.Buydid.

   v2 modified on October 2022 includes
   support for fahrenheit sensor
   on A1 and some code refactor.
*/

int interruptPin = 2;
int fan_pwm_pin = 9;
int buzzerPin = 12;
int A0Pin = A0;
int A1Pin = A1;
int D4Pin = 4;
int D6Pin = 6;

int celsiusTemperatureSensorOneAdcValue = 0;
int fahrenheitTemperatureSensorTwoAdcValue = 0;

float celsiusTemperatureSensorOneVoltageOutput = 0.0;
float fahrenheitTemperatureSensorTwoVoltageOutput = 0.0;

float celsiusTemperatureSensorOneOutput = 0.0;
float fahrenheitTemperatureSensorTwoOutput = 0.0;

float temperatureInsideEnclosure = 0.0;

//threeRuleValue = 1023/2.493 where 2.493 is EXTERNAL AREF value.
//1023 ADCValue = 2.493V.
const float threeRuleValue = 410.3;

const float conversionValue = 0.5555;
const float temperatureDifferenceThreshold = 2.00;

//LM34CZ adds 0.010V every 1 degree fahrenheit at its output
//LM35CZ adds 0.010V every 1 degree celsius at its output
//at 25 degree celsius for example its output is 0.25V
const float scaleValue = 0.010;

const int temperatureValues[] = {5, 6, 7, 8, 9, 10, 11, 12, 13, 14,
                                 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
                                 25, 26, 27, 28, 29, 30, 31, 32, 33, 34,
                                 35, 36, 37, 38, 39, 40, 41, 42, 43, 44,
                                 45, 46, 47, 48, 49, 50
                                };

const int minimumTemperature = 5;
const int maximumTemperature = 50;
const int middleTemperature = 27;

//this signal produces 2 pulses per revolution
volatile int fan_fg_signal_pulses = 0;

int timeInterval = 1000;
unsigned long elapsed_time;

int fanPwmValue = 0;
//int fan_rpm = 0;

String fan_hardware_test = "";

void setup() {

  TCCR1A = 0; //undo the configuration done by ...
  TCCR1B = 0; //... the arduino core library
  TCNT1 = 0; //reset timer
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11); //undo default timers
  TCCR1B = _BV(WGM13) | _BV(CS10); //for pins 9 and 10
  ICR1 = 320; //pwm will be from 0 to 320

  analogReference(EXTERNAL);
  pinMode(A0Pin, INPUT);
  pinMode(A1Pin, INPUT);
  pinMode(fan_pwm_pin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(D4Pin, OUTPUT);
  pinMode(D6Pin, OUTPUT);

  OCR1A = 0; //we set pwm to 0 out of 320 on pin 9.
  OCR1B = 0; //we set pwm to 0 out of 320 on pin 10.

  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(interruptPin), pulseCounter, RISING);

  fanDiagnostics();

  delay(1000);
}

void loop() {

  celsiusTemperatureSensorOneAdcValue = analogRead(A0Pin);
  celsiusTemperatureSensorOneAdcValue = analogRead(A0Pin);

  fahrenheitTemperatureSensorTwoAdcValue = analogRead(A1Pin);
  fahrenheitTemperatureSensorTwoAdcValue = analogRead(A1Pin);

  celsiusTemperatureSensorOneVoltageOutput = (celsiusTemperatureSensorOneAdcValue / threeRuleValue);
  fahrenheitTemperatureSensorTwoVoltageOutput = (fahrenheitTemperatureSensorTwoAdcValue / threeRuleValue);

  celsiusTemperatureSensorOneOutput = (celsiusTemperatureSensorOneVoltageOutput / scaleValue);
  fahrenheitTemperatureSensorTwoOutput = (fahrenheitTemperatureSensorTwoVoltageOutput / scaleValue);

  temperatureInsideEnclosure = temperatureFromSensors(celsiusTemperatureSensorOneOutput,
                               fahrenheitTemperatureSensorTwoOutput);

  Serial.print("Temperature Inside Enclosure: ");
  Serial.println(temperatureInsideEnclosure);

  fanPwmValue = temperatureToPwmValue (temperatureInsideEnclosure, temperatureValues);

  updatePwmValueOnPin(9, fanPwmValue);

  boardStatus(fanPwmValue);

  fanStatus(fan_fg_signal_pulses);
  fan_fg_signal_pulses = 0;

  delay(5000);
}

void fanDiagnostics() {
  //FAN MODEL QFR0812SH-CX13
  delay(100);
  for (int pwm = 128; pwm <= 128; pwm += 128) {
    OCR1A = pwm;
    delay(3000);
    unsigned long time_amount = millis();
    fan_fg_signal_pulses = 0;
    while ((millis() - time_amount) < timeInterval) {
      //we pause exactly 1000ms to calculate the amount of pulses in 1 sec
    }
  }
  //fan_rpm = (fan_fg_signal_pulses * 30);
  //with PWM lead wire unconnected, signal pulses are between 130-135.
  //in the diagnostics pulses are between 53-55, if we have less than 53 pulses or
  //more than 57, there's a problem on the board either with PWM signal, FG signal
  //or the fan is deffective.
  if ((fan_fg_signal_pulses > 52) && (fan_fg_signal_pulses < 57)) {
    fan_hardware_test = "PASSED";
  }
  else {
    fan_hardware_test = "FAILED";
  }
  OCR1A = 0;
  fan_fg_signal_pulses = 0;
  //fan_rpm = 0;
}

int temperatureToPwmValue (float temperatureReading, int temperatureRanges[]) {
  if (inRange(temperatureReading, minimumTemperature, maximumTemperature)) {
    float lowerRangeDifferenceValue = 0.000;
    int pwmValue = 0;
    int index = 0;
    int indexJ = 0;
    int upperLimitRange = 0;
    //if the temperature reading is less than or equal the temperature value
    //of the element at the middle of the array, we only search for index = 0
    //to the upperLimitRange = 22 which is half size of the array - 1.
    //indexJ has the same value as index.
    if (temperatureReading <= middleTemperature) {
      upperLimitRange = (((sizeof(temperatureValues) / 2) - 1) / 2 - 1);
    }
    else {
      //if the temperature reading is greater than or equal the temperature value
      //of the element at the middle of the array, we only search from index = 22
      //to the upperLimitRange = 46 which is the size opf the array - 1.
      //indexJ has the same value as index
      index = (((sizeof(temperatureValues) / 2) - 1) / 2 - 1);
      indexJ = (((sizeof(temperatureValues) / 2) - 1) / 2 - 1);
      upperLimitRange = ((sizeof(temperatureValues) / 2) - 1);
    }
    for (int indexI = index; indexI < upperLimitRange; indexI++) {
      int indexITemperature = temperatureRanges[indexI];
      if (temperatureReading >= indexITemperature) {
        indexJ = indexJ + 1;
        int indexJTemperature = temperatureRanges[indexJ];
        if (temperatureReading <= indexJTemperature) {
          lowerRangeDifferenceValue = ((temperatureReading / 100) - (indexITemperature / 100));
          if (lowerRangeDifferenceValue <= 0.005) {
            int pwmValueI = (7 * indexI) + 5;
            pwmValue = pwmValueI;
          }
          else {
            int pwmValueJ = (7 * indexJ) + 5;
            pwmValue = pwmValueJ;
          }
          return pwmValue;
        }
      }
    }
  }
  //this value was random selected.
  //if the function returns this value is either because the pwmValue was not calculated
  //correctly, or there is a temperature reading out of bounds.
  return 15797;
}

void updatePwmValueOnPin(int pinNumber, int value) {
  if (value != 15797) {
    if (pinNumber == 9) {
      OCR1A = value;
    }
  }
}

bool inRange(float reading, int minimumValue, int maximumValue) {
  int temperature = reading;
  return ((minimumValue <= reading) && (reading <= maximumValue));
}

void pulseCounter() {
  fan_fg_signal_pulses++;
}

void boardStatus(int value) {
  if ((value != 15797) && (fan_hardware_test.equals("FAILED"))) {
    digitalWrite(D4Pin, LOW);
    digitalWrite(D6Pin, HIGH);
    //faultAlarm();
    Serial.println("Fan Hardware Test: NOT PASSED");
    Serial.println("Temperature Sensors: WORKING");
  }
  else if ((value == 15797) && (fan_hardware_test.equals("FAILED"))) {
    digitalWrite(D4Pin, HIGH);
    digitalWrite(D6Pin, HIGH);
    faultAlarm();
    Serial.println("Fan Hardware Test: NOT PASSED");
    Serial.println("Temperature Sensors: ERROR DETECTED");
  }
  else if ((value == 15797) && (fan_hardware_test.equals("PASSED"))) {
    digitalWrite(D4Pin, HIGH);
    digitalWrite(D6Pin, LOW);
    faultAlarm();
    Serial.println("Fan Hardware Test: PASSED");
    Serial.println("Temperature Sensors: ERROR DETECTED");
  }
  else if ((value != 15797) && (fan_hardware_test.equals("PASSED"))) {
    digitalWrite(D4Pin, LOW);
    digitalWrite(D6Pin, LOW);
    Serial.println("Fan Hardware Test: PASSED");
    Serial.println("Temperature Sensors: WORKING");
  }
}

void fanStatus(int pulses) {
  if (pulses == 0) {
    digitalWrite(D6Pin, HIGH);
    faultAlarm();
    Serial.println("Fan Status: NOT SPINNING");
  }
}

void faultAlarm() {
  tone(buzzerPin, 652);
  delay(500);
  noTone(buzzerPin);
  delay(50);
  tone(buzzerPin, 652);
  delay(500);
  noTone(buzzerPin);
  delay(50);
}

float temperatureFromSensors (float sensorOneReading, float sensorTwoReading) {

  //we set this random temperature because if the readings not match the if conditions
  //then the temperature will be detected out of bounds as we want for control.
  float outputTemperature = -99.14;

  float sensorTwoTemperatureInCelsius = fahrenheitToCelsiusConversion(sensorTwoReading);

  if ((sensorOneReading - sensorTwoTemperatureInCelsius > 0) &&
      (sensorOneReading - sensorTwoTemperatureInCelsius <= temperatureDifferenceThreshold)) {
    outputTemperature = sensorTwoTemperatureInCelsius +
                        (sensorOneReading - sensorTwoTemperatureInCelsius);
  }

  if ((sensorTwoTemperatureInCelsius - sensorOneReading > 0) &&
      (sensorTwoTemperatureInCelsius - sensorOneReading <= temperatureDifferenceThreshold)) {
    outputTemperature = sensorOneReading +
                        (sensorTwoTemperatureInCelsius - sensorOneReading);
  }
  return outputTemperature;
}

float fahrenheitToCelsiusConversion(float temperatureReading) {
  return ((temperatureReading - 32) * conversionValue);
}

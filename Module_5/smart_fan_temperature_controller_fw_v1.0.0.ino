// ==============================================================================================
// Smart Fan Temperature Controller
// Copyright (c) 2022-2026 Marcos Buydid. All rights reserved.
// Unauthorized use, reproduction, or distribution of this software, in whole or in part,
// is strictly prohibited without prior written permission from the copyright holder.
// ==============================================================================================

#include <avr/wdt.h>

// Set 0 for production, 1 for debug.
#define DEBUG_FLAG 0

// -----Pin Definitions -------------------------------------------------------------------------

const uint8_t FAN_FG_INTERRUPT_PIN = 2;
const uint8_t FAN_PWM_PIN = 9;
const uint8_t BUZZER_PIN = 12;
const uint8_t CELSIUS_TEMPERATURE_SENSOR_PIN = A0;
const uint8_t FAHRENHEIT_TEMPERATURE_SENSOR_PIN = A1;
const uint8_t LED_BOARD_STATUS_PIN = 4;
const uint8_t LED_FAN_STATUS_PIN = 6;

// -----ADC & Sensor Constants ------------------------------------------------------------------

// ADC full scale (1023) / external AREF voltage (2.493V)
const float ADC_TO_VOLTAGE_FACTOR = 410.3f;

// LM35CZ and LM34CZ produce 10mV per degree of their scale
// At 25 degree celsius LM35CZ output is 0.25V.
const float SENSOR_MV_PER_DEGREE = 0.010f;

// Fahrenheit-to-Celsius conversion factor (5.0/9.0)
const float FAHRENHEIT_TO_CELSIUS_CONVERSION_FACTOR = 0.5555f;

// -----Temperature Operating Range Inside Enclosure---------------------------------------------

const int CELSIUS_MINIMUM_TEMPERATURE = 5;
const int CELSIUS_MAXIMUM_TEMPERATURE = 50;

// Sentinel returned when temperature read is out of range
const float TEMPERATURE_OUT_OF_RANGE = -99.0f;

// Sensors should agree within 5°C
const float MAX_SENSOR_DIVERGENCE_CELSIUS = 5.0f;

// Sentinel returned when readings between sensors differ
// on more than 5°C
const float SENSOR_READING_DIVERGENCE_ERROR = -97.0f;

// -----PWM Configuration------------------------------------------------------------------------

// Timer1 is configured for phase/frequency-correct PWM with ICR1 = 320 (~25 kHz at 16 MHz)
const int PWM_TOP = 320;

const int PWM_AT_MINIMUM_TEMPERATURE = 5;

// Linear step per degree Celsius: (320 - 5) / (50 - 5) ≈ 7
const int PWM_STEP_PER_DEGREE = 7;

// Sentinel returned if temperature is out of range when mapping temperature to PWM
const int TEMPERATURE_PWM_ERROR = -1;

// Sentinel returned if divergence in reading between sensors is detected
const int TEMPERATURE_DIVERGENCE_ERROR = -2;

// -----Fan Test Constants-----------------------------------------------------------------------

//Selected PWM value to use in fan test
const int FAN_TEST_PWM = 128;

const int FAN_TEST_MINIMUM_PULSES = 52;
const int FAN_TEST_MAXIMUM_PULSES = 57;

// Time delay before start the test
const unsigned long FAN_TEST_INITIAL_DELAY_MS = 300UL;

// Time given for the fan to reach stable RPM
const unsigned long FAN_TEST_SPINUP_MS = 3000UL;

// Counting window for FG pulses
const unsigned long FAN_TEST_COUNT_WINDOW_MS = 1000UL;

// -----Buzzer Constants-------------------------------------------------------------------------

const int BUZZER_FREQUENCY_HZ = 652;
const unsigned long BUZZER_BEEP_ON_MS = 500UL;
const unsigned long BUZZER_BEEP_OFF_MS = 50UL;

// -----Setup Interval---------------------------------------------------------------------------

const unsigned long SETUP_DELAY_MS = 1000UL;

// -----Main Loop Interval-----------------------------------------------------------------------

const unsigned long LOOP_DELAY_MS = 5000UL;

// -----Sustained High PWM Detection-------------------------------------------------------------

// PWM value considered as maximum fan speed
const int MAX_PWM_THRESHOLD = PWM_TOP;

// Consecutive loop cycles at maximum PWM before trigger the alarm
const int MAX_HIGH_PWM_CYCLES = 2;

// -----Global State-----------------------------------------------------------------------------

// Incremented by ISR on every RISING edge of the FG signal.
// Declared as volatile because it is written in an ISR and read in the main loop.
volatile int fanFGSignalPulses = 0;

bool fanTestPassed = false;

int consecutiveMaxPwmCycles = 0;

// -----ISR--------------------------------------------------------------------------------------

// Counts each rising edge on the FG line.
// The fan produces 2 pulses per revolution.
void onFgPulse() {
  fanFGSignalPulses++;
}

void setupTimer1_25kHz() {

  // Configure Timer1 for phase/frequency-correct PWM at ~25kHz.
  // WGM13 + WGM11: mode 10 (phase/freq correct, TOP = ICR1)
  // COM1A1: non-inverting output on OC1A (pin 9)
  // COM1B1: non-inverting output on OC1B (pin 10, unused)
  // CS10: no prescaler -> f_PWM = f_CPU / (2 × ICR1) = 16MHz/(2×320) ≈ 25kHz

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = PWM_TOP;
  OCR1A = 0;  //fan off at startup
  OCR1B = 0;
}

void configurePins() {
  pinMode(CELSIUS_TEMPERATURE_SENSOR_PIN, INPUT);
  pinMode(FAHRENHEIT_TEMPERATURE_SENSOR_PIN, INPUT);
  pinMode(FAN_PWM_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_BOARD_STATUS_PIN, OUTPUT);
  pinMode(LED_FAN_STATUS_PIN, OUTPUT);
}

void setup() {

  MCUSR = 0; // clear the reset flags, including WDRF
  wdt_disable(); // the watchdog now can actually be turned off

#if DEBUG_FLAG
  Serial.begin(9600);
#endif

  setupTimer1_25kHz();

  analogReference(EXTERNAL);

  configurePins();

  attachInterrupt(digitalPinToInterrupt(FAN_FG_INTERRUPT_PIN), onFgPulse, RISING);

  performFanStartupTest();

  delay(SETUP_DELAY_MS);

  wdt_enable(WDTO_8S);

}

void loop() {

#if DEBUG_FLAG
  Serial.println(F("----- cycle -----"));
#endif

  wdt_reset();

  float celsiusSensorTemperature = readTemperatureFromCelsiusSensor();
  float fahrenheitSensorTemperature = readTemperatureFromFahrenheitSensor();

  float averageTemperature = calculateAverageTemperatureFromSensors(celsiusSensorTemperature,
                             fahrenheitSensorTemperature);

  int fanPWMValue = temperatureToPWM(averageTemperature);
  updateFanPWM(fanPWMValue);

  monitorPwmCycles(fanPWMValue);

  // Atomically copy and reset the pulse counter.
  // fanFGSignalPulses is 16-bit on AVR, without disabling interrupts, the ISR
  // could fire between the two bytes of the read/write, corrupting the value.
  noInterrupts();
  int pulses = fanFGSignalPulses;
  fanFGSignalPulses = 0;
  interrupts();

  monitorBoardHardware(fanPWMValue, pulses);

#if DEBUG_FLAG
  Serial.print(F("Average Temperature: "));
  Serial.println(averageTemperature);
  Serial.print(F("Fan PWM: "));
  Serial.println(fanPWMValue);
  Serial.print(F("OCR1A applied: "));
  Serial.println(OCR1A);
#endif

  delay(LOOP_DELAY_MS);

}

void performFanStartupTest() {
  //FAN MODEL: DELTA QFR0812SH-CX13
  delay(FAN_TEST_INITIAL_DELAY_MS);
  OCR1A = FAN_TEST_PWM;
  delay(FAN_TEST_SPINUP_MS);

  noInterrupts();
  fanFGSignalPulses = 0;
  interrupts();

  unsigned long countWindowStart = millis();
  while ((millis() - countWindowStart) < FAN_TEST_COUNT_WINDOW_MS) {
    //pause exactly FAN_TEST_COUNT_WINDOW_MS to calculate the amount of pulses
  }

  noInterrupts();
  int detectedFGPulses = fanFGSignalPulses;
  fanFGSignalPulses = 0;
  interrupts();

  OCR1A = 0; //fan off after

  // Fan RPM = fg_signal_pulses * 30.
  // With PWM lead wire unconnected, signal pulses are between 130-135.
  // During the test with FAN_TEST_PWM value, fan pulses are between 53-55. If we have less than 53
  // or more than 56, there's a problem either on the board with the PWM signal, FG signal or the
  // fan is deffective.
  fanTestPassed = (detectedFGPulses > FAN_TEST_MINIMUM_PULSES) &&
                  (detectedFGPulses < FAN_TEST_MAXIMUM_PULSES);

#if DEBUG_FLAG
  Serial.print(F("Detected FGPulses: "));
  Serial.println(detectedFGPulses);
  Serial.print(F("Test Result: "));
  Serial.println(fanTestPassed ? F("PASSED") : F("NOT PASSED"));
#endif
}

float readTemperatureFromCelsiusSensor() {
  analogRead(CELSIUS_TEMPERATURE_SENSOR_PIN); //first read is discarded
  int adcValue = analogRead(CELSIUS_TEMPERATURE_SENSOR_PIN);
  float voltage = adcValue / ADC_TO_VOLTAGE_FACTOR;
  return voltage / SENSOR_MV_PER_DEGREE;
}

float readTemperatureFromFahrenheitSensor() {
  analogRead(FAHRENHEIT_TEMPERATURE_SENSOR_PIN); //first read is discarded
  int adcValue = analogRead(FAHRENHEIT_TEMPERATURE_SENSOR_PIN);
  float voltage = adcValue / ADC_TO_VOLTAGE_FACTOR;
  return voltage / SENSOR_MV_PER_DEGREE;
}

float calculateAverageTemperatureFromSensors(float celsiusSensorReading,
    float fahrenheitSensorReading) {

  float fahrenheitReadingToCelsius = fahrenheitToCelsius(fahrenheitSensorReading);

  bool celsiusReadingValid = inRange(celsiusSensorReading,
                                     CELSIUS_MINIMUM_TEMPERATURE, CELSIUS_MAXIMUM_TEMPERATURE);
  bool fahrenheitReadingValid = inRange(fahrenheitReadingToCelsius,
                                        CELSIUS_MINIMUM_TEMPERATURE, CELSIUS_MAXIMUM_TEMPERATURE);

  if (!celsiusReadingValid || !fahrenheitReadingValid) {
#if DEBUG_FLAG
    Serial.println(F("Temperature out of range"));
#endif
    return TEMPERATURE_OUT_OF_RANGE;
  }

  if (fabsf(celsiusSensorReading - fahrenheitReadingToCelsius) > MAX_SENSOR_DIVERGENCE_CELSIUS) {
#if DEBUG_FLAG
    Serial.println(F("Reading divergence between sensors"));
    Serial.print(F("Celsius sensor reading: "));
    Serial.println(celsiusSensorReading);
    Serial.print(F("Fahrenheit sensor (as C): "));
    Serial.println(fahrenheitReadingToCelsius);
#endif
    return SENSOR_READING_DIVERGENCE_ERROR;
  }

  return (celsiusSensorReading + fahrenheitReadingToCelsius) / 2.0f;
}

// Maps a temperature in Celsius to a Timer1 PWM compare value using a linear relationship:
// PWM = PWM_AT_MINIMUM_TEMPERATURE + PWM_STEP_PER_DEGREE ×
// (round(temperature) − CELSIUS_MINIMUM_TEMPERATURE)
int temperatureToPWM(float temperatureReading) {
  if (temperatureReading == SENSOR_READING_DIVERGENCE_ERROR) {
    return TEMPERATURE_DIVERGENCE_ERROR;
  }

  if (!inRange(temperatureReading, CELSIUS_MINIMUM_TEMPERATURE, CELSIUS_MAXIMUM_TEMPERATURE)) {
    return TEMPERATURE_PWM_ERROR;
  }

  int temperatureDegreeOffset = (int)ceil(temperatureReading) - CELSIUS_MINIMUM_TEMPERATURE;

  return PWM_AT_MINIMUM_TEMPERATURE + (PWM_STEP_PER_DEGREE * temperatureDegreeOffset);
}

void updateFanPWM(int pwmValue) {
  if (pwmValue >= 0 && pwmValue <= PWM_TOP) {
    OCR1A = pwmValue;
  }
}

float fahrenheitToCelsius(float temperatureReading) {
  return (temperatureReading - 32.0f) * FAHRENHEIT_TO_CELSIUS_CONVERSION_FACTOR;
}

bool inRange(float reading, int minimumValue, int maximumValue) {
  return (reading >= static_cast<float>(minimumValue)) &&
         (reading <= static_cast<float>(maximumValue));
}

void monitorBoardHardware(int pwmValue, int fanPulses) {
  bool fanTestFailed = !fanTestPassed;
  bool fanNotSpinning = (fanPulses == 0);
  bool pwmTemperatureError = (pwmValue == TEMPERATURE_PWM_ERROR);
  bool temperatureDivergenceError = (pwmValue == TEMPERATURE_DIVERGENCE_ERROR);
  bool sustainedHighPwm = isSustainedHighPwmDetected();

  digitalWrite(LED_FAN_STATUS_PIN, (fanTestFailed || fanNotSpinning) ? HIGH : LOW);
  digitalWrite(LED_BOARD_STATUS_PIN, (pwmTemperatureError || temperatureDivergenceError
                                      || sustainedHighPwm) ? HIGH : LOW);

  if (fanNotSpinning || sustainedHighPwm) {
#if DEBUG_FLAG
    Serial.println(F("Fan not spinning or sustained high pwm detected"));
#endif
    triggerFaultAlarm();
  }
}

void triggerFaultAlarm() {
  tone(BUZZER_PIN, BUZZER_FREQUENCY_HZ);
  delay(BUZZER_BEEP_ON_MS);
  noTone(BUZZER_PIN);
  delay(BUZZER_BEEP_OFF_MS);

  tone(BUZZER_PIN, BUZZER_FREQUENCY_HZ);
  delay(BUZZER_BEEP_ON_MS);
  noTone(BUZZER_PIN);
}

void monitorPwmCycles(int pwmValue) {
  if (pwmValue >= MAX_PWM_THRESHOLD) {
    if (consecutiveMaxPwmCycles < MAX_HIGH_PWM_CYCLES) {
      consecutiveMaxPwmCycles++; // no overflow after ~55 hours
    }
  } else if (pwmValue >= 0) {
    consecutiveMaxPwmCycles = 0; // only a valid lower PWM resets the counter
  }
}

bool isSustainedHighPwmDetected() {
  return consecutiveMaxPwmCycles >= MAX_HIGH_PWM_CYCLES;
}

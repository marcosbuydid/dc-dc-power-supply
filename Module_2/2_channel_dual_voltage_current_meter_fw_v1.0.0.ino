// ==============================================================================================
// 2 Channel Dual Voltage-Current Meter
// Copyright (c) 2021-2026 Marcos Buydid. All rights reserved.
// Unauthorized use, reproduction, or distribution of this software, in whole or in part,
// is strictly prohibited without prior written permission from the copyright holder.
// ==============================================================================================

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>

#define FW_VERSION "1.0.0"

// -----Build Options ---------------------------------------------------------------------------

// Set 0 for production, 1 for debug.
// In debug builds, raw ADC averages and results are printed.
#define DEBUG_FLAG 0

// -----Pin Definitions -------------------------------------------------------------------------

constexpr uint8_t LCD_RS_PIN = 7;
constexpr uint8_t LCD_EN_PIN = 8;
constexpr uint8_t LCD_D4_PIN = 9;
constexpr uint8_t LCD_D5_PIN = 10;
constexpr uint8_t LCD_D6_PIN = 11;
constexpr uint8_t LCD_D7_PIN = 12;

constexpr uint8_t CH1_VOLTAGE_PIN = A3;
constexpr uint8_t CH1_CURRENT_PIN = A2;
constexpr uint8_t CH2_VOLTAGE_PIN = A0;
constexpr uint8_t CH2_CURRENT_PIN = A1;
constexpr uint8_t VREF_TEMPERATURE_PIN = A6;

// Turn on while the channel shows more than 0.00 A or its current is over range ("OL").
constexpr uint8_t CH1_LOAD_STATUS_LED_PIN = 3;
constexpr uint8_t CH2_LOAD_STATUS_LED_PIN = 5;

constexpr uint8_t ONBOARD_LED_PIN = 13; // Nano on-board LED, not used

// Not used pins get the internal pull-up so they don't float.
// D0/D1 (serial) and A7 (analog-only, cannot be configured) are left out.
constexpr uint8_t UNUSED_PINS[] = { 2, 4, 6, A4, A5 };

// -----ADC acquisition -------------------------------------------------------------------------

// Calibrated counts-per-volt for the external AREF (MAX6250, 4.999 V).
// Nominal value is 1023 / 4.999 = 204.64, 204.7 is the calibrated value.
// All voltage and current calibration below was measured with 204.7.
constexpr float ADC_COUNTS_PER_VOLT = 204.7f;
constexpr float ADC_FULL_SCALE_COUNTS = 1023.0f;

// Reads thrown away after switching the ADC multiplexer to a new channel.
// Let the sample-and-hold capacitor settle to the new channel's voltage.
constexpr uint8_t ADC_SETTLING_READS = 2;

// Samples averaged per measurement. 64 samples take about 7.5 ms, which
// covers about 1,500 switching cycles at 200 kHz. Random noise drops by
// sqrt(64) = 8x.
constexpr uint8_t MEASUREMENT_OVERSAMPLE_COUNT  = 64;
constexpr uint8_t TEMPERATURE_OVERSAMPLE_COUNT  = 16;

constexpr uint8_t MAX_OVERSAMPLE_COUNT = 64;
static_assert(MEASUREMENT_OVERSAMPLE_COUNT <= MAX_OVERSAMPLE_COUNT, "Too many samples");
static_assert(TEMPERATURE_OVERSAMPLE_COUNT <= MAX_OVERSAMPLE_COUNT, "Too many samples");
static_assert(MEASUREMENT_OVERSAMPLE_COUNT >= 1 &&
              TEMPERATURE_OVERSAMPLE_COUNT >= 1, "At least one sample");

// Samples further than this from the median are treated as outliers and ignored.
// 8 counts = 39 mV at the ADC pin: about 43 mA on current, 158 mV on voltage.
// Normal noise (1-2 counts) is always far inside this limit.
// Set to 1023 to disable outlier rejection (plain average of all samples).
constexpr uint16_t OUTLIER_LIMIT_COUNTS = 8;

// Maximum random delay between ADC samples: 0 to 7 us (see generateSampleJitterUs).
constexpr uint8_t SAMPLE_JITTER_MASK_US = 0x07;

// -----Voltage calibration ----------------------------------------------------------------------

// AD8605 operational amplifier gain (nominal 4.03). The extra 0.006 comes from PCB traces
// and amplifier's gain error.
constexpr float OP_AMP_GAIN = 4.036f;

// Voltage calibration: displayed = measured x VOLTAGE_CAL_GAIN + VOLTAGE_CAL_OFFSET_MV.
// Set GAIN = 1.0 and OFFSET = 0.0 for no compensation.
constexpr float VOLTAGE_CAL_GAIN = 0.99578f;
constexpr float VOLTAGE_CAL_OFFSET_MV = 31.0f;

// Displayed voltage is rounded to the nearest 10 mV starting at 0.790 V.
constexpr float VOLTAGE_MIN_MV = 790.0f;
constexpr float VOLTAGE_STEP_MV = 10.0f;

// A new 10 mV step is only accepted once the reading is this far past the
// step boundary. This stops the last digit toggling on a boundary.
constexpr float VOLTAGE_HYSTERESIS_MV = 3.0f;

// -----Current calibration ----------------------------------------------------------------------

// Each step in the current table is 0.01 A.
constexpr float CURRENT_STEP_A = 0.01f;

// A neighbouring 0.01 A step is only accepted once the current-sense voltage is
// this far past the midpoint between the two steps. 1.5 mV is about 0.3
// ADC counts, or about 1.7 mA.
constexpr float CURRENT_HYSTERESIS_MV = 1.5f;

// Lookup table of MAX9919NASA+ (precision current amplifier) output voltages in mV, 501 entries.
// Index N corresponds to N / 100 A: from 0.00 A (0 mV) to 5.00 A (4451 mV).
const uint16_t CURRENT_TO_VOLTAGE_TABLE_MV[] PROGMEM = {
  0, 12, 21, 30, 39, 48, 57, 66, 75, 84, 93, 102, 111,
  119, 128, 137, 146, 155, 164, 173, 182, 191, 201, 209, 218, 227,
  236, 245, 254, 263, 272, 281, 289, 298, 307, 316, 325, 334, 343,
  352, 361, 370, 379, 388, 397, 406, 415, 424, 433, 442, 450, 459,
  468, 477, 486, 495, 504, 513, 522, 531, 540, 549, 558, 567, 576,
  585, 594, 603, 612, 616, 625, 634, 643, 652, 661, 670, 679, 688,
  697, 706, 715, 724, 733, 742, 751, 760, 768, 777, 785, 794, 803,
  812, 821, 830, 839, 848, 857, 866, 875, 884, 893, 902, 911, 920,
  929, 938, 946, 955, 964, 973, 982, 991, 1000, 1009, 1018, 1027, 1036,
  1044, 1053, 1062, 1071, 1080, 1089, 1098, 1107, 1115, 1124, 1133, 1142, 1151,
  1160, 1169, 1178, 1187, 1196, 1205, 1214, 1223, 1231, 1240, 1249, 1258, 1267,
  1275, 1284, 1293, 1302, 1311, 1320, 1329, 1338, 1347, 1356, 1365, 1374, 1383,
  1392, 1401, 1410, 1418, 1427, 1436, 1444, 1453, 1462, 1471, 1480, 1489, 1498,
  1507, 1516, 1525, 1534, 1543, 1552, 1561, 1570, 1579, 1588, 1597, 1605, 1614,
  1623, 1632, 1641, 1650, 1659, 1668, 1677, 1686, 1695, 1703, 1712, 1721, 1730,
  1739, 1748, 1757, 1766, 1774, 1783, 1792, 1801, 1810, 1819, 1828, 1837, 1846,
  1855, 1864, 1873, 1882, 1890, 1899, 1908, 1917, 1926, 1935, 1943, 1952, 1961,
  1970, 1979, 1988, 1997, 2006, 2015, 2024, 2033, 2042, 2051, 2060, 2069, 2076,
  2085, 2094, 2102, 2111, 2120, 2129, 2138, 2147, 2156, 2165, 2174, 2183, 2192,
  2201, 2210, 2219, 2228, 2237, 2246, 2255, 2264, 2272, 2281, 2290, 2299, 2308,
  2317, 2326, 2335, 2344, 2353, 2362, 2371, 2380, 2389, 2398, 2407, 2416, 2425,
  2433, 2442, 2451, 2460, 2469, 2478, 2487, 2496, 2505, 2514, 2523, 2532, 2538,
  2547, 2556, 2565, 2574, 2583, 2592, 2600, 2609, 2618, 2627, 2636, 2645, 2654,
  2663, 2672, 2681, 2690, 2699, 2708, 2717, 2726, 2735, 2744, 2753, 2762, 2770,
  2779, 2788, 2797, 2806, 2815, 2824, 2833, 2842, 2851, 2860, 2869, 2878, 2887,
  2896, 2905, 2911, 2920, 2928, 2937, 2946, 2955, 2964, 2973, 2982, 2991, 3000,
  3009, 3018, 3027, 3036, 3045, 3054, 3063, 3072, 3081, 3090, 3098, 3107, 3116,
  3125, 3134, 3143, 3152, 3161, 3170, 3179, 3188, 3197, 3206, 3215, 3224, 3233,
  3242, 3251, 3259, 3268, 3277, 3286, 3295, 3304, 3313, 3322, 3331, 3340, 3349,
  3358, 3367, 3376, 3385, 3394, 3403, 3412, 3421, 3429, 3438, 3447, 3456, 3465,
  3474, 3483, 3492, 3501, 3510, 3519, 3528, 3537, 3546, 3555, 3564, 3571, 3580,
  3588, 3597, 3606, 3615, 3624, 3633, 3642, 3651, 3660, 3669, 3678, 3687, 3696,
  3705, 3714, 3723, 3732, 3741, 3750, 3758, 3767, 3776, 3785, 3794, 3803, 3812,
  3821, 3830, 3839, 3848, 3857, 3866, 3875, 3884, 3893, 3902, 3911, 3920, 3928,
  3937, 3946, 3955, 3964, 3973, 3982, 3991, 4000, 4009, 4018, 4027, 4034, 4043,
  4052, 4061, 4070, 4079, 4087, 4096, 4105, 4114, 4123, 4132, 4141, 4150, 4159,
  4168, 4177, 4186, 4195, 4204, 4213, 4222, 4231, 4240, 4249, 4257, 4266, 4275,
  4284, 4293, 4302, 4311, 4320, 4329, 4338, 4347, 4356, 4365, 4374, 4383, 4392,
  4401, 4407, 4415, 4424, 4433, 4442, 4451
};

constexpr uint16_t CURRENT_TO_VOLTAGE_TABLE_SIZE =
  sizeof(CURRENT_TO_VOLTAGE_TABLE_MV) / sizeof(CURRENT_TO_VOLTAGE_TABLE_MV[0]);
constexpr uint16_t CURRENT_TO_VOLTAGE_TABLE_LAST_INDEX = CURRENT_TO_VOLTAGE_TABLE_SIZE - 1;
static_assert(CURRENT_TO_VOLTAGE_TABLE_SIZE == 501, "Current to voltage table must have 501 entries");

// -----REF02AP voltage reference temperature protection -----------------------------------------

// The TEMP pin outputs about 0.630 V at 25°C and about 0.690 V at 60°C.
// The IC is rated to 85°C. Alert threshold is set at 0.680 V (about 54°C) and turns off
// below 0.670 V (about 48°C).
constexpr float VREF_TEMPERATURE_ALERT_ON_V = 0.680f;
constexpr float VREF_TEMPERATURE_ALERT_OFF_V = 0.670f;

static_assert(VREF_TEMPERATURE_ALERT_OFF_V < VREF_TEMPERATURE_ALERT_ON_V,
              "Temperature alert OFF threshold must be below the ON threshold");

// -----Timing and display -----------------------------------------------------------------------

constexpr uint16_t MEASUREMENT_PERIOD_MS = 560;
constexpr uint16_t SPLASH_SCREEN_DURATION_MS = 2000;
constexpr uint8_t  LCD_COLUMNS = 16;
constexpr uint8_t  LCD_ROWS = 2;

constexpr int16_t NO_STEP = -1;   // "no previous step": disables hysteresis

// -----Types ------------------------------------------------------------------------------------

enum class ReadingStatus : uint8_t {
  Valid,
  OverRange
};

struct Reading {
  float value;
  ReadingStatus status;
};

struct Channel {
  uint8_t number;
  uint8_t voltagePin;
  uint8_t currentPin;
  uint8_t ledPin;
  uint8_t lcdRow;
  int16_t lastVoltageStep;
  int16_t lastCurrentStep;
};

Channel channels[] = {
  { 1, CH1_VOLTAGE_PIN, CH1_CURRENT_PIN, CH1_LOAD_STATUS_LED_PIN, 1, NO_STEP, NO_STEP },
  { 2, CH2_VOLTAGE_PIN, CH2_CURRENT_PIN, CH2_LOAD_STATUS_LED_PIN, 0, NO_STEP, NO_STEP },
};
constexpr uint8_t CHANNEL_COUNT = sizeof(channels) / sizeof(channels[0]);

// -----Globals ------------------------------------------------------------------------------------

LiquidCrystal lcd(LCD_RS_PIN, LCD_EN_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

uint32_t lastMeasurementMs = 0;
bool     vRefTemperatureAlertActive = false;

#if DEBUG_FLAG
uint32_t debugCycleCount = 0;   // measurement cycles since reset; printed on every debug line
#endif

// Reset cause, captured before main() runs (see captureResetFlagsAndDisableWatchdog).
uint8_t resetFlags __attribute__((section(".noinit")));

// -----Function signatures ------------------------------------------------------------------------

void captureResetFlagsAndDisableWatchdog() __attribute__((naked, used, section(".init3")));
uint8_t generateSampleJitterUs();
float readAdcAverage(uint8_t pin, uint8_t sampleCount);
bool isAdjacentStep(int16_t newStep, int16_t lastStep);
Reading computeVoltage(float adcAverage, int16_t &lastStep);
float getVoltageFromTableMv(uint16_t index);
int16_t findNearestCurrentStep(float currentSenseMv);
Reading computeCurrent(float adcAverage, int16_t &lastStep);
bool shouldVRefTemperatureAlertBeActive(bool alertActive, float vRefTemperatureVoltage);
void calculateAndDisplayChannelMeasurements(Channel &channel);
void performMeasurementsOnChannels();
void updateVRefTemperatureAlertState();
void resetChannelsHysteresis();
void formatChannelRow(char *row, const Reading &voltage, const Reading &current);
void showSplashScreen();
void showVRefTemperatureAlert();
#if DEBUG_FLAG
void printTemperatureDebugOutput(float vRefTemperatureVoltage);
void printChannelDebugOutput(const Channel &channel, float voltageAdc, float currentAdc,
                             const Reading &voltage, const Reading &current);
#endif

// -----Early startup ------------------------------------------------------------------------------

// Runs before main(), in the .init3 startup section. After a watchdog
// reset, the watchdog stays enabled with a 16 ms timeout. It must be
// disabled here, before the 2 s splash screen, or the board resets again.
// Optiboot may clear MCUSR before this code runs; the reset cause then
// reads 0, but the watchdog is still disabled safely.
void captureResetFlagsAndDisableWatchdog() {
  resetFlags = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

// -----Setup --------------------------------------------------------------------------------------

void setup() {

  analogReference(EXTERNAL);

  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    pinMode(channels[i].voltagePin, INPUT);
    pinMode(channels[i].currentPin, INPUT);
  }

  // Turn off the digital input buffers on the analog pins in use. This
  // lowers ADC noise (ATmega328P datasheet, section 24.9.5).
  // A6 is analog-only and has no digital buffer.
  DIDR0 = _BV(ADC0D) | _BV(ADC1D) | _BV(ADC2D) | _BV(ADC3D);

  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    pinMode(channels[i].ledPin, OUTPUT);
    digitalWrite(channels[i].ledPin, LOW);
  }

  for (uint8_t pin : UNUSED_PINS) {
    pinMode(pin, INPUT_PULLUP);
  }

  pinMode(ONBOARD_LED_PIN, OUTPUT);
  digitalWrite(ONBOARD_LED_PIN, LOW);

#if DEBUG_FLAG
  Serial.begin(115200);
  Serial.print(F("FW " FW_VERSION " reset flags 0x"));
  Serial.println(resetFlags, HEX);
#endif

  lcd.begin(LCD_COLUMNS, LCD_ROWS);

  showSplashScreen();

  delay(SPLASH_SCREEN_DURATION_MS);

  lcd.clear();

  wdt_enable(WDTO_2S);
}

// -----Main loop ----------------------------------------------------------------------------------

void loop() {

  // Watchdog is fed once per pass. If anything below hangs, the loop
  // stops passing through here and the watchdog resets the MCU within 2 seconds.
  wdt_reset();

  const uint32_t now = millis();
  if (now - lastMeasurementMs < MEASUREMENT_PERIOD_MS) {
    return;
  }
  lastMeasurementMs = now;

  performMeasurementsOnChannels();
}

void showSplashScreen() {
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print(F("STARTING"));
  if (resetFlags & _BV(WDRF)) {
    lcd.setCursor(1, 1);
    lcd.print(F("WATCHDOG RESET"));
  } else {
    lcd.setCursor(3, 1);
    lcd.print(F("PLEASE WAIT"));
  }
}

void performMeasurementsOnChannels() {

#if DEBUG_FLAG
  debugCycleCount++;
#endif

  updateVRefTemperatureAlertState();

  if (vRefTemperatureAlertActive) {
    return;
  }

  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    calculateAndDisplayChannelMeasurements(channels[i]);
  }
}

void updateVRefTemperatureAlertState() {
  const float vRefTemperatureVoltage =
    readAdcAverage(VREF_TEMPERATURE_PIN, TEMPERATURE_OVERSAMPLE_COUNT) / ADC_COUNTS_PER_VOLT;

  const bool newAlertState =
    shouldVRefTemperatureAlertBeActive(vRefTemperatureAlertActive, vRefTemperatureVoltage);

  if (newAlertState != vRefTemperatureAlertActive) {
    vRefTemperatureAlertActive = newAlertState;

    // Measurements are paused during the alert, so the previous steps are stale.
    resetChannelsHysteresis();

    if (newAlertState) {
      showVRefTemperatureAlert();
    } else {
      lcd.clear();
    }
  }
#if DEBUG_FLAG
  printTemperatureDebugOutput(vRefTemperatureVoltage);
#endif
}

void resetChannelsHysteresis() {
  for (uint8_t i = 0; i < CHANNEL_COUNT; i++) {
    channels[i].lastVoltageStep = NO_STEP;
    channels[i].lastCurrentStep = NO_STEP;
  }
}

void calculateAndDisplayChannelMeasurements(Channel &channel) {
  const float voltageAdc = readAdcAverage(channel.voltagePin, MEASUREMENT_OVERSAMPLE_COUNT);
  const float currentAdc = readAdcAverage(channel.currentPin, MEASUREMENT_OVERSAMPLE_COUNT);

  const Reading voltage = computeVoltage(voltageAdc, channel.lastVoltageStep);
  const Reading current = computeCurrent(currentAdc, channel.lastCurrentStep);

  const bool loadActive = (current.status == ReadingStatus::OverRange) || (current.value > 0.0f);
  digitalWrite(channel.ledPin, loadActive ? HIGH : LOW);

  char row[LCD_COLUMNS + 1];
  formatChannelRow(row, voltage, current);
  lcd.setCursor(0, channel.lcdRow);
  lcd.print(row);

#if DEBUG_FLAG
  printChannelDebugOutput(channel, voltageAdc, currentAdc, voltage, current);
#endif
}

// Returns the average of sampleCount ADC conversions as a float, ignoring outliers.
// The samples are sorted to find the median. Samples further than
// OUTLIER_LIMIT_COUNTS from the median (for example a sample caught on a
// switching transient) are discarded, the rest are averaged. Normal noise is
// well inside the limit, so it is averaged exactly like a plain mean.
// Keeping the fraction gives resolution finer than one ADC count.
float readAdcAverage(uint8_t pin, uint8_t sampleCount) {
  for (uint8_t i = 0; i < ADC_SETTLING_READS; i++) {
    analogRead(pin);
  }

  if (sampleCount > MAX_OVERSAMPLE_COUNT) {
    sampleCount = MAX_OVERSAMPLE_COUNT;
  }

  uint16_t samples[MAX_OVERSAMPLE_COUNT];
  for (uint8_t i = 0; i < sampleCount; i++) {
    samples[i] = analogRead(pin);
    delayMicroseconds(generateSampleJitterUs());
  }

  // Insertion sort: fast enough for 64 values.
  for (uint8_t i = 1; i < sampleCount; i++) {
    const uint16_t value = samples[i];
    uint8_t j = i;
    while (j > 0 && samples[j - 1] > value) {
      samples[j] = samples[j - 1];
      j--;
    }
    samples[j] = value;
  }

  const uint16_t median = samples[sampleCount / 2];
  uint32_t sum = 0;
  uint8_t kept = 0;
  for (uint8_t i = 0; i < sampleCount; i++) {
    const uint16_t distance = (samples[i] > median) ? samples[i] - median : median - samples[i];
    if (distance <= OUTLIER_LIMIT_COUNTS) {
      sum += samples[i];
      kept++;
    }
  }
  return (float)sum / kept;   // kept >= 1: the median itself is always kept
}

// Returns a pseudo-random number from 0 to 7. That number is used as a delay in microseconds
// between consecutive ADC samples.
// Needed to avoid phase-locked sampling.
// Both DC-DC converters in the design switch at 200 kHz, so the ripple repeats every 5 µs.
// One analogRead() takes about 112 µs. That means each sample lands 112 µs after the
// previous one, which is 22.4 ripple periods later.
// The key is where each sample falls inside the 5 µs ripple cycle. With a fixed 112 µs interval
// (112 ÷ 5 leaves a remainder of 2 µs).
// With 0–7 µs of random jitter added, each sample lands at an unpredictable point in the cycle.
// 7 µs is longer than one full 5 µs period, so every position in the cycle is equally likely
// and the 64 samples are spread evenly over the ripple waveform.
uint8_t generateSampleJitterUs() {
  static uint16_t state = 0xACE1;
  state ^= state << 7;
  state ^= state >> 9;
  state ^= state << 8;
  return state & SAMPLE_JITTER_MASK_US;
}

bool shouldVRefTemperatureAlertBeActive(bool alertActive, float vRefTemperatureVoltage) {
  if (alertActive) {
    return vRefTemperatureVoltage >= VREF_TEMPERATURE_ALERT_OFF_V;
  }
  return vRefTemperatureVoltage >= VREF_TEMPERATURE_ALERT_ON_V;
}

void showVRefTemperatureAlert() {
  lcd.clear();
  lcd.setCursor(4, 0);
  lcd.print(F("VREF-TEMP"));
  lcd.setCursor(6, 1);
  lcd.print(F("ALERT"));
}

// Converts an averaged voltage-channel ADC value into the displayed voltage:
// calibrated, rounded to the nearest 10 mV step, with hysteresis at step boundaries.
Reading computeVoltage(float adcAverage, int16_t &lastStep) {
  if (adcAverage >= ADC_FULL_SCALE_COUNTS - 0.5f) {
    lastStep = NO_STEP;   // ADC saturated: true voltage is unknown
    return { 0.0f, ReadingStatus::OverRange };
  }

  const float measuredMv = (adcAverage / ADC_COUNTS_PER_VOLT) * OP_AMP_GAIN * 1000.0f;
  const float channelVoltageMv = measuredMv * VOLTAGE_CAL_GAIN + VOLTAGE_CAL_OFFSET_MV;

  if (channelVoltageMv < VOLTAGE_MIN_MV) {
    lastStep = NO_STEP;
    return { 0.0f, ReadingStatus::Valid };
  }

  int16_t step = (int16_t)((channelVoltageMv + VOLTAGE_STEP_MV / 2.0f) / VOLTAGE_STEP_MV);

  if (isAdjacentStep(step, lastStep)) {
    // Boundary between two neighbouring steps: halfway between them.
    const int16_t upperStep = (step > lastStep) ? step : lastStep;
    const float boundaryMv = (upperStep - 0.5f) * VOLTAGE_STEP_MV;
    if (fabs(channelVoltageMv - boundaryMv) < VOLTAGE_HYSTERESIS_MV) {
      step = lastStep;
    }
  }
  lastStep = step;

  return { step * VOLTAGE_STEP_MV / 1000.0f, ReadingStatus::Valid };
}

int16_t findNearestCurrentStep(float currentSenseMv) {
  uint16_t low  = 0; // invariant: table[low] <= currentSenseMv
  uint16_t high = CURRENT_TO_VOLTAGE_TABLE_LAST_INDEX; // invariant: table[high] >= currentSenseMv

  while (high - low > 1) {
    const uint16_t mid = low + (high - low) / 2;
    if (getVoltageFromTableMv(mid) <= currentSenseMv) {
      low = mid;
    } else {
      high = mid;
    }
  }

  const float midpointMv = (getVoltageFromTableMv(low) + getVoltageFromTableMv(high)) * 0.5f;
  return (currentSenseMv < midpointMv) ? low : high;
}

// Converts an averaged current-channel ADC value into load current.
Reading computeCurrent(float adcAverage, int16_t &lastStep) {
  const float currentSenseMv = (adcAverage / ADC_COUNTS_PER_VOLT) * 1000.0f;

  if (currentSenseMv > getVoltageFromTableMv(CURRENT_TO_VOLTAGE_TABLE_LAST_INDEX)) {
    lastStep = NO_STEP; // above 5.00 A: outside the calibrated range
    return { 0.0f, ReadingStatus::OverRange };
  }

  int16_t step = findNearestCurrentStep(currentSenseMv);

  if (isAdjacentStep(step, lastStep)) {
    const int16_t lowerStep = (step < lastStep) ? step : lastStep;
    const float   boundaryMv = (getVoltageFromTableMv(lowerStep) +
                                getVoltageFromTableMv(lowerStep + 1)) * 0.5f;
    if (fabs(currentSenseMv - boundaryMv) < CURRENT_HYSTERESIS_MV) {
      step = lastStep;
    }
  }
  lastStep = step;

  return { step * CURRENT_STEP_A, ReadingStatus::Valid };
}

float getVoltageFromTableMv(uint16_t index) {
  return (float)pgm_read_word(&CURRENT_TO_VOLTAGE_TABLE_MV[index]);
}

// Builds one complete 16-character LCD row, so it is written in one pass
// and no old characters are left behind.
// Examples (16 columns):
// "12.030V  0.720A " normal
// "    OL   0.720A " voltage above ADC range
// "12.030V     OL  " current above 5.00 A
void formatChannelRow(char *row, const Reading &voltage, const Reading &current) {
  memset(row, ' ', LCD_COLUMNS);
  row[LCD_COLUMNS] = '\0';

  char field[8];

  if (voltage.status == ReadingStatus::OverRange) {
    memcpy_P(&row[0], PSTR("    OL"), 6);
  } else {
    dtostrf(voltage.value, 6, 3, field); // Example " 5.030" or "12.030"
    memcpy(&row[0], field, 6);
    row[6] = 'V';
  }

  if (current.status == ReadingStatus::OverRange) {
    memcpy_P(&row[9], PSTR("   OL"), 5);
  } else {
    dtostrf(current.value, 5, 3, field); // Example "0.720"
    memcpy(&row[9], field, 5);
    row[14] = 'A';
  }
}

bool isAdjacentStep(int16_t newStep, int16_t lastStep) {
  return (lastStep != NO_STEP) && (newStep - lastStep == 1 || lastStep - newStep == 1);
}

// Debug output example
// Each measurement cycle prints one temperature line
// followed by one line per channel.
// All lines start with the cycle number:
// #42 t=2104ms VREF_TEMP=0.645V
// #42 CH1 vAdc=611.00 iAdc=131.60 V=12.030 I=0.720
// #42 CH2 vAdc=254.00 iAdc=0.00 V=5.020 I=0.000
#if DEBUG_FLAG
void printTemperatureDebugOutput(float vRefTemperatureVoltage) {
  Serial.print('#');
  Serial.print(debugCycleCount);
  Serial.print(F(" t="));
  Serial.print(millis());
  Serial.print(F("ms VREF_TEMP="));
  Serial.print(vRefTemperatureVoltage, 3);
  Serial.println(vRefTemperatureAlertActive ? F("V ALERT") : F("V"));
}

void printChannelDebugOutput(const Channel &channel, float voltageAdc, float currentAdc,
                             const Reading &voltage, const Reading &current) {
  Serial.print('#');
  Serial.print(debugCycleCount);
  Serial.print(F(" CH"));
  Serial.print(channel.number);
  Serial.print(F(" vAdc=")); Serial.print(voltageAdc, 2);
  Serial.print(F(" iAdc=")); Serial.print(currentAdc, 2);

  Serial.print(F(" V="));
  if (voltage.status == ReadingStatus::OverRange) {
    Serial.print(F("OL"));
  } else {
    Serial.print(voltage.value, 3);
  }

  Serial.print(F(" I="));
  if (current.status == ReadingStatus::OverRange) {
    Serial.println(F("OL"));
  } else {
    Serial.println(current.value, 3);
  }
}
#endif

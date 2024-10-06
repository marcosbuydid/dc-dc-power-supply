
/*
   Created on Dec 2021 by Marcos A. Buydid.

   Jul 2023 version, change the way voltages
   are calculated on channels.
*/

#include <LiquidCrystal.h>

const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//threeRuleValue = 1023/4.999 where 4.999 is EXTERNAL AREF value.
//1023 ADCValue = 4.999V.
const float threeRuleValue = 204.7;

//operational amplifier gain is 4.03 on formula.
//real gain on op-amps is 4.036 due to internal traces resistance
//and gain error amplifier.
//voltage input on A0 and A3 is multiplied by 4.036.
const float operationalAmplifierGain = 4.036;

const float minimumVoltageBasedOnCurrent = 0.000;
const float maximumVoltageBasedOnCurrent = 4.451;
const float middleVoltageBasedOnCurrent = 2.228;

//IC6 (REF02AP) temperature voltage output is 0.630v at 25 degree celsius,
//and 0.690v when is about 60 degree celsius.
//the maximum temperature ic6 can operate is 85 degree celsius.
const float ic6TemperatureAlertThreshold = 0.680;

const float threshold = 0.800;

int A0Pin = A0;
int A1Pin = A1;
int A2Pin = A2;
int A3Pin = A3;
int A6Pin = A6;

int D3Pin = 3;
int D5Pin = 5;

int ch1ADCValue = 0;
int ch2ADCValue = 0;

float ch1VoltageReading = 0.0;
float ch2VoltageReading = 0.0;

float ch1VoltageOutput = 0.0;
float ch2VoltageOutput = 0.0;

int ch1ADCValueBasedOnCurrent = 0;
int ch2ADCValueBasedOnCurrent = 0;

float ch1VoltageBasedOnCurrentReading = 0.000;
float ch2VoltageBasedOnCurrentReading = 0.000;

float ch1CurrentOutput = 0.000;
float ch2CurrentOutput = 0.000;

int analog6ADCValue = 0;
float analog6VoltageInput = 0.0;

/*
  The array contains voltage values translated from 0 to 4.451A based on the current reading of the precision current amplifier MAX9919NASA+.
  In order to see in which voltage range a reading falls off, first we must divide each number value into 1000 while we move along the array.
  Once we get the range from a reading, the current value is given by the position of the lower or upper number of the range divided by 100
  depending on how close the reading value is of one or the other.
  Example: if reading is 0.008v, range would be 0.000-0.012. 0.008v is more near of 0.012 than 0.000 because 0.008 - 0.000 = 0.008
  and 0.012 - 0.008 = 0.004, (0.004 <= 0.005).
  Current value would be 0.01A (0.01 = position of 12 in the array divided by 100).
*/
const int voltageBasedOnCurrentValues[] = {0, 12, 21, 30, 39, 48, 57, 66, 75, 84, 93, 102, 111, 119, 128,
                                           137, 146, 155, 164, 173, 182, 191, 201, 209, 218, 227, 236, 245, 254, 263, 272, 281, 289, 298,
                                           307, 316, 325, 334, 343, 352, 361, 370, 379, 388, 397, 406, 415, 424, 433, 442, 450, 459, 468,
                                           477, 486, 495, 504, 513, 522, 531, 540, 549, 558, 567, 576, 585, 594, 603, 612, 616, 625, 634,
                                           643, 652, 661, 670, 679, 688, 697, 706, 715, 724, 733, 742, 751, 760, 768, 777, 785, 794, 803,
                                           812, 821, 830, 839, 848, 857, 866, 875, 884, 893, 902, 911, 920, 929, 938, 946, 955, 964, 973,
                                           982, 991, 1000, 1009, 1018, 1027, 1036, 1044, 1053, 1062, 1071, 1080, 1089, 1098, 1107, 1115,
                                           1124, 1133, 1142, 1151, 1160, 1169, 1178, 1187, 1196, 1205, 1214, 1223, 1231, 1240, 1249, 1258,
                                           1267, 1275, 1284, 1293, 1302, 1311, 1320, 1329, 1338, 1347, 1356, 1365, 1374, 1383, 1392, 1401,
                                           1410, 1418, 1427, 1436, 1444, 1453, 1462, 1471, 1480, 1489, 1498, 1507, 1516, 1525, 1534, 1543,
                                           1552, 1561, 1570, 1579, 1588, 1597, 1605, 1614, 1623, 1632, 1641, 1650, 1659, 1668, 1677, 1686,
                                           1695, 1703, 1712, 1721, 1730, 1739, 1748, 1757, 1766, 1774, 1783, 1792, 1801, 1810, 1819, 1828,
                                           1837, 1846, 1855, 1864, 1873, 1882, 1890, 1899, 1908, 1917, 1926, 1935, 1943, 1952, 1961, 1970,
                                           1979, 1988, 1997, 2006, 2015, 2024, 2033, 2042, 2051, 2060, 2069, 2076, 2085, 2094, 2102, 2111,
                                           2120, 2129, 2138, 2147, 2156, 2165, 2174, 2183, 2192, 2201, 2210, 2219, 2228, 2237, 2246, 2255,
                                           2264, 2272, 2281, 2290, 2299, 2308, 2317, 2326, 2335, 2344, 2353, 2362, 2371, 2380, 2389, 2398,
                                           2407, 2416, 2425, 2433, 2442, 2451, 2460, 2469, 2478, 2487, 2496, 2505, 2514, 2523, 2532, 2538,
                                           2547, 2556, 2565, 2574, 2583, 2592, 2600, 2609, 2618, 2627, 2636, 2645, 2654, 2663, 2672, 2681,
                                           2690, 2699, 2708, 2717, 2726, 2735, 2744, 2753, 2762, 2770, 2779, 2788, 2797, 2806, 2815, 2824,
                                           2833, 2842, 2851, 2860, 2869, 2878, 2887, 2896, 2905, 2911, 2920, 2928, 2937, 2946, 2955, 2964,
                                           2973, 2982, 2991, 3000, 3009, 3018, 3027, 3036, 3045, 3054, 3063, 3072, 3081, 3090, 3098, 3107,
                                           3116, 3125, 3134, 3143, 3152, 3161, 3170, 3179, 3188, 3197, 3206, 3215, 3224, 3233, 3242, 3251,
                                           3259, 3268, 3277, 3286, 3295, 3304, 3313, 3322, 3331, 3340, 3349, 3358, 3367, 3376, 3385, 3394,
                                           3403, 3412, 3421, 3429, 3438, 3447, 3456, 3465, 3474, 3483, 3492, 3501, 3510, 3519, 3528, 3537,
                                           3546, 3555, 3564, 3571, 3580, 3588, 3597, 3606, 3615, 3624, 3633, 3642, 3651, 3660, 3669, 3678,
                                           3687, 3696, 3705, 3714, 3723, 3732, 3741, 3750, 3758, 3767, 3776, 3785, 3794, 3803, 3812, 3821,
                                           3830, 3839, 3848, 3857, 3866, 3875, 3884, 3893, 3902, 3911, 3920, 3928, 3937, 3946, 3955, 3964,
                                           3973, 3982, 3991, 4000, 4009, 4018, 4027, 4034, 4043, 4052, 4061, 4070, 4079, 4087, 4096, 4105,
                                           4114, 4123, 4132, 4141, 4150, 4159, 4168, 4177, 4186, 4195, 4204, 4213, 4222, 4231, 4240, 4249,
                                           4257, 4266, 4275, 4284, 4293, 4302, 4311, 4320, 4329, 4338, 4347, 4356, 4365, 4374, 4383, 4392,
                                           4401, 4407, 4415, 4424, 4433, 4442, 4451
                                          };

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.setCursor(4, 0);
  lcd.print("STARTING");
  lcd.setCursor(3, 1);
  lcd.print("PLEASE WAIT");
  delay(2000);
  lcd.begin(16, 2);
  analogReference(EXTERNAL);
  pinMode(A0Pin, INPUT);
  pinMode(A1Pin, INPUT);
  pinMode(A2Pin, INPUT);
  pinMode(A3Pin, INPUT);
  pinMode(A6Pin, INPUT);
  pinMode(D3Pin, OUTPUT);
  pinMode(D5Pin, OUTPUT);
}

void loop() {

  delay(10);

  analog6ADCValue = analogRead(A6Pin);
  analog6ADCValue = analogRead(A6Pin);

  ch1ADCValue = analogRead(A3Pin);
  ch1ADCValue = analogRead(A3Pin);
  ch1ADCValueBasedOnCurrent = analogRead(A2Pin);
  ch1ADCValueBasedOnCurrent = analogRead(A2Pin);

  ch2ADCValue = analogRead(A0Pin);
  ch2ADCValue = analogRead(A0Pin);
  ch2ADCValueBasedOnCurrent = analogRead(A1Pin);
  ch2ADCValueBasedOnCurrent = analogRead(A1Pin);

  analog6VoltageInput = analog6ADCValue / threeRuleValue;

  if (!temperatureAboveThreshold(analog6VoltageInput, ic6TemperatureAlertThreshold)) {
    ch1VoltageReading = (ch1ADCValue / threeRuleValue) * operationalAmplifierGain;
    ch1VoltageOutput = voltageOnChannel(ch1VoltageReading);
    ch1VoltageBasedOnCurrentReading = ch1ADCValueBasedOnCurrent / threeRuleValue;
    ch1CurrentOutput = voltageToCurrent(ch1VoltageBasedOnCurrentReading, voltageBasedOnCurrentValues);

    ch2VoltageReading = (ch2ADCValue / threeRuleValue) * operationalAmplifierGain;
    ch2VoltageOutput = voltageOnChannel(ch2VoltageReading);
    ch2VoltageBasedOnCurrentReading = ch2ADCValueBasedOnCurrent / threeRuleValue;
    ch2CurrentOutput = voltageToCurrent(ch2VoltageBasedOnCurrentReading, voltageBasedOnCurrentValues);

    ledStatus(ch1CurrentOutput, ch2CurrentOutput);

    lcd.setCursor(0, 0);
    lcd.print(ch2VoltageOutput, 4);
    lcd.setCursor(6, 0);
    lcd.print("V");
    lcd.setCursor(9, 0);
    lcd.print(ch2CurrentOutput, 4);
    lcd.setCursor(14, 0);
    lcd.print("A");

    lcd.setCursor(0, 1);
    lcd.print(ch1VoltageOutput, 4);
    lcd.setCursor(6, 1);
    lcd.print("V");
    lcd.setCursor(9, 1);
    lcd.print(ch1CurrentOutput, 4);
    lcd.setCursor(14, 1);
    lcd.print("A");
  }
  else {
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("IC6-TEMP");
    lcd.setCursor(6, 1);
    lcd.print("ALERT");
  }
  delay(550);
}


float voltageToCurrent(float voltageReading, int voltageRanges[]) {
  if (inRange(voltageReading, minimumVoltageBasedOnCurrent, maximumVoltageBasedOnCurrent)) {
    float lowerRangeDifferenceValue = 0.000;
    float current = 0.000;
    int index = 0;
    int indexJ = 0;
    int upperLimitRange = 0;
    if (voltageReading <= middleVoltageBasedOnCurrent) {
      upperLimitRange = (((sizeof(voltageBasedOnCurrentValues) / 2) - 1) / 2 - 1);
      // if the voltage reading is less than or equal the voltage value of
      //the element at the middle of the array, we only search from index = 0
      //to the upperLimitRange = 249 which is half size of the array - 1.
      //indexJ has the same value as index.
    }
    else {
      index = (((sizeof(voltageBasedOnCurrentValues) / 2) - 1) / 2 - 1);
      indexJ = (((sizeof(voltageBasedOnCurrentValues) / 2) - 1) / 2 - 1);
      upperLimitRange = ((sizeof(voltageBasedOnCurrentValues) / 2) - 1);
      // if the voltage reading is greater than or equal the voltage value of
      //the element at the middle of the array, we only search from index = 249
      //to the upperLimitRange = 500 which is the size of the array - 1.
      //indexJ has the same value as index.
    }
    for (int indexI = index; indexI < upperLimitRange; indexI++) {
      float vIndexIVoltage = (float) voltageRanges[indexI];
      if (voltageReading >= (vIndexIVoltage / 1000)) {
        indexJ = indexJ + 1;
        float vIndexJVoltage = (float) voltageRanges[indexJ];
        if (voltageReading <= (vIndexJVoltage / 1000)) {
          lowerRangeDifferenceValue = voltageReading - (vIndexIVoltage / 1000);
          if (lowerRangeDifferenceValue <= 0.005) {
            float currentI = (float) indexI / 100;
            current = currentI;
          }
          else {
            float currentJ = (float) indexJ / 100;
            current = currentJ;
          }
          return current;
        }
      }
    }
  }
  //this value was random selected.
  //if the function return this value is either because current was not calculated correctly,
  //or there is a voltage reading out of bounds.
  return 999.23;
}

bool inRange(float reading, float minimumValue, float maximumValue) {
  return ((minimumValue <= reading) && (reading <= maximumValue));
}

bool temperatureAboveThreshold(float value, float threshold) {
  return (value >= threshold);
}

float voltageOnChannel(float voltageReading) {
  float voltageOutput = 0.0;
  if (voltageBelowThreshold(voltageReading, threshold)) {
    if (voltageReading > 0.760 && voltageReading <= 0.790) {
      voltageOutput = 0.790;
    }
    if (voltageReading > 0.790 && voltageReading < 0.800) {
      //above 0.790 index = 0, 0.790 + 0.030 = 0.810
      voltageOutput = 0.810;
    }
  }
  else {
    int offsetVoltageIndex = (voltageReading * 1000 - 800) / 10;
    float roundedVoltage = (float)(offsetVoltageIndex * 10 + 800) / 1000;
    float offsetVoltage = offsetVoltageBasedOnRange(roundedVoltage);
    voltageOutput = roundedVoltage + offsetVoltage;
  }
  return voltageOutput;
}

bool voltageBelowThreshold(float voltageValue, float threshold) {
  return voltageValue < threshold;
}

//This offset is added to adc voltage reading due to pcb traces and wires resistance.
float offsetVoltageBasedOnRange(float voltageValue) {
  float offsetVoltage = 0.0;
  if (voltageValue > 0.800 && voltageValue <= 4.150) {
    offsetVoltage = 0.030;
  }
  else if (voltageValue > 4.150 && voltageValue <= 5.980) {
    offsetVoltage = 0.010;
  }
  else if (voltageValue > 8.761 && voltageValue <= 13.01) {
    offsetVoltage = -0.010;
  }
  else if (voltageValue > 13.01 && voltageValue <= 15.02) {
    offsetVoltage = -0.020;
  }
  else if (voltageValue > 15.02 && voltageValue <= 16.03) {
    offsetVoltage = -0.030;
  }
  else if (voltageValue > 16.03 && voltageValue <= 21.00) {
    offsetVoltage = -0.040;
  }
  return offsetVoltage;
}

void ledStatus(float voltageValueOne, float voltageValueTwo) {
  if ((voltageValueOne > 0.000) && (voltageValueOne != 999.23)) {
    digitalWrite(D3Pin, HIGH);
  }
  else {
    digitalWrite(D3Pin, LOW);
  }

  if ((voltageValueTwo > 0.000) && (voltageValueTwo != 999.23)) {
    digitalWrite(D5Pin, HIGH);
  }
  else {
    digitalWrite(D5Pin, LOW);
  }
}

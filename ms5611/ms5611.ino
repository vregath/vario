
#include <Wire.h>

#define LED_PIN LED_BUILTIN
#define BEEP_PIN PIN3

#define MS5611_ADDRESS          0x76
#define MS5611_RESET_CMD        0x1E
#define MS5611_PROM_READ        0xA2
#define MS5611_ADC_READ         0x00
#define MS5611_TEMP_CONV        0x50
#define MS5611_PRESS_CONV       0x40

int resolution = 0x8;
int resDelay = 10;
uint16_t fc[6];
int32_t TEMP2;

float seaLevelPress;
int ledStatus = 0;

#define ALTITUDE_BUFFER_SIZE   31
float altitudes[ALTITUDE_BUFFER_SIZE * 2];
uint64_t times[ALTITUDE_BUFFER_SIZE * 2];
uint16_t altitudeBufferIdx = 0;

#define AVG_ALTITUDE_BUFFER_SIZE   5
float avgAltitudes[AVG_ALTITUDE_BUFFER_SIZE * 2];
uint64_t avgTimes[AVG_ALTITUDE_BUFFER_SIZE * 2];
uint16_t avgAltitudeBufferIdx = 0;

uint64_t timeDivider = 10;
uint64_t timeCounter = 0;

uint16_t beepStartInit = 1000;
uint16_t beepFrequencyInit = 1;
uint16_t beepStartCounter = beepStartInit;
uint16_t beepFrequency = beepFrequencyInit;
int beepFrequencyStatus = 0;
int beepStatus = 0;
int doBeep = 0;
int doBeepOff = 0; 

void setup() {
  
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);
  Wire.begin();

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(MS5611_RESET_CMD);
  Wire.endTransmission();
  delay(100);

  readPROM();
  
  uint32_t pressure = readPressure();

  seaLevelPress = seaLevelPressure(pressure, 200);

  pinMode(BEEP_PIN, OUTPUT);

  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 100;
  TCCR1B |= (1 << WGM12); // CTCmode
  TCCR1B &= ~(1 << CS12); // 8 prescaler
  TCCR1B |= (1 << CS11); // 8 prescaler
  TCCR1B &= ~(1 << CS10); // 8 prescaler
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
  
}

void loop() {
  times[altitudeBufferIdx] = timeCounter;
  times[altitudeBufferIdx + ALTITUDE_BUFFER_SIZE] = timeCounter;
  uint32_t pressure = readPressure();
  altitudes[altitudeBufferIdx] = altitude(pressure, seaLevelPress);
  altitudes[altitudeBufferIdx + ALTITUDE_BUFFER_SIZE] = altitudes[altitudeBufferIdx];
  altitudeBufferIdx = altitudeBufferIdx < ALTITUDE_BUFFER_SIZE - 1 ? altitudeBufferIdx + 1 : 0;

  int i;
  float avgAlt = 0;
  for(i=altitudeBufferIdx;i<altitudeBufferIdx+ALTITUDE_BUFFER_SIZE;i++) {
    avgAlt += altitudes[i];
  }
  avgAlt /= (float)ALTITUDE_BUFFER_SIZE;

  avgAltitudes[avgAltitudeBufferIdx] = avgAlt;
  avgAltitudes[avgAltitudeBufferIdx + AVG_ALTITUDE_BUFFER_SIZE] = avgAlt;
  avgTimes[avgAltitudeBufferIdx] = times[altitudeBufferIdx + (ALTITUDE_BUFFER_SIZE >> 1)];
  avgTimes[avgAltitudeBufferIdx + AVG_ALTITUDE_BUFFER_SIZE] = avgTimes[avgAltitudeBufferIdx];
  avgAltitudeBufferIdx = avgAltitudeBufferIdx < AVG_ALTITUDE_BUFFER_SIZE - 1 ? avgAltitudeBufferIdx + 1 : 0;

  float w = 1.f/(float)AVG_ALTITUDE_BUFFER_SIZE;
  float xx, yy, xx2, yy2, xxyy;
  xx = yy = xx2 = yy2 = xxyy = 0;

  float avgX = 0; float avgY = 0;
  for(i=0;i<AVG_ALTITUDE_BUFFER_SIZE;i++) {
    avgX += avgTimes[i];
    avgY += avgAltitudes[i];
  }

  avgX *= w;
  avgY *= w;

  for(i=0;i<AVG_ALTITUDE_BUFFER_SIZE;i++) {
    float x = avgTimes[i] - avgX;
    float y = (avgAltitudes[i] - avgY) * 1000;
    xx += x; yy += y;
    xx2 += x*x; yy2 += y*y; xxyy += x*y;
  }
  
  xx *= w; yy *= w; xx2 *= w; yy2 *= w; xxyy *= w;

  float dx2 = xx2 - xx * xx;
  float dy2 = yy2 - yy * yy;
  float dxy = xxyy - xx * yy;
  float tt = (float)(atan2(2*dxy, dx2-dy2) * 0.5f);

  int k = tt * 10.f;

  if (k < 3 && k > -3) {
    doBeepOff = 1;
  } else {
    beepStartInit = k < 30 ? 3000 - k * 100 : 10;
    beepFrequencyInit = k < 0 ? 30 : 4;
    doBeepOff = 0;
    doBeep = 1;
  }
  
}

ISR(TIMER1_COMPA_vect) {

  if (doBeep) {

    if (!--beepStartCounter) {
      if (doBeepOff) {
        doBeep = 0;
        doBeepOff = 0;
        beepStatus = 0;
        beepFrequency = beepFrequencyInit;
      } else {
        beepStatus = beepStatus ? 0 : 1;
      }
      beepStartCounter = beepStartInit;
    }
  
    if (beepStatus) {
      if (!--beepFrequency) {
        beepFrequencyStatus = beepFrequencyStatus ? 0 : 1;
        beepFrequency = beepFrequencyInit;
      }
    }
  
    if (beepFrequencyStatus) {
      digitalWrite(BEEP_PIN, HIGH);
    } else {
      digitalWrite(BEEP_PIN, LOW);
    }
  }

  if (!--timeDivider) {
    timeCounter++;
    timeDivider = 10;
  }
}

float altitude(float pressure, float seaLevelPress) {
    return (44330.0f * (1.0f - pow((float)pressure / (float)seaLevelPress, 0.1902949f)));
}

float seaLevelPressure(float pressure, float altitude) {
    return ((float)pressure / pow(1.0f - ((float)altitude / 44330.0f), 5.255f));
}

uint32_t readPressure() {
  uint32_t D1 = readRawPressure();

  uint32_t D2 = readRawTemperature();
  int32_t dT = D2 - (uint32_t)fc[4] * 256;

  int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
  int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

  int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

  int64_t OFF2, SENS2;

  OFF2 = 0;
  SENS2 = 0;
/*
  if (TEMP < 2000)
  {
      OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
      SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
  }

  if (TEMP < -1500)
  {
      OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
      SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
  }
*/
  OFF = OFF - OFF2;
  SENS = SENS - SENS2;
  uint32_t P = (D1 * SENS / 2097152 - OFF) / 32768;

  return P;
}

uint32_t readTemperature() {
  uint32_t D2 = readRawTemperature();
  int32_t dT = D2 - (uint32_t)fc[4] * 256;

  int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

  TEMP2 = 0;
/*
  if (TEMP < 2000) {
    TEMP2 = (dT * dT) / (2 << 30);
  }
*/
  TEMP = TEMP - TEMP2;
  return TEMP;
}

void readPROM(void) {
  for (uint8_t offset = 0; offset < 6; offset++) {
    fc[offset] = readRegister16(MS5611_PROM_READ + (offset * 2));
  }
}

uint32_t readRawTemperature() {
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(MS5611_TEMP_CONV + resolution);
  Wire.endTransmission();

  delay(resDelay);

  return readRegister24(MS5611_ADC_READ);
}

uint32_t readRawPressure() {
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(MS5611_PRESS_CONV + resolution);
  Wire.endTransmission();

  delay(resDelay);

  return readRegister24(MS5611_ADC_READ);
}

uint16_t readRegister16(uint8_t req) {
  uint16_t value;
  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.write(req);
  Wire.endTransmission();

  Wire.beginTransmission(MS5611_ADDRESS);
  Wire.requestFrom(MS5611_ADDRESS, 2);
  while(!Wire.available()) {};
  uint8_t vha = Wire.read();
  uint8_t vla = Wire.read();
  Wire.endTransmission();

  value = vha << 8 | vla;

  return value;
}

uint32_t readRegister24(uint8_t req) {
    uint32_t value;
    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.write(req);
    Wire.endTransmission();

    Wire.beginTransmission(MS5611_ADDRESS);
    Wire.requestFrom(MS5611_ADDRESS, 3);
    while(!Wire.available()) {};
    uint8_t vxa = Wire.read();
    uint8_t vha = Wire.read();
    uint8_t vla = Wire.read();
    Wire.endTransmission();

    value = ((int32_t)vxa << 16) | ((int32_t)vha << 8) | vla;

    return value;
}


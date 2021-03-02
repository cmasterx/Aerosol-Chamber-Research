#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>

#ifndef BAUDRATE
#define BAUDRATE 9600
#endif

#define NUM_SENSORS 3

#define START_PIN 2

// BME sensors
Adafruit_BME280 bme[NUM_SENSORS];
int8_t addressBME[] = { 0x76, 0x77,0x76 };

/* Recording */
enum RECORDING_INTERVAL {
  HALF_SEC = 7811u,
  ONE_SEC = 15624u,
};

// recording state
bool record = false;
float recordingInterval = 1000;   // time in ms
bool takeMeasurement = false;

// LED
bool isLEDOn = false;
bool ledChangeState = false;

void changeMUXAddress(uint8_t bus) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

void changeState() {
  record = !record;
}

void toggleLED() {
  isLEDOn = !isLEDOn;
  digitalWrite(13, isLEDOn);
}

void setTimer1(RECORDING_INTERVAL time) {
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;//initialize counter value to 0
  OCR1A = time; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  TIMSK1 |= (1 << OCIE1A);

  // enable interrupt
  sei();
}

void setup() {
  Serial.begin(BAUDRATE);

  initializes i2c
  Wire.begin();
  
  // initializes bme sensors
  for (int i = 0; i  < sizeof(bme) / sizeof(Adafruit_BME280), ++i) {
    if (!bme[i].begin(addressBME[i])) {
      char buffer[64];
      sprintf(buffer, "BME Index [%i] cannot be initialized with address %x", , i, bme[i]);
      Serial.println(buffer);
      for(;;);
    }
  }

  // initializes pins and interrupts
  pinMode(START_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(START_PIN), changeState, FALLING);
  pinMode(13, OUTPUT);
  setTimer1(RECORDING_INTERVAL::HALF_SEC);
}

void loop() {
  // put your main code here, to run repeatedly:

  if (ledChangeState) {
    ledChangeState = false;
    toggleLED();
  }
}

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  ledChangeState = true;
  takeMeasurement = true;
}

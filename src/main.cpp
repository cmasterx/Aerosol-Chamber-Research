#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#ifndef BAUDRATE
#define BAUDRATE 9600
#endif

#define NUM_SENSORS        3
#define START_PIN          2
#define MUX_ADDR        0x70
#define SD_PIN             4
#define SAFE_BYPASS_PIN    8

#define MAX_FILE_COUNTER 1000

// BME sensors
Adafruit_BME280 bme[NUM_SENSORS];
uint8_t addressBME[] = { 0x76, 0x76,0x76 };
uint8_t muxBusBME[] =   { 0x00, 0x01, 0x02 };

/* Recording */
enum RECORDING_INTERVAL {
  HALF_SEC      = 1,
  ONE_SEC       = 2,
  TWO_SEC       = 4,
  FIVE_SEC      = 10,
  TEN_SEC       = 20,
  FIFTEEN_SEC   = 30,
  THIRTY_SEC    = 60,
  SIXTY_SEC     = 120,
  ONETWENTY_SEC = 240
};

// recording state
bool record = false;
unsigned long recordingInterval = RECORDING_INTERVAL::HALF_SEC;   // time in ms
bool takeMeasurement = false;

// LED
bool isLEDOn = false;
bool ledChangeState = false;

// SD Card
File file;

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

void printBME(uint8_t sensorIdx) {

  changeMUXAddress(sensorIdx);
  Adafruit_BME280 &bmes = bme[sensorIdx];

  // char buffer[32];
  // sprintf(buffer, "BME %d:", sensorIdx);
  // Serial.println(buffer);

  // Serial.print("Temperature = ");
	// Serial.print((bmes.readTemperature() * 9.0f / 5.0f ) + 32);
	// Serial.println("*F");

	// Serial.print("Pressure = ");
	// Serial.print(bmes.readPressure() / 100.0F);
	// Serial.println("hPa");

	// Serial.print("Humidity = ");
	// Serial.print(bmes.readHumidity());
	// Serial.println("%");

  file.print((bmes.readTemperature() * 9.0f / 5.0f ) + 32);
  file.print(',');
  file.print(bmes.readPressure() / 100.0F);
  file.print(',');
  file.print(bmes.readHumidity());
  file.print(',');
  
}

void setTimer1(unsigned int time) {
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
  Serial.println("Initializing...");
  // initializes i2c
  Wire.begin();

  // initialize bypass pin
  // pinMode(SAFE_BYPASS_PIN, INPUT_PULLUP);
  
  // initializes bme sensors
  for (int i = 0; i  < sizeof(bme) / sizeof(Adafruit_BME280); ++i) {
    changeMUXAddress(muxBusBME[i]);

    if (!bme[i].begin(addressBME[i])) {
      char buffer[64];
      sprintf(buffer, "BME Index [%i] cannot be initialized with address %x", i, bme[i]);
      Serial.println(buffer);
      for(;;);
    }

  // open file
  // file = SD.open("data.csv", FILE_WRITE);
  // if (!file) {
  //   Serial.println("Unable to open the file");
  //   for(;;);
  // }
  }

  // set up SD card
  if (!SD.begin(SD_PIN)) {
    // Serial.println("Cannot initialize SD card reader");
    for (;;);
  }

  // looks for a valid file
  // char filename[64];
  // for (unsigned int i = 0; i < 1000; ++i) {
  //   sprintf(filename, "csv_data_%d.csv", i + 1);
  //   if (!SD.exists(filename)) {
  //     file = SD.open(filename, FILE_WRITE);
  //   }
  // }

  char filename[] = "File.csv";
  file = SD.open(filename, FILE_WRITE);
  
  if (!file) {
    Serial.println("Unable to open a file. Either clean the SD card or something else went wrong");
    while(1);
  }
  else {
    char buff[72];
    sprintf(buff, "Successfully created file %s", filename);
    Serial.println(buff);
    file.close();
  }
  
  // initializes pins and interrupts
  pinMode(START_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(START_PIN), changeState, FALLING);
  pinMode(13, OUTPUT);
  setTimer1(7811);    // timer interrups every half seconds
  Serial.println("Initializing Done");
}


void loop() {
  // put your main code here, to run repeatedly:

  if (ledChangeState) {
    ledChangeState = false;
    toggleLED();

    file = SD.open("data.csv", FILE_WRITE);
    printBME(0);
    printBME(1);
    printBME(2);

    file.println("");
    file.close();
  }
}

unsigned long counter = 0;

ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  ++counter;
  counter %= 240;

  if ((counter % recordingInterval == 0)) {
    ledChangeState = true;
  }
}

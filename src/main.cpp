#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#ifndef BAUDRATE
#define BAUDRATE 9600
#endif

#define NUM_SENSORS              3
#define START_PIN                2
#define MUX_ADDR              0x70
#define SD_PIN                  10
#define SAFE_BYPASS_PIN          9
#define LED_RECORDING_PIN        8

#define MAX_FILE_COUNTER 1000


/* Screen vars */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

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

#define INFO_DATA F("========== Aerosol Chamber Data Logger ==========\
\n\
\n- Made by Charlemagne Wong '21 CECN\
\n- Code and designs found at: https://github.com/cmasterx/Aerosol-Chamber-Research\
\n\
\nData logger -> records temperature, humidity, and air pressure to a microSD Card as a CSV file.\
\nThe logger records sensor data every 0.5 seconds.\
\nFor this data logger to function, all 3 BME sensors must be connected. This data logger\
\ncan accept up to 4 BME sensors but will require minor reprogramming on the board firmare.\
\nThis data logger can be modified to accept up to 24 BME sensors. The data logger can also be\
\nreprogrammed to log other sensors.\
\n\
\n\
\n[ microSD Card Notice ]\
\nThe microSD card cannot be larger than 32GB and must be formatted to FAT32.\
\n\
\n\
\n[ Instructions ]\
\n1. Plug in all 3 Sensors to the correct port\
\n  1.1 VDD pins connect to VDD rail on the data logger\
\n  1.2 GND pins connect to GND rail on the data logger\
\n  1.3 Connect the SCL and SDA pin to the sensor input pins\
\n    Pinout is shown below\
\n    X is an unused pin\
\n    SCL is the SCL pin on the BME sensor\
\n    SDA is the SDA pin on the sensor\
\n    SCLx is the SCL pin of sensor #x\
\n\
\n\
\n   Pin       Connection\
\n   1         X - No connection\
\n   2         X - No connection\
\n   3         SCL3\
\n   4         SDA3\
\n   5         SCL2\
\n   6         SDA2\
\n   7         SCL1\
\n   8         SDA1\
\n\
\n2. Insert the MicroSD card\
\n3. Insert the microUSB cable to the data logger\
\n4. Press the button located at the top of the board to start recording. The LED should start flashing,\
\n   indicating that logger is logging sensor data \
\n5. Press the same button as step 4 to stop recording. The LED should stop flashing.\
\n\
\nRefer to the link above for full detailed instructions.\
\n\
\n\
\n[ Important Information] \
\n- The data logger is resettable with the reset button found at the bottom side of the data logger\
\n- The microSD card can be ejected without turning the data logger off. Just ensure that:\
\n  - Make sure the data logger is not recording before removing microSD card\
\n  - After inserting the microSD card, press the reset button\
\n\
\n\
\n[ LED ]\
\n- LED Off                   -> device is not recording or is powered off\
\n- LED Flashing              -> data logger is logging data to the microSD card\
\n- LED Solid On (No Flashing -> data logger cannot initilaize. Check if all sensors are all connected and plugged in correctly,\
\n                               microSD card is plugged in and is setup correctly (according to [ microSD Card Notice]) \
\n")\

// BME sensors
Adafruit_BME280 bme[NUM_SENSORS];
uint8_t addressBME[] = { 0x76, 0x76,0x76 };
uint8_t muxBusBME[] =   { 0x02, 0x03, 0x04 };

// recording state
bool record = false;
unsigned long recordingInterval = RECORDING_INTERVAL::HALF_SEC;   // time in ms
bool takeMeasurement = false;

// LED
bool isLEDOn = false;
bool ledChangeState = false;

// SD Card
File file;

// sreen instance
// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,&Wire, OLED_RESET);

/**
 * @brief Change i2c mux address to specified address
 * 
 * @param bus mux address to switch
 */
void changeMUXAddress(uint8_t bus) {
  Wire.beginTransmission(0x70);
  Wire.write(1 << bus);
  Wire.endTransmission();
}

/**
 * @brief Button interrupt handler - handles when button pin 8 is pressed
 *        Toggles the recording state, whether to log data or not
 */
void changeState() {
  static unsigned long lastPressed = 0;   // time since button last pressed
  const unsigned long epsilon = 300;       // button state can only change after 15ms

  unsigned long currentTime = millis();

  // if button pressed is after epsilon, change record state
  if (currentTime - lastPressed > epsilon) {
    record = !record;
    Serial.println("Button Pressed");
  }

  lastPressed = currentTime;
}

/**
 * @brief When function is called, LED will toggle from on to off or vice-versa
 * 
 */
void toggleLED() {
  isLEDOn = !isLEDOn;
  digitalWrite(LED_RECORDING_PIN, isLEDOn);
}

void printBME(uint8_t sensorIdx) {

  // switch mux of specified sensor and aquire bme object instance
  changeMUXAddress(muxBusBME[sensorIdx]);
  Adafruit_BME280 &bmes = bme[sensorIdx];

  // obtains reading from sensor
  float temperatue = bmes.readTemperature();
  float pressure = bmes.readPressure();
  float humidity = bmes.readHumidity();

  // writes sensor readings to file
  file.print((temperatue * 9.0f / 5.0f ) + 32);
  file.print(',');
  file.print(pressure / 100.0F);
  file.print(',');
  file.print(humidity);
  file.print(',');
  
  // prints sensor reading
  #if defined(DEBUG) || defined(VERBOSE)
  Serial.print("Sensor idx: ");
  Serial.print(sensorIdx);
  Serial.print(" - ");
  Serial.print((temperatue * 9.0f / 5.0f ) + 32);
  Serial.print(',');
  Serial.print(pressure / 100.0F);
  Serial.print(',');
  Serial.print(humidity);
  Serial.print(", ");
  #endif
  
}

/**
 * @brief Sets the hardware timer
 * 
 * @param time interval that interrupt will be called
 */
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
  // enable led pin
  pinMode(LED_RECORDING_PIN, OUTPUT);
  digitalWrite(LED_RECORDING_PIN, HIGH);
  delay(400);
  digitalWrite(LED_RECORDING_PIN, LOW);
  
  // initializes i2c
  Wire.begin();

  // initialize bypass pin
  // pinMode(SAFE_BYPASS_PIN, INPUT_PULLUP);
  
  Serial.println("I2C initialized");

  // initializes bme sensors
  for (int i = 0; i < sizeof(bme) / sizeof(Adafruit_BME280); ++i) {
    changeMUXAddress(muxBusBME[i]);

    if (!bme[i].begin(addressBME[i])) {

      digitalWrite(LED_RECORDING_PIN, HIGH);
      char str_buff[16];
      sprintf(str_buff, "BME Fail ID: %d", i);
      Serial.println(str_buff);
      for(;;);
    }
  }

  // set up SD card
  if (!SD.begin(SD_PIN)) {

    digitalWrite(LED_RECORDING_PIN, HIGH);
    Serial.println("SD Fail");
    for (;;);
  }

  // initializes info file
  SD.remove("info.txt");
  File f = SD.open("info.txt", FILE_WRITE);
  f.print(INFO_DATA);
  f.close();

  Serial.println("SD Initialized");
  
  // initializes pins and interrupts
  pinMode(START_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(START_PIN), changeState, FALLING);
  setTimer1(7811);    // timer interrups every half second

  // delay to remove some button debounce and show ready state
  Serial.println("Init Pass");
  pinMode(8, OUTPUT);
    for (unsigned int i = 0; i < 2; ++i) {
    digitalWrite(LED_RECORDING_PIN, HIGH);
    delay(125);
    digitalWrite(LED_RECORDING_PIN, LOW);
    delay(250);
  }
  digitalWrite(LED_RECORDING_PIN, HIGH);
  delay(1000);
  digitalWrite(LED_RECORDING_PIN, LOW);
  
  record = false;
}


void loop() {
  if (record && ledChangeState) {
    ledChangeState = false;
    toggleLED();

    file = SD.open("data.csv", FILE_WRITE);
    file.print(millis());
    file.print(',');

    for (int i = 0; i < NUM_SENSORS; ++i) {
      printBME(i);
    }

    file.print('\n');
    file.close();
    Serial.println("Recording");
  }
  else if (!record) {
    isLEDOn = true;
    toggleLED();
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

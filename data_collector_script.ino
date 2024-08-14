#include <Arduino_LSM9DS1.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// Label identifiers
volatile int currentLabel = 0;
const int LABEL_NONE = 0;
const int LABEL_DOWNHILL = 1;
const int LABEL_UPHILL = 2;
const int LABEL_DOWNSTAIRS = 3;
const int LABEL_UPSTAIRS = 4;
const int LABEL_FLATGROUND = 5;


// Constants for button pins
const int downhill_button = 7;
const int uphill_button = 6;
const int down_stairs_button = 5;
const int up_stairs_button = 4;
const int flat_ground_button = 3;

// Interrupt service routines (ISRs)
void setDownhill() { currentLabel = LABEL_DOWNHILL; }
void setUphill() { currentLabel = LABEL_UPHILL; }
void setDownStairs() { currentLabel = LABEL_DOWNSTAIRS; }
void setUpStairs() { currentLabel = LABEL_UPSTAIRS; }
void setFlatGround() { currentLabel = LABEL_FLATGROUND; }

#define SAMPLING_RATE 20  // 20 Hz sampling rate
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Acceleration in g's");
  Serial.println("X\tY\tZ");

  unsigned status;
    
  // default settings
  status = bme.begin(0x77);

  if (!status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
  }

  // Initialize the pushbutton pins as an input
  pinMode(downhill_button, INPUT_PULLUP);
  pinMode(uphill_button, INPUT_PULLUP);
  pinMode(down_stairs_button, INPUT_PULLUP);
  pinMode(up_stairs_button, INPUT_PULLUP);
  pinMode(flat_ground_button, INPUT_PULLUP);

  // Attach interrupts to the button pins
  attachInterrupt(digitalPinToInterrupt(downhill_button), setDownhill, FALLING);
  attachInterrupt(digitalPinToInterrupt(uphill_button), setUphill, FALLING);
  attachInterrupt(digitalPinToInterrupt(down_stairs_button), setDownStairs, FALLING);
  attachInterrupt(digitalPinToInterrupt(up_stairs_button), setUpStairs, FALLING);
  attachInterrupt(digitalPinToInterrupt(flat_ground_button), setFlatGround, FALLING);
}

void loop() {

  // Check button presses and set current label
  /*
  checkButtonPress(downhill_button, "downhill", prev_downhill_button_state);
  checkButtonPress(uphill_button, "uphill", prev_uphill_button_state);
  checkButtonPress(down_stairs_button, "downstairs", prev_down_stairs_button_state);
  checkButtonPress(up_stairs_button, "upstairs", prev_up_stairs_button_state);
  checkButtonPress(flat_ground_button, "flat ground", prev_flat_ground_button_state);*/
  
  float x, y, z, altitude;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

    Serial.print(String(x, 6));
    Serial.print(',');
    Serial.print(String(y, 6));
    Serial.print(',');
    Serial.print(String(z, 6));
    Serial.print(',');
    Serial.print(String(altitude, 6));
    Serial.print(',');

    // Convert the current label to a string
    switch (currentLabel) {
      case LABEL_DOWNHILL:
        Serial.println("downhill");
        break;
      case LABEL_UPHILL:
        Serial.println("none");
        break;
      case LABEL_DOWNSTAIRS:
        Serial.println("downstairs");
        break;
      case LABEL_UPSTAIRS:
        Serial.println("upstairs");
        break;
      case LABEL_FLATGROUND:
        Serial.println("flat ground");
        break;
      default:
        Serial.println("none");
        break;
    }
    
  }
  delay(1000 / SAMPLING_RATE);
}

/*
void checkButtonPress(int buttonPin, String label, bool &prevButtonState) {
  bool buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH && prevButtonState == LOW) {  // Button press detected
    currentLabel = label;
    Serial.println("Label set to: " + currentLabel);
  }
  prevButtonState = buttonState;
}*/

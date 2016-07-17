#include <Wire.h>
#include "Co2Meter_K33.h"
#include <SPI.h>
#include <SD.h>
#include "RTClib.h"

int co2Addr = 0x68;
const int chipSelect = 10;
const int motorPin = 4;
RTC_DS1307 RTC; // define the Real Time Clock object

Co2Meter_K33 k33;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  pinMode(13, OUTPUT); // We will use this pin as a read‐indicator;
  Serial.print("Initializing SD card...");
  pinMode(motorPin, OUTPUT); //setting up motor
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(10, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  // connect to RTC
  if (!RTC.begin()) {
    Serial.println("RTC Failed");
  }
}
void loop() {
  DateTime now; //creates a DateTime Object to hold the time. We will be storing data from clock here

  // We keep the sample period >25s or so, else the sensor will start ignoring sample requests. wakeSensor();
  digitalWrite(motorPin, HIGH); //turns pump on
  k33.initPoll();
  //delay(60000); //wait one minute
  delay(16000); //wait 16 seconds
  wakeSensor();
  double tempValue = k33.readTemp();
  delay(20);
  wakeSensor();
  double rhValue = k33.readRh();
  delay(20);
  wakeSensor();
  double co2Value = k33.readCo2();
  wakeSensor();
  now = RTC.now(); //reads current time from the Clock
  long timeStamp = now.unixtime(); //parses that DateTime Object into a UTC Timestamp
  if (co2Value >= 0) {
    Serial.print("Time: ");
    Serial.print(timeStamp);
    Serial.print("CO2: ");
    Serial.print(co2Value);
    Serial.print("ppm Temp: ");
    Serial.print(tempValue);
    Serial.print("C Rh: ");
    Serial.print(rhValue);
    Serial.println("%");
    if (co2Value >= 0) {
      File dataFile = SD.open("testdata1.txt", FILE_WRITE);
      // if the file is available, write to it:
      if (dataFile) {
        dataFile.print(timeStamp);
        dataFile.print(",");
        dataFile.print(tempValue);
        dataFile.print(",");
        dataFile.print(rhValue);
        dataFile.print(",");
        dataFile.print(co2Value);
        dataFile.println();
        dataFile.close();
        // print to the serial port too:
        Serial.println("sdWritten");
      }
    }
    else {
      Serial.println("Checksum failed / Communication failure");
      delay(9000);
    }
  }
  digitalWrite(motorPin, LOW); //turning off motor
  for (int i = 0; i < 1; i++) {
    delay(60000);
  }
}
void wakeSensor() {
  // This command serves as a wakeup to the CO2 sensor, for K33‐ELG/BLG Sensors Only
  // You'll have the look up the registers for your specific device, but the idea here is simple:
  // 1. Disabled the I2C engine on the AVR
  // 2. Set the Data Direction register to output on the SDA line
  // 3. Toggle the line low for ~1ms to wake the micro up. Enable I2C Engine
  // 4. Wake a millisecond.

  TWCR &= ~(1 << 2); // Disable I2C Engine
  DDRC |= (1 << 4); // Set pin to output mode
  PORTC &= ~(1 << 4); // Pull pin low
  delay(1);
  PORTC |= (1 << 4); // Pull pin high again
  TWCR |= (1 << 2); // I2C is now enabled
  delay(1);
}

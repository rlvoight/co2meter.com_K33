#include <Wire.h>
#include <SPI.h>
#include <SD.h>   //changed <> to ""
#include "RTClib.h"
#include "Co2Meter_K33.h"

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
  /*to set the clock use this line of code
   * but it will set it to the time on your computer
   * So if you want it to convert correctly without a time zone offset
   * you should set your time zone on your computer to be Greenich Mean Time or GMT-0
   * 
   * Once the time is set, you can comment this line out, the clock should stay set 
   * and count accurately as long as the backup battery is in the shield.
   */
  
  RTC.adjust(DateTime((__DATE__), (__TIME__))); //sets the RTC to the computer time.
}

void loop() {
  DateTime now; //creates a DateTime Object to hold the time. We will be storing data from clock here

  // We keep the sample period >25s or so, else the sensor will start ignoring sample requests. wakeSensor();
  digitalWrite(motorPin, HIGH); //turns pump on.
  delay(60000); //wait one minute. rlv uncommented
  k33.initPoll(); //This is basically wakes the sensor up
  delay(16000); //wait 16 seconds
  double tempValue = k33.readTemp();
  delay(20);
  double rhValue = k33.readRh();
  delay(20);
  double co2Value = k33.readCo2();

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
      File dataFile = SD.open("testdata.txt", FILE_WRITE);
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

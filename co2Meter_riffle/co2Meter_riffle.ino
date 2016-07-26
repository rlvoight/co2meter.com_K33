#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"    //  https://github.com/greiman/SdFat
#include "EnableInterrupt.h"  //  https://github.com/GreyGnome/EnableInterrupt
#include "DS3231.h"   //  https://github.com/kinasmith/DS3231
#include "LowPower.h"   //  https://github.com/rocketscream/Low-Power
#include "Co2Meter_K33.h" //  https://github.com/kinasmith/co2meter.com_K33

DS3231 rtc;
SdFat sd;
SdFile myFile;
Co2Meter_K33 k33;

const int DEBUG = 1; //Enable or disable Serial Printing with this line. 1 is enabled, 0 is disabled.
const int led = 9; //Feedback LED
const int bat_v_pin = A3;
const int bat_v_enable = 4; //enable pin for bat. voltage read
const int rtc_int = 5; //rtc interrupt pin
const int sd_pwr_enable = 6; //enable pin for SD power
const int hdr_pwr_enable = 8; //enable pin for header power
const int chipSelect = 7; //SPI Chip Select for SD Card
const int airPumpPin = 3;

long measurement_interval = 60 * 1; //60 seconds/minute * # of minutes
long airPump_flush_time = 1000 * 1; //1000 milliseconds/second * # of seconds

void setup() {
  if (DEBUG) Serial.begin(9600);
  Wire.begin();
  rtc.begin();
  pinMode(rtc_int, INPUT_PULLUP); //rtc needs the interrupt line to be pulled up
  if (DEBUG) rtc.adjust(DateTime((__DATE__), (__TIME__))); //Adjust automatically
  pinMode(led, OUTPUT);
  pinMode(chipSelect, OUTPUT);
  pinMode(bat_v_enable, OUTPUT);
  digitalWrite(bat_v_enable, HIGH); //Turn off Battery Reading
  pinMode(sd_pwr_enable, OUTPUT);
  pinMode(airPumpPin, OUTPUT); //setting up motor
  if (DEBUG) Serial.println("----Setup Done, Starting Measurement Cycle---");
}

void loop() {
  digitalWrite(airPumpPin, HIGH); //turns pump on.
  delay(airPump_flush_time); //milliseconds

  DateTime now = rtc.now(); //get the current time
  DateTime nextAlarm = DateTime(now.unixtime() + measurement_interval);
  if (DEBUG) {
    Serial.print("Now: ");
    Serial.print(now.unixtime());
    Serial.print(" Alarm Set for: ");
    Serial.println(nextAlarm.unixtime());
    Serial.flush();
  }
  k33.initPoll();
  delay(16000);
  double tempValue = k33.readTemp();
  delay(20);
  double rhValue = k33.readRh();
  delay(20);
  double co2Value = k33.readCo2();
  long timeStamp = now.unixtime(); //parses that DateTime Object into a UTC Timestamp
  if (DEBUG) {
    Serial.print("Time: ");
    Serial.print(timeStamp);
    Serial.print(" CO2: ");
    Serial.print(co2Value);
    Serial.print("ppm Temp: ");
    Serial.print(tempValue);
    Serial.print("C Rh: ");
    Serial.print(rhValue);
    Serial.println("v");
    Serial.flush();
  }
  writeToSd(now.unixtime(), tempValue, rhValue, co2Value);
  if (DEBUG) {
    Serial.print("SD Card Written. Sleeping for ");
    Serial.print(measurement_interval);
    Serial.print(" seconds.");
    Serial.println();
    Serial.println("---------------------------------");
    Serial.flush();
  }
  digitalWrite(airPumpPin, LOW); //turning off motor
  enterSleep(nextAlarm); //Sleep until saved time
}


///////////////////////////////////////////////////
//Function:  void rtc_interrupt()
//Excecutes: Wakes the MCU from sleep on Alarm Pin Change from RTC
///////////////////////////////////////////////////
void rtc_interrupt() {
  disableInterrupt(rtc_int); //first it Disables the interrupt so it doesn't get retriggered
}


///////////////////////////////////////////////////
//Function:  void enterSleep(Time Wake up)
//Excecutes: Turns off SD Card Power, Sets Wake Alarm and Interrupt, and Powers down the MCU
///////////////////////////////////////////////////
void enterSleep(DateTime& dt) { //argument is Wake Time as a DateTime object
  delay(50); //Wait for file writing to finish. 10ms works somethings, 20 is more stable
//  digitalWrite(sd_pwr_enable, HIGH); //This breaks some cards for some reason(???)
//  delay(100); //wait for SD Card to power down
  rtc.clearAlarm(); //resets the alarm interrupt status on the rtc
  enableInterrupt(rtc_int, rtc_interrupt, FALLING); //enables the interrupt on Pin5
  rtc.enableAlarm(dt); //Sets the alarm on the rtc to the specified time (using the DateTime Object passed in)
  delay(1); //wait for a moment for everything to complete
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); //power down everything until the alarm fires
}

///////////////////////////////////////////////////
//Function: float getBat_v(AnalogPin, EnablePin)
//Returns: Battery Voltage from Analog Pin 3
///////////////////////////////////////////////////
float getBat_v(byte p, byte en) {
  float v;
  digitalWrite(en, LOW); //write mosfet low to enable read
  delay(10); //wait for it to settle
  v = analogRead(p); //read voltage
  delay(10); //wait some more...for some reason
  digitalWrite(en, HIGH); //disable read circuit
  v = (v * (3.3 / 1024.0)) * 2.0; //calculate actual voltage
  return v;
}

///////////////////////////////////////////////////
//Function: void blink(LEDPin, Time)
//Excecutes: Blinks a digital Pin
///////////////////////////////////////////////////
void blink(byte PIN, int DELAY_MS) {
  digitalWrite(PIN, HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN, LOW);
  delay(DELAY_MS);
}

///////////////////////////////////////////////////
//Function: writeToSd(timeStamp, temp, rh, co2);
//Excecutes: Powers on SD Card, and records the give values into "data.csv"
//Notes: The delay times are important. The SD Card initializations
//     will fail if there isn't enough time between writing and sleeping
///////////////////////////////////////////////////
void writeToSd(long t, float temp, float rh, float co2) {
  digitalWrite(led, HIGH); //LED ON, write cycle start
  /**** POWER ON SD CARD ****/
  digitalWrite(sd_pwr_enable, LOW); //Turn power to SD Card On
  delay(150); //wait for power to stabilize (!!) 10ms works sometimes
  /**** INIT SD CARD ****/
  if (DEBUG) Serial.print("SD Card Initializing...");
  while (!sd.begin(chipSelect)) {  //init. card
    if (DEBUG) Serial.println("Failed!");
    //    while (1); //if card fails to init. the led will stay lit.
//    while (!sd.begin(chipSelect))
      delay(5000);
  }
  if (DEBUG) Serial.println("Success");
  /**** OPEN FILE ****/
  if (DEBUG) Serial.print("File Opening...");
  if (!myFile.open("data.csv", O_RDWR | O_CREAT | O_AT_END)) {  //open file
    if (DEBUG) Serial.println("Failed!");
    while (1);
  }
  if (DEBUG) Serial.println("Success");
  /**** WRITE TO FILE ****/
  myFile.print(t);
  myFile.print(",");
  myFile.print(temp);
  myFile.print(",");
  myFile.print(rh);
  myFile.print(",");
  myFile.print(co2);
  myFile.println();
  myFile.close();
  digitalWrite(led, LOW); //LED will stay on if something broke
}

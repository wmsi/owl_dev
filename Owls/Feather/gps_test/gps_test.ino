// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>
#include <TinyGPS++.h>

/* for feather m0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0


// define send rate, sensor IDs and pins
#define SENDDELAY     5000

#define LIGHTID       "01"
#define LIGHTPIN      A0
#define LIGHTSENSOR   true

#define TEMPID        "02"
#define TEMPPIN       A1
#define TEMPSENSOR    true

#define GPSRX         3
#define GPSTX         4
#define GPS           true
#define DATE          false
#define TIME          true
#define GPSBAUD       9600

// battery voltage checker
#define VBATPIN       A7

#define LEDPIN 13

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;

void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(9600);

  //start serial for GPS
  Serial1.begin(GPSBAUD);
  delay(100);

  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(5, false);

  // check and print battery power
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measuredvbat);

  pinMode(LEDPIN, OUTPUT);
}

void loop()
{
    static String send_string;
  static int send_length = 0;
  static char send_buffer[RH_RF95_MAX_MESSAGE_LEN];
  
  // This sketch displays information every time a new sentence is correctly encoded.
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      displayInfo();
      send_string = addGPSData();
    }
  }

  if(LIGHTSENSOR) {
    float light_reading = analogRead(LIGHTPIN) * (1000.0/1023);
    send_string = addSensorData(send_string, LIGHTID, light_reading, 0);
  }

  if(TEMPSENSOR) {
    float voltage = (analogRead(TEMPPIN) * 0.00322581);
    float tempC = (voltage - 0.5) * 100.0;
    send_string = addSensorData(send_string, TEMPID, tempC, 2);
  }

   send_length = min(RH_RF95_MAX_MESSAGE_LEN, send_string.length());
  Serial.print("about to send: "); Serial.println(send_string);
  for (byte i = 0; i < send_length; i++) {
    send_buffer[i] = send_string[i];
  }

  Serial.print("Sending "); Serial.println(send_buffer);
  rf95.send((uint8_t *)send_buffer, send_length);
  
  //    Serial.println("Waiting for packet to complete..."); 
  delay(20);
  rf95.waitPacketSent();

  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected"));
    while(true);
  }

  delay(SENDDELAY);
}

void displayInfo()
{
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}


String addGPSData() {
  String send_string = "LOC:";
  if(gps.location.isValid()) {
    send_string = String(send_string + String(gps.location.lat(), 5));
    send_string = String(send_string + ",");
    send_string = String(send_string + String(gps.location.lng(), 5));
  } else {
    send_string = String(send_string + "ERR;");
  }

  if(DATE) {
    send_string = String(send_string + "DAT:");
    if(gps.date.isValid()) {
      send_string = String(send_string + String(gps.date.month())); send_string = String(send_string + "/");
      send_string = String(send_string + String(gps.date.day())); send_string = String(send_string + "/");
      send_string = String(send_string + String(gps.date.year())); send_string = String(send_string + "; ");
    } else {
      send_string = String(send_string + "ERR;"); 
    }
  }

  if (TIME) {
    send_string = String(send_string + "TIM:");
    if(gps.time.isValid()) {
      if (gps.time.hour() < 10) send_string = String(send_string + "0"); 
      send_string = String(send_string + String(gps.time.hour()));
      send_string = String(send_string + ":");
      if (gps.time.minute() < 10) send_string = String(send_string + "0"); 
      send_string = String(send_string + String(gps.time.minute()));
      send_string = String(send_string + ";");
//      if (gps.time.second() < 10) send_string = String(send_string + "0"); 
//      send_string = String(send_string + String(gps.time.second()));
//      send_string = String(send_string + ";");
    } else {
      send_string = String(send_string + "ERR;");
    }
  }
  return send_string;
}

// add data from a sensor to the upcoming transmission. 
// Transmission format is [sensor id]:[sensor value],...,[sensor value];
String addSensorData(String send_string, String id, float reading, int dec_places) {
  send_string = String(send_string + id);
  send_string = String(send_string + ":");
  send_string = String(send_string + String(reading, dec_places));
  send_string = String(send_string + ";");
  return send_string;
}


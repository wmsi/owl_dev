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
#define TXPOWER   23

// define send rate, sensor IDs and pins
#define SENDDELAY     3000

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
  rf95.setTxPower(TXPOWER, false);

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
  static int send_time;
  static bool got_location = false;

  if(GPS && !got_location) {
    while (Serial1.available() > 0) {
      if (gps.encode(Serial1.read())) {
        send_string = addGPSData();
        got_location = true;
      }
    }
  }

  if(millis() < SENDDELAY || millis() - send_time > SENDDELAY) {
    if(LIGHTSENSOR) {
      float light_reading = analogRead(LIGHTPIN) * (1000.0/1023);
      send_string = addSensorData(send_string, LIGHTID, light_reading, 0);
    }
  
    if(TEMPSENSOR) {
      float voltage = (analogRead(TEMPPIN) * 0.00322581);
      float tempC = (voltage - 0.5) * 100.0;
      send_string = addSensorData(send_string, TEMPID, tempC, 2);
    }
    // add more sensors here
  
    // send transmission
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
    // Now wait for a reply
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
  
  //  Serial.println("Waiting for reply..."); 
    delay(20);
    if (rf95.waitAvailableTimeout(1000)) { 
      // Should be a reply message for us now   
      if (rf95.recv(buf, &len)) {
        Serial.print("Got reply: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);    
      } else {
        Serial.println("Receive failed");
      }
    } else {
      Serial.println("No reply, is there a listener around?");
    }
  
    send_time = millis();
    send_string = "";
    got_location = false;
  }
}

String addGPSData() {
  String send_string = "LOC:";
  if(gps.location.isValid()) {
    send_string = String(send_string + String(gps.location.lat(), 4));
    send_string = String(send_string + ",");
    send_string = String(send_string + String(gps.location.lng(), 4));
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
    send_string = String(send_string + ";TIM:");
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

// Blink an LED for a given number of ms
void Blink(byte PIN, int DELAY_MS)
{
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
}

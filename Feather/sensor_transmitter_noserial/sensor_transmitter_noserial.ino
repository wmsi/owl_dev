// Feather9x_TX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (transmitter)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_RX

#include <SPI.h>
#include <RH_RF95.h>

/* for feather m0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
#define TXPOWER   23

// define send rate, sensor IDs and pins
#define SENDDELAY     1000

#define LIGHTID       "01"
#define LIGHTPIN      A0
#define LIGHTSENSOR   true

#define TEMPID        "02"
#define TEMPPIN       A1
#define TEMPSENSOR    false

// battery voltage checker
#define VBATPIN       A7

#define LEDPIN 13

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

//  while (!Serial);
//  Serial.begin(9600);
//  delay(100);
//
//  Serial.println("Feather LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
//    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
//    Serial.println("setFrequency failed");
    while (1);
  }
//  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
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
//  Serial.print("VBat: " ); Serial.println(measuredvbat);

  pinMode(LEDPIN, OUTPUT);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop()
{
  static String send_string;
  static int send_length = 0;
  static char send_buffer[RH_RF95_MAX_MESSAGE_LEN];

  send_string = String(String(millis(), DEC) + ";");
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

  // send transmission. tranmission must be formatted as a char array
  send_length = send_string.length();
  for (byte i = 0; i < send_length; i++) {
    send_buffer[i] = send_string[i];
  }

  rf95.send((uint8_t *)send_buffer, send_length);
  
  rf95.waitPacketSent();
  Blink(LEDPIN, 500);
  
  // Maybe add code to handle a reply

  delay(SENDDELAY);
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

    

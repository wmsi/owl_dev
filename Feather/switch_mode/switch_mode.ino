// Feather9x_RX
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messaging client (receiver)
// with the RH_RF95 class. RH_RF95 class does not provide for addressing or
// reliability, so you should only use RH_RF95 if you do not need the higher
// level messaging abilities.
// It is designed to work with the other example Feather9x_TX

#include <SPI.h>
#include <RH_RF95.h>

/* for feather m0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// battery voltage checker
#define VBATPIN       A7

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

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Blinky on receipt
#define LED 13

// Resonant frequency for audio feedback
#define CENTER_FREQ 2000
#define SPEAKER 5

// set mode to transmit/ receive
#define MODE_PIN 6
static bool send_mode;

void setup() 
{
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  pinMode(SPEAKER, OUTPUT);
  pinMode(MODE_PIN, INPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(9600);
  delay(100);
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    while (1);
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    while (1);
  }

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

    // check and print battery power
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
//  Serial.print("VBat: " ); Serial.println(measuredvbat);

  if(digitalRead(MODE_PIN)) {
    send_mode = 1; //send
//    Serial.println("send mode");
  } else {
    send_mode = 0;
//    Serial.println("receive mode");
  }

}

void loop()
{
  if(send_mode) {
    static String send_string;
    static int send_length = 0;
    static char send_buffer[RH_RF95_MAX_MESSAGE_LEN];
    static long send_time = -1;
  
    if(send_time == -1 || millis() - send_time > SENDDELAY) {
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
      for (byte i = 0; i < send_length; i++) {
        send_buffer[i] = send_string[i];
      }
    
      rf95.send((uint8_t *)send_buffer, send_length);
      
      delay(20);
      rf95.waitPacketSent();
      Blink(LED, 500);
      
      // Maybe add code to handle a reply
  
    
      send_time = millis();
      send_string = "";
    }
  } else {                // receive mode
    if (rf95.available())
    {
      // Should be a message for us now   
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      
      if (rf95.recv(buf, &len))
      {
        digitalWrite(LED, HIGH);
//        RH_RF95::printBuffer("Received: ", buf, len);
//        Serial.print("Got: ");
//        Serial.println((char*)buf);
//         Serial.print("RSSI: ");
//        Serial.println(rf95.lastRssi(), DEC);
        delay(10);
        // Send a reply
        uint8_t data[] = "And hello back to you";
        rf95.send(data, sizeof(data));
        rf95.waitPacketSent();
//        Serial.println("Sent a reply");
        digitalWrite(LED, LOW);
        audioFeedback(rf95.lastRssi());
      }
    }
  }
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

void audioFeedback(int rssi) {
  int scale_factor = 10;
  int center_rssi  = 75;

  int output_freq = CENTER_FREQ + (center_rssi + rssi)*scale_factor;
  tone(SPEAKER, output_freq, 500);
}


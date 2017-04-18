// See the hook-up guide for wiring instructions:
// https://learn.sparkfun.com/tutorials/rfm69hcw-hookup-guide

// Uses the RFM69 library by Felix Rusu, LowPowerLab.com
// Original library: https://www.github.com/lowpowerlab/rfm69
// SparkFun repository: https://github.com/sparkfun/RFM69HCW_Breakout

// Include the RFM69 and SPI libraries:
#include <RFM69.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Addresses for this node. CHANGE THESE FOR EACH NODE!
#define NETWORKID     0   // Must be the same for all nodes
#define MYNODEID      2   // My node ID
#define TONODEID      1   // Destination node ID

// RFM69 frequency, uncomment the frequency of your module:
//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ

// Transmission power (0-31 = 5-20 dBm)
#define TXPOWER       31

// AES encryption (or not):
#define ENCRYPT       true // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):
#define USEACK        true // Request ACKs or not

// Packet sent/received indicator LED (optional):
#define LED           9 // LED positive pin
#define GND           8 // LED ground pin

// define send rate, sensor IDs and pins
#define SENDDELAY     4000

#define LIGHTID       "01"
#define LIGHTPIN      A0
#define LIGHTSENSOR   true

#define TEMPID        "02"
#define TEMPPIN       A1
#define TEMPSENSOR    true

#define GPSRX         3
#define GPSTX         4
#define GPS           true
#define DATE          true
#define TIME          true
#define GPSBAUD       9600

// Create a library object for our RFM69HCW module:
RFM69 radio;

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(GPSRX, GPSTX);

void setup() {

  // Set up the indicator LED (optional):
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
  pinMode(GND,OUTPUT);
  digitalWrite(GND,LOW);

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBAUD);

  // Initialize the RFM69HCW:
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW
  radio.setPowerLevel(TXPOWER);

  // Turn on encryption if desired:
  if (ENCRYPT)
    radio.encrypt(ENCRYPTKEY);
}

void loop() {
  static String send_string = "";
  static int send_length = 0;
  static char send_buffer[RF69_MAX_DATA_LEN];
  static long send_time = -1;
  static bool got_location = false;

  if(GPS && !got_location) {
    while (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        send_string = addGPSData();
        got_location = true;
      }
    }
  }

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
    send_length = min(send_string.length(), RF69_MAX_DATA_LEN);
  
  //  Serial.print("sending to node ");
  //  Serial.print(TONODEID, DEC);
  //  Serial.print(", message [");
    for (byte i = 0; i < send_length; i++) {
  //    Serial.print(send_string[i]);
      send_buffer[i] = send_string[i];
    }
  //  Serial.println("]");
  
    // There are two ways to send packets. If you want
    // acknowledgements, use sendWithRetry():
    if (USEACK) {
      radio.sendWithRetry(TONODEID, send_buffer, send_length);
    } else {
      radio.send(TONODEID, send_buffer, send_length);
    }
  
    Blink(LED,10); 
    send_time = millis();
    send_string = "";
    got_location = false;
  }
}

String addGPSData() {
  String send_string = "LOC:";
  if(gps.location.isValid()) {
    send_string = String(send_string + String(gps.location.lat(), 6));
    send_string = String(send_string + ",");
    send_string = String(send_string + String(gps.location.lng(), 6));
  } else {
    send_string = String(send_string + "ERR");
  }

  if(DATE) {
    send_string = String(send_string + ";DAT: ");
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
      send_string = String(send_string + ":");
      if (gps.time.second() < 10) send_string = String(send_string + "0"); 
      send_string = String(send_string + String(gps.time.second()));
      send_string = String(send_string + ";");
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


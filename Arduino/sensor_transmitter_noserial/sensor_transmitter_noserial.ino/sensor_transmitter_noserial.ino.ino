// See the hook-up guide for wiring instructions:
// https://learn.sparkfun.com/tutorials/rfm69hcw-hookup-guide

// Uses the RFM69 library by Felix Rusu, LowPowerLab.com
// Original library: https://www.github.com/lowpowerlab/rfm69
// SparkFun repository: https://github.com/sparkfun/RFM69HCW_Breakout

// Include the RFM69 and SPI libraries:
#include <RFM69.h>
#include <SPI.h>

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
#define SENDDELAY     1000

#define LIGHTID       "01"
#define LIGHTPIN      A0
#define LIGHTSENSOR   true

#define TEMPID        "02"
#define TEMPPIN       A1
#define TEMPSENSOR    true

// Create a library object for our RFM69HCW module:
RFM69 radio;

void setup()
{
  // Open a serial port so we can send keystrokes to the module:
//  Serial.begin(9600);
//  Serial.print("Node ");
//  Serial.print(MYNODEID,DEC);
//  Serial.println(" ready");  

  // Set up the indicator LED (optional):
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
  pinMode(GND,OUTPUT);
  digitalWrite(GND,LOW);

  // Initialize the RFM69HCW:
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower(); // Always use this for RFM69HCW
  radio.setPowerLevel(TXPOWER);

  // Turn on encryption if desired:
  if (ENCRYPT)
    radio.encrypt(ENCRYPTKEY);
}

void loop()
{
  static String send_string = "";
  static int send_length = 0;
  static char send_buffer[62];

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

  // send transmission
  send_length = send_string.length();

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
  if (USEACK)  {
    if (radio.sendWithRetry(TONODEID, send_buffer, send_length)) {  
//      Serial.println("ACK received!");
//      Serial.print("RSSI: ");
//      Serial.println(radio.RSSI);
    } else {
//      Serial.println("no ACK received");
    }
  } else {
    radio.send(TONODEID, send_buffer, send_length);
  }

  send_length = 0; // reset the packet
  Blink(LED,250);
    
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


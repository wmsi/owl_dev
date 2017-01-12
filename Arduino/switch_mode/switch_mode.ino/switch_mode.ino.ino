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
#define MYNODEID      1   // My node ID
#define TONODEID      2   // Destination node ID

// RFM69 frequency, uncomment the frequency of your module:
//#define FREQUENCY   RF69_433MHZ
#define FREQUENCY     RF69_915MHZ
#define RFM69_RST     9

// Transmission power (0-31 = 5-20 dBm)
#define TXPOWER       31

// AES encryption (or not):
#define ENCRYPT       true // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):
#define USEACK        true // Request ACKs or not

// define send rate, sensor IDs and pins
#define SENDDELAY     500

#define LIGHTID       "01"
#define LIGHTPIN      A0
#define LIGHTSENSOR   true

#define TEMPID        "02"
#define TEMPPIN       A1
#define TEMPSENSOR    true

// indicator LEDs
const int LED[4] = {5,6,7,8};

// Resonant frequency for audio feedback
#define CENTER_FREQ 2000
#define SPEAKER 3

// mode select switch
#define MODESELECT    4
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

RFM69 radio;

void setup() {
  // Open a serial port so we can send keystrokes to the module:
  Serial.begin(9600);
  Serial.print("Node ");
  Serial.print(MYNODEID,DEC);
  Serial.println(" ready");  
  
  // Hard Reset the RFM module
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  // Initialize radio
  radio.initialize(FREQUENCY,MYNODEID,NETWORKID);
  radio.setHighPower();    // Only for RFM69HCW & HW!
  radio.setPowerLevel(TXPOWER); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
  // promiscuous mode allows this unit to receive all transmissions on the network
  radio.promiscuous();
  radio.encrypt(ENCRYPTKEY);

  // setup LEDs and speaker as outputs
  for(int i=0; i<4; i++) {
    pinMode(LED[i], OUTPUT);
  }
  pinMode(SPEAKER, OUTPUT); 

  // select mode to transmit or receive based on a switch
  // on pin [MODESELECT]. This could also be moved to loop
  // to allow for 
  pinMode(MODESELECT, INPUT);
  if(digitalRead(MODESELECT)) {
    mode = RECEIVE;
    Serial.print("\nReceiving at ");
    Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
    Serial.println(" MHz");
  } else {
    mode = TRANSMIT;
    Serial.print("\nTransmitting at ");
    Serial.print(FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
    Serial.println(" MHz");
  }
}

void loop() {
  if(mode == TRANSMIT) {
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
  
    Serial.print("sending to node ");
    Serial.print(TONODEID, DEC);
    Serial.print(", message [");
    for (byte i = 0; i < send_length; i++) {
      Serial.print(send_string[i]);
      send_buffer[i] = send_string[i];
    }
    Serial.println("]");
  
    // There are two ways to send packets. If you want
    // acknowledgements, use sendWithRetry():
    if (USEACK)
    {
      if (radio.sendWithRetry(TONODEID, send_buffer, send_length)) {
        Serial.println("ACK received!");
        Serial.print("RSSI: ");
        Serial.println(radio.RSSI);
        BlinkSS(radio.RSSI, 250);
      } else
        Serial.println("no ACK received");
    } else {
      radio.send(TONODEID, send_buffer, send_length);
    }  
    Blink(byte(LED[3]), 250);
  
    send_length = 0; // reset the packet
  
    delay(SENDDELAY);
  } else {              // receive mode
    // In this section, we'll check with the RFM69HCW to see
    // if it has received any packets:
    if (radio.receiveDone()) {
      // Print out the information:
  
      Serial.print("received from node ");
      Serial.print(radio.SENDERID, DEC);
      Serial.print(", message [");
  
      // The actual message is contained in the DATA array,
      // and is DATALEN bytes in size:
      for (byte i = 0; i < radio.DATALEN; i++)
        Serial.print((char)radio.DATA[i]);
  
      // RSSI is the "Receive Signal Strength Indicator",
      // smaller numbers mean higher power.
      Serial.print("], RSSI ");
      Serial.println(radio.RSSI);
  
      // Send an ACK if requested.
      // (You don't need this code if you're not using ACKs.)
      if (radio.ACKRequested()) {
        radio.sendACK();
        Serial.println("ACK sent");
      }
      BlinkSS(radio.RSSI, 250);
      audioFeedback(radio.RSSI, 250);
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

// Blink 4 LEDs to show signal strength
void BlinkSS(int rssi, int delay_ms) {
  int num_leds = 1;
  if(rssi > -50) {
    num_leds = 4; 
  } else if(rssi > -70) {
    num_leds = 3;
  } else if(rssi > -50) {
    num_leds = 2;
  }
  for(int i=num_leds-1; i>=0; i--) 
    digitalWrite(LED[i], HIGH);

  delay(delay_ms);
  
  for(int i=0; i<4; i++)
    digitalWrite(LED[i], LOW);
}

void audioFeedback(int rssi) {
  int scale_factor = 10;
  int center_rssi  = 75;

  int output_freq = CENTER_FREQ + (center_rssi + rssi)*scale_factor;
  tone(SPEAKER, output_freq);
}

void audioFeedback(int rssi, int duration) {
  int scale_factor = 10;
  int center_rssi  = 75;

  int output_freq = CENTER_FREQ + (center_rssi + rssi)*scale_factor;
  tone(SPEAKER, output_freq, duration);
}

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
//#define MYNODEID      1   // My node ID
//#define TONODEID      2   // Destination node ID
static int MYNODEID;
static int TONODEID;

// RFM69 frequency, uncomment the frequency of your module:
#define FREQUENCY     RF69_915MHZ

// Transmission power (0-31 = 5-20 dBm)
#define TXPOWER       31

// AES encryption (or not):
#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):
#define USEACK        true // Request ACKs or not

// define send rate, sensor IDs and pins
#define SENDDELAY     250
static long send_time = -1;
const int rssi_n = 4;
static int rssi_q[rssi_n];

#define LIGHTID       "01"
#define LIGHTPIN      A0
#define LIGHTSENSOR   false

#define TEMPID        "02"
#define TEMPPIN       A1
#define TEMPSENSOR    false

// indicator LEDs
//const int LED[4] = {5,6,7,8};
#define LED           9 // LED positive pin
#define GND           8 // LED ground pin

// Resonant frequency for audio feedback
#define CENTER_FREQ   2000
#define SPEAKER       3
#define SPEAKER_EN    9

// mode select switch
#define MODESELECT    4
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

// check battery voltage
#define VBAT          A0
#define VMAX          4.2
#define VMIN          2.8     // from datasheet for 3.7V, 400mAh LiPo battery

#define SERIAL        true
#define LCD           false

RFM69 radio;

void setup() {
  // Open a serial port so we can send keystrokes to the module:
  Serial.begin(9600);
  
  // Hard Reset the RFM module (only for adafruit modules)
  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  // select mode to transmit or receive based on a switch
  // on pin [MODESELECT]. This could also be moved to loop
  // to allow for 
  pinMode(MODESELECT, INPUT);
  delay(250);
  if(digitalRead(MODESELECT)) {
    mode = RECEIVE;
    MYNODEID = 1;
    TONODEID = 2;
    Serial.println("Receive Mode Selected");
  } else {
    mode = TRANSMIT;
    MYNODEID = 2;
    TONODEID = 1;
    Serial.print("Transmit Mode Selected");
  }
  if(SERIAL) 
    Serial.print("Node "); Serial.print(MYNODEID,DEC); Serial.println(" ready");  

  // Initialize radio
  radio.initialize(FREQUENCY,MYNODEID,NETWORKID);
  radio.setHighPower();    // Only for RFM69HCW & HW!
  radio.setPowerLevel(TXPOWER); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
  // promiscuous mode allows this unit to receive all transmissions on the network
  radio.promiscuous();
  radio.encrypt(ENCRYPTKEY);

  // setup LEDs and speaker as outputs
//  for(int i=0; i<4; i++) {
//    pinMode(LED[i], OUTPUT);
//  }
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
  pinMode(GND,OUTPUT);
  digitalWrite(GND,LOW);
  pinMode(SPEAKER, OUTPUT); 
  pinMode(SPEAKER_EN, OUTPUT);
  digitalWrite(SPEAKER_EN, HIGH);   // don't enable the speaker yet

  for (int i = 0; i < rssi_n; i++) {
    rssi_q[i] = 0;
  }

//  flashVoltage();
}

void loop() {
  int rssi;
  if(mode == TRANSMIT) {
    static String send_string = "";
    static int send_length = 0;
    static char send_buffer[62];
  
    send_string = String(String(millis(), DEC) + ";");

    Serial.print(send_string); Serial.print(" ");
    
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
    if (USEACK) {
      if (radio.sendWithRetry(TONODEID, send_buffer, send_length, 1)) {
        Serial.println("ACK received!");
        Serial.print("RSSI: ");
        Serial.println(radio.RSSI);
//        BlinkSS(radio.RSSI, 250);
      } else
        Serial.println("no ACK received");
    } else {
      radio.send(TONODEID, send_buffer, send_length);
    }  
    Blink(LED, SENDDELAY/2);
  
    send_length = 0; // reset the packet
  
    delay(SENDDELAY);
    
  } else {              // receive mode
    delay(10);
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
      rssi = pushRSSI(radio.RSSI);
  
      // Send an ACK if requested.
      // (You don't need this code if you're not using ACKs.)
      if (radio.ACKRequested()) {
        radio.sendACK();
        Serial.println("ACK sent");
      }
//      BlinkSS(radio.RSSI, 250);
//      audioFeedback(rssi);
      overwriteBeep();
      Blink(LED, SENDDELAY/2);
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
  tone(SPEAKER, output_freq);
}

void audioFeedback(int rssi, int duration) {
  int scale_factor = 10;
  int center_rssi  = 85;

  int output_freq = CENTER_FREQ + (center_rssi + rssi)*scale_factor;
  tone(SPEAKER, output_freq, duration);
}

void overwriteBeep() {
  digitalWrite(SPEAKER_EN, LOW);
  tone(SPEAKER,CENTER_FREQ,300);
  digitalWrite(SPEAKER_EN, HIGH);
}

// average the last n rssi values
int pushRSSI(int new_rssi) {
  int n = 0;
  float sum = 0;
  for (int i = 0; i < rssi_n - 1; i++)
    rssi_q[i] = rssi_q[i + 1];
  rssi_q[rssi_n - 1] = new_rssi;
  for (int i = 0; i < rssi_n; i++) {
    if (rssi_q[i] != 0) {
      sum += rssi_q[i];
      n += 1;
    }
  }
  return sum / n;
}

const long InternalReferenceVoltage = 1062;  // Adjust this value to your board's specific internal BG voltage
 

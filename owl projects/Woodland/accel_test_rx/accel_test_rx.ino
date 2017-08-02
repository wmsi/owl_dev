/* This software is intended to be run as an eample Owl activity for the 
 *  Woodland school. When powered on, each Owl will start up as either a
 *  transmitter or receiver, depending on the Mode Select switch. In 
 *  receive mode, the Owl searches for waypoints and releases a new compass 
 *  (pointing toward the transmitter) at each waypoint. The transmitter sits
 *  in a central location, broadcasting updates as the timer runs down. When 
 *  found, a button press on the transmitter will notify all receivers that it 
 *  has been retrieved.
 * 
 */


// include libraries for GPS and LCD setup
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Include the RFM69 and SPI libraries:
#include <RFM69.h>
#include <SPI.h>

#define LCD     false
#define SERIAL  false

// Choose two Arduino pins to use for software serial
// The GPS Shield uses D2 and D3 by default when in DLINE mode
#define GPSRX 5
#define GPSTX 4
#define LCDRX 7
#define LCDTX 6

// create a softwareSerial object to interface with the LCD
// we can use the same Rx and Tx because GPS only transmits, while LCD only receives

SoftwareSerial lcdSerial(LCDRX,LCDTX); // Rx, Tx
// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(GPSRX, GPSTX);

// Resonant frequency for audio feedback
#define CENTER_FREQ   2000
#define SPEAKER       3
#define SPEAKER_EN    9
boolean toggle = false;
boolean speaker_state = false;
int tone_length = 0;
int post_scale = 0;
int counter = 0;

// mode select switch
#define MODESELECT    4
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

// push to beep
#define PUSHTOBEEP    7

// The Skytaq EM-506 GPS module included in the GPS Shield Kit
// uses 4800 baud by default
int GPSBaud = 9600;

// Create a TinyGPS++ object called "gps"
// Addresses for this node. CHANGE THESE FOR EACH NODE!
#define NETWORKID     0   // Must be the same for all nodes
//#define MYNODEID      1   // My node ID
//#define TONODEID      2   // Destination node ID
static int MYNODEID;
static int TONODEID;

#define FREQUENCY     RF69_915MHZ

// Transmission power (0-31 = 5-20 dBm)
#define TXPOWER       31

// AES encryption (or not):
#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):
#define USEACK        true // Request ACKs or not

RFM69 radio;

void setup() {
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  if(LCD)
    lcdSerial.begin(9600);
  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);

  clearLCD();

  pinMode(MODESELECT, INPUT);
  pinMode(SPEAKER, OUTPUT); 
  pinMode(SPEAKER_EN, OUTPUT);
  digitalWrite(SPEAKER_EN, HIGH);   // don't enable the speaker yet
  pinMode(PUSHTOBEEP, INPUT_PULLUP);

  radioSetup();
  establishContact();
}

void loop() {
  String send_string;
  String message = "";
  if (radio.receiveDone()) {
    message = printMessage();
    Serial.println(message);
    if (radio.ACKRequested()) {
      radio.sendACK();
    }
  }
}


int getDistance(long lat_diff, long lng_diff) {
//  Serial.print("differences: "); Serial.print(lat_diff); Serial.print(", "); Serial.println(lng_diff);
  long raw_distance = sqrt(lat_diff*lat_diff + lng_diff*lng_diff);
//  Serial.print("distance: "); Serial.println(raw_distance);
  int distance_m= raw_distance*11;
  return raw_distance;
}

boolean radioSend(String send_string) {
  boolean ack_recd = false;
  byte send_length = send_string.length();
  static char send_buffer[62];
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
      ack_recd = true;
      Serial.println("ACK received!");
      Serial.print("RSSI: ");
      Serial.println(radio.RSSI);
    } else
      Serial.println("no ACK received");
  } else {
    radio.send(TONODEID, send_buffer, send_length);
  } 
  return ack_recd;
}

String printMessage() {
  char message[radio.DATALEN + 1];
//  Serial.print("received from node ");
//  Serial.print(radio.SENDERID, DEC);
//  Serial.print(", message [");

  // The actual message is contained in the DATA array,
  // and is DATALEN bytes in size:
  for (byte i = 0; i < radio.DATALEN; i++) {
//    Serial.print((char)radio.DATA[i]);
    message[i] = (char)radio.DATA[i];
  }
  message[radio.DATALEN] = '\0';

  // RSSI is the "Receive Signal Strength Indicator",
  // smaller numbers mean higher power.
//  Serial.print("], RSSI ");
//  Serial.println(radio.RSSI);

  return message;
}

void clearLCD() {
  lcdSerial.write(254); // move cursor to beginning of first line
  lcdSerial.write(128);

  lcdSerial.write("                "); // clear display
  lcdSerial.write("                ");
//  delay(50);
}

void writeLine(String str, int line, int pos) {
  char temp[16] = "               ";
  int num_chars = str.length();
  lcdSerial.write(254); // move cursor to beginning of first line
  switch(line) {
    case 2:
      lcdSerial.write(191+pos);
    case 1:
      lcdSerial.write(128+pos);
  }
  delay(10);
  for(int i=0; i<num_chars; i++) {
    temp[i] = str[i];
  }
  lcdSerial.write(temp);
//  delay(100);
}

void overwriteBeep() {
  digitalWrite(SPEAKER_EN, LOW);
  tone(SPEAKER,CENTER_FREQ,300);
//  analogWrite(SPEAKER, 127);
  digitalWrite(SPEAKER_EN, HIGH);
}

ISR(TIMER1_COMPA_vect){
  counter++;
  if(counter >= post_scale) {
    toggle = !toggle;
    counter = 0;
    if(toggle && post_scale != 0) {
      int frequency = map(post_scale,5,80,1500,1000);
      analogWrite(3, 127);
//      tone(3,frequency);
    } else {
      analogWrite(3, 0);
//      noTone(3);
    }
  }
}

void radioSetup() {
  if(digitalRead(MODESELECT)) {
    mode = RECEIVE;
    MYNODEID = 1;
    TONODEID = 2;
  } else {
    mode = TRANSMIT;
    MYNODEID = 2;
    TONODEID = 1;
  }
 
  // Initialize radio
  radio.initialize(FREQUENCY,MYNODEID,NETWORKID);
  radio.setHighPower();    // Only for RFM69HCW & HW!
  radio.setPowerLevel(TXPOWER); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
  // promiscuous mode allows this unit to receive all transmissions on the network
  radio.promiscuous();
  radio.encrypt(ENCRYPTKEY);

}

void establishContact() {
  while (Serial.available() <= 0) {
  Serial.println("A");   // send a capital A
  delay(300);
  }
}

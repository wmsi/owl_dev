/*  This Script is a basic rundown with only the bare essentials in coding
 *   the owl to be able to transmit a signal to another owl. 
 */


// include libraries for GPS and LCD setup
#include <SoftwareSerial.h>

// Include the RFM69 and SPI libraries:
#include <RFM69.h>
#include <SPI.h>

// Let the Arduino know there is an LCD and Serial connection
#define LCD     true
#define SERIAL  true

// Choose two Arduino pins to use for software serial
#define LCDRX 7
#define LCDTX 6

// mode select switch
#define MODESELECT    4
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

// push to beep
#define BUTTON    7
#define LOOP        true

#define NETWORKID     0   // Must be the same for all nodes
//#define MYNODEID      1   // My node ID
//#define TONODEID      2   // Destination node ID
static int MYNODEID;
static int TONODEID;

// AES encryption (or not):
#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes

#define FREQUENCY     RF69_915MHZ

// Transmission power (0-31 = 5-20 dBm)
#define TXPOWER       31

// Use ACKnowledge when sending messages (or not):
#define USEACK        true // Request ACKs or not

RFM69 radio;  //create radio object
const int s7sTx = 8;
const int s7sRx = 7;
SoftwareSerial s7s(s7sRx, s7sTx); //create serial object for interfacing with LCD

void setup() {
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);
  
  s7s.begin(9600);

  //clearDisplay();

  pinMode(MODESELECT, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  radioSetup();
}


void loop() {
  
  if (digitalRead(BUTTON) == LOW){
    broadcastMessage();
    delay(2000);
  }


}

boolean radioSend(String send_string) {
  boolean ack_recd = false;
  byte send_length = send_string.length();
  static char send_buffer[62];
  for (byte i = 0; i < send_length; i++) {
    send_buffer[i] = send_string[i];
  }
  if (USEACK) {
    if (radio.sendWithRetry(TONODEID, send_buffer, send_length, 1)) {
      ack_recd = true;
    }
  } else {
    radio.send(TONODEID, send_buffer, send_length);
  } 
  return ack_recd;
}

//Sends message - allows to alter which message and then send via radio
void broadcastMessage() {
  String send_string = "Hello World!";
  
  if(radioSend(send_string)) {  
    int last_broadcast = millis();
  }
}

// Send the clear display command (0x76)
//  This will clear the display and reset the cursor
void clearDisplay() {
  s7s.write(0x76);  // Clear display command
}

// Turn on any, none, or all of the decimals.
//  The six lowest bits in the decimals parameter sets a decimal 
//  (or colon, or apostrophe) on or off. A 1 indicates on, 0 off.
//  [MSB] (X)(X)(Apos)(Colon)(Digit 4)(Digit 3)(Digit2)(Digit1)
void setDecimals(byte decimals) {
  s7s.write(0x77);
  s7s.write(decimals);
}

//Radio Setup Function
void radioSetup() {
  mode = TRANSMIT;
  MYNODEID = 2;
  TONODEID = 1;
  
  // Initialize radio
  radio.initialize(FREQUENCY,MYNODEID,NETWORKID);
  radio.setHighPower();    // Only for RFM69HCW & HW!
  radio.setPowerLevel(TXPOWER); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
  // promiscuous mode allows this unit to receive all transmissions on the network
  radio.promiscuous();
  radio.encrypt(ENCRYPTKEY);

}



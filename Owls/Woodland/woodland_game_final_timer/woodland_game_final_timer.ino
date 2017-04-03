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
#define GPS     false
#define SERIAL  true

// Choose two Arduino pins to use for software serial
// The GPS Shield uses D2 and D3 by default when in DLINE mode
#define GPSRX 5
#define GPSTX 4
#define LCDRX 7
#define LCDTX 6

// create a softwareSerial object to interface with the LCD
// we can use the same Rx and Tx because GPS only transmits, while LCD only receives

// mode select switch
#define MODESELECT    4
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

// push to beep
#define BLUEWIRE    7
#define REDWIRE     6
#define LOOP        true

const int allotted_time = 25; //minutes
const int broadcast_interval = 540; //seconds
const int s7sTx = 8;
const int s7sRx = 7;
SoftwareSerial s7s(s7sRx, s7sTx);
unsigned long last_update = 0;
unsigned long last_broadcast = 0;
boolean flash_state = false;
boolean stop_display = false;
unsigned int stop_time = 0;

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
  
  s7s.begin(9600);

  clearDisplay();

  pinMode(MODESELECT, INPUT);
  pinMode(BLUEWIRE, INPUT_PULLUP);
  pinMode(REDWIRE, INPUT_PULLUP);

  radioSetup();
}

void loop() {
  String send_string;
  String message = "";
  if (radio.receiveDone()) {
    message = printMessage();
  }
  // update countdown timer
  updateDisplay();

  // broadcast time update
  if((millis()-last_broadcast)/1000 >= broadcast_interval) {
    Serial.println("time udpate");
    broadcastTime();
  }

  if(digitalRead(BLUEWIRE)) {
    radioSend("owl found");
    stop_display = true;
  }
  if(digitalRead(REDWIRE)) {
    fastCount();
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

String printMessage() {
  char message[radio.DATALEN + 1];
  for (byte i = 0; i < radio.DATALEN; i++) {
    message[i] = (char)radio.DATA[i];
  }
  message[radio.DATALEN] = '\0';
  if (radio.ACKRequested()) {
    radio.sendACK();
  }

  return message;
}

void updateDisplay() {
  if(millis() - last_update >= 1000) {
    int start_time = allotted_time*60;
    int elapsed_time = millis()/1000;
    int time_remaining = start_time - elapsed_time;
    while(LOOP && time_remaining < 0) {
      time_remaining += 60;
    }
    
    if(stop_display) {
      if(stop_time == 0)
        stop_time = start_time - elapsed_time;
      time_remaining = stop_time;
      flash_state = !flash_state;
    }
    
    unsigned int display_time = max((100*(time_remaining/60) + time_remaining%60),0);
    writeTime(display_time);
  }
}

// count down at double the speed until the blue wire is pulled
void fastCount() {
  int time_remaining = allotted_time*60 - millis()/1000;
  while(time_remaining < 0) {
    time_remaining += 60;
  }
  unsigned long last_write = millis();
  unsigned int display_time;
  while(time_remaining > 0) {
    if(millis() - last_write >= 350) {
      display_time = 100*(time_remaining/60) + time_remaining%60;
      writeTime(display_time);
      time_remaining--;
      last_write = millis();
    }
    if(digitalRead(BLUEWIRE))
      break;
  }
  // flash the last time on the clock forever
  while(1) {
    if(millis() - last_write > 1000) {
      flash_state = !flash_state;
      writeTime(display_time);
      last_write = millis();
    }
  }
}

void writeTime(unsigned int display_time) {
  char tempString[10];
  sprintf(tempString, "%4d", display_time);
  if(display_time < 60)
    tempString[1] = '0';
  if(display_time < 10)
    tempString[2] = '0';
  if(flash_state) {
    for(int i=0;i<4;i++)
      tempString[i] = '-';
  }
  s7s.print(tempString);
  setDecimals(0b00010000);
  last_update = millis();
}

void broadcastTime() {
  String send_string;
  int time_remaining = allotted_time - (millis()/60000);
  if(time_remaining <= 0) 
    send_string = "Out of time!";
  else
    send_string = String(String(time_remaining) + " min. left!");

  // keep trying on future iterations until ack received
  if(radioSend(send_string)) {  
    last_broadcast = millis();
  }
}

// Turn on any, none, or all of the decimals.
//  The six lowest bits in the decimals parameter sets a decimal 
//  (or colon, or apostrophe) on or off. A 1 indicates on, 0 off.
//  [MSB] (X)(X)(Apos)(Colon)(Digit 4)(Digit 3)(Digit2)(Digit1)
void setDecimals(byte decimals) {
  s7s.write(0x77);
  s7s.write(decimals);
}

// Send the clear display command (0x76)
//  This will clear the display and reset the cursor
void clearDisplay() {
  s7s.write(0x76);  // Clear display command
}

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


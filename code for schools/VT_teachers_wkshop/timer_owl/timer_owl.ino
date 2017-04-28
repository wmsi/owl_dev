/* This software is intended to be run as an example GPS game for a
    teachers' workshop at the Lyndon town school. In seeker mode, the 
    Owl searches for waypoints, always displaying the distance to the 
    nearest waypoint and emitting faster beeps (with a button press)
    the closer you get. It then releases a new compass bearing 
    (pointing toward the hidden Owl) at each waypoint.
    The hidden Owl sits in a central location, broadcasting updates as the
    timer runs down. When found, a button press on the transmitter will
    notify all receivers that it has been retrieved.

    The final lock code is 9536
*/

// include libraries for GPS and LCD setup
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Include the RFM69 and SPI libraries:
#include <RFM69.h>
#include <SPI.h>
#define GPS     false
#define SERIAL  true

// Choose two Arduino pins to use for software serial
// The GPS Shield uses D2 and D3 by default when in DLINE mode
#define GPSRX 5
#define GPSTX 4

// mode select switch
#define MODESELECT    4
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

// push to beep
#define BUTTON        7

// create a softwareSerial object to talk to the display
#define s7sTx     8
#define s7sRx     7
SoftwareSerial s7s(s7sRx, s7sTx);

// setup variables for updating the timer
#define RESTART     false     // restart the timer continually with one minute left
const int allotted_time = 90; //minutes
const int broadcast_interval = 600; //seconds
unsigned long last_update = 0;
unsigned long last_broadcast = 0;
unsigned long last_flash = 0;
boolean flash_state;
boolean stop_display = false;
unsigned int stop_time = 0;

// setup variables for the internal speaker
// SPEAKER_EN should be connected to the ground pin of the speaker
// when SPEAKER_EN is LOW (gnd) the speaker can make noise
#define CENTER_FREQ   2048    // resonant frequency
#define SPEAKER       3
#define SPEAKER_EN    9
boolean toggle = false;
boolean speaker_state = false;
int tone_length = 0;
int post_scale = 0;
int counter = 0;

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
#define USEACK        false // Request ACKs or not

RFM69 radio;

void setup() {
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);
  
  s7s.begin(9600);

  clearDisplay();

  pinMode(MODESELECT, INPUT);
  pinMode(SPEAKER, OUTPUT);
  pinMode(SPEAKER_EN, OUTPUT);
  digitalWrite(SPEAKER_EN, LOW);

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
  
  // check for push to beep message
  if(message.indexOf("beep") != -1) {
    overwriteBeep();
  }
  
  // broadcast time update
  if((millis()-last_broadcast)/1000 >= broadcast_interval) {
    Serial.println("time update");
    broadcastTime();
  }

  // check if button was pressed, signalling unit found
  if(digitalRead(BUTTON) == LOW) {
    radioSend("owl found");
    stop_display = true;
  }
}

/*
 * This function enables the speaker and then plays a tone for 300 ms.
 * Tone can be played with the tone() function (which is louder but 
 * sometimes sees interference from softwareSerial) or with PWM via
 * analogWrite.
 */
void overwriteBeep() {
  analogWrite(SPEAKER, 127);
  delay(100);
  analogWrite(SPEAKER, 0);
}
/////////////////////////// FUNCTIONS FOR THE 7 SEGMENT DISPLAY //////////////////////////

/*
 * updateDisplay() writes a new time to the display every second. Usually
 * this is just the allotted time - the amount of time the sketch has
 * been running. However, if RESTART has been set to true, the timer will
 * keep adding a minute instead of running down to 0.
 */
void updateDisplay() {
  if(millis() - last_update >= 1000) {
    int start_time = allotted_time*60;
    int elapsed_time = millis()/1000;
    int time_remaining = start_time - elapsed_time;
    while(RESTART && time_remaining < 0) {
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

/*
 * write a time to the display. display time must be an unsigned integer between
 * 0 and 9999. For most timer applications the last two digits should be less 
 * than 60. E.g. display_time = 549 -> 5:49
 */
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

/*
 * broadcast a time update to the seeking Owls. This function will get called
 * regularly as defined by broadcast_time. This function could also be used
 * to send clues to the seekers as the timer runs down.
 */
void broadcastTime() {
  if(SERIAL)
    Serial.println("sending time update");
  String send_string;
  int time_remaining = allotted_time - (millis()/60000);
  if(time_remaining <= 0) 
    send_string = "Out of time!";
  else
    send_string = String(String(time_remaining) + " min. left!");

  // keep trying on future iterations until ack received
  if(!USEACK || (USEACK &&radioSend(send_string))) {  
    last_broadcast = millis();
  }
}

// Turn on any, none, or all of the decimals.
//  The six lowest bits in the decimals parameter sets a decimal 
//  (or colon, or apostrophe) on or off. A 1 indicates on, 0 off.
//  [MSB] (X)(X)(Apos)(Colon)(Digit 4)(Digit 3)(Digit2)(Digit1)
void setDecimals(byte decimals)
{
  s7s.write(0x77);
  s7s.write(decimals);
}

// Send the clear display command (0x76)
//  This will clear the display and reset the cursor
void clearDisplay()
{
  s7s.write(0x76);  // Clear display command
}


/*
   Send out a message via the radio board. If USEACK is set to true,
   wait for an acknowledgment as dictated by sendWithRetry(). 
   From RFM69.h:
   sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, 
                 uint8_t retries=2, uint8_t retryWaitTime=40);

   return true if an ACK was received
*/
bool radioSend(String send_string) {
  bool ack_recd = false;
  static int send_length = 0;
  static char send_buffer[RF69_MAX_DATA_LEN];

  send_length = min(send_string.length(),RF69_MAX_DATA_LEN);
  if(SERIAL) {
    Serial.print("sending to node "); Serial.print(TONODEID, DEC); Serial.print(", message: ");
    Serial.println(send_string);
  }
  for (byte i = 0; i < send_length; i++) {
    send_buffer[i] = send_string[i];
  }

  // There are two ways to send packets. If you want
  // acknowledgements, use sendWithRetry():
  if (USEACK) {
    if (radio.sendWithRetry(TONODEID, send_buffer, send_length, 1)) {
      if(SERIAL) {
        Serial.println("ACK received!");
        Serial.print("RSSI: "); Serial.println(radio.RSSI);
      }
      ack_recd = true;
    } else if(SERIAL) 
      Serial.println("No ACK received");
  } else {
    radio.send(TONODEID, send_buffer, send_length);
  } 
  return ack_recd;
}

/*
 * Process a new message from the radio board. Send ACK if ACK
 * is turned on, and print the message over Serial if SERIAL
 * is set to true. 
 */
String printMessage() {
  char message[radio.DATALEN + 1];
  // The actual message is contained in the DATA array,
  // and is DATALEN bytes in size:
  for (byte i = 0; i < radio.DATALEN; i++) {
    message[i] = (char)radio.DATA[i];
  }
  message[radio.DATALEN] = '\0';

  if (SERIAL) {
    Serial.print("received from node ");
      Serial.print(radio.SENDERID, DEC);
      Serial.print(", message: "); 
      Serial.println(message);
  }
  if (radio.ACKRequested()) {
    radio.sendACK();
    if (SERIAL)
      Serial.println("ACK sent");
  }
  return message;
}

/*
 * Currently this sketch only contains code to run in receive mode, 
 * since adding support for the timer display overloads the DIO pins.
 * To restore mode functionality simply uncomment the code in setup
 * and add mode handling in loop()
 */
void radioSetup() {
  if (digitalRead(MODESELECT)) {
    mode = RECEIVE;
    MYNODEID = 1;
    TONODEID = 2;
  } else {
    mode = TRANSMIT;
    MYNODEID = 2;
    TONODEID = 1;
  }
  if(SERIAL)
    Serial.print(String(mode==RECEIVE ? "Receive" : "Transmit") + " mode selected");
      
  // Initialize radio
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower();    // Only for RFM69HCW & HW!
  radio.setPowerLevel(TXPOWER); // power output ranges from 0 (5dBm) to 31 (20dBm)

  // promiscuous mode allows this unit to receive all transmissions on the network
  radio.promiscuous();
  radio.encrypt(ENCRYPTKEY);
}


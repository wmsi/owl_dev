/* This is the basic script to run an owl that receives a signal and then displays it onto 
 *  an LCD display. It requires a transmitting owl in order to get a 
 *  message.
 */

// include libraries for GPS and LCD setup
#include <SoftwareSerial.h>

// Include the RFM69 and SPI libraries:
#include <RFM69.h>
#include <SPI.h>

#define LCD     true
#define SERIAL  true

// Choose two Arduino pins to use for software serial
#define LCDRX 7
#define LCDTX 6

// button pin
#define BUTTON    7
#define DEBOUNCE  200

// Radio stuff
#define NETWORKID     0   // Must be the same for all nodes
static int MYNODEID;
static int TONODEID;
#define FREQUENCY     RF69_915MHZ
#define TXPOWER       31
#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes
#define USEACK        true // Request ACKs or not

#define CENTER        512

// create a softwareSerial object to interface with the LCD
// we can use the same Rx and Tx because GPS only transmits, while LCD only receives

SoftwareSerial lcdSerial(LCDRX,LCDTX); // Rx, Tx

//create a radio object
RFM69 radio;

void setup() {
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  if(LCD)
    lcdSerial.begin(9600);
    clearLCD();
    
  // Start the software serial port at the GPS's default baud
 // gpsSerial.begin(GPSBaud);

  radioSetup();

  pinMode(BUTTON, INPUT_PULLUP);
}

void loop() {
  //get message from transmitting owl then print it
  String message = "";
  if (radio.receiveDone()) {
    message = printMessage();
  }
  writeLine(message, 1, 0);
  delay(2000);  
}

/*                        LCD Stuff                       */
//Function to write a line to LCD
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
}
//Clears LCD screen
void clearLCD() {
  lcdSerial.write(254); // move cursor to beginning of first line
  lcdSerial.write(128);

  lcdSerial.write("                "); // clear display
  lcdSerial.write("                ");
}

//////////////               radio functions            ///////////////

void radioSetup() {
  MYNODEID = 1;
  TONODEID = 2;
  
  // Initialize radio
  radio.initialize(FREQUENCY,MYNODEID,NETWORKID);
  radio.setHighPower();    // Only for RFM69HCW & HW!
  radio.setPowerLevel(TXPOWER); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
  // promiscuous mode allows this unit to receive all transmissions on the network
  radio.promiscuous();
  radio.encrypt(ENCRYPTKEY);

}

//Bool function that send radio message and lets know if succeeded
boolean radioSend(String send_string) {
  boolean ack_recd = false;
  byte send_length = send_string.length();
  static char send_buffer[62];
  for (byte i = 0; i < send_length; i++) {
    send_buffer[i] = send_string[i];
  }

  // There are two ways to send packets. If you want
  // acknowledgements, use sendWithRetry():
  if (USEACK) {
    if (radio.sendWithRetry(TONODEID, send_buffer, send_length, 1)) {
      ack_recd = true;
    }
  } else {
    radio.send(TONODEID, send_buffer, send_length);
  } 
  return ack_recd;
}

//Returns Message received by radio 
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

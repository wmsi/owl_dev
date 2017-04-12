/*
 * This example demonstrates basic transmission/ reception of
 * radio messages using the RFM board. Transmit/ receive mode
 * is designated at startup by the MODESELECT switch, and messages
 * can be sent by pressing a button or at regularly timed
 * intervals. By default the message is the number of milliseconds
 * that the program has been running.
 */
 
// Include the RFM69 and SPI libraries for using the RFM board
#include <RFM69.h>
#include <SPI.h>

// mode select switch will tell the unit to enter transmit or receive mode
#define MODESELECT    4
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

// Addresses for this node, assigned at startup based on transmit/ receive mode
#define NETWORKID     0   // Must be the same for all nodes
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

// all-purpose button. In this sketch it used to send a 
// "button pressed!" message every time it is pressed
#define BUTTON        7

// how often do we want to send messages? (in milliseconds)
#define SENDDELAY     1000

void setup() {
  Serial.begin(9600);

  pinMode(MODESELECT, INPUT);
//  pinMode(SPEAKER, OUTPUT);

  // Enable the built-in pullup resistor and read a button press
  // any time it gets connected to ground
  pinMode(BUTTON, INPUT_PULLUP);
  
  radioSetup();
}

void loop() {
  static unsigned long last_send = 0;
  if(mode == RECEIVE) {
    String message = "";
    if (radio.receiveDone()) {
      message = printMessage();

      // In a real application this is where the message would be
      // processed. The built-in function string.indexOf() is useful
      // for parsing messages.
    }
  } else {
    if(millis() - last_send > SENDDELAY) {
      
      // replace this with an actual message
      String send_string = String(millis());
      radioSend(send_string);
      last_send = millis();
    }
    if(digitalRead(BUTTON) == LOW) {
      radioSend("button pressed!");
    }
  }
}


/*
   Broadcast a message via the radio board. If USEACK is set to true,
   wait for an acknowledgment as dictated by sendWithRetry(). From 
   RFM69.h:
   sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, 
                 uint8_t retries=2, uint8_t retryWaitTime=40);

   return true if an ACK was received
*/
bool radioSend(String send_string) {
  bool ack_recd = false;
  byte send_length = min(send_string.length(),RF69_MAX_DATA_LEN);
  char send_buffer[RF69_MAX_DATA_LEN];

  Serial.print("sending to node "); Serial.print(TONODEID, DEC); Serial.print(", message: ");
  Serial.println(send_string);
  for (byte i = 0; i < send_length; i++) {
    send_buffer[i] = send_string[i];
  }

  // There are two ways to send packets. If you want
  // acknowledgements, use sendWithRetry():
  if (USEACK) {
    if (radio.sendWithRetry(TONODEID, send_buffer, send_length, 1)) {
      Serial.println("ACK received!");
      Serial.print("RSSI: "); Serial.println(radio.RSSI);
      ack_recd = true;
    } else
      Serial.println("No ACK received");
  } else {
    radio.send(TONODEID, send_buffer, send_length);
  } 
  return ack_recd;
}

/*
   If a new message has been received on the RFM board,
   convert it to a String and sent an acknowledgment
   message, if requested.  Return the received message, 
   cast as a String
*/
String printMessage() {
  char message[radio.DATALEN + 1];
  // The actual message is contained in the DATA array,
  // and is DATALEN bytes in size:
  for (byte i = 0; i < radio.DATALEN; i++) {
    message[i] = (char)radio.DATA[i];
  }
  message[radio.DATALEN] = '\0';

  Serial.println(message);
  if (radio.ACKRequested()) {
    radio.sendACK();
    Serial.println("ACK sent");
  }
  return message;
}


/*
 * Initialize the radio board into transmit or receive mode.
 * By default, radio.setPowerLevel() is used to set the output
 * power as high as possible, and the radio is set to promiscuous 
 * mode. If you want a radio unit to only receive messages addressed
 * its node, get rid of the line radio.promiscuous().
 * 
 * The encryption option has been kept around from the Sparkfun 
 * example sketches for this board but probably isn't necessary 
 * for Owl applications.
 */
void radioSetup() {
  if (digitalRead(MODESELECT)) {
    mode = RECEIVE;
    MYNODEID = 1;
    TONODEID = 2;
    Serial.println("Receive Mode Selected");
  } else {
    mode = TRANSMIT;
    MYNODEID = 2;
    TONODEID = 1;
    Serial.println("Transmit Mode Selected");
  }
  
  // Initialize radio
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower();    // Only for RFM69HCW & HW!
  radio.setPowerLevel(TXPOWER); // power output ranges from 0 (5dBm) to 31 (20dBm)

  // promiscuous mode allows this unit to receive all transmissions on the network
  radio.promiscuous();
  radio.encrypt(ENCRYPTKEY);
}

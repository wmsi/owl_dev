/*
 * This example demonstrates basic transmission/ reception of
 * radio messages using the Feather with LoRa radio. Transmit/ 
 * receive mode is designated at startup by the MODESELECT switch, 
 * and messages can be sent by pressing a button or at regularly 
 * timed intervals. By default the message is the number of 
 * milliseconds that the program has been running.
 */

// Include the RFM69 and SPI libraries for using the RFM board
#include <SPI.h>
#include <RH_RF95.h>

/* for feather m0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Create an instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
#define TXPOWER   23

// mode select switch
#define MODESELECT    6
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

// define send rate, sensor IDs and pins
#define SENDDELAY     1000

// all-purpose button. In this sketch it used to send a 
// "button pressed!" message every time it is pressed
#define BUTTON        6
 
void setup() {
  Serial.begin(9600);

  pinMode(RFM95_RST, OUTPUT);
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
    if (rf95.available()) {
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
  }
}


/*
   If a new message has been received on the LoRa board,
   convert it to a String and send an acknowledgment
   message. Return the received message, cast as a String
*/
String printMessage() {
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  String message = "";
  
  if (rf95.recv(buf, &len)) {
    message = String((char*)buf);
    Serial.print("Received: "); Serial.println(message);
    Serial.print("RSSI: "); Serial.println(rf95.lastRssi(), DEC);

    // Send a reply
    uint8_t data[] = "ACK";
    rf95.send(data, sizeof(data));
    rf95.waitPacketSent();
  }
  return message;
}

/*
   Broadcast a message via the radio board. Note that, unlike the 
   Arduino-based Owls, The LoRa boards do not have acknowledgments
   built in to their protocol. Therefore in order for the transmit
   unit to receive a reply, the receiver must process the incoming 
   message and send a reply.

   From RHGenericDriver.h:
   waitAvailableTimeout(uint16_t timeout); 
    Starts the receiver and blocks until a received message is 
    available or a timeout
    \param[in] timeout Maximum time to wait in milliseconds.
    \return true if a message is available
*/
bool radioSend(String send_string) {
  int send_length = min(RH_RF95_MAX_MESSAGE_LEN, send_string.length());
  char send_buffer[RH_RF95_MAX_MESSAGE_LEN];
  bool reply_recd = false;
  
  for (byte i = 0; i < send_length; i++) {
    send_buffer[i] = send_string[i];
  }

  Serial.print("sending message: "); Serial.println(send_string);
  rf95.send((uint8_t *)send_buffer, send_length);
  
  delay(20);
  rf95.waitPacketSent();
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(250)) { 
    if (rf95.recv(buf, &len)) {
      Serial.print("Reply RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);  
      reply_recd = true;
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply.");
  }
}

/*
 * Initialize the radio board into transmit or receive mode.
 * By default, radio.setTxPower() is used to set the output
 * power as high as possible.
 * 
 */
void radioSetup() {
  if(digitalRead(MODESELECT)) {
    mode = RECEIVE;
    Serial.println("Receive Mode Selected");
  } else {
    mode = TRANSMIT;
    Serial.println("Transmit Mode Selected");
  }
  
  // manuallly reset the radio module
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  rf95.init();
  rf95.setFrequency(RF95_FREQ);
  rf95.setTxPower(TXPOWER, false);
}


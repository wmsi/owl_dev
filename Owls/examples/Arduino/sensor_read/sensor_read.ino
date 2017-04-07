#include <RFM69.h>
#include <SPI.h>

// Addresses for this node. CHANGE THESE FOR EACH NODE!
#define NETWORKID     0   // Must be the same for all nodes
static int MYNODEID = 2;  // My node ID
static int TONODEID = 1;  // Destination node ID

// mode select switch
#define MODESELECT    4
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

// RFM69 frequency
#define FREQUENCY     RF69_915MHZ

// Transmission power (0-31 = 5-20 dBm)
#define TXPOWER       31

// AES encryption (or not):
#define ENCRYPT       true // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):
#define USEACK        true // Request ACKs or not

// define send rate, sensor IDs and pins
#define SENDDELAY     500
unsigned long last_send = 0;

#define LIGHTID       "01"
#define LIGHTPIN      A0
#define LIGHTSENSOR   true

#define TEMPID        "02"
#define TEMPPIN       A1
#define TEMPSENSOR    true

#define ACCELID       "03"
#define ACCELPINX     A1
#define ACCELPINY     A2
#define ACCELPINZ     A3
#define ACCEL         false

// Create a library object for our RFM69HCW module:
RFM69 radio;

//turn Serial prints on or off. Sometimes Serial can cause errors
//when not connected to the computer, thoough this behavior has
//never been observed with Arduinos
#define SERIAL        true

void setup() {
  // Open a serial port so we can send keystrokes to the module:
  if(SERIAL)
    Serial.begin(9600);
  
  pinMode(MODESELECT, INPUT);
  
  radioSetup();
}

void loop() {
  String send_string;
  String message;
  if(mode == TRANSMIT) {
    if(millis() - last_send > SENDDELAY) {
      send_string = String(String(millis(), DEC) + ";");
      send_string = readSensors();
  
      radioSend(send_string); 
    }
  } else {
    // In this section, we'll check with the RFM69HCW to see
    // if it has received any packets:
    if (radio.receiveDone()) {
      message = printMessage();
    }
    /*
     * At this point sensor data may be processed by the receiver
     * or stored for later use. You can use message.indexOf() to
     * parse the String by searching for colons, semicolons, and commas
     */
  }
}

/*
 * Read all of the sensors designated at the top of the sketch and
 * package their data into send_string. Note that some massaging
 * of data will be necessary (i.e. map(reading,0,1023,0,newMax))
 * in order to fit all of the information into one radio transmission.
 */
String readSensors() {
  String send_string;
  if(LIGHTSENSOR) {
    String light_reading = String(map(analogRead(LIGHTPIN),0,1023,0,100));
    send_string = addSensorData(send_string, LIGHTID, light_reading);
  }
  
  if(TEMPSENSOR) {
    float voltage = (analogRead(TEMPPIN) * 0.00322581);
    float tempC = (voltage - 0.5) * 100.0;
    String temp_reading = String(tempC,2);
    send_string = addSensorData(send_string, TEMPID, temp_reading);
  }

  if(ACCEL) {
    int accelx = map(analogRead(ACCELPINX),0,1023,0,100);
    int accely = map(analogRead(ACCELPINY),0,1023,0,100);
    int accelz = map(analogRead(ACCELPINZ),0,1023,0,100);
    String accel_reading = String(String(accelx) + "," + String(accely) + "," + String(accelz));
    send_string = addSensorData(send_string, ACCELID, accel_reading);
  }
  return send_string;
}

/*
 * add data from a sensor to the upcoming transmission. 
 * Transmission format is [sensor id]:[sensor value],...,[sensor value];
 */
String addSensorData(String send_string, String id, String reading) {
  send_string = String(send_string + id);
  send_string = String(send_string + ":");
  send_string = String(send_string + reading);
  send_string = String(send_string + ";");
  return send_string;
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
  static int send_length = 0;
  static char send_buffer[62];

  send_length = min(send_string.length(),62);
  Serial.print("sending to node "); Serial.print(TONODEID, DEC); Serial.print(", message: ");
  Serial.println(send_string);
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
 * Initialize the radio board into transmit or receive mode.
 * By default, radio.setPowerLevel() is used to set the output
 * power as high as possible, and the radio is set to promiscuous 
 * mode. If you want a radio unit to only receive messages addressed
 * its node, get rid of the line radio.promiscuous().
 * 
 * The encryption option has been kept around from the example 
 * sketches for this board but probably isn't necessary for
 * Owl applications.
 */
void radioSetup() {
  
  if(digitalRead(MODESELECT)) {
    mode = RECEIVE;
    MYNODEID = 1;
    TONODEID = 2;
    if(SERIAL)
      Serial.println("Receiving at 915 MHz");
  } else {
    mode = TRANSMIT;
    MYNODEID = 2;
    TONODEID = 1;
    if(SERIAL)
      Serial.println("Transmitting at 915 MHz");
  }
  if(SERIAL)
    Serial.print("Node "); Serial.print(MYNODEID,DEC); Serial.println(" ready");  

  // Initialize radio
  radio.initialize(FREQUENCY,MYNODEID,NETWORKID);
  radio.setHighPower();    // Only for RFM69HCW & HW!
  radio.setPowerLevel(TXPOWER); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
  // promiscuous mode allows this unit to receive all transmissions on the network
  radio.promiscuous();
  if(ENCRYPT)
    radio.encrypt(ENCRYPTKEY);
}


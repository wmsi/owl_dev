/*
 * This example demonstrates how to collect GPS data and broadcast it 
 * using the RFM69HCW breakout board
 */
 
// Include the RFM69 and SPI libraries for using the RFM board
#include <RFM69.h>
#include <SPI.h>

// Include TinyGPS and SoftwareSerial libraries for using the GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// mode select switch will tell the unit to enter transmit or receive mode
#define MODESELECT    4
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

// Addresses for this node. CHANGE THESE FOR EACH NODE!
#define NETWORKID     0   // Must be the same for all nodes
static int MYNODEID;   // My node ID
static int TONODEID;   // Destination node ID

// RFM69 frequency, uncomment the frequency of your module:
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

// SENDDELAY defines how often to send messages (1000 = 1 send/ second)
#define SENDDELAY     30000

#define GPSRX         5
#define GPSTX         4
#define GPS           true
#define DATE          false
#define TIME          true
#define GPSBAUD       9600
#define NUMDECS       4       //number of decimals to include

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(GPSRX, GPSTX);

//enable or disable SERIAL for debug messages
#define SERIAL true

void setup()
{
  Serial.begin(9600);

  pinMode(MODESELECT, INPUT);
//  pinMode(SPEAKER, OUTPUT);

  // Enable the built-in pullup resistor and read a button press
  // any time it gets connected to ground
  pinMode(BUTTON, INPUT_PULLUP);
  
  radioSetup();
  
  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBAUD);
}

void loop()
{
  if(mode == TRANSMIT) {
    static String send_string = "";
    static unsigned long send_time = 0;
    static bool sat_acquired = false;
    
    if(GPS) {
      while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
          send_string = addGPSData();
          sat_acquired = true;
        }
      }
    }
    
    if(millis() - send_time > SENDDELAY) {
      if(sat_acquired) {
        if(DATE)
          send_string = addDate(send_string);
        if(TIME)
          send_string = addTime(send_string);
      } else {
        send_string = "waiting for satellites...";
      }
      radioSend(send_string);
      
      send_time = millis();
    }
  } else {
    String message = "";
    if (radio.receiveDone()) {
      message = printMessage();

      // In a real application this is where the message would be
      // processed. The built-in function string.indexOf() is useful
      // for parsing messages.
    }
  }
}

/*
 * Add the GPS location data to the send string. The GPS can also provide
 * information about time, date, and altitude. For more information on 
 * these funcitons check out TeinyGPS++.h
 */
String addGPSData() {
  String send_string = "LOC:";
  if(gps.location.isValid()) {
    send_string = String(send_string + String(gps.location.lat(), 5));
    send_string = String(send_string + ",");
    send_string = String(send_string + String(gps.location.lng(), 5));
  } else {
    send_string = String(send_string + "ERR");
  }
  send_string = send_string + ";";
  return send_string;
}

/*
 * Add the time as reported by the GPS to the send_string. Note that 
 * the time is always in the GMT timezone and 
 */
String addDate(String send_string) {
  send_string = String(send_string + "DAT:");
  if(gps.date.isValid()) {
    send_string = String(send_string + String(gps.date.month())); send_string = String(send_string + "/");
    send_string = String(send_string + String(gps.date.day())); send_string = String(send_string + "/");
    send_string = String(send_string + String(gps.date.year())); send_string = String(send_string + "; ");
  } else {
    send_string = String(send_string + "ERR;"); 
  }

  return send_string;
}

/*
 * Add the time as reported by the GPS to the send_string. Note that 
 * the time is always in the GMT timezone and 
 */
String addTime(String send_string) {
  send_string = String(send_string + "TIM:");
  if(gps.time.isValid()) {
    
    if (gps.time.hour() < 10) 
      send_string = String(send_string + "0"); 
    send_string = String(send_string + String(gps.time.hour()));
    send_string = String(send_string + ":");
    
    if (gps.time.minute() < 10) 
      send_string = String(send_string + "0"); 
    send_string = String(send_string + String(gps.time.minute()));
    send_string = String(send_string + ":");
    
    if (gps.time.second() < 10) 
      send_string = String(send_string + "0"); 
    send_string = String(send_string + String(gps.time.second()));
    send_string = String(send_string + ";");
  } else {
    send_string = String(send_string + "ERR;");
  }
  return send_string;
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

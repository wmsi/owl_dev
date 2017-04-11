/*
 * This example demonstrates how to collect GPS data and broadcast it 
 * using the RFM69HCW breakout board
 */
// Include the RFM69 and SPI libraries:
#include <RFM69.h>
#include <SPI.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Addresses for this node. CHANGE THESE FOR EACH NODE!
#define NETWORKID     0   // Must be the same for all nodes
#define MYNODEID      2   // My node ID
#define TONODEID      1   // Destination node ID

// RFM69 frequency, uncomment the frequency of your module:
#define FREQUENCY     RF69_915MHZ

// Transmission power (0-31 = 5-20 dBm)
#define TXPOWER       31

// AES encryption (or not):
#define ENCRYPT       false // Set to "true" to use encryption
#define ENCRYPTKEY    "TOPSECRETPASSWRD" // Use the same 16-byte key on all nodes

// Use ACKnowledge when sending messages (or not):
#define USEACK        true // Request ACKs or not

// Packet sent/received indicator LED (optional):
#define LED           9 // LED positive pin
#define GND           8 // LED ground pin

// SENDDELAY defines how often to send messages (1000 = 1 send/ second)
#define SENDDELAY     1000

#define GPSRX         3
#define GPSTX         4
#define GPS           true
#define DATE          false
#define TIME          true
#define GPSBAUD       9600
#define NUMDECS       4       //number of decimals to include

// Create a library object for our RFM69HCW module:
RFM69 radio;

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(GPSRX, GPSTX);

void setup()
{
  // Open a serial port so we can send keystrokes to the module:
  Serial.begin(9600);
  Serial.print("Node ");
  Serial.print(MYNODEID,DEC);
  Serial.println(" ready");  

  // Set up the indicator LED (optional):
  pinMode(LED,OUTPUT);
  digitalWrite(LED,LOW);
  pinMode(GND,OUTPUT);
  digitalWrite(GND,LOW);

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBAUD);
}

void loop()
{
  String send_string = "";
  static unsigned long send_time = 0;
  bool got_location = false;
  
  if(GPS && !got_location) {
    while (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        send_string = addGPSData();
        got_location = true;
      }
    }
  }
  
  if(got_location && millis() - send_time > SENDDELAY) {
    radioSend(send_string);
    
    Blink(LED,10); 
    send_time = millis();
  }
}

/*
 * 
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

  if(DATE) {
    send_string = String(send_string + ";DAT:");
    if(gps.date.isValid()) {
      send_string = String(send_string + String(gps.date.month())); send_string = String(send_string + "/");
      send_string = String(send_string + String(gps.date.day())); send_string = String(send_string + "/");
      send_string = String(send_string + String(gps.date.year())); send_string = String(send_string + "; ");
    } else {
      send_string = String(send_string + "ERR;"); 
    }
  }

  if (TIME) {
    send_string = String(send_string + "TIM:");
    if(gps.time.isValid()) {
      if (gps.time.hour() < 10) send_string = String(send_string + "0"); 
      send_string = String(send_string + String(gps.time.hour()));
      send_string = String(send_string + ":");
      if (gps.time.minute() < 10) send_string = String(send_string + "0"); 
      send_string = String(send_string + String(gps.time.minute()));
      send_string = String(send_string + ";");
//      if (gps.time.second() < 10) send_string = String(send_string + "0"); 
//      send_string = String(send_string + String(gps.time.second()));
//      send_string = String(send_string + ";");
    } else {
      send_string = String(send_string + "ERR;");
    }
  }
  return send_string;
}

// Blink an LED for a given number of ms
void Blink(byte PIN, int DELAY_MS)
{
  digitalWrite(PIN,HIGH);
  delay(DELAY_MS);
  digitalWrite(PIN,LOW);
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
 * Initialize the radio board into transmit or receive mode.
 * By default, radio.setPowerLevel() is used to set the output
 * power as high as possible, and the radio is set to promiscuous 
 * mode. If you want a radio unit to only receive messages addressed
 * its node, get rid of the line radio.promiscuous().
 * 
 * The encryption option has been kept around from the example 
 * sketches for this board but probably isn't necessary for
 * Owl applications.
 *
 * Currently this sketch only contains code to run in transmit mode. 
 *   To restore mode functionality 
 * simply add mode handling in loop() (e.g. if(mode == TRANSMIT) { ... } else {...})
 */
void radioSetup() {
  // Initialize radio
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower();    // Only for RFM69HCW & HW!
  radio.setPowerLevel(TXPOWER); // power output ranges from 0 (5dBm) to 31 (20dBm)

  // promiscuous mode allows this unit to receive all transmissions on the network
  radio.promiscuous();
  radio.encrypt(ENCRYPTKEY);

}

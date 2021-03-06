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

#define GPS     false
#define LCD     false
#define SERIAL  false
#define DIFF    false
#define NUM_DEC 5
#define SCALE   100000

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

// mode select switch
#define MODESELECT    4
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

// all purpose button and optional status LED
#define BUTTON    7
#define LED       9
#define GND       8

// The Skytaq EM-506 GPS module included in the GPS Shield Kit
// uses 4800 baud by default
int GPSBaud = 9600;
unsigned long dot_update = 0;
byte num_dots = 1;

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;
double my_location[2] = {-71.749028,44.295546};
double correction[2] = {0,0};
unsigned long avg_size = 0;

// Addresses for this node. CHANGE THESE FOR EACH NODE!
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

void setup() {
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  if(LCD)
    lcdSerial.begin(9600);
  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED, OUTPUT);
  pinMode(GND, OUTPUT);
  digitalWrite(GND, LOW);

  radioSetup();
  if(LCD) {
    clearLCD();
    writeLine("Waiting for", 1, 2);
    writeLine("Serial", 2, 6);
  }
  establishContact();
}

void loop() {
  String send_string;
  String message = "";
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      myLocation();
    }
  }
  if (radio.receiveDone()) {
    message = getMessage();
    printCoords(message);
    if (radio.ACKRequested()) {
      radio.sendACK();
    }
    flashLED();
  } 
  if(SERIAL && Serial.available() > 0)
    printMyLocation();
}

void printCoords(String message) {
  int comma_index = getIndex(message, ",");
  float message_lng = message.substring(0,comma_index).toFloat();
  float message_lat = message.substring(comma_index+1).toFloat();
  long x_coord;
  long y_coord;

  if(DIFF) {
    x_coord = getCoord(my_location[0] + correction[0], message_lng);
    y_coord = getCoord(my_location[1] + correction[1], message_lat);
  } else {
    x_coord = getCoord(my_location[0], message_lng);
    y_coord = getCoord(my_location[1], message_lat);   
  }
  Serial.println(String(String(x_coord) + "," + String(y_coord)));
}

void myLocation() {
  if(gps.location.isValid()) {
    if(DIFF) {
      if(gps.location.lng() != my_location[0]) 
        correction[0] = gps.location.lng() - my_location[0];
      if(gps.location.lat() != my_location[1])
        correction[1] = gps.location.lat() - my_location[1];
    } else {
      my_location[0] = (my_location[0]*avg_size + gps.location.lng())/(avg_size+1);
      my_location[1] = (my_location[1]*avg_size + gps.location.lat())/(avg_size+1);
      avg_size++;
    }
    
    if(LCD) {
      clearLCD();
      writeLine(String(my_location[0],NUM_DEC), 1, 0);
      writeLine(String(my_location[1],NUM_DEC), 2, 0);
    }
  } else {
    waitScreen();
  } 
}

void printMyLocation() {
  while(Serial.available())
    Serial.read();
    
  Serial.print(my_location[0],NUM_DEC); Serial.print(", "); 
  Serial.println(my_location[1],NUM_DEC);
}

long getCoord(float center_pt, float new_pt) {
  float gps_diff = new_pt - center_pt;
  return gps_diff*SCALE;
}

int getDistance(long lat_diff, long lng_diff) {
//  Serial.print("differences: "); Serial.print(lat_diff); Serial.print(", "); Serial.println(lng_diff);
  long raw_distance = sqrt(lat_diff*lat_diff + lng_diff*lng_diff);
//  Serial.print("distance: "); Serial.println(raw_distance);
  int distance_m= raw_distance*11;
  return raw_distance;
}

int getIndex(String string, String sub){
  for(int i=0; i<string.length(); i++) {
    if(string.substring(i,i+1) == sub)
      return i;
  }
  return -1;
}


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

String getMessage() {
  char message[radio.DATALEN + 1];

  // The actual message is contained in the DATA array,
  // and is DATALEN bytes in size:
  for (byte i = 0; i < radio.DATALEN; i++) {
//    Serial.print((char)radio.DATA[i]);
    message[i] = (char)radio.DATA[i];
  }
  message[radio.DATALEN] = '\0';
  return message;
}

// Gets called while we wait for satellites to be acquired.
// Dot animation ensures that Owl is running correctly
void waitScreen() {
  bool write_serial = false;
  if(millis() - dot_update > 1000) {
    num_dots++;
    write_serial = true;
    dot_update = millis();
    if(num_dots == 4)
      num_dots = 1;
  }
  String status = "Acquiring";
  for(int i=0; i<num_dots; i++) {
    status = String(status + ".");
  }
  if(LCD)   
    writeLine(status,1,0);
  else if(SERIAL && write_serial)
    Serial.println(status);
}

void clearLCD() {
  lcdSerial.write(254); // move cursor to beginning of first line
  lcdSerial.write(128);

  lcdSerial.write("                "); // clear display
  lcdSerial.write("                ");
}

void writeLine(String str, int line, int pos) {
  char temp[16] = "               ";
  int num_chars = min(str.length()+pos, 16-pos);
  lcdSerial.write(254); // move cursor to beginning of first line
  switch(line) {
    case 1:
      lcdSerial.write(128);
    case 2:
      lcdSerial.write(192);
  }
  delay(10);
  for(int i=0; i<num_chars; i++) {
    temp[i+pos] = str[i];
  }
  lcdSerial.write(temp);
//  delay(100);
}

void flashLED() {
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
}

void radioSetup() {
  mode = RECEIVE;
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

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.read();
    Serial.println("A");   // send a capital A
    delay(300);
  }
}

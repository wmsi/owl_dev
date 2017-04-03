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

#define LCD     true
#define SERIAL  true

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

// Resonant frequency for audio feedback
#define CENTER_FREQ   2000
#define SPEAKER       3
#define SPEAKER_EN    9
boolean toggle = false;
boolean speaker_state = false;
int tone_length = 0;
int post_scale = 0;
int counter = 0;

// mode select switch
#define MODESELECT    4
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

// push to beep
#define PUSHTOBEEP    8

// The Skytaq EM-506 GPS module included in the GPS Shield Kit
// uses 4800 baud by default
int GPSBaud = 9600;
unsigned long dot_update = 0;
byte num_dots = 1;

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;
double target_lat = 44.280417;
double target_lng = -71.691528;

const int num_waypts = 6;
const double waypt_lat_lng[num_waypts][2] = {
  {44.280599,-71.691327},
  {44.281840,-71.691408},
  {44.283301,-71.692905},
  {44.281770,-71.694268},
  {44.280071,-71.692702},
  {44.283229,-71.694266}
};

const int bearings[num_waypts] = {
  318,
  266,
  178,
  90,
  357,
  144
};

//const int allotted_time = 10; //minutes
//const int broadcast_interval = 10; //seconds
//const int s7sTx = 8;
//const int s7sRx = 7;
//SoftwareSerial s7s(s7sRx, s7sTx);
//unsigned long last_update = 0;
//unsigned long last_broadcast = 0;

// Addresses for this node. CHANGE THESE FOR EACH NODE!
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
  cli();//stop interrupts

  TCCR1A = 0;
  TCCR1B = (1<<CS12) | (1<<WGM12); // 256 prescaler
  OCR1A = 311;                      // = (8*10^6) / (100*256) - 1
  TIMSK1 = (1<<OCIE1A);
  
  sei();//allow interrupts
  
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  if(LCD)
    lcdSerial.begin(9600);
  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);

//  clearDisplay();
  clearLCD();

  pinMode(MODESELECT, INPUT);
  pinMode(SPEAKER, OUTPUT); 
  pinMode(SPEAKER_EN, OUTPUT);
  digitalWrite(SPEAKER_EN, HIGH);   // don't enable the speaker yet
  pinMode(PUSHTOBEEP, INPUT_PULLUP);

  radioSetup();
}

void loop() {
  String send_string;
  String message = "";
  if (radio.receiveDone()) {
    message = printMessage();
    Serial.print("received: "); Serial.println(message);
    if (radio.ACKRequested()) {
      radio.sendACK();
      Serial.println("ACK sent");
    }
  }
  if(mode == RECEIVE) {
    // keep checking the GPS
    while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
  //      displayInfo();
        checkProximity();
      }
    }

    // send push to beep to the hidden unit
    if(digitalRead(PUSHTOBEEP) == LOW) {
      send_string = "beep";
      if(radioSend(send_string) && SERIAL) {
        Serial.print("RSSI: "); Serial.println(radio.RSSI);
      }
    }

    // check if a time update has been received
    if(message.indexOf("min. left!") != -1) {
      clearLCD();
      writeLine(message,1,0);
      while(digitalRead(PUSHTOBEEP) != LOW) {
        delay(10);
      }
    }

    if(message.indexOf("Out of time!") != -1) {
            clearLCD();
      writeLine(message,1,0);
      while(digitalRead(PUSHTOBEEP) != LOW) {
        delay(10);
      }
    }

    // check if the transmitter was found
    if(message.indexOf("owl found") != -1) {
      clearLCD();
      writeLine("Hidden Owl",1,3);
      writeLine("Found!",2,5);
//      while(digitalRead(PUSHTOBEEP) != LOW) {
      while(1) {
        clearLCD();
        delay(250);
        writeLine("Hidden Owl",1,3);
        writeLine("Found!",2,5);
        delay(750);
      }
    }
    
  // transmit (hidden) mode
  } else {
//    // check for push to beep
//    if(message.indexOf("beep") != -1) {
//      overwriteBeep();
//    }
//
//    // update countdown timer
//    updateDisplay();
//
//    // broadcast time update
//    if((millis()-last_broadcast)/1000 >= broadcast_interval) {
//      Serial.println("time udpate");
//      broadcastTime();
//    }
//
//    // check if button was pressed, signalling unit found
//    if(digitalRead(PUSHTOBEEP) == LOW) {
//      radioSend("owl found");
//    }
  }
}

void checkProximity() {
  if(gps.location.isValid()) {
    
    int min_distance = -1;
    int index = -1;
    for(int i=0; i<num_waypts; i++) {
      long lat_diff = 10000*waypt_lat_lng[i][0] - 10000*gps.location.lat();
      long lng_diff = 10000*waypt_lat_lng[i][1] - 10000*gps.location.lng();
      int distance = getDistance(lat_diff, lng_diff);
      if(min_distance == -1 || distance < min_distance) {
        min_distance = distance;
        index = i;
      }
    }

    clearLCD();
    post_scale = map(min_distance,1,15,5,80);
    if(min_distance < 2) {
      writeLine("bearing: ", 1, 0);
      writeLine(String(bearings[index]),2,0);
    } else {
      writeLine("min distance:",1,0);
      writeLine(String(String(min_distance) + ", " + String(index)),2,0);
    }
  } else {
    waitScreen();
  }
}

int getDistance(long lat_diff, long lng_diff) {
//  Serial.print("differences: "); Serial.print(lat_diff); Serial.print(", "); Serial.println(lng_diff);
  long raw_distance = sqrt(lat_diff*lat_diff + lng_diff*lng_diff);
//  Serial.print("distance: "); Serial.println(raw_distance);
  int distance_m= raw_distance*11;
  return raw_distance;
}

boolean radioSend(String send_string) {
  boolean ack_recd = false;
  byte send_length = send_string.length();
  static char send_buffer[62];
  Serial.print("sending to node ");
  Serial.print(TONODEID, DEC);
  Serial.print(", message [");
  for (byte i = 0; i < send_length; i++) {
    Serial.print(send_string[i]);
    send_buffer[i] = send_string[i];
  }
  Serial.println("]");

  // There are two ways to send packets. If you want
  // acknowledgements, use sendWithRetry():
  if (USEACK) {
    if (radio.sendWithRetry(TONODEID, send_buffer, send_length, 1)) {
      ack_recd = true;
      Serial.println("ACK received!");
      Serial.print("RSSI: ");
      Serial.println(radio.RSSI);
    } else
      Serial.println("no ACK received");
  } else {
    radio.send(TONODEID, send_buffer, send_length);
  } 
  return ack_recd;
}

String printMessage() {
  char message[radio.DATALEN + 1];
//  Serial.print("received from node ");
//  Serial.print(radio.SENDERID, DEC);
//  Serial.print(", message [");

  // The actual message is contained in the DATA array,
  // and is DATALEN bytes in size:
  for (byte i = 0; i < radio.DATALEN; i++) {
//    Serial.print((char)radio.DATA[i]);
    message[i] = (char)radio.DATA[i];
  }
  message[radio.DATALEN] = '\0';

  // RSSI is the "Receive Signal Strength Indicator",
  // smaller numbers mean higher power.
//  Serial.print("], RSSI ");
//  Serial.println(radio.RSSI);

  return message;
}

// Gets called while we wait for satellites to be acquired.
// Dot animation ensures that Owl is running correctly
void waitScreen() {
  if(millis() - dot_update > 1000) {
    num_dots++;
    dot_update = millis();
    if(num_dots == 4)
      num_dots = 1;
  }
  String status = "Acquiring";
  for(int i=0; i<num_dots; i++) {
    status = String(status + ".");
  }
  writeLine(status,1,0);
}

void clearLCD() {
  lcdSerial.write(254); // move cursor to beginning of first line
  lcdSerial.write(128);

  lcdSerial.write("                "); // clear display
  lcdSerial.write("                ");
//  delay(50);
}

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
//  delay(100);
}

//void updateDisplay() {
//  if(millis() - last_update >= 1000) {
//    char tempString[10];
//    int start_time = allotted_time*60;
//    int elapsed_time = millis()/1000;
//    int time_remaining = start_time - elapsed_time;
//    int display_time = 100*(time_remaining/60) + time_remaining%60;
//  
//    sprintf(tempString, "%4d", display_time);
//    s7s.print(tempString);
//    setDecimals(0b00010000);
//    last_update = millis();
//    if(SERIAL) {
//      Serial.println(tempString);
//    }
//  }
//}
//
//void broadcastTime() {
//  String send_string;
//  int time_remaining = allotted_time - (millis()/60000);
//  send_string = String(String(time_remaining) + "min. left!");
//
//  // keep trying on future iterations until ack received
//  if(radioSend(send_string)) {  
//    last_broadcast = millis();
//  }
//}
//
//// Turn on any, none, or all of the decimals.
////  The six lowest bits in the decimals parameter sets a decimal 
////  (or colon, or apostrophe) on or off. A 1 indicates on, 0 off.
////  [MSB] (X)(X)(Apos)(Colon)(Digit 4)(Digit 3)(Digit2)(Digit1)
//void setDecimals(byte decimals)
//{
//  s7s.write(0x77);
//  s7s.write(decimals);
//}
//
//// Send the clear display command (0x76)
////  This will clear the display and reset the cursor
//void clearDisplay()
//{
//  s7s.write(0x76);  // Clear display command
//}

void overwriteBeep() {
  digitalWrite(SPEAKER_EN, LOW);
  tone(SPEAKER,CENTER_FREQ,300);
//  analogWrite(SPEAKER, 127);
  digitalWrite(SPEAKER_EN, HIGH);
}

ISR(TIMER1_COMPA_vect){
  counter++;
  if(counter >= post_scale) {
    toggle = !toggle;
    counter = 0;
    if(toggle && post_scale != 0) {
      int frequency = map(post_scale,5,80,1500,1000);
      analogWrite(3, 127);
//      tone(3,frequency);
    } else {
      analogWrite(3, 0);
//      noTone(3);
    }
  }
}

void radioSetup() {
  if(digitalRead(MODESELECT)) {
    mode = RECEIVE;
    MYNODEID = 1;
    TONODEID = 2;
    Serial.println("Receive Mode Selected");
  } else {
    mode = TRANSMIT;
    MYNODEID = 2;
    TONODEID = 1;
    Serial.print("Transmit Mode Selected");
  }
  if(SERIAL) 
    Serial.print("Node "); Serial.print(MYNODEID,DEC); Serial.println(" ready");  

  // Initialize radio
  radio.initialize(FREQUENCY,MYNODEID,NETWORKID);
  radio.setHighPower();    // Only for RFM69HCW & HW!
  radio.setPowerLevel(TXPOWER); // power output ranges from 0 (5dBm) to 31 (20dBm)
  
  // promiscuous mode allows this unit to receive all transmissions on the network
  radio.promiscuous();
  radio.encrypt(ENCRYPTKEY);

}


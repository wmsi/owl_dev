/*
 * This is the final draft of the Owl program for the Woodland
 * Olders' game. This sketch is intended for the units that 
 * participants will have while playing the game. It handles
 * the GPS Draw station and flippy-flop mini game.
 */

// include libraries for GPS and LCD setup
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Include the RFM69 and SPI libraries:
#include <RFM69.h>
#include <SPI.h>

#define GPS     true
#define LCD     true
#define SERIAL  true

// Choose two Arduino pins to use for software serial
// The GPS Shield uses D2 and D3 by default when in DLINE mode
#define GPSRX 5
#define GPSTX 4
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
// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(GPSRX, GPSTX);

// The Skytaq EM-506 GPS module included in the GPS Shield Kit
// uses 4800 baud by default
int GPSBaud = 9600;
unsigned long dot_update = 0;
byte num_dots = 1;

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;
double my_location[2] = {-71.768792,44.295530};
const int stage_loc[2] = {-71.748118,44.296684};
bool acquired = false;
bool new_write = true;
unsigned long last_event = 0;     // for all-purpose debouncing

const int steps[] = {2,0,1,3,4,3,5,0,1};
const int num_steps = 9;
const String key[] = {"TOP","BOTTOM","LEFT","RIGHT","FRONT","BACK"}; // {+x,-x,+y,-y,+z,-z}
int step_time = 1000;
boolean step_displayed = false;
unsigned long step_start;
int step_index = 0;
bool start_over = true;
int max_mistake_time = 1000;

RFM69 radio;

void setup() {
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  if(LCD)
    lcdSerial.begin(9600);
    clearLCD();
    
  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);

  radioSetup();

  pinMode(BUTTON, INPUT_PULLUP);
}

void loop() {
  String send_string;
  String message = "";
  if (radio.receiveDone()) {
    message = printMessage();
  }
  
  // check for GPS update
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      myLocation();
    }
  }

  if ((my_location[0] > -71.75 && my_location[0] < -71.745) &&
      (my_location[1] < 44.296 && my_location[1] > 44.293)) {
//if(true) {
    GPSDraw();
  } else if(runFlippyFlop()) {
    flippyFlop();
  }
  
}

//////////////        GPS-related functions         ///////////////////

void myLocation() {
  if(gps.location.isValid()) {
    acquired = true;
    my_location[0] = gps.location.lng();
    my_location[1] = gps.location.lat();
  } else if(LCD) {
    waitScreen();
  }
}

void GPSDraw() {
  String send_string;
  if(acquired) {
    if(new_write && millis() - last_event > step_time) {
      clearLCD();
      writeLine("Press button",1,2);
      writeLine("for new point",2,2);
      new_write = false;
    }
    if(digitalRead(BUTTON) == LOW && millis() - last_event > DEBOUNCE) {
      clearLCD();
      new_write = true;
      last_event = millis();
      writeLine("Sending...",1,3);
      send_string = String(String(my_location[0],5) + "," + String(my_location[1],5));
      radioSend(send_string);
    }
  }
}

int getDistance(long lat_diff, long lng_diff) {
//  Serial.print("differences: "); Serial.print(lat_diff); Serial.print(", "); Serial.println(lng_diff);
  long raw_distance = sqrt(lat_diff*lat_diff + lng_diff*lng_diff);
//  Serial.print("distance: "); Serial.println(raw_distance);
  int distance_m= raw_distance*11;
  return raw_distance;
}

//////////////         flippy flop functions        /////////////////

bool runFlippyFlop() {
  long lng_diff = my_location[0]-stage_loc[0];
  long lat_diff = my_location[1]-stage_loc[1];
  if(getDistance(lat_diff, lng_diff) < 3) {
    clearLCD();
    writeLine("Ready for your",1,1);
    writeLine("next challenge?",2,1);
  }
  unsigned long start_time = millis();
  while(millis() - start_time < 2000) {
    if(digitalRead(BUTTON) == LOW)
      return true;
  }
  return false;
}

void flippyFlop() {
  Serial.println("running flippy flop...");
  bool game_over = false;
  byte orientation;
  while(!game_over) {
    orientation = getOrientation();
//    displayStep();
    if(LCD && !step_displayed)
      writeStep();
    
    if(orientation == steps[step_index]) {
      if(step_start == 0) {
        step_start = millis();
      } else if(millis() - step_start > step_time) {
        Serial.println("got it!");
        step_index++;
        step_start = 0;
        step_displayed = false;
        if(step_index == num_steps) {
          displayDone();
          last_event = 0;
        }
      }
    } else {
      step_start = 0;
      if(step_index != 0 && orientation == steps[step_index-1]) {
        last_event = 0;
      } else if(step_index != 0 && start_over) {
        startOver();   
      }
    }
  }
}

byte getOrientation() {
  int x = analogRead(0);       // read analog input pin 0
  int y = analogRead(1);       // read analog input pin 1
  int z = analogRead(2);       // read analog input pin 2

  int up_reading;
  if(max(x,max(y,z)) - CENTER > CENTER - min(x,min(y,z))) {
    up_reading = max(x,max(y,z));
    if(x == up_reading) {
      return 1;
    } else if(y == up_reading) {
      return 3;
    } else {
      return 5;
    }
  } else {
    up_reading = min(x,min(y,z));
    if(x == up_reading) {
      return 0;
    } else if(y == up_reading) {
      return 2;
    } else {
      return 4;
    }
  }
}

String toCode(String this_step) {
  String temp = "";
  byte code;
  for(int i=0; i<this_step.length(); i++) {
    code = byte(this_step[i])-64;
    temp = String(temp + String(code));
    if(i != this_step.length()-1)
      temp = String(temp + "-");
  }
  return temp;
}

void startOver() {
  if(last_event == 0) {
    last_event = millis();
  } else if(millis() - last_event > max_mistake_time) {
    clearLCD();
    writeLine("Start Over :(",1,2);
    step_index = 0;
    step_displayed = false;
    delay(1500);
    last_event = 0;
  }
}

void writeStep() {
  clearLCD();
//  char temp[16];
  String current_step = toCode(key[steps[step_index]]);
  if(current_step.length() >= 16) {
    String part1 = current_step.substring(0,14);
    String part2 = current_step.substring(14);
    writeLine(part1,1,0);
    writeLine(part2,2,0);
  } else {
    writeLine(current_step,1,0);
  }
  step_displayed = true;
}

void welcomeScreen() {
  clearLCD();
  writeLine("Welcome to",1,3);
  writeLine("Flippy Flop!",2,2);
}

void displayDone() {
  Serial.println("you win!");
  if(LCD) {
    clearLCD();
    writeLine("You Win!",1,4);
    writeLine("Next Number: 9",2,0);
  }
  while(1);
}

//////////////               LCD functions            ///////////////

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

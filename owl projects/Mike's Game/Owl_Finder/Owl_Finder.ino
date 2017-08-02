/* This is the basic script to run an owl that receives a signal and then displays it onto 
 *  an LCD display. It requires a transmitting owl in order to get a 
 *  message.
 */

// include libraries for GPS and LCD setup
#include <SoftwareSerial.h>
#include <TinyGPS++.h> 

// Include the RFM69 and SPI libraries:
#include <RFM69.h>
#include <SPI.h>

#define LCD     true
#define SERIAL  true
#define GPS true

const int MAX_DISTANCE = 100;
// Choose two Arduino pins to use for software serial
#define GPSRX 5
#define GPSTX 4
#define LCDRX 7
#define LCDTX 6

// button pin
#define BUTTON    7
#define DEBOUNCE  200

// Initialize speaker variables. This takes a bit of setup because the
// speaker will always run in the background, though the user won't hear it
// until they press a button to complete the connection
#define SPEAKER       3
boolean toggle = false;
boolean speaker_state = false;
int tone_length = 0;
int post_scale = 0;
int counter = 0;


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
SoftwareSerial gpsSerial(GPSRX, GPSTX);
SoftwareSerial lcdSerial(LCDRX,LCDTX); // Rx, Tx

unsigned long dot_update = 0;
byte num_dots = 1;

int GPSBaud = 9600;

TinyGPSPlus gps;

bool acquired = false;

//create a radio object
RFM69 radio;

double my_lt = 0;
double my_lg = 0;
double Latitude = 0;
double Longitude = 0;
double distance =0;

void setup() {
  // set up interrupts to allow for timing speaker output in the background.
  // Using interrupts allow for more precise timing control than doing this in loop
  // for more info check out this instructables:
  // http://www.instructables.com/id/Arduino-Timer-Interrupts/
  cli();//stop interrupts

  TCCR1A = 0;
  TCCR1B = (1 << CS12) | (1 << WGM12); // 256 prescaler
  OCR1A = 311;                      // = (8*10^6) / (100*256) - 1
  TIMSK1 = (1 << OCIE1A);

  sei();//allow interrupts
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);

  if(LCD)
    lcdSerial.begin(9600);
    clearLCD();
    
  // Start the software serial port at the GPS's default baud
 // gpsSerial.begin(GPSBaud);
  gpsSerial.begin(GPSBaud);
  
  radioSetup();

  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(SPEAKER, OUTPUT);
}

void loop() {
  //get message from transmitting owl then print it
  String send_string;
  String message = "";
  while(!acquired){
    if (radio.receiveDone()) {
      message = printMessage();
    }
    convertDouble(message);
    if (Latitude != 0 and Longitude != 0){
     acquired = true;
     delay(2000);  
    }
    else
      writeLine("wait");
  }
    // keep checking the GPS
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      checkProximity();
    }
  }

  // send push to beep to the hidden unit
  if (digitalRead(BUTTON) == LOW) {
    send_string = "beep";
    if (radioSend(send_string) && SERIAL) {
      Serial.print("RSSI: "); Serial.println(radio.RSSI);
    }
  }
}


/*                        LCD Stuff                       */
/*
 * Shortcut function to write a line to the first line of the LCD
 */
void writeLine(String str) {
  writeLine(str, 1, 0);
}
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
/*
 * This function is meant to be called while the GPS is trying to
 * acquire satellites. The screen will display "Acquiring..." with
 * the number of dots incrementing from 1-3 on repeat. This little
 * animation ensures that the Owl is still running when it may 
 * otherwise appear to have frozen.
 */
void waitScreen() {
  if (millis() - dot_update > 1000) {
    num_dots++;
    dot_update = millis();
    if (num_dots == 4)
      num_dots = 1;
  }
  String status = "Acquiring";
  for (int i = 0; i < num_dots; i++) {
    status = String(status + ".");
  }
  writeLine(status, 1, 0);
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
/*                        GPS Functions                   */
/*
 * Use the GPS location to check our proximity to every waypoint in
 * the list and determine which waypoint is closest. Display the
 * nearest waypoint number and the distance on the screen.
 */
void checkProximity() {
  // check if we have a fix on enough satellites to provide a location
  long Raw_Lat = 0;
  long Raw_Long = 0;
  if (gps.location.isValid()) {
    long min_distance = -1;
    int index = -1;
    clearLCD();
    delay(1500);
    getLocation();
    distance = gps.distanceBetween(my_lt, my_lg, Latitude, Longitude);
    if (min_distance == -1 || distance < min_distance){
      min_distance = distance;
    }
    // set post_scale for speaker beep frequency. This is checked in the interrupt
    // routine to decide when to toggle the speaker on/ off
    post_scale = map(min_distance, 1, MAX_DISTANCE, 5, 80);     

    displayDistance();
  } else {
    waitScreen();
  }
}

void getLocation(){
  //Gets the decimal values of the lat and long
  my_lt =(gps.location.rawLat().billionths * 0.000000001);
  my_lg =(gps.location.rawLng().billionths * 0.000000001);

  double Raw_deg_lat = gps.location.rawLat().deg;
  double Raw_deg_lng = gps.location.rawLng().deg;
  double real_Lat = gps.location.lat() - Raw_deg_lat;
  double real_Lng = gps.location.lng() - Raw_deg_lng;
  //add the whole number value to the latitude and longitude
  if (real_Lat  > 0) { 
    my_lt += Raw_deg_lat;
  }
  else {
      my_lt += (Raw_deg_lat); //because the gps rounds to degreee, checks to see if rounded up or not
  }
  if (real_Lng > 0){
      my_lg += Raw_deg_lng;
  }
  else {
    my_lg += (Raw_deg_lng);
  }
  my_lg = 0 - my_lg;
}

void checkLocation(){
  double Raw_Lat = gps.location.rawLat().deg;
  if (Raw_Lat != 0){
    acquired = true;
  }
}
/*
 * Calculate the approximate distance between two points, based on 
 * the difference in latitude and longitude. Note that this does not 
 * provide the actual distance, which would have to take into account
 * the curvature of the earth
 */
int getDistance(double lat_diff, double lng_diff) {
  double raw_distance = sqrt(lat_diff * lat_diff + lng_diff * lng_diff);
  int distance_m = raw_distance * 11;
  return raw_distance;
}



/*
 *  Display a message on the screen depending on the distance to the
 *  nearest waypoint. If we are greater than MAX_DISTANCE away from
 *  the nearest waypoint, display a message showing that we are out 
 *  of range. If we are within 1 unit of distance (about 11 m) from
 *  the nearest waypoint, release a bearing. Otherwise show a number
 *  of asterisks to represent the distance.
 */
void displayDistance() {
  if (distance > MAX_DISTANCE){
    writeLine("Out of Range :(");
  }
  else { 
    if (distance < 8) {
      writeLine("Look Around", 1, 0);
    }
    else {
      int num_asterisks = map(distance, 0, MAX_DISTANCE, 1, 8);
      String asterisks = "";
      for (int i = 0; i < num_asterisks; i++){
        asterisks = asterisks + "*";
      }
      writeLine(String("Owl"), 1, 0);
      writeLine(String("dist: " + asterisks), 2, 0);
      delay(1500);
      }
  }
}
/*                        Other                           */

/*
 * Converts the Latitude and Longitude coordinates given from a string
 * to a double and then moves the decimal over
 */
void convertDouble(String message){
  String Lat = message.substring(0, 11);
  String Lng = message.substring(12);
  Latitude = Lat.toDouble() * 0.000001;
  Longitude = Lng.toDouble() * 0.000001;
  Longitude = Longitude * -1;
}
/*
 * Interrupt routine to handle proximity beeping. Beep spacing
 * will increase proportionately the closer you are to the 
 * nearest waypoint. Originally beeps were also supposed to 
 * increase in pitch as well, but this feature was disabled
 * due to interference from softwareSerial
 */
ISR(TIMER1_COMPA_vect) {
  counter++;
  if (counter >= post_scale) {
    toggle = !toggle;
    counter = 0;
    if (toggle && post_scale != 0) {
      int frequency = map(post_scale, 5, 80, 1500, 1000);
      analogWrite(3, 127);
    } else {
      analogWrite(3, 0);
    }
  }
}


/* This is a basic script to introduce you to GPS receiving.  The purpose
 *  is to show you how to get your own Lat/Long coordinates and how to format
 *  them in a way that can be interpreted.
 */


// include libraries for using the GPS and LCD
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Use these to limit unnecessary output. Sometimes the Arduino will 
// get confused if it tries to write to Serial or an LCD when nothing
// is connected
#define LCD     true
#define SERIAL  true

// Choose Arduino pins to use for software serial with the GPS and LCD
// only GPSRX and LCDTX will actually be wired up
#define GPSRX 5
#define GPSTX 4
#define LCDRX 7
#define LCDTX 6

// create a softwareSerial object to interface with the LCD
SoftwareSerial lcdSerial(LCDRX, LCDTX);
// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(GPSRX, GPSTX);

// all-purpose button. In this sketch it used for "push to beep", ie it sends
// a signal to the hidden owl to beep so that the seekers can hopefully
// hear where it is.
#define BUTTON        7

// These two variables are used for the little animation that happens while
// the GPS is waiting to acquire satellites. This was originally designed
// so that you know the unit didn't freeze when it's trying to get a connection
unsigned long dot_update = 0;
byte num_dots = 1;

// mode select switch (unimportant for this code version, which only receives)
#define MODESELECT    4
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

// Define the baud rate for communicating with the GPS module
int GPSBaud = 9600;

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;


bool acquired = false;

void setup() {
  // put your setup code here, to run once:
  // Start the Arduino hardware serial port at 9600 baud
  if (SERIAL)
    Serial.begin(9600);

  if (LCD) {
    lcdSerial.begin(9600);
    clearLCD();
  }

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);


  pinMode(MODESELECT, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);


}

void loop() {

  // keep checking the GPS
  if (Found){
    while (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        if (!acquired){
          checkLocation();
          waitScreen();
        }
        while((digitalRead(BUTTON) != LOW) && (acquired)){
          writeLine("Push button for", 1, 0);
          writeLine("Location" , 2, 0);
        }
        getLocation();
      }
    }
  }
}

/*
 * Use the GPS location to check our proximity to every waypoint in
 * the list and determine which waypoint is closest. Display the
 * nearest waypoint number and the distance on the screen.
 */
bool Found() {
  // check if we have a fix on enough satellites to provide a location
  if (gps.location.isValid()) {
    clearLCD();
    return true;     
  } else {
    waitScreen();
    return false;
  }
}

void getLocation(){
  //Gets the decimal values of the lat and long
  double Raw_Lat =(gps.location.rawLat().billionths * 0.000000001);
  double Raw_Long =(gps.location.rawLng().billionths * 0.000000001);

  double Raw_deg_lat = gps.location.rawLat().deg;
  double Raw_deg_lng = gps.location.rawLng().deg;
  double real_Lat = gps.location.lat() - Raw_deg_lat;
  double real_Lng = gps.location.lng() - Raw_deg_lng;
  //add the whole number value to the latitude and longitude
  if (real_Lat  > 0) { 
    Raw_Lat += Raw_deg_lat;
  }
  else {
      Raw_Lat += (Raw_deg_lat); //because the gps rounds to degreee, checks to see if rounded up or not
  }
  if (real_Lng > 0){
      Raw_Long += Raw_deg_lng;
  }
  else {
    Raw_Long += (Raw_deg_lng);
  }
  Raw_Long = 0 - Raw_Long;
  if (Raw_Lat != 0){
    writeLine(String(Raw_Lat*1000000), 1, 0); //prints solely to see actual value since limited to two decimal places
    writeLine(String(Raw_Long*1000000), 2, 0);
    delay(3000);
  }
}

void checkLocation(){
  double Raw_Lat = gps.location.rawLat().deg;
  if (Raw_Lat != 0){
    acquired = true;
  }
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

/*
 * clear the LCD screen from all text
 */
void clearLCD() {
  lcdSerial.write(254); // move cursor to beginning of first line
  lcdSerial.write(128);

  lcdSerial.write("                "); // clear display
  lcdSerial.write("                ");
}

/*
 * Shortcut function to write a line to the first line of the LCD
 */
void writeLine(String str) {
  writeLine(str, 1, 0);
}

/*
 * Write a single line of text (String str) to the LCD screen,
 * beginning at position 'pos' of line 'line'
 * line: 1-2
 * pos: 0-15
 */
void writeLine(String str, int line, int pos) {
  char temp[16] = "               ";
  int num_chars = str.length();
  lcdSerial.write(254); // move cursor to beginning of first line
  switch (line) {
    case 2:
      lcdSerial.write(191 + pos);
    case 1:
      lcdSerial.write(128 + pos);
  }
  delay(10);
  for (int i = 0; i < num_chars; i++) {
    temp[i] = str[i];
  }
  lcdSerial.write(temp);
}


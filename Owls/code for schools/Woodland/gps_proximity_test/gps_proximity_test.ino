#include <TinyGPS++.h>
#include <SoftwareSerial.h>
/*
 This example uses software serial and the TinyGPS++ library by Mikal Hart
 Based on TinyGPSPlus/DeviceExample.ino by Mikal Hart
 Modified by acavis
*/

// Choose two Arduino pins to use for software serial
// The GPS Shield uses D2 and D3 by default when in DLINE mode
#define GPSRX 5
#define GPSTX 4
#define LCDRX 7
#define LCDTX 6

#define LCD true

//speaker stuff
boolean toggle = false;
boolean speaker_state = false;
int tone_length = 0;
int post_scale = 0;
int counter = 0;


// The Skytaq EM-506 GPS module included in the GPS Shield Kit
// uses 4800 baud by default
int GPSBaud = 9600;

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

// create a softwareSerial object to interface with the LCD
// we can use the same Rx and Tx because GPS only transmits, while LCD only receives
SoftwareSerial lcdSerial(LCDRX,LCDTX); // Rx, Tx
// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(GPSRX, GPSTX);

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

  clearLCD();
}

void loop() {
  // This sketch displays information every time a new sentence is correctly encoded.
//  post_scale = map(analogRead(A0),0,1023,5,80);
//  int frequency = map(post_scale,5,80,1500,1000);
//  
//  if(toggle != speaker_state) {
//    speaker_state = toggle;
//    if(speaker_state) {
//      analogWrite(3, 127);
////      tone(3,frequency);
//    } else {
//      analogWrite(3, 0);
////      noTone(3);
//    }
//  }
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
//      displayInfo();
      checkProximity();
    }
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
    
    post_scale = map(min_distance,1,15,5,80);
    if(min_distance < 2) {
      writeLine("bearing: ", 1, 0);
      writeLine(String(bearings[index]),2,0);
    } else {
      writeLine("min distance:",1,0);
      writeLine(String(String(min_distance) + ", " + String(index)),2,0);
    }
  } else {
    writeLine("acquiring...",1,0);
  }
}

int getDistance(long lat_diff, long lng_diff) {
//  Serial.print("differences: "); Serial.print(lat_diff); Serial.print(", "); Serial.println(lng_diff);
  long raw_distance = sqrt(lat_diff*lat_diff + lng_diff*lng_diff);
//  Serial.print("distance: "); Serial.println(raw_distance);
  int distance_m= raw_distance*11;
  return raw_distance;
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

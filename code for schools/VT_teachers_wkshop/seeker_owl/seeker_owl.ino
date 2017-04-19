/* This software is intended to be run as an example Owl activity for the
    Woodland school. In receive mode, the Owl searches for waypoints and
    releases a new compass (pointing toward the transmitter) at each waypoint.
    The transmitter sits in a central location, broadcasting updates as the
    timer runs down. When found, a button press on the transmitter will
    notify all receivers that it has been retrieved.
*/

// include libraries for using the GPS and LCD
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Include the RFM69 and SPI libraries for the RFM hookup board:
#include <RFM69.h>
#include <SPI.h>

// Use these to limit unnecessary output. Sometimes the Arduino will 
// get confused if it tries to write to Serial or an LCD when nothing
// is connected
#define LCD     true
#define SERIAL  true

// Choose Arduino pins to use for software serial with the GPS and LCD
#define GPSRX 5
#define GPSTX 4
#define LCDRX 7
#define LCDTX 6

// create a softwareSerial object to interface with the LCD
SoftwareSerial lcdSerial(LCDRX, LCDTX);
// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(GPSRX, GPSTX);

// Initialize speaker variables. This takes a bit of setup because the
// speaker will always run in the background, though the user won't hear it
// until they press a button to complete the connection
#define SPEAKER       3
boolean toggle = false;
boolean speaker_state = false;
int tone_length = 0;
int post_scale = 0;
int counter = 0;

// mode select switch (unimportant for this code version, which only receives)
#define MODESELECT    4
#define TRANSMIT      0
#define RECEIVE       1
static bool mode;

// all-purpose button. In this sketch it used for "push to beep", ie it sends
// a signal to the hidden owl to beep so that the seekers can hopefully
// hear where it is.
#define BUTTON        7

// Define the baud rate for communicating with the GPS module
int GPSBaud = 9600;

// These two variables are used for the little animation that happens while
// the GPS is waiting to acquire satellites. This was originally designed
// so that you know the unit didn't freeze when it's trying to get a connection
unsigned long dot_update = 0;
byte num_dots = 1;

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;

// This (receiving) unit will release a new bearing for each waypoint on the list
// when the GPS approaches within 33 ft. of that waypoint. Bearings should
// correspond directly to waypoints (e.g. bearing 0 released at waypoint 0, etc.)
const int num_waypts = 7;
const double waypt_lat_lng[num_waypts][2] = {
  {44.545895, -71.984866},      // fire hydrant
  {44.545933, -71.986062},      // weird corner
  {44.546633, -71.987152},      // back corner
  {44.546968, -71.986414},      // wood chip shed
  {44.547151, -71.984912},      // blue tunnels
  {44.546900, -71.984490},      // climbing wall
  {44.546206, -71.984745}       // green jxn boxes
};

const int bearings[num_waypts] = {
  322,
  35,
  97,
  127,
  213,
  240,
  297
};

// Addresses for this node, assigned at startup based on transmit/ receive mode
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

/* setup() initializes interrupts for timing beeps, begins software Serial
 *  communication with the GPS, and initializes the radio board. setup()
 *  will also start up LCD and Serial communication, if desired
 */
void setup() {
  // set up interrupts to allow for timing speaker output in the background
  cli();//stop interrupts

  TCCR1A = 0;
  TCCR1B = (1 << CS12) | (1 << WGM12); // 256 prescaler
  OCR1A = 311;                      // = (8*10^6) / (100*256) - 1
  TIMSK1 = (1 << OCIE1A);

  sei();//allow interrupts

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
  pinMode(SPEAKER, OUTPUT);
  // Enable the built-in pullup resistor and read a button press
  // any time it gets connected to ground
  pinMode(BUTTON, INPUT_PULLUP);

  radioSetup();
}

/* loop will continue to run for as long as the sketch is running.
    On each iterations, this function checks for new radio messages,
    GPS data, or a button press. It then updates the screen to reflect
    the location relative to the nearest waypoint, or display an alert
    from a time udpate or "Owl Found" message
*/
void loop() {
  String send_string;
  String message = "";
  if (radio.receiveDone()) {
    message = printMessage();
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

  if (message != "")
    processMessage(message);
}

/*
 * Use the GPS location to check our proximity to every waypoint in
 * the list and determine which waypoint is closest. Display the
 * nearest waypoint number and the distance on the screen.
 */
void checkProximity() {
  if (gps.location.isValid()) {

    long min_distance = -1;
    int index = -1;
    for (int i = 0; i < num_waypts; i++) {
      long lat_diff = 10000 * waypt_lat_lng[i][0] - 10000 * gps.location.lat();
      long lng_diff = 10000 * waypt_lat_lng[i][1] - 10000 * gps.location.lng();
      long distance = getDistance(lat_diff, lng_diff);
      if (min_distance == -1 || distance < min_distance) {
        min_distance = distance;
        index = i;
      }
    }

    clearLCD();
    post_scale = map(min_distance, 1, 15, 5, 80);
    min_distance *= 33;
    if (min_distance < 2) {
      writeLine("bearing: ", 1, 0);
      writeLine(String(bearings[index]), 2, 0);
    } else {
      writeLine(String("waypoint: " + String(index + 1)), 1, 0);
      writeLine(String("distance: " + String(min_distance)), 2, 0);
    }
  } else {
    waitScreen();
  }
}

/*
 * Calculate the approximate distance between two points, based on 
 * the difference in latitude and longitude. Note that this does not 
 * provide the actual distance, which would have to take into account
 * the curvature of the earth
 */
int getDistance(long lat_diff, long lng_diff) {
  long raw_distance = sqrt(lat_diff * lat_diff + lng_diff * lng_diff);
  int distance_m = raw_distance * 11;
  return raw_distance;
}

/*
   Broadcast a message via the radio board. If USEACK is set to true,
   wait for an acknowledgment as dictated by sendWithRetry(). From 
   RFM69.h:
   sendWithRetry(uint8_t toAddress, const void* buffer, uint8_t bufferSize, 
                 uint8_t retries=2, uint8_t retryWaitTime=40);

   return true if an ACK was received
*/
boolean radioSend(String send_string) {
  boolean ack_recd = false;
  byte send_length = send_string.length();
  static char send_buffer[RF69_MAX_DATA_LEN];
  if (SERIAL)
    Serial.println("sending: " + send_string);
  for (byte i = 0; i < send_length; i++) {
    send_buffer[i] = send_string[i];
  }

  // There are two ways to send packets. If you want
  // acknowledgements, use sendWithRetry():
  if (USEACK) {
    if (radio.sendWithRetry(TONODEID, send_buffer, send_length, 1)) {
      ack_recd = true;
      if (SERIAL)
        Serial.println("ACK received!");
    } else if (SERIAL) {
      Serial.println("no ACK received");
    }
  } else {
    radio.send(TONODEID, send_buffer, send_length);
  }
  return ack_recd;
}

/*
   If a new message has been received on the RFM board,
   convert it to a String and sent an acknowledgment
   message, if requested
*/
String printMessage() {
  char message[radio.DATALEN + 1];
  // The actual message is contained in the DATA array,
  // and is DATALEN bytes in size:
  for (byte i = 0; i < radio.DATALEN; i++) {
    message[i] = (char)radio.DATA[i];
  }
  message[radio.DATALEN] = '\0';

  if (SERIAL)
    Serial.println(message);
  if (radio.ACKRequested()) {
    radio.sendACK();
    if (SERIAL)
      Serial.println("ACK sent");
  }
  return message;
}

/*
 * Read a new message from the radio board and decide what action
 * to take as a result. 
 * 
 * **Note that it may be desirable to change the final section of
 * this function in order to allow the user to clear the screen
 * once the hidden owl has been found. It is currently set up to 
 * end the program, so that once an "owl found" message has been
 * received this unit will need to be restarted to play again.
 */
void processMessage(String message) {

  if (message.indexOf("min. left!") != -1) {
    clearLCD();
    writeLine(message, 1, 0);
    while (digitalRead(BUTTON) != LOW) {
      delay(10);
    }
  } else if (message.indexOf("Out of time!") != -1) {
    clearLCD();
    writeLine(message, 1, 0);
    while (digitalRead(BUTTON) != LOW) {
      delay(10);
    }
  } else if (message.indexOf("owl found") != -1) {
    clearLCD();
    writeLine("Hidden Owl", 1, 3);
    writeLine("Found!", 2, 5);
    //      while(digitalRead(PUSHTOBEEP) != LOW) {
    while (1) {
      clearLCD();
      delay(250);
      writeLine("Hidden Owl", 1, 3);
      writeLine("Found!", 2, 5);
      delay(750);
    }
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

/*
 * Currently this sketch only contains code to run in receive mode, 
 * since adding support for the timer display overloads the DIO pins.
 * To restore mode functionality simply uncomment the code in setup
 * and add mode handling in loop()
 */
void radioSetup() {
  if (digitalRead(MODESELECT)) {
    mode = RECEIVE;
    MYNODEID = 1;
    TONODEID = 2;
  } else {
    mode = TRANSMIT;
    MYNODEID = 2;
    TONODEID = 1;
  }
  if(SERIAL)
    Serial.print(String(mode==RECEIVE ? "Receive" : "Transmit") + " mode selected");
      
  // Initialize radio
  radio.initialize(FREQUENCY, MYNODEID, NETWORKID);
  radio.setHighPower();    // Only for RFM69HCW & HW!
  radio.setPowerLevel(TXPOWER); // power output ranges from 0 (5dBm) to 31 (20dBm)

  // promiscuous mode allows this unit to receive all transmissions on the network
  radio.promiscuous();
  radio.encrypt(ENCRYPTKEY);
}


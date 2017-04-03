#define MAXBLINK   3000

int brightness = 0;
unsigned long last_fade = 0;
int fade_amount = 5;
const int fade_step = 10; // milliseconds
int blink_time = MAXBLINK;
int pin_out = 0;
unsigned long last_blink = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);

  pinMode(3, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(3) == LOW) {
    blink_time = 50;
  }
  if(blink_time < MAXBLINK) {
    flashSequence();
  } else {
    fade();
  }
  
}

void flashSequence() {
  if(millis() - last_blink > blink_time) {
    clearLEDs();
    digitalWrite(pin_out, HIGH);
    pin_out++;
    pin_out %= 3;
    last_blink = millis();
    blink_time += blink_time/5;
    if(blink_time > MAXBLINK)
      clearLEDs();
  }
}

void fade() {
  if(millis() - last_fade > fade_step) {
    brightness += fade_amount;
    analogWrite(0, brightness);
    if(brightness >= 255 || brightness <= 0)
      fade_amount = -fade_amount;
    last_fade = millis();
  }
}

void clearLEDs() {
  digitalWrite(0, LOW);
  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
}

int brightness = 0;
unsigned long last_fade = 0;
int fade_amount = 5;
const int fade_step = 10; // milliseconds

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
    flashSequence();
  } else {
    fade();
  }
  
}

void flashSequence() {
  digitalWrite(0, HIGH);
  delay(250);
  digitalWrite(0, LOW);
  digitalWrite(1, HIGH);
  delay(250);
  digitalWrite(1, LOW);
  digitalWrite(2, HIGH);
  delay(250);
  digitalWrite(2, LOW);
}

void fade() {
  if(last_fade - millis() > fade_step) {
    brightness += fade_amount;
    analogWrite(0, brightness);
    if(brightness >= 255 || brightness <= 0)
      fade_amount = -fade_amount;
    last_fade = millis();
  }
}


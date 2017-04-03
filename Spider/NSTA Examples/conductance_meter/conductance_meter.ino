// output a different response based on the voltage read through an analog input

#define THRESHOLD   800

int brightness = 0;
int fade_amount = 5;
int fade_step = 10;
unsigned long last_step = 0;
int pin_out = 0;
bool new_fade = true;     // use this to clear the LEDs when switching from blink to fade

void setup() {
  // put your setup code here, to run once:
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
}

void loop() {
  int conductance = analogRead(A2);
  if(conductance > THRESHOLD) {
    if(new_fade)
      clearLEDs();
    fade();
  } else {
    int blink_time = map(conductance,0,800,50,1000);
    flash(blink_time);
  }
}

void fade() {
  new_fade = false;
  if(millis() - last_step > fade_step) {
    brightness += fade_amount;
    analogWrite(0, brightness);
    if(brightness >= 255 || brightness <= 0)
      fade_amount = -fade_amount;
    last_step = millis();
  }
}

void flash(int blink_time) {
  new_fade = true;
  if(millis() - last_step > blink_time) {
    clearLEDs();
    digitalWrite(pin_out, HIGH);
    pin_out++;
    pin_out %= 3;
    last_step = millis();
  }
}

void clearLEDs() {
  digitalWrite(0, LOW);
  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
}


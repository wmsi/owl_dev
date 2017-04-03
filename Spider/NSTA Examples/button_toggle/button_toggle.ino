// toggle between different flash patterns using a finger bridge 
// over the analog input

int threshold = 500;
int last_debounce = 0;
int debounce_time = 150;  // milliseconds
bool pressed = false;

byte pattern = 0;
const byte num_patterns = 4;

unsigned long last_step = 0;
int step_number = 0;

int brightness = 0;
int fade_amount = 5;

void setup() {
  // put your setup code here, to run once:
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(analogRead(A3) < threshold) {
    if(pressed) {
      if(millis() - last_debounce > debounce_time) {
        nextPattern();
        pressed = false;
      }
    } else {
      pressed = true;
      last_debounce = millis();
    }
  } else {
    pressed = false;
  }
  
  switch(pattern) {
    case 0:
      flashPattern0();
      break;
    case 1:
      flashPattern1();
      break;
    case 2:
      flashPattern2();
      break;
    case 3:
      flashPattern3();
  }
}

void nextPattern() {
  pattern++;
  pattern %= num_patterns;
  step_number = 0;
  clearLEDs();
}

void flashPattern0() {
  int step_length = 500;
  int num_steps = 3;
  if(millis() - last_step > step_length) {
    clearLEDs();
    digitalWrite(step_number, HIGH);
    step_number++;
    step_number %= num_steps;
    last_step = millis();
  }
}

void flashPattern1() {
  int step_length = 10;
  if(millis() - last_step > step_length) {
    brightness += fade_amount;
    analogWrite(0, brightness);
    analogWrite(1, 255-brightness);
    if(brightness >= 255 || brightness <= 0)
      fade_amount = -fade_amount;
    last_step = millis();
  }
}

void flashPattern2() {
  int step_length = 750;
  int num_steps = 6;
  if(millis() - last_step > step_length) {
    switch(step_number) {
      case 0:
        digitalWrite(0, HIGH);
        break;
      case 1:
        digitalWrite(1, HIGH);
        break;
      case 2:
        digitalWrite(2, HIGH);
        break;
      case 3:
        digitalWrite(0, LOW);
        break;
      case 4:
        digitalWrite(1, LOW);
        break;
      case 5: 
        digitalWrite(2, LOW);
    }
    step_number++;
    step_number %= num_steps;
    last_step = millis();
  }
}

void flashPattern3() {
  int step_length = 100;
  if(millis() - last_step > step_length) {
    clearLEDs();
    int led = random(0,3);
    digitalWrite(led, HIGH);
    last_step = millis();
  }
}

void clearLEDs() {
  digitalWrite(0, LOW);
  digitalWrite(1, LOW);
  digitalWrite(2, LOW);
}


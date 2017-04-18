//https://www.instructables.com/id/Arduino-Timer-Interrupts/

boolean toggle = false;
boolean speaker_state = false;
int tone_length = 0;
int post_scale = 5;
int counter = 0;

void setup() {

  pinMode(10, OUTPUT);

  cli();//stop interrupts

  TCCR1A = 0;
  TCCR1B = (1<<CS12) | (1<<WGM12); // 256 prescaler
  OCR1A = 311;                      // = (8*10^6) / (100*256) - 1
  TIMSK1 = (1<<OCIE1A);
  
  sei();//allow interrupts

  Serial.begin(9600);

}//end setup

void loop() {
  post_scale = map(analogRead(A0),0,1023,2,80);
  int frequency = map(post_scale,2,80,1500,1000);
  
  if(toggle != speaker_state) {
    speaker_state = toggle;
    if(speaker_state) {
      tone(3,frequency);
    } else {
      noTone(3);
    }
  }
}


ISR(TIMER1_COMPA_vect){//timer1 interrupt 1Hz toggles pin 13 (LED)
//generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  counter++;
  if(counter >= post_scale) {
    toggle = !toggle;
    counter = 0;
  }
}

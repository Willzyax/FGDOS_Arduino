void setup() {
  // put your setup code here, to run once:
//  pinMode(3, OUTPUT);
//  pinMode(11,OUTPUT);
//
//  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
//  TCCR2B = _BV(WGM22) | _BV(CS21); // set CS2x to the desired bit to change divider for frequency (CS21 = 8)
//  OCR2A = 63; // freq = 16MHz / divider / (OCR2A +1) (= 31.25 kHz)
//  OCR2B = 31; // duty cycle = OCR2B+1 / OCR2A+1

  // to test, prefer not to use timer 0, it is used by other Arduino functions (like millis)
  //pinMode(5,OUTPUT);
  //TCCR0A = _BV(COM0A0) |  _BV(WGM01) | _BV(WGM00);
  //TCCR0B = _BV(WGM02) | _BV(CS01); // set CS2x to the desired bit to change divider for frequency (CS21 = 8)
  //OCR0A = 63; // freq = 16MHz / divider / (OCR2A +1) (= 31.25 kHz)
  //OCR0B = 31; // duty cycle = OCR2B+1 / OCR2A+1

  // set pin 9 to output, it uses timer1 comp A ( see pinout )
  // WGM register bits all to 1 to select fast PWM and use OCR
  // more details, see atmega328 datasheet
  pinMode(9,OUTPUT);
  TCCR1A = _BV(COM1A0) |  _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // set CS2x to the desired bit to change divider for frequency (CS21 = 8)
  OCR1A = 31; // freq = 16MHz / divider / (OCR2A +1) (= 31.25 kHz)
  OCR1B = 15; // duty cycle = OCR2B+1 / OCR2A+1
}

void loop() {
  // put your main code here, to run repeatedly:

}

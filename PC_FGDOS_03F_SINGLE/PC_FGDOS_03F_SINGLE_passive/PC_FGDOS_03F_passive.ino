/*
 * SETUP
 * SS1 active, SS2 passive, for most test no recharges are necessary
 * SS2 is activated before SS1 reaches threshold and read
 * both recharge and disconnect again
 */
//-----------------------------------------------------------------------------------------------------------------------
// extra needed libraries
#include <SPI.h>
// header file where libraries, constants, masks, pins, etc are defined
#include "definitions.h"

// variables
unsigned long int sens_freq_1 = 0, sens_freq_2 = 0;
unsigned long int ref_freq_1 = 0, ref_freq_2 = 0;
unsigned long int data32;
int temperature_1, temperature_2, recharge_count_1, recharge_count_2, data16;
byte i = 0, reg, data8;
bool flag_print = true, flag_receive = false, flag_request = false, first_run = true;

//-----------------------------------------------------------------------------------------------------------------------
// MAIN SETUP
void setup() {
  // set up serial connection with PC
  Serial.begin(250000); // higher for faster usart transfer, be careful not to lose data when transferring too fast
  while(!Serial);

  // activate sensor by setting nsbty pins high
  // for complete disconnection, disconnect vcc and vccd, eg through MOSFET connected to pin PASSIVE
  // check physically to see which sensor PASSIVE is connected to
  pinMode(NSTBY_1,OUTPUT);
  digitalWrite(NSTBY_1,HIGH);
  pinMode(PASSIVE,OUTPUT);
  digitalWrite(PASSIVE,LOW);
  
   /* 
   * --------------- PWM GENERATION ------------------------
   * Set the registers so the PWM is generated at the desired frequency. For more details see either
   * https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
   * Atmega32p datasheet
   * Current mode is: Fast PWM Mode with OCRA top
   * _BV(x) shifts bits x times to the left and sets this one to 1
   * Pin 9 is used now, which uses timer 1. Timer 2 is needed for pin 3 (see Atmel_FGDOS code)
   * Refrain from using timer 0, it is used for Arduino functions
   */
  pinMode(PWM_PIN, OUTPUT);
  TCCR1A = _BV(COM1A0) |  _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11); // set CS2x to the desired bit to change divider for frequency (CS21 = 8)
  OCR1A = 31; // freq = 16MHz / divider / (OCR2A +1) (= 31.25 kHz)
  OCR1B = 15; // duty cycle = OCR2B+1 / OCR2A+1
  
  // --------------- SPI SETUP ------------------------
  // for details concering SPI library see https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  pinMode(SS1, OUTPUT); //chip enable, active low
  pinMode(SS2, OUTPUT);
  pinMode(10,OUTPUT); //set SS pin to output to prevent Arduino from going into slave mode
  digitalWrite(SS1, HIGH);
  digitalWrite(SS2, HIGH);

  // --------------- SENSOR SETUP ------------------------
  // set up the sensors basic properties, see function for more details
  // first wait for the PWM to settle, this is necessary as it is used for window!!!!
  fgdos_init(SS2);
  fgdos_init(SS1); 

  // ISR upon new data for SS2
  pinMode(NIRQ2, INPUT); //interrupt request data ready, active low
  attachInterrupt(digitalPinToInterrupt(NIRQ2), collect_data_SS2, LOW);

  // Set SS1 to passive
  digitalWrite(PASSIVE,HIGH);

  
}

//-----------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
void loop() {
  
  /* the normal sequence of interrupts is stopped when sensor 2 reaches threshold
  *  Notice that a range is added to the threshold. This is to prevent sensor 1 going outside of the linear region
  *  (and this sensors' behaviour is not perfectly known due to its passive operation)
  *  The added factor is taken as 10% of the average of the linear range
  *  The recharge has to be stopped again by setting FCH
   */
  if (sens_freq_2 < THRESHOLD_FREQ+F_RANGE){
    noInterrupts();
    
    // activate and read SS1 and print out
    digitalWrite(PASSIVE,LOW);
    wait(1); // TO CHECK IF NEEDED
    collect_data_SS1();
    
    // start recharge and continuously print out new frequencies
    activate_recharge(SS1);
    activate_recharge(SS2);
    do {
      collect_data_SS1();
      collect_data_SS2();
      // wait at least 2 windows + 10 % for new data (in millisec: 2/WINDOW_FACTOR*1000)
      wait(2/WINDOW_FACTOR*1000);
    } while((sens_freq_1 < TARGET_FREQ-F_RANGE) && (sens_freq_2 < TARGET_FREQ-F_RANGE));

    // turn off recharge
    deactivate_recharge(SS1);
    deactivate_recharge(SS2);
    
    // deactivate SS1
    digitalWrite(PASSIVE,HIGH);

    interrupts();
  }
  
}

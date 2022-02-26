/*
 * TO DO
 *  - test recharge, only if discharge is possible though maybe?
 *  - see if sensor registers maintained after restartup
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
  // check physically to see which sensor PASSIVE controls
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

  // ISR upon new data
  pinMode(NIRQ1, INPUT); //interrupt request data ready, active low
  pinMode(NIRQ2, INPUT);
  attachInterrupt(digitalPinToInterrupt(NIRQ1), collect_data_SS1, LOW);
  attachInterrupt(digitalPinToInterrupt(NIRQ2), collect_data_SS2, LOW);
}

//-----------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
void loop() {

  //------------- SS2 ------------- 
  // use this to test without interrupts
  /*
  //digitalWrite(NSTBY_2,LOW); // see if still works with standby sensor
  //digitalWrite(PASSIVE,HIGH); // see if still works with passive sensor
  temperature = read_reg(SS2,x0_TEMP);
  recharge_count = read_reg(SS2,x1_RECHARGE_COUNT);
  collect_freq(SS2,&sens_freq,&ref_freq);
  print_meas_short(temperature,sens_freq,ref_freq,recharge_count,2);
  delay(3000);
  // wait at least 2 windows + 10 % for new data! (in millisec: 2/WINDOW_FACTOR*1000)
  //wait(2/WINDOW_FACTOR*1000);
  */

  // use this to test recharge and discharge (CHECK IF POSSIBLE WITH MY SENSOR VERSION)
  // pin connections!
  /*
  if (first_run){
    // disconnect recharging system
    write_reg(sensor,xD_RECHARGE_REF,0x07);
    // set target and threshold augmented
    byte reg_target = floor(110000/WINDOW_FACTOR/1024);
    write_reg(SS2,x9_TARGET,reg_target);
    Serial.print("target ("); Serial.print(TARGET_FREQ); Serial.print(") , "); Serial.println((reg_target << 13)*WINDOW_FACTOR);
    byte reg_threshold = round(100000/WINDOW_FACTOR/1024);
    write_reg(SS2,xA_THRESHOLD,reg_threshold);
    Serial.print("threshold ("); Serial.print(THRESHOLD_FREQ); Serial.print(") , "); Serial.println((reg_threshold << 13)*WINDOW_FACTOR);
    // reconnect recharging system
    write_reg(sensor,xD_RECHARGE_REF,0x47);
    
    // see if recharge ongoing
    Serial.print(RECHARGE REGISTER: ); Serial.println(read_reg(SS_2,x1_RECHARGE_COUNT));
    delay(5000);

    // reset target and threshold
    // disconnect recharging system
    write_reg(sensor,xD_RECHARGE_REF,0x07);
    // set target and threshold augmented
    byte reg_target = floor(TARGET_FREQ/WINDOW_FACTOR/1024);
    write_reg(SS2,x9_TARGET,reg_target);
    Serial.print("target ("); Serial.print(TARGET_FREQ); Serial.print(") , "); Serial.println((reg_target << 13)*WINDOW_FACTOR);
    byte reg_threshold = round(THRESHOLD_FREQ/WINDOW_FACTOR/1024);
    write_reg(SS2,xA_THRESHOLD,reg_threshold);
    Serial.print("threshold ("); Serial.print(THRESHOLD_FREQ); Serial.print(") , "); Serial.println((reg_threshold << 13)*WINDOW_FACTOR);
    // reconnect recharging system
    write_reg(sensor,xD_RECHARGE_REF,0x47);
    
    first_run = false;
    }
    */
  
}

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
unsigned long int sens_freq = 0;
unsigned long int ref_freq = 0;
unsigned long int data32;
int temperature, recharge_count, data16;
byte i = 0, reg, data8,sensor;
bool flag_print = true, flag_receive = false, flag_request = false, first_run = true, read_window = true;
long int window_1, window_2;
float window_factor_1, window_factor_2;
String sens_1, sens_2;

//-----------------------------------------------------------------------------------------------------------------------
// MAIN SETUP
void setup() {
  // set up serial connection with PC
  Serial.begin(250000); // higher for faster usart transfer, be careful not to lose data when transferring too fast
  while(!Serial);
  
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

  // wait a bit first for pwm to settle
  wait(500);
  
}

//-----------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
void loop() {

  if (read_window){
    // the window has been set by another program, so read this first to determine window factor
    // also read the sensitivity settings
    window_1 = convert_to_pulses(read_reg(SS1,xB_RECHARGE_WINDOW) & 0x0C);
    window_2 = convert_to_pulses(read_reg(SS2,xB_RECHARGE_WINDOW) & 0x0C);
    Serial.print("window pulses 1: "); Serial.println(window_1);
    Serial.print("window pulses 2: "); Serial.println(window_2);
    window_factor_1 = CK_FREQ/window_1;
    window_factor_2 = CK_FREQ/window_2;
    
    sens_1 = convert_to_sens(read_reg(SS1,xC_CHARGE_SENS) & 0x07);
    sens_2 = convert_to_sens(read_reg(SS2,xC_CHARGE_SENS) & 0x07); 
    Serial.print("sensitivity 1: "); Serial.println(sens_1);
    Serial.print("sensitivity 2: "); Serial.println(sens_2);
    // you could also try to read recharge settings, but does not seem necessary for now

    read_window = false;
  }

  
  //------------- SS1 ------------- 
  sensor = SS1;
  temperature = read_reg(sensor,x0_TEMP);
  recharge_count = read_reg(sensor,x1_RECHARGE_COUNT);
  if ((recharge_count & 0x80) == 0x80){
    Serial.println("recharging");
  }
  if (recharge_count == 0x7F){
    Serial.println("reset counter");
    write_reg(sensor,x1_RECHARGE_COUNT,0x00);
  }
  collect_freq(sensor,&sens_freq,&ref_freq,window_factor_1);
  print_meas_short(temperature,sens_freq,ref_freq,recharge_count,abs(sensor-9));

  delay(1000);
  
  //------------- SS2 ------------- 
  sensor = SS2;
  temperature = read_reg(sensor,x0_TEMP);
  recharge_count = read_reg(sensor,x1_RECHARGE_COUNT);
  if ((recharge_count & 0x80) == 0x80){
    Serial.println("recharging");
  }
  if (recharge_count == 0x7F){
    Serial.println("reset counter");
    write_reg(sensor,x1_RECHARGE_COUNT,0x00);
  }
  collect_freq(sensor,&sens_freq,&ref_freq,window_factor_2);
  print_meas_short(temperature,sens_freq,ref_freq,recharge_count,abs(sensor-9));
  
  delay(1000);
  // wait at least 2 windows + 10 % for new data! (in millisec: 2/WINDOW_FACTOR*1000)
  //wait(2/WINDOW_FACTOR*1000);
 
}

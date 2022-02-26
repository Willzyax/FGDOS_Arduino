//-----------------------------------------------------------------------------------------------------------------------
// extra needed libraries
#include <SPI.h>
// header file where libraries, constants, masks, pins, etc are defined
#include "definitions.h"

// variables
unsigned long int sens_freq = 0, ref_freq = 0, target_freq = 0, threshold_freq = 0;
unsigned long int data32;
int temperature, recharge_count, data16;
byte i = 0, reg, data8,sensor;
bool flag_print = true, flag_receive = false, flag_request = false, first_run = true;
bool flag_discharge_1 = true, flag_discharge_2 = true, flag_active_1 = true, flag_active_2 = true, flag_recharge_1 = true, flag_recharge_2 = true;
String command_str;
char command;

//-----------------------------------------------------------------------------------------------------------------------
// MAIN SETUP
void setup() {
  // set up serial connection with PC
  Serial.begin(250000); // higher for faster usart transfer, be careful not to lose data when transferring too fast
  while(!Serial);

  // pin to control switch for sensor 1 power connection (passive) and pin to pull sensor 1 into standby
  pinMode(PASSIVE,OUTPUT);
  digitalWrite(PASSIVE,LOW);
  pinMode(NSTBY_1,OUTPUT);
  digitalWrite(NSTBY_1,HIGH);
  pinMode(NSTBY_2,OUTPUT);
  digitalWrite(NSTBY_2,HIGH);
  
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

}

//-----------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
void loop() {

  if(Serial.available()){
      //command = Serial.readStringUntil('\n');
      //command.trim(); // remove leading and trailing characters (spaces when no char specified
      while(Serial.available()){
        command_str = Serial.readString();
      }
      command = command_str[0];
      
      switch (command){
        case '1':
          collect_data(SS1);
          break;
        case '2':
          collect_data(SS2);
          break;
          
        case 't':
          // T for target and threshold
          collect_range(SS1,&target_freq,&threshold_freq);
          Serial.print("TARGET: "); Serial.print(target_freq); 
          Serial.print(", THRESHOLD: "); Serial.println(threshold_freq);
          break; 
        case 'T':
          collect_range(SS2,&target_freq,&threshold_freq);
          Serial.print("TARGET: "); Serial.print(target_freq); 
          Serial.print(", THRESHOLD: "); Serial.println(threshold_freq);
          break;
          
        case 'a':
          if (!flag_active_1){
            digitalWrite(PASSIVE,LOW);
            //digitalWrite(NSTBY_1,HIGH);
            Serial.println("ACTIVE 1");
          } else {
            digitalWrite(PASSIVE,HIGH);
            //digitalWrite(NSTBY_1,LOW);
            Serial.println("PASSIVE 1");
          }
          break;
        case 'A':
          if (!flag_active_2){
            digitalWrite(NSTBY_2,HIGH);
            Serial.println("ACTIVE 2");
          } else {
            digitalWrite(NSTBY_2,LOW);
            Serial.println("PASSIVE 2");
          }
          break;
        
        case 'r':
          if (flag_recharge_1){
            recharge_enable(SS1);
            Serial.println("RECHARGING STARTED");
            flag_recharge_1 = false;  
          } else {
            recharge_disable(SS1);
            Serial.println("RECHARGING STOPPED");
            flag_recharge_1 = true;
          }
          break;
        case 'R':
          if (flag_recharge_2){
            recharge_enable(SS2);
            Serial.println("RECHARGING STARTED");
            flag_recharge_2 = false;  
          } else {
            recharge_disable(SS2);
            Serial.println("RECHARGING STOPPED");
            flag_recharge_2 = true;
          }
          break;

        case 'd':
          if (flag_discharge_1){
            discharge_enable(SS1);
            Serial.println("discharging...");
            flag_discharge_1 = false;  
          } else {
            discharge_disable(SS1);
            Serial.println("DISCHARGING STOPPED");
            flag_discharge_1 = true;
          }
          break;
        case 'D':
          if (flag_discharge_2){
            discharge_enable(SS2);
            Serial.println("discharging...");
            flag_discharge_2 = false;  
          } else {
            discharge_disable(SS2);
            Serial.println("DISCHARGING STOPPED");
            flag_discharge_2 = true;
          }
          break;
          
        case 'i':
          // target, threshold, nirq pin, ... are not set again. Only ref, window, sens
          Serial.print("RESET: ");
          fgdos_init(SS1);
          break;
        case 'I':
          // target, threshold, nirq pin, ... are not set again. Only ref, window, sens
          Serial.print("RESET: ");
          fgdos_init(SS2);
          break;

        case 'l':
          write_reg(SS1,xC_CHARGE_SENS,0x7C);
          Serial.println("SENS 1 LOW");
          break;         
        case 'L':
          write_reg(SS2,xC_CHARGE_SENS,0x7C);
          Serial.println("SENS 2 LOW");
          break;
        case 'h':
          write_reg(SS1,xC_CHARGE_SENS,0x79);
          Serial.println("SENS 1 HIGH");
          break;
        case 'H':
          write_reg(SS2,xC_CHARGE_SENS,0x79);
          Serial.println("SENS 2 HIGH");
          break;
          
        default:
          Serial.print("unknown command: ");
          Serial.println(command);
          break;
      }
    }
}

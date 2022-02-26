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
byte i = 0, reg, data8;
bool flag = true, flag_receive = false, flag_request = false;
String command_str;
char command;

//-----------------------------------------------------------------------------------------------------------------------
// MAIN SETUP
void setup() {
  Serial.begin(250000); // higher for faster usart transfer, be careful not to lose data when transferring too fast
  while(!Serial);

  pinMode(SWITCH,OUTPUT);
  digitalWrite(SWITCH,LOW);

   /* 
   * --------------- PWM GENERATION ------------------------
   * Set the registers so the PWM is generated at the desired frequency. For more details see either
   * https://www.arduino.cc/en/Tutorial/SecretsOfArduinoPWM
   * Atmega32p datasheet
   * Current mode is: Fast PWM Mode with OCRA top
   * _BV(x) shifts bits x times to the left and sets this one to 1
   */
  pinMode(3, OUTPUT);
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS21); // set CS2x to the desired bit to change divider for frequency (CS21 = 8)
  OCR2A = 63; // freq = 16MHz / divider / (OCR2A +1) (= 31.25 kHz)
  OCR2B = 31; // duty cycle = OCR2B+1 / OCR2A+1
  
  // --------------- SPI SETUP ------------------------
  // for details concering SPI library see https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  pinMode(SS1, OUTPUT); //chip enable, active low
  pinMode(SS2, OUTPUT);
  pinMode(10,OUTPUT); //set SS pin to output to prevent Arduino from going into slave mode
  digitalWrite(SS1, HIGH);
  // disable internal pullups
  digitalWrite(SDA, LOW);
  digitalWrite(SCL, LOW);

  // --------------- SENSOR SETUP ------------------------
  // already done normally, however, if power was disconnected this has to be done again (can be done via command)
  
}

//-----------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
void loop() {

   if(Serial.available()){
    //command = Serial.readStringUntil('\n');
    //command.trim(); // remove leading and trailing characters (spaces when no char specified)
    while(Serial.available()){
      command_str = Serial.readString();
    }
    command = command_str[0];
  
    switch (command){
      case '1':
        //Serial.print(command);
        Serial.println("SENSOR 1 NOT CONNECTED");
        break;
      case '2':
        collect_data_SS2();
        break;
      case 'R':
        // simple: target, threshold,... are not set again. Only ref, window, sens, nirq
        //Serial.print("RESET SIMPLE: ");
        //fgdos_init_simple(SS2);
        Serial.print("RESET: ");
        fgdos_init(SS2);
        
        break;
      case 'A':
        Serial.println("activated");
        digitalWrite(SWITCH,LOW);
        digitalWrite(SS2, HIGH);
        break;
 
      case 'D':
        digitalWrite(SWITCH,HIGH);
        digitalWrite(SS2, LOW);
        Serial.println("deactivated");
        break;
        
      default:
        Serial.print("unknown command: ");
        Serial.println(command);
        break;
    }
  }
  // wait at least 2 windows + 10 % for new data! (in millisec: 2/WINDOW_FACTOR*1000)
  //wait(2/WINDOW_FACTOR*1000);
  // extra delay to prevent too much output
  //delay(1000);
}

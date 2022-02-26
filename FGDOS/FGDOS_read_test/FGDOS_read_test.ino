/* TO DO 
 *    - use window measurement cycles as a timeout for the frequency write function?
 * 
*/
//--------------------------------------------------------------------
#include <SPI.h>
//--------------------------------------------------------------------
// Micro pins
// pins for slave selection
#define SS1 9 
#define SS2 8
//--------------------------------------------------------------------
// Useful masks
// WR and RD are combined wit the address via bitwise OR to set the first 2 bits to 01 (write) or 10 (read)
// frequency mask to select the correct bits from the frequency registers
#define WR 0x40
#define RD 0x80
#define FREQ_MASK 0x3FFFF
//--------------------------------------------------------------------
// Function definitions
unsigned int read_reg(byte sensor, byte reg);
bool collect_freq(byte sensor, long int *sens_freq, long int *ref_freq);
//--------------------------------------------------------------------
// variables
long int sens_freq;
long int ref_freq;
//--------------------------------------------------------------------
// Main setup
void setup() {
  Serial.begin(9600);
  while(!Serial);
  
  // SPI setup
  // for details concering SPI library see https://www.arduino.cc/en/Reference/SPI
  SPI.beginTransaction(SPISettings(32000, MSBFIRST, SPI_MODE0));
  SPI.begin();
  pinMode(SS1, OUTPUT); //chip enable, active low
  pinMode(SS2, OUTPUT);
  pinMode(10,OUTPUT); //set SS pin to output to prevent Arduino from going into slave mode
  digitalWrite(SS1, HIGH);
  digitalWrite(SS2, HIGH);
   
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
}


//--------------------------------------------------------------------
// Main loop
void loop() {
  // read the temperature and frequency registers
  Serial.print("Temperature (sensor 1445 offset = 8): "); Serial.println(read_reg(SS2,0x00));
  delay(1000);
  
  if(!collect_freq(SS2,&sens_freq,&ref_freq)){
    Serial.println("reading failed, timeout");
  }
  Serial.print("Sensor and Reference Frequencies : ");
  Serial.print(sens_freq); Serial.print(" | ");
  Serial.print(ref_freq); Serial.println(" | ");
  delay(1000);
}


//--------------------------------------------------------------------
// FUNCTIONS

/* 
 * Fuction to read a registry from the sensor.  
 * first send the address, then read what is being send back
 * apparently the automatic register incrementation (necessary to read more than one register with a single request) 
 * doesn't work properly. Reads have been limited to one reg at a time. (note from CERN)
 */
unsigned int read_reg(byte sensor, byte reg){
  byte data = 0;
  byte adr = reg|RD;
  
  digitalWrite(sensor, LOW);
  SPI.transfer(adr);
  delayMicroseconds(10);
  data = SPI.transfer(0x00); 
  digitalWrite(sensor, HIGH);

  return data;
}

bool collect_freq(byte sensor, long int *sens_freq, long int *ref_freq){
  bool have_sens_freq = false;
  bool have_ref_freq = false;
  byte freq_reg[3];
  unsigned int time_start=millis(); // to prevent getting stuck

  while(!have_sens_freq || !have_ref_freq){
    if(!have_sens_freq){
      // sensor frequency
      freq_reg[0] = read_reg(SS2,0x08);
      freq_reg[1] = read_reg(SS2,0x07);
      freq_reg[2] = read_reg(SS2,0x06);
      // for bitshifting over 16, first cast to long, otherwise bits get dropped
      *sens_freq = ((long)(freq_reg[0] << 8 | freq_reg[1] ) <<8 | freq_reg[2] ) & FREQ_MASK;
      have_sens_freq = true;
    }
    if(!have_ref_freq){
      // reference frequency
      freq_reg[0] = read_reg(SS2,0x05);
      freq_reg[1] = read_reg(SS2,0x04);
      freq_reg[2] = read_reg(SS2,0x03);
      *ref_freq = ((long)(freq_reg[0] << 8 | freq_reg[1] ) <<8 | freq_reg[2] ) & FREQ_MASK;
      have_ref_freq = true;
    }
    if (millis()-time_start>1000){
      return false;
    }
  }
  
//  Serial.print("Reference frequency registers (0x05,0x04,0x03): "); 
//  Serial.print(freq_reg[0],BIN); Serial.print(" | ");
//  Serial.print(freq_reg[1],BIN); Serial.print(" | ");
//  Serial.print(freq_reg[2],BIN); Serial.println(" | ");
//  Serial.print("Frequency reconstructed: "); Serial.println(*ref_freq,BIN); Serial.print(" | "); Serial.println(*ref_freq);

  return true;
}

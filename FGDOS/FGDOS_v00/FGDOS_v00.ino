//-----------------------------------------------------------------------------------------------------------------------
#include <SPI.h>

// Micro pins
// pins for slave selection
#define SS1 9 
#define SS2 8 // 

// Useful masks
// WR and RD are combined wit the address via bitwise OR to set the first 2 bits to 01 (write) or 10 (read)
// frequency mask to select the correct bits from the frequency registers
#define WR 0x40
#define RD 0x80
#define FREQ_MASK 0x3FFFF

// Register definitions
#define REF_WINDOW 0x0B
#define CHARGE_SENS 0x0C
#define RECHARGE 0x0D
#define NIRQ_ENGATE 0x0E
#define TARGET 0x09
#define THRESHOLD 0x0A

// handy constant definitions (these depend on settings in Arduino and sensor!)
#define CK_FREQ 31250 // depends on the settings of the PWM
#define WINDOW_PULSES 8192.0f // depends on window register settings!
#define WINDOW_FACTOR (CK_FREQ/WINDOW_PULSES)

// Function definitions
unsigned int read_reg(byte sensor, byte reg);
void write_reg(byte sensor, byte reg, byte data);
bool collect_freq(byte sensor, long int *sens_freq, long int *ref_freq);
void fgdos_init(byte sensor);
void wait(int microsecs);
void print_freq(byte freq_reg[3],unsigned long int freq_value,char f_type);
void print_meas_full(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, byte sensor);
void print_meas_short(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, byte sensor);

// variables
unsigned long int sens_freq = 0;
unsigned long int ref_freq = 0;
int temperature;
byte i = 0;
bool flag = true;

//-----------------------------------------------------------------------------------------------------------------------
// MAIN SETUP
void setup() {
  Serial.begin(9600);
  while(!Serial);

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
  
  // SPI setup
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
  //fgdos_init(SS1); // for sensor 1 some miso and mosi lines are still connected...
}

//-----------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
void loop() {

  // Sensor 2
  // read the temperature and frequency registers
  temperature = read_reg(SS2,0x00) - 87;
  if(!collect_freq(SS2,&sens_freq,&ref_freq)){
    Serial.println("reading failed, timeout");
  }
//  print_meas_full(temperature,sens_freq,ref_freq,2);
  print_meas_short(temperature,sens_freq,ref_freq,2);

//  store values in arrays
//  sens_freqs[i] = sens_freq;
//  ref_freqs[i] = ref_freq;
//  temperatures[i] = temperature;
//  i++;
  delay(5000);
}


//-----------------------------------------------------------------------------------------------------------------------
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
  //delayMicroseconds(10);
  data = SPI.transfer(0x00); 
  digitalWrite(sensor, HIGH);

  return data;
}

// write to a register
void write_reg(byte sensor, byte reg, byte data){
  byte adr = reg|WR;
  digitalWrite(sensor, LOW);
  SPI.transfer(adr);
  SPI.transfer(data);
  digitalWrite(sensor, HIGH);
}

// read the frequency registers and reconstruct their values
// frequency = registers / window pulses amount * ck frequency
bool collect_freq(byte sensor, long int *sens_freq, long int *ref_freq){
  bool have_sens_freq = false;
  bool have_ref_freq = false;
  byte freq_reg[3];
  unsigned int time_start=millis(); // to prevent getting stuck
  int x = 1000;// increase amount of time to wait if serial.prints are used! Otherwise set to 1

  while(!have_sens_freq || !have_ref_freq){
    if(!have_sens_freq){
      // sensor frequency
      freq_reg[0] = read_reg(sensor,0x08);
      freq_reg[1] = read_reg(sensor,0x07);
      freq_reg[2] = read_reg(sensor,0x06);
      // for bitshifting over 16, first cast to long, otherwise bits get dropped
      *sens_freq = (((unsigned long)(freq_reg[0] << 8 | freq_reg[1] ) <<8 | freq_reg[2] ) & FREQ_MASK );
      have_sens_freq = true;
      //print_freq(freq_reg,*sens_freq,'S');
      *sens_freq = *sens_freq * WINDOW_FACTOR;
    }
    if(!have_ref_freq){
      // reference frequency
      freq_reg[0] = read_reg(sensor,0x05);
      freq_reg[1] = read_reg(sensor,0x04);
      freq_reg[2] = read_reg(sensor,0x03);
      *ref_freq = (((unsigned long)(freq_reg[0] << 8 | freq_reg[1] ) <<8 | freq_reg[2] ) & FREQ_MASK );
      have_ref_freq = true;
      //print_freq(freq_reg,*ref_freq,'R');
      *ref_freq = *ref_freq * WINDOW_FACTOR;

      
    }
    if (millis()-time_start>(4*1/WINDOW_FACTOR*x)){
      return false;
    }
  }
  return true;
}

void fgdos_init(byte sensor){
  Serial.print("-SENSOR "); Serial.println(abs(sensor-10));
  
  // set the reference oscillator and window measurement amount of pulses settings
  // bits (6:4) for ref and (3:2) for window (bit counting from lsb to msb)
  // reference set to 100 (= ??) and windows set to 10 (8192 pulses)(00=32768 ck pulses per window)
  write_reg(sensor,REF_WINDOW,0x48);
  Serial.print("-window_factor "); Serial.println(WINDOW_FACTOR);
  
  // manual recharge off and sesitivity to low
  // MSB to switch on or off manual recharge, 3 LSBs to set sensitivity (100 low, 001 high)
  write_reg(sensor,CHARGE_SENS,0x79);
  Serial.println("-sensitivity high");
  
  // no new data avialable signal via nirq pin (mnrev bit 6), nirq setting to push-pull (nirqoc bit 1), 
  // measurement window to count clocks (engate bit 0)
  write_reg(sensor,NIRQ_ENGATE,0x30);
  
  // disconnect recharging system before setting targets
  write_reg(sensor,RECHARGE,0x00);
  
  // wait for reference to stabilise, eg 4 measurement windows (= 4*32768 pulses at 31.25 kHz = 4194 millisecs)
  wait(4/WINDOW_FACTOR*1000); // 4194
  
  // read reference register ,select 8 MSBs and set them to target register
  // or just pick 50 kHz advised from Sealicon
  //unsigned long int target = (((read_reg(sensor,0x05) << 8 | read_reg(sensor,0x04) ) <<8 | read_reg(sensor,0x03) ) & FREQ_MASK );
  //byte reg_target = (target & 0x3FC00) >> 10;
  unsigned long int target = 50000;
  byte reg_target = round(target/WINDOW_FACTOR/1023); // dividing by 1023 is bitshifting by 10 ;)
  write_reg(sensor,TARGET,reg_target);
  Serial.print("-target "); Serial.println(target);
  //Serial.print("-registry "); Serial.println(reg_target);
  
  // set threshold to 30 kHz equivalent (0x1D = 29, when converted to 8 MSBs of sens_freq (18 bit) = 29696)
  // do not forget to apply window factor! (lower alternative: 0x08 = 8192)
  unsigned long int threshold = 30000/WINDOW_FACTOR;
  byte reg_threshold = (threshold & 0x3FC00) >> 10;
  write_reg(sensor,THRESHOLD,reg_threshold);
  Serial.print("-threshold "); Serial.println(threshold);
  //Serial.print("registry "); Serial.println(reg_threshold);
  
  // enable interrupt upon new data (for now no interrupt pin is actually being used
  write_reg(sensor,NIRQ_ENGATE,0x30);
  
  // also enable automatic recharging again
  write_reg(sensor,RECHARGE,0x41);
  Serial.println();
}

void wait(int millisecs){
  int t1 = millis();
  while(millis()-t1<millisecs);
}

void print_freq(byte freq_reg[3],unsigned long int freq_value,char f_type){
  switch (f_type){
    case 'S':
      Serial.print("SENSOR  frequency registers (0x05,0x04,0x03): ");
      break;
    case 'R':
      Serial.print("REFERENCE frequency registers (0x05,0x04,0x03): ");
      break;
    default:
      Serial.print("error in frequency type");
      break;
  }
  Serial.print(freq_reg[0],HEX); Serial.print(" | ");
  Serial.print(freq_reg[1],HEX); Serial.print(" | ");
  Serial.print(freq_reg[2],HEX); Serial.println(" | ");
  Serial.print("Frequency reconstructed HEX "); Serial.print(freq_value,HEX);
  Serial.print(" | DEC "); Serial.println(freq_value);
}

void print_meas_full(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, byte sensor){
  Serial.print("SENSOR "); Serial.println(sensor);
  Serial.print("\t temperature (sensor 1445 offset = 87): "); Serial.println(temperature);
  Serial.print("\t sensor and reference frequencies : ");
  Serial.print(sens_freq); Serial.print(" | ");
  Serial.print(ref_freq); Serial.println(" | ");
  }
  
void print_meas_short(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, byte sensor){
  flag ? Serial.println("T , Fs , Fr"),flag=false:0;
  Serial.print(temperature);Serial.print(" , ");
  Serial.print(sens_freq);Serial.print(" , ");
  Serial.println(ref_freq);
  }

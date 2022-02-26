/*
 * NOTE: no need to define variables, etc here
 * Arduino (alphabetically) compiles and includes what was first compiled into the next ones
 * see bottom for I2C ISR functions
 */
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

/* 
 * write to a register 
 */
void write_reg(byte sensor, byte reg, byte data){
  byte adr = reg|WR;
  digitalWrite(sensor, LOW);
  SPI.transfer(adr);
  SPI.transfer(data);
  digitalWrite(sensor, HIGH);
}

/* 
 * read the frequency registers and reconstruct their values
 * frequency = registers / window pulses amount * ck frequency
 * also stores them in the I2C data array
 */
void collect_freq(byte sensor,long unsigned int *sens_freq, long unsigned int *ref_freq,float window_factor){
  bool have_sens_freq = false;
  bool have_ref_freq = false;
  byte freq_reg[3];
  unsigned int time_start=millis(); // to prevent getting stuck
  int x = 1000;// increase amount of time to wait if serial.prints are used! Otherwise set to 1
  
  // first check to see if no recharge is going on
  while(!have_sens_freq || !have_ref_freq){
    if(!have_sens_freq){
      // sensor frequency
      freq_reg[0] = read_reg(sensor,0x08);
      freq_reg[1] = read_reg(sensor,0x07);
      freq_reg[2] = read_reg(sensor,0x06);
      // for bitshifting over 16, first cast to long, otherwise bits get dropped
      *sens_freq = (((unsigned long)(freq_reg[0] << 8 | freq_reg[1] ) << 8 | freq_reg[2] ) & FREQ_MASK );
      have_sens_freq = true;
      //print_freq(freq_reg,*sens_freq,'S');
      *sens_freq = *sens_freq * window_factor;
    }
    if(!have_ref_freq){
      // reference frequency
      freq_reg[0] = read_reg(sensor,0x05);
      freq_reg[1] = read_reg(sensor,0x04);
      freq_reg[2] = read_reg(sensor,0x03);
      *ref_freq = (((unsigned long)(freq_reg[0] << 8 | freq_reg[1] ) << 8 | freq_reg[2] ) & FREQ_MASK );
      have_ref_freq = true;
      //print_freq(freq_reg,*ref_freq,'R');
      *ref_freq = *ref_freq * window_factor;
    }
    if ((millis()-time_start>(4*1/window_factor*x) && !have_ref_freq && !have_sens_freq)){
      Serial.println("reading failed, timeout");
    }
  }
}

void collect_data_SS1(){
  //unsigned long int sens_freq = 0, ref_freq = 0;
  //int temperature, recharge_count;
  collect_freq(SS1,&sens_freq,&ref_freq,window_factor_1);
  temperature = read_reg(SS1,x0_TEMP);
  recharge_count = read_reg(SS1,x1_RECHARGE_COUNT); // only 7 LSBs are used for counting
  print_meas_short(temperature,sens_freq,ref_freq,recharge_count,1);
}
void collect_data_SS2(){
  //unsigned long int sens_freq = 0, ref_freq = 0;
  //int temperature, recharge_count;
  collect_freq(SS2,&sens_freq,&ref_freq,window_factor_2);
  temperature = read_reg(SS2,x0_TEMP);
  recharge_count = read_reg(SS2,x1_RECHARGE_COUNT); // only 7 LSBs are used for counting
  print_meas_short(temperature,sens_freq,ref_freq,recharge_count,2);
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
  
void print_meas_short(int temperature, unsigned long int sens_freq, unsigned long int ref_freq, byte reg_recharge_counter, byte sensor){
  flag_print ? Serial.println("Sensor, Temp , F_sens , F_ref, Rech_Count"),flag_print=false:0;
  Serial.print(sensor);Serial.print(" , ");
  Serial.print(temperature);Serial.print(" , ");
  Serial.print(sens_freq);Serial.print(" , ");
  Serial.print(ref_freq);Serial.print(" , ");
  Serial.println(reg_recharge_counter);
  }

void float_to_bytes(float val,byte* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    byte temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}

void wait(int millisecs){
  long int t1 = millis();
  while(millis()-t1<millisecs);
}

long int convert_to_pulses(int window){
  switch (window){
    case 0x0C:
      return 4096;
    case 0x08:
      return 8192;
    case 0x04:
      return 16384;
    case 0x00:
      return 32768;
  }
}

String convert_to_sens(int sens){
  switch (sens){
    case 0x04:
      return "LOW";
    case 0x01:
      return "HIGH";
  }
}

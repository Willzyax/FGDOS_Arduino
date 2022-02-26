///-----------------------------------------------------------------------------------------------------------------------
// extra needed libraries
#include <SPI.h>
#include <FGD_03F.h>

// variables
unsigned long int sens_freq_1 = 0, sens_freq_2 = 0;
unsigned long int ref_freq_1 = 0, ref_freq_2 = 0;
int temperature_1, temperature_2, recharge_count_1, recharge_count_2;
byte i = 0;
bool first_run = true;
bool flag_isr = FGD_ISR;

//-----------------------------------------------------------------------------------------------------------------------
// MAIN SETUP
void setup() {
  // set up serial connection with PC
  Serial.begin(250000); // higher for faster usart transfer, be careful not to lose data when transferring too fast
  while(!Serial);

  standby_passive_pins_setup();
  pwm_setup();
  spi_setup();
  // wait for everything to settle (MOSFET, PWM, Serial)
  wait(1000);
  
  // --------------- SENSOR SETUP ------------------------
  // set up the sensors basic properties, see function for more details
  // first wait for the PWM to settle, this is necessary as it is used for window!!!!
  Serial.println("---------------------");
  fgdos_init(SS2);
  fgdos_init(SS1); 

  // ISR upon new data, interrupt request data ready, active low
  // Arduino pullup used and sensor set to open collector (can only pull low)
  if (flag_isr){
    pinMode(NIRQ_1, INPUT_PULLUP); 
    pinMode(NIRQ_2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(NIRQ_1), collect_data_SS1, LOW);
    attachInterrupt(digitalPinToInterrupt(NIRQ_2), collect_data_SS2, LOW);
  }
}

//-----------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
void loop() {
  
  /*
  delay(2000);
  print_meas_short(temperature_2,sens_freq_2,ref_freq_2,recharge_count_2,2);
  print_meas_short(temperature_1,sens_freq_1,ref_freq_1,recharge_count_1,1);
  */
  
  //------------- SS2 ------------- 
  // use this to test without interrupts
  /*
  //digitalWrite(NSTBY_2,LOW); // see if still works with standby sensor
  //digitalWrite(PASSIVE,HIGH); // see if still works with passive sensor
  temperature_2 = read_reg(SS2,x0_TEMP);
  recharge_count_2 = read_reg(SS2,x1_RECHARGE_COUNT);
  collect_freq(SS2,&sens_freq_2,&ref_freq_2,WINDOW_FACTOR);
  print_meas_short(temperature_2,sens_freq_2,ref_freq_2,recharge_count_2,2);
  delay(1000);
  // wait at least 2 windows + 10 % for new data! (in millisec: 2/WINDOW_FACTOR*1000)
  //wait(2.1/WINDOW_FACTOR*1000);
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
    
    // test to see how long the mosfet has to settle, note the init function has waits included 
    while(i<5){
      collect_data(SS2, &temperature_2, &recharge_count_2, &sens_freq_2, &ref_freq_2, WINDOW_FACTOR);
      // wait at least 2 windows + 10 % for new data! (in millisec: 2/WINDOW_FACTOR*1000)
      wait(2/WINDOW_FACTOR*1000);
      i = i+1;
    }
    i = 0;
    digitalWrite(PASSIVE,HIGH);
    while(i<5){
      collect_data(SS2, &temperature_2, &recharge_count_2, &sens_freq_2, &ref_freq_2, WINDOW_FACTOR);
      // wait at least 2 windows + 10 % for new data! (in millisec: 2/WINDOW_FACTOR*1000)
      wait(250);
      i = i+1;
    }
    i = 0;
    wait(2000);
    digitalWrite(PASSIVE,LOW);
    wait(750);
    fgdos_init(SS2);
    //wait(750);
    while(i<5){
      collect_data(SS2, &temperature_2, &recharge_count_2, &sens_freq_2, &ref_freq_2, WINDOW_FACTOR);
      // wait at least 2 windows + 10 % for new data! (in millisec: 2/WINDOW_FACTOR*1000)
      wait(250);
      i = i+1;
    }
    
}

//------------------------------------- ISR -------------------------------------
void collect_data_SS1(){
  collect_data(SS1, &temperature_1, &recharge_count_1, &sens_freq_1, &ref_freq_1, WINDOW_FACTOR);
}
void collect_data_SS2(){
  collect_data(SS2, &temperature_2, &recharge_count_2, &sens_freq_2, &ref_freq_2, WINDOW_FACTOR);
}

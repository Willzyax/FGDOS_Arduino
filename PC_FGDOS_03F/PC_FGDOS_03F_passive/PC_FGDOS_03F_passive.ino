/*
 * SETUP
 * SS1 active, SS2 passive/standby, for most test no recharges are necessary
 * standby: register setting | passive: mosfet disconnect
 * several methods can be used, see explanation in each method for details
 * METHOD 1: use SS1 as ref, manual recharge
 * METHOD 2: use SS1 as ref, auto recharge
 * METHOD 3: use sens estimate, auto recharge
 * 
 * NOTES:
 * during the active period interrupts signal is disconnected
 * the command noInterrupts is NOT to be used, since this disables all interrupts and messes with wait and serial functions
 */

//-----------------------------------------------------------------------------------------------------------------------
// extra needed libraries
#include <SPI.h>
#include <FGD_03F.h>

// settings
#define METHOD 3 // method 1, 2 or 3
#define F_RANGE (TARGET_FREQ-THRESHOLD_FREQ)*0.1 // can be used to set limits for passive sensor, 10% of range
unsigned int sens_estimate = 9690; // Hz/Gy
float dose_rate = 0.008558; // Gy/s
float time_recharge = ((TARGET_FREQ-THRESHOLD_FREQ)/sens_estimate)/dose_rate; // s
float time_limit = 1000*time_recharge; // 10% error margin (random), ms
bool flag_manual_1 = false;
bool flag_manual_2 = false;

// variables
unsigned long int sens_freq_1 = 0, sens_freq_2 = 0, ref_freq_1 = 0, ref_freq_2 = 0;
int temperature_1, temperature_2, recharge_temp_1, recharge_temp_2;
byte recharge_count_1, recharge_count_2;
unsigned int time_start;
bool flag_isr = FGD_ISR;
bool flag_passive;

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
  fgdos_init(SS1); 
  fgdos_init(SS2);
  
  // print frequencies a first time and then set SS2 to passive
  collect_data_SS1();
  collect_data_SS2();
  
  // put into passive AND standby
  digitalWrite(PASSIVE,HIGH);
  Serial.println("PASSIVE 2");
  digitalWrite(NSTBY_2,LOW);
  Serial.println("STANDBY 2");
  flag_passive = true;

  // start measuring time in case you work with time limit for recharges
  // set auto to passive in case of manual recharge method
  if (METHOD == 1){
    auto_recharge_disable(SS1);
    Serial.println("AUTO RECHARGE 1 passive");
  }
  if (METHOD == 3){
    time_start = millis(); // ms
    Serial.print("recharge time and start time: "); Serial.print(time_recharge); Serial.print("  "); Serial.println(time_start);
    //write_reg(sensor,x1_RECHARGE_COUNT,0x00);
    //Serial.println("reset recharge counter 2");
  }
  
  // ISR upon new data for SS1
  // also for passive sensor, since in passive mode IRSs are not triggered
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

  //select METHOD to use
  switch (METHOD){
    case 1:
      // METHOD 1: USE THRESHOLD AND MANUAL RECHARGE --------------------------------------------------
      /* the normal sequence of interrupts is stopped when sensor 2 reaches threshold
      *  Notice that a range can be added to the threshold. This is to prevent sensor 1 going outside of the linear region
      *  (and this sensors' behaviour is not perfectly known due to its passive operation)
      *  The added factor is taken as 10% of the linear range
      *  The recharge has to be stopped again by setting FCH
      *  You can also use the sensor's standby mode instead of the mosfet switching (which takes time)
      *  note that in this approach both sensors are manually recharged!
       */
       
      if (sens_freq_1 < THRESHOLD_FREQ+F_RANGE){
        // option to use threshold +F_RANGE or not
        // turn of the interrupt measuring during this process
        detachInterrupt(digitalPinToInterrupt(NIRQ_1));
        
        // activate and read SS2 and print out
        // time is needed for MOSFET to settle (750 ms) as it seems from tests, can also be placed after init function
        // you can ignore but then your first measurement is 0...
        digitalWrite(PASSIVE,LOW);
        Serial.println("ACTIVE 2");
        digitalWrite(NSTBY_2,HIGH);
        Serial.println("ON 2");
        wait(750);
        // you have to set registers first again, make sure to enter passive mode to prevent direct recharge
        fgdos_init_variable(SS1,SENS,"passive"); 
        fgdos_init_variable(SS2,SENS,"passive"); 
        collect_data_SS1();
        collect_data_SS2();
        
        // start recharge and continuously print out new frequencies while target -F_range is not reached
        // stop as soon as one of the two goes over it
        recharge_enable(SS1);
        recharge_enable(SS2);
        flag_manual_1 = true;
        flag_manual_2 = true;
        
        do {
          collect_data_SS1();
          collect_data_SS2();

          if (sens_freq_1>TARGET_FREQ){
            recharge_disable(SS1);  
            flag_manual_1 = false;  
          }
          if (sens_freq_2>TARGET_FREQ){
            recharge_disable(SS2);    
            flag_manual_2 = false;
          }
          // wait at least 2 windows + 10 % for new data (in millisec: 2/WINDOW_FACTOR*1000)
          wait(2.1/WINDOW_FACTOR*1000);
        } while(flag_manual_1 || flag_manual_2);
        
        // deactivate SS2
        Serial.println("PASSIVE 2");
        digitalWrite(PASSIVE,HIGH);
        Serial.println("STANDBY 2");
        digitalWrite(NSTBY_2,LOW);
    
        attachInterrupt(digitalPinToInterrupt(NIRQ_1), collect_data_SS1, LOW);
      }
      break;
      
    case 2:
      // METHOD 2: USE THRESHOLD AND AUTO RECHARGE --------------------------------------------------
      if (sens_freq_1 < THRESHOLD_FREQ+F_RANGE){
        // option to use threshold +F_RANGE or not
        // turn of the interrupt measuring during this process
        detachInterrupt(digitalPinToInterrupt(NIRQ_1));
        
        // activate and read SS2 and print out
        // time is needed for MOSFET to settle (750 ms) as it seems from tests, can also be placed after init function
        // you can ignore but then your first measurement is 0...
        digitalWrite(PASSIVE,LOW);
        Serial.println("ACTIVE 2");
        digitalWrite(NSTBY_2,HIGH);
        Serial.println("ON 2");
        wait(750);
        // you have to set registers first again, make sure to enter passive mode to prevent direct recharge
        fgdos_init_variable(SS2,SENS,"passive"); 
        collect_data_SS1();
        collect_data_SS2();
    
        // store the amount of recharges
        recharge_temp_1 = recharge_count_1 & 0x7F;
        recharge_temp_2 = recharge_count_2 & 0x7F;
        
        // activate auto recharge and wait for both sensors to complete recharge
        auto_recharge_enable(SS2);
        Serial.println("AUTO RECHARGE 2 active");
        
        do {
          collect_data_SS1();
          collect_data_SS2();
          // wait at least 2 windows + 10 % for new data (in millisec: 2/WINDOW_FACTOR*1000)
          wait(2.1/WINDOW_FACTOR*1000);
        } while(((recharge_count_1 & 0x7F) <= recharge_temp_1) && ((recharge_count_2 & 0x7F) <= recharge_temp_2));
        
        // deactivate SS2
        Serial.println("PASSIVE 2");
        digitalWrite(PASSIVE,HIGH);
        Serial.println("STANDBY 2");
        digitalWrite(NSTBY_2,LOW);
    
        attachInterrupt(digitalPinToInterrupt(NIRQ_1), collect_data_SS1, LOW);
      }
      break;

      case 3:
        // METHOD 3: USE SENSITIVITY ESTIMATION TO ESTIMATE TIME PERIOD --------------------------------------------------
        /*
         * other option is to estimate/determine sensitivity another way and estimate time after which recharge is necessary
         * see global variables to set sensitivity estimation and dose rate and setup for start time determination
         * The comparison of the recharge counter would be an issue if it overflows. But this counter is put to 0
         * when the sensor comes out of standby anyway. However, to be sure, recharge counter was reset in the beginning
         */
        if (((millis()-time_start)>time_limit) && flag_passive){         
          // activate and read SS2 and print out
          // time is needed for MOSFET to settle (750 ms) as it seems from tests, can also be placed after init function
          // you can ignore but then your first measurement is 0...
          digitalWrite(PASSIVE,LOW);
          Serial.println("ACTIVE 2");
          digitalWrite(NSTBY_2,HIGH);
          Serial.println("ON 2");
          wait(750);
          // you have to set registers first again, make sure to enter passive mode to prevent direct recharge
          fgdos_init_variable(SS2,SENS,"passive");
          wait(250);
          collect_data_SS1();
          collect_data_SS2();     
          // store the amount of recharges
          recharge_temp_2 = recharge_count_2 & 0x7F;
          // let the sensor auto recharge
          auto_recharge_enable(SS2);
          Serial.println("AUTO RECHARGE 2 active");
          flag_passive = false;
        }
        
        // make sure measurements are collected during recharge
        if (((recharge_count_1 & 0x80) == 0x80) || ((recharge_count_2 & 0x80) == 0x80)){
          if ((recharge_count_1 & 0x80) == 0x80){collect_data(SS1, &temperature_1, &recharge_count_1, &sens_freq_1, &ref_freq_1, WINDOW_FACTOR);}
          if ((recharge_count_2 & 0x80) == 0x80){collect_data(SS2, &temperature_2, &recharge_count_2, &sens_freq_2, &ref_freq_2, WINDOW_FACTOR);}
          wait(2.1/WINDOW_FACTOR*1000);
          Serial.println("extra loop running");
        }

        // set back to passive after recharge
        if (!flag_passive && (recharge_temp_2 < (recharge_count_2 & 0x7F)) && ((recharge_count_2 & 0x80) != 0x80)){
          // deactivate SS2
          Serial.println("PASSIVE 2");
          digitalWrite(PASSIVE,HIGH);
          Serial.println("STANDBY 2");
          digitalWrite(NSTBY_2,LOW);
          flag_passive = true;
          time_start = millis();
        }
        break;
    }
}

//------------------------------------- ISR -------------------------------------
void collect_data_SS1(){
  collect_data(SS1, &temperature_1, &recharge_count_1, &sens_freq_1, &ref_freq_1, WINDOW_FACTOR);
}
void collect_data_SS2(){
  collect_data(SS2, &temperature_2, &recharge_count_2, &sens_freq_2, &ref_freq_2, WINDOW_FACTOR);
}

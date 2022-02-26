
//-----------------------------------------------------------------------------------------------------------------------
// extra needed libraries
#include <SPI.h>
#include <FGD_03F.h>

// variables
unsigned long int sens_freq_1 = 0, sens_freq_2 = 0, target_freq = 0, threshold_freq = 0;
unsigned long int ref_freq_1 = 0, ref_freq_2 = 0;
char command;
int temperature_1, temperature_2, recharge_count_1, recharge_count_2;

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

  Serial.println("---------------------");
}

//-----------------------------------------------------------------------------------------------------------------------
// MAIN LOOP
void loop() {

  command = get_command();
  set_command(command, &sens_freq_1, &sens_freq_2, &target_freq, &threshold_freq, &ref_freq_1, &ref_freq_2,
      &temperature_1, &temperature_2, &recharge_count_1, &recharge_count_2, false);
      
}

// Micro pins SPI
// pins for slave selection SPI
#define SS1 7 
#define SS2 8
#define PWM_PIN 9
#define PASSIVE 6
#define NSTBY_1 4
#define NSTBY_2 5

// Useful masks
// WR and RD are combined wit the address via bitwise OR to set the first 2 bits to 01 (write) or 10 (read)
// frequency mask to select the correct bits from the frequency registers
#define WR 0x40
#define RD 0x80
#define FREQ_MASK 0x3FFFF

// Register definitions
#define x0_TEMP 0x00
#define x1_RECHARGE_COUNT 0x01
#define x9_TARGET 0x09
#define xA_THRESHOLD 0x0A
#define xB_RECHARGE_WINDOW 0x0B
#define xC_CHARGE_SENS 0x0C
#define xD_RECHARGE_REF 0x0D
#define xE_NIRQ_ENGATE 0x0E

// sensor constant definitions (these depend on settings in Arduino and sensor!)
#define CK_FREQ 31250 // depends on the settings of the PWM
#define WINDOW_PULSES 4096.0f // depends on window register settings!
#define WINDOW_FACTOR (CK_FREQ/WINDOW_PULSES)
#define BYTES_SEND 13 // amount of bytes to send over I2C

// register overall valid settings
// nirq setting to push-pull (nirqoc bit 1), measurement window to count clocks (engate bit 0)
// edirt bit to 1
#define xE_settings 0x04
// set the reference oscillator and window measurement amount of pulses settings
// bits (8:4) for recharging (enable autorech, internal pump at VB, rech by pump, 0) and (3:2) for window (bit counting from lsb to msb)
// reference set to 100 (= ??) and windows set to 11 (4096 pulses)(00=32768 ck pulses per window)
// TDIV (bit 0) to 1 for more precise frequency range
// CD default, 4D to disable auto recharge
#define xB_settings 0xCD

// CHOOSE SENSITIVITY
#define HIGHSENS
//#define LOWSENS

#ifdef HIGHSENS
  #define SENS "HIGH"
  // these values are not exact! Since only 5 bits are used (depends on TDIV)
  #define THRESHOLD_FREQ 50000 
  #define TARGET_FREQ 90000
  // manual recharge off and sesitivity to high
  // MSB to switch on or off manual recharge, 3 LSBs to set sensitivity (100 low, 001 high)
  #define xC_settings 0x79
  // disconnect recharging system before setting targets, set pump level (for shortest recharge time bit 3:0 to 111)
  // the recharge voltage of 100 (16.5V) seems to be good, otherwise to fast and overshoots in recharging
  // set reference close to target, according to sensitivity  (should be bit 3 to 1 for highsens, but does not work)
  #define xD_settings_off 0x04
  // allow recharges again
  #define xD_settings_on 0x44
#endif

#ifdef LOWSENS
  #define SENS "LOW"
  // these values are not exact! Since only 5 bits are used (depends on TDIV)
  #define THRESHOLD_FREQ 140000 
  #define TARGET_FREQ 180000
  // manual recharge off and sesitivity to low
  // MSB to switch on or off manual recharge, 3 LSBs to set sensitivity (100 low, 001 high)
  #define xC_settings 0x7C
  // nirq setting to push-pull (nirqoc bit 1), measurement window to count clocks (engate bit 0)
  // edirt to 1 (no measuring during SPI to reduce noise)
  #define xE_settings 0x04
  // disconnect recharging system before setting targets, set pump level (for shortest recharge time bit 3:0 to 111)
  // the recharge voltage of 100 (16.5V) seems to be good, otherwise to fast and overshoots in recharging
  // set reference close to target, according to sensitivity  (should be bit 3 to 1 for highsens, but does not work)
  #define xD_settings_off 0x04
  // allow recharges again
  #define xD_settings_on 0x44
#endif

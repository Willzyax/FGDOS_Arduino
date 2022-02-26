// Micro pins SPI
// pins for slave selection SPI
#define SS1 7 
#define SS2 8
#define PWM_PIN 9

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
#define BYTES_SEND 13 // amount of bytes to send over I2C


#include <SPI.h>
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

// buffer to store incomming SPI info, variable to store measurement
int data, measure = 400;
byte buf_in[2];
byte buf_out[2] = {0,1};
byte byte_in;
byte byte_out = 66;
// volatile parameters that can be changed by ISR, since these variables can be changed beyond the main loop
volatile byte pos_in = 0, c = 0, pos_out= 0;
volatile boolean spi_called = false;
// pins
int MEAS;
// others
bool flag = false;

void setup() {
  Serial.begin (19200);

  // set up build in LED (uses D13, which is needed for SPI...
  //pinMode(LED_BUILTIN, OUTPUT);
  // measurement pin 
  pinMode(MEAS,INPUT);
  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);

  // now turn on interrupts
  SPI.attachInterrupt();
}

// SPI interrupt routine
ISR (SPI_STC_vect){
  /*
  // part of process to send and/or receive an int, other part in main loop --------------------
  if(pos_out < sizeof buf_out){
    if (pos_out == sizeof buf_in){
      SPDR = buf_out[0]; 
      pos_in = 0;
    }
    SPDR = buf_out[pos_out++];
  }
  if (pos_in < sizeof buf_in){
    if (pos_in == sizeof buf_in -1){
      SPDR = buf_out[0]; 
      pos_out = 1;
    }
    buf_in [pos_in++] = SPDR; 
  } 
  */
  /*
  // process to read an array of data from buffer -----------------------------------------------
  // works only if master sends 1 large stream?
  while(pos < sizeof buf_in){
    c = SPDR;  // grab byte from SPI Data Register and add to buffer if room
    if (pos < sizeof buf_in){
      buf_in [pos++] = c; 
    }  // end of room available
  }
  */

  // process to do a simultaneous transfer (using spi_transfer in atmel) ------------------------
  if(++pos_out < sizeof buf_out){
    SPDR = buf_out[pos_out];
  }
  if (pos_in < sizeof buf_in){
    buf_in [pos_in++] = SPDR; 
  } 
  
  flag = false;
  spi_called = true;
}

void loop() {
  
  if (spi_called){
    data = buf_in[1]<<8 | buf_in[0];
    Serial.print("buffer in 1: "); Serial.print(buf_in[0]);
    Serial.print("| buffer in 2: "); Serial.print(buf_in[1]);
    Serial.print("| data: "); Serial.println(data);

    data = buf_out[0]<<8 | buf_out[1];
    Serial.print("buffer out 1: "); Serial.print(buf_out[0]);
    Serial.print("| buffer out 2: "); Serial.print(buf_out[1]);
    Serial.print("| data: "); Serial.println(data);

    buf_out[0] = highByte(measure);
    buf_out[1] = lowByte(measure);
    measure ++;
    pos_in = 0;
    pos_out = 0;
    spi_called = false;
    SPDR = buf_out[0];
    }
    
}


#include <SPI.h>

#define lowByte(w) ((uint8_t) ((w) & 0xff))
#define highByte(w) ((uint8_t) ((w) >> 8))

// buffer to store incomming SPI info, variable to store measurement
int measure = 666;
byte buf[2];
byte byte_in;
byte byte_out = 66;
char command;
// volatile parameters that can be changed by ISR, since these variables can be changed beyond the main loop
volatile byte pos = 0, c = 0;
volatile boolean process_it;
byte counter = 0;
// pins
int MEAS;

// others
bool flag = false;

void setup() {
  Serial.begin (9600);   // debugging

  // set up build in LED (uses D13, which is needed for SPI...
  //pinMode(LED_BUILTIN, OUTPUT);

  // measurement pin 
  pinMode(MEAS,INPUT);
  
  // have to send on master in, *slave out*
  pinMode(MISO, OUTPUT);
  // turn on SPI in slave mode
  SPCR |= _BV(SPE);
  
  // get ready for an interrupt 
  pos = 0;   // buffer empty
  process_it = false;

  // now turn on interrupts
  SPI.attachInterrupt();

}

// SPI interrupt routine
ISR (SPI_STC_vect){

  // process to read an array of data from buffer
  /*while(pos < sizeof buf){
    c = SPDR;  // grab byte from SPI Data Register and add to buffer if room
    if (pos < sizeof buf){
      buf [pos++] = c; 
    }  // end of room available
    delay(1); // small delay, otherwise same byte read twice
  }
  */

  // process to store data in buffer that can be read by master
  /*if (!flag){
    measure ++;
    //buf[0] = SPI.transfer(highByte(measure));
    SPDR = highByte(measure);
    flag = true;
  } 
  else {
    SPDR = lowByte(measure);
    flag = false;
    process_it = true;
  */
  
  // process to perform read and write, first read SPDR then assign new (byte) value
  SPDR = byte_out;
  if (!flag){    
    command = SPDR;
    flag = true;
  } 
  else {
    switch (command) {
    case 'R':
      // read from slave
      SPDR = byte_out;
      byte_out++;
      break;
    case 'W':
      // write to slave
      byte_in = SPDR;
      break;
    default:
      // invalid command
      command = 'X';
      SPDR = 66;
      break;
      }
    flag = false;
    process_it = true;
  }
}

void loop() {
  
  /*
  counter ++;
  delay(30);
  if (counter == 100){
    Serial.println("LOOP...");
    counter = 0;
  }
  */
 
  if (process_it){
    /*Serial.print("data: ");
    for (byte i = 0; i < sizeof buf; i++){
      Serial.print(buf[i]); Serial.print(" | ");
    }*/
    pos = 0;
    process_it = false;
    Serial.println(command);
    Serial.print("byte in: "); Serial.println(byte_in);
    Serial.print("byte out: "); Serial.println(byte_out);
    /*
    Serial.print("measure (value, highbit, lowbit: "); Serial.print(measure); 
    Serial.print(" | "); Serial.print(highByte(measure));
    Serial.print(" | "); Serial.println(lowByte(measure));
    */
    }  // end of flag set

  //delay(1000);
  //measure = analogRead(MEAS);
  //Serial.println(measure);
}

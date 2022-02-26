#include <SPI.h>
// 

//--------------------------------------------------------------------
// Micro pins
#define SS1 9 
#define SS2 8
#define LEDR 3

byte data_s = 0;
byte data_r = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while(!Serial);
  
  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  pinMode(SS1, OUTPUT); //chip enable, active low
  pinMode(SS2, OUTPUT);
  pinMode(10,OUTPUT); //set SS pin to output to prevent Arduino from going into slave mode
  digitalWrite(SS1, HIGH);
  digitalWrite(SS2, HIGH);

  pinMode(LEDR, OUTPUT);
  digitalWrite(LEDR, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  data_r = 0;
  data_r = SPI.transfer(data_s);
  Serial.println(data_r);
  data_s++;
  delay(1000);
  
}

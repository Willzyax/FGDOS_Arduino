#include <Wire.h>


void setup()
{

  Serial.begin(9600); // open the serial port at 9600 bps:
    
  Wire.begin(0x25); 
  Wire.onReceive(receiveEvent); 
  Wire.onRequest(requestEvent);
}

  byte data = 0;
  String input;
  char inp;

void loop()
{
  Serial.print("Test Run \n");
  delay(5000);
   
}

void receiveEvent(int howMany)
{
  Serial.print("INPUT: "); Serial.println(data);
  data = 0;


  while( Wire.available()){
    data = Wire.read();
  }   
  
  
}

void requestEvent()
{
  Serial.println("data out");
  byte myArray[2];
  myArray[0] = (data >> 8) & 0xFF;
  myArray[1] = data & 0xFF;
  Wire.write(myArray,2);
}

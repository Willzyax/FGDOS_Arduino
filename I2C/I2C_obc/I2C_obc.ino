#include <Wire.h>
#define LED_R 13  // Red LED
#define LED_G 12  // Green LED

void setup()
{
  pinMode(LED_R, OUTPUT); // Declare the LED as an output
  pinMode(LED_G, OUTPUT); // Declare the LED as an output
  pinMode(A0, INPUT) ; // Declare the voltage read pin as input
  Serial.begin(9600); // open the serial port at 9600 bps:
    
  Wire.begin(0x25); 
  //Wire.setClock(1000000);
  Wire.onReceive(receiveEvent); 
  Wire.onRequest(requestEvent);
}

  int input;
  char ADCChar;
  int ADCValue;
  byte ADCArray[2];
  int ADCnew;
  byte test = 7;

void loop()
{
  digitalWrite(LED_R, HIGH); // Turn the LED on
  //Serial.print("Test Run \n");
  delay(1000);
  
  digitalWrite(LED_R, LOW); // Turn the LED on
  delay(1000);
  
  ADCValue = analogRead(A0)*5/1.023;
  //ADCChar = (char)ADCValue; 
  ADCArray[0] = (ADCValue >> 8) & 0xFF;
  ADCArray[1] = ADCValue & 0xFF;
  /*
  Serial.print("ADCValue and ADCArray in binary \t");
  Serial.print(ADCValue,BIN); Serial.print("\t");
  Serial.print(ADCArray[0],BIN); Serial.print("\t");
  Serial.print(ADCArray[1],BIN); Serial.println("\t");
  */
  ADCnew = ADCArray[0];
  ADCnew = (ADCnew<<8) | ADCArray[1];
  /*
  Serial.print("ADCValue and ADC reconstructed \t");
  Serial.print(ADCValue); Serial.print("\t"); Serial.println(ADCnew);
  */
}

void receiveEvent(int howMany)
{
  //digitalWrite(LED_G, HIGH); // Turn the LED on
  //delay(100);  
  int input[2];
  int i = 0;
  int received;

  while( Wire.available()){
    input[i] = Wire.read();
    ++i;
  }

  received = (input[1]<<8) | input[0];
  Serial.print("INPUT (value, binary): "); Serial.print(received);
  Serial.print("\t"); Serial.print(input[1],BIN); Serial.println(input[0],BIN);

  //digitalWrite(LED_G, LOW); // Turn the LED off
}

void requestEvent()
{  
  digitalWrite(LED_G, HIGH); // Turn the LED on
  
  ADCValue = analogRead(A0)*5/1.023;
  ADCArray[0] = (ADCValue >> 8) & 0xFF;
  ADCArray[1] = ADCValue & 0xFF;
  
  Wire.write(ADCArray,2);
  
  Serial.print("OUTPUT (value, binary): "); Serial.print(ADCValue); 
  Serial.print("\t"); Serial.println(ADCValue,BIN);
  //Serial.print("voltage \t"); Serial.print(ADCValue); Serial.print("\t char \t"); Serial.println(ADCChar); 
  
  digitalWrite(LED_G, LOW); // Turn the LED off
}

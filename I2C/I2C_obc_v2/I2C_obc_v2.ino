#include <Wire.h>
#define LED_R 13  // Red LED
#define LED_G 12  // Green LED
int ADCValue;
byte ADCArray[2];
int ADCnew;
int r_data;

void setup()
{
  pinMode(LED_R, OUTPUT); // Declare the LED as an output
  pinMode(LED_G, OUTPUT); // Declare the LED as an output
  pinMode(A0, INPUT) ; // Declare the voltage read pin as input
  Serial.begin(9600); // open the serial port at 9600 bps:
    
  Wire.begin(0x25); // set slave address
  //Wire.setClock(1000000);
  Wire.onReceive(receiveEvent); 
  Wire.onRequest(requestEvent);
}

void loop()
{
  digitalWrite(LED_R, HIGH); // Turn the LED on
  delay(500);
  
  digitalWrite(LED_R, LOW); // Turn the LED on
  delay(1000);

  Serial.print("Data Send: "); Serial.print(ADCValue); Serial.print("\t");
  Serial.print("Data Received: "); Serial.println(r_data);
/*  
  ADCValue = analogRead(A0)*5/1.023;
  ADCArray[0] = (ADCValue >> 8) & 0xFF;
  ADCArray[1] = ADCValue & 0xFF;
  ADCnew = (ADCArray[0]<<8) | ADCArray[1];
 
  Serial.print("ADCValue and ADC reconstructed \t");
  Serial.print(ADCValue); Serial.print("\t"); Serial.println(ADCnew);
*/
}

void receiveEvent(int howMany)
{
  digitalWrite(LED_G, HIGH); // Turn the LED on
  int r_data_array[2];
  int i = 0;
  
  while( Wire.available()){
    r_data_array[i] = Wire.read();
    ++i;
  }

  r_data = (r_data_array[1]<<8) | r_data_array[0];
//  Serial.print("INPUT (value, binary): "); Serial.print(r_data);
//  Serial.print("\t"); Serial.print(r_data_array[1],BIN); Serial.println(r_data_array[0],BIN);

  delay(50);
  digitalWrite(LED_G, LOW); // Turn the LED off
}

void requestEvent()
{  
  digitalWrite(LED_G, HIGH); // Turn the LED on
  
  ADCValue = analogRead(A0)*5/1.023;
  ADCArray[0] = (ADCValue >> 8) & 0xFF;
  ADCArray[1] = ADCValue & 0xFF;
  
  Wire.write(ADCArray,2);
  
  //Serial.print("OUTPUT (value, binary): "); Serial.print(ADCValue); 
  //Serial.print("\t"); Serial.println(ADCValue,BIN);
  //Serial.print("voltage \t"); Serial.print(ADCValue); Serial.print("\t char \t"); Serial.println(ADCChar); 

  delay(50);
  digitalWrite(LED_G, LOW); // Turn the LED off
}

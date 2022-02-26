char command;

void setup() {
  // set up serial connection with PC
  Serial.begin(250000); // higher for faster usart transfer, be careful not to lose data when transferring too fast
  while(!Serial);
}

void loop() {
  if(Serial.available()){
    //command = Serial.readStringUntil('\n');
    //command.trim(); // remove leading and trailing characters (spaces when no char specified
    while(Serial.available()){
      command = Serial.read();
      delay(1);  
    }
    
    switch (command){
      case '1':
        Serial.print(command);
        Serial.println(" testing sensor 1: ");
        break;
      case '2':
        Serial.print(command);
        Serial.println(" testing sensor 2");
        break;
      default:
        Serial.print(command);
        Serial.println(" unknown command");
        break;
    }
  }
}

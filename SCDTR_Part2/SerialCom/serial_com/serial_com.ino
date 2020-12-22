#include <SoftwareSerial.h>
SoftwareSerial mySerial(1, 0);


void setup() {
  Serial.begin(9600);   
  mySerial.begin(9600);
}

 

void loop() {
    mySerial.print("HELLO\n\r"); // To PI 
    
    if(mySerial.available()) { // From PI to Arduino Serial
      Serial.print("Echo :" + mySerial.readString());   
    };
    delay(800);                
}

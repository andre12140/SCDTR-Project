
void setup() {  
  Serial.begin(1000000);
}

 
void loop() {
    
    if(Serial.available()) { // From PI to Arduino Serial
      Serial.print(Serial.readString());  // From Arduino to PI 
    };
    delay(200);                
}

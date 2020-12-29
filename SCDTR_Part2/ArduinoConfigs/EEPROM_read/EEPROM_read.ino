// DESCRIPTION: Reads the EEPROM and verifies if the it is corrupted or not by calculating the checksum of the previously stored variables

#include <EEPROM.h>

#define ID_ADDR 0 //byte
// LDR VARS
#define M_ADDR 1   // Float
#define B_ADDR 5// Float
// R2 MODELATION
#define R2_BASE_ADDR 9// Float
#define R2_EXP_ADDR 13// Float
// TAU MODELATION
#define TAU_A_ADDR 17// Float
#define TAU_B_ADDR 21// Float
#define TAU0_ADDR 25// Float
// PI CONTROLLER
#define KP_ADDR 29
#define KI_ADDR 33

#define CHECKSUM_ADDR 37 //Float 


// ARDUINO/LDR VARIABLES
byte ID; //byte
float M;   // Float
float B;// Float
float R2_BASE;// Float
float R2_EXP;// Float
float TAU_A;// Float
float TAU_B;// Float
float TAU0;// Float
float KP;
float KI;
float CHECKSUM; //Float 
float cmp_checksum; // checksum calculated from stored vars;
void setup() {
  Serial.begin(9600);
  
  EEPROM.get(ID_ADDR, ID);
  EEPROM.get(M_ADDR, M);
  EEPROM.get(B_ADDR, B);
  EEPROM.get(R2_BASE_ADDR, R2_BASE);
  EEPROM.get(R2_EXP_ADDR, R2_EXP);
  EEPROM.get(TAU_A_ADDR, TAU_A);
  EEPROM.get(TAU_B_ADDR, TAU_B);
  EEPROM.get(TAU0_ADDR, TAU0);
  EEPROM.get(KP_ADDR, KP);
  EEPROM.get(KI_ADDR, KI);
  EEPROM.get(CHECKSUM_ADDR, CHECKSUM);
  
  cmp_checksum = (float)ID + M + B + R2_BASE + R2_EXP + TAU_A + TAU_B + TAU0 + KP + KI;
  if (cmp_checksum != CHECKSUM){
    Serial.println("DATA IS CORRUPTED!");
    }

  else{
    Serial.print("DATA IS UNCORRUPTED!");
    Serial.print("My ID is ");
    Serial.print(ID);
    }
    delay(100);
    exit(0);
}

void loop() {
  // put your main code here, to run repeatedly:

}

// DESCRIPTION: Writes to the EEPROM the desired values. To add values a new address memory must be declares having ina ccount variable size (bytes)

#include <EEPROM.h>

// --- EEPROM VARIABLE ADDRESSES ---
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

// --- VARIABLE DECLARATION ---
// ARDUINO/LDR VARIABLES
byte ID = 1; //byte
// LDR VARS
float M = 1.2;   // Float
float B = 500.123;// Float
// R2 MODELATION
float R2_BASE = 0.09;// Float
float R2_EXP = 13.0001;// Float
// TAU MODELATION
float TAU_A = 170.0;// Float
float TAU_B = 2100.12;// Float
float TAU0 = 2;
// PI CONTROLLER
float KP = 0.1;
float KI = 10;

float CHECKSUM =0; //Float 



uint8_t id;
float f;

int addr =0;
void setup() {
  
  // RUN ONLY ONCE OR WHEN DATA IS CORRUPTED
  EEPROM.put(ID_ADDR, ID);
  EEPROM.put(M_ADDR, M);
  EEPROM.put(B_ADDR, B);
  EEPROM.put(R2_BASE_ADDR, R2_BASE);
  EEPROM.put(R2_EXP_ADDR, R2_EXP);
  EEPROM.put(TAU_A_ADDR, TAU_A);
  EEPROM.put(TAU_B_ADDR, TAU_B);
  EEPROM.put(TAU0_ADDR, TAU0);
  EEPROM.put(KP_ADDR, KP);
  EEPROM.put(KI_ADDR, KI);
  

  CHECKSUM = (float)ID + M + B + R2_BASE + R2_EXP + TAU_A + TAU_B + TAU0 + KP + KI;
  EEPROM.put(CHECKSUM_ADDR, CHECKSUM);
  
Serial.begin(9600);
  delay(1000); 
}

void loop() {
  Serial.println("DONE WRITING TO EEPROM!");
  delay(100);
  exit(0);
}

#ifndef EEPROM_data_H_
#define EEPROM_data_H_

#include <Arduino.h>

// --- ARDUINO/LDR VARIABLES ADDRESSES ---
#define ID_ADDR 0 //byte
// LDR VARS
#define M_ADDR 1 // Float
#define B_ADDR 5 // Float
// R2 MODELATION
#define R2_BASE_ADDR 9 // Float
#define R2_EXP_ADDR 13 // Float
// TAU MODELATION
#define TAU_A_ADDR 17 // Float
#define TAU_B_ADDR 21 // Float
#define TAU0_ADDR 25  // Float
// PI CONTROLLER
#define KP_ADDR 29
#define KI_ADDR 33

#define CHECKSUM_ADDR 37 //Float

// --- ARDUINO/LDR VARIABLES ---
float CHECKSUM; //Float

#endif
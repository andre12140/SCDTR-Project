#include <Arduino.h>
#include <stdlib.h>
#include <math.h>
#include "system.h"
#include "simulator.h"
#include "control.h"
#include "userInterface.h"
#include "state.h"

System sys;
Simulation simulator;
Control controller;
UserInterface user;
upperLevel desk_occ;
lowLevel desk_free;

void setup() {
 
 Serial.begin(1000000); // initialize serial communications at 1Mbps

  TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz (Timer 2 | 8 bit)
  //Phase correct
  TCCR2A = TCCR2A & B11111100 | B00000001;
  TCCR2B = TCCR2B & B11110111;
  //Normal port operation, OC2A disconnected. COM2A1,COM2A0 = 0
  TCCR2A = TCCR2A & B00111111;
  //Normal port operation, OC2B disconnected. COM2B1, COM2B0 = 0
  TCCR2A = TCCR2A & B11001111;
  //To ensure compatibility with future devices, this bit (FOC2A,FOC2B) must be set to zero when TCCR2B is written when operating in PWM mode.
  TCCR2B = TCCR2B & B00111111;
  

  Serial.println("PWM\tLUX\tExp.Tau\tTh.Tau");
}


void loop() {

  sys.stepGenerator();
  sys.readingsLDR();
/*   Serial.print(millis());
  Serial.print(",   ");
  Serial.print(sys.PWM);
  Serial.print(",   ");
  Serial.println(sys.Lux); */
  sys.tauCalculator();

  //Serial.println("loop");

}
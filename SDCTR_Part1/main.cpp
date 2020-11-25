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

  cli();  // disable interrupts
  TCCR1A = 0;   // clear register
  TCCR1B = 0;   // clear register
  TCNT1 = 0;    // reset counter 
  OCR1A = 19999; // Ensure 100 Hz sampling frequency (Ts = 10 ms)
  TCCR1B |= (1 << WGM12); // CTC On
  TCCR1B |= (1 << CS01); // Set prescaler for 8
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei(); //enable interrupts

  // Calibrates one time only, everytime the system reboots
  sys.calibrationG();
}

ISR(TIMER1_COMPA_vect){
  sys.samp = true;
  // At each sampling time read from LDR
  sys.readingsLDR();
  } 

void loop() {

  // After calibration and at each sampling period apply control 
  if (sys.samp)
  {
    // Detect intructions from user 
    user.newInput();
    // When in free state change the reference to lowRef
    if(desk_free.flag){sys.x_des = desk_free.lowRef; sys.newRef = true;}
    // When in occupied state change the reference to upperRef
    if(desk_occ.flag){sys.x_des = desk_occ.upperRef; sys.newRef = true;}   

    // Send Fixed PWM value to LED
    if(user.setPWM){analogWrite(sys.analogOutPin, sys.PWM);}
    // Control the System
    else
    {
      // Decoupled Feedforward + Feedback Control
      controller.Decoupled_Fb_Ff_Control(simulator.luxSimulator(sys.x_des));
    }
    controller.dataDisplay();
    sys.samp = false;
  }
}




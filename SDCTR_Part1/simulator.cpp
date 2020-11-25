#include <Arduino.h>
#include "system.h"
#include "simulator.h"

extern System sys;

float Simulation::luxSimulator(float _x_des){
  // Tau Experimenta lModel
  //Lux_st = G*PWM; // use in case of stepGenerator
  Lux_st = _x_des;
  if (Lux_st == 0){tauModel_st = 22000;} 
  else {tauModel_st = -3622*log(Lux_st)+19455;}
  R2_st = 65088*pow(Lux_st, -0.702);

  // Record the time, initial voltage, and steady state voltage each step change
  if (sys.newRef) 
  {
    ti = micros();
    vi = vf;
    vf = sys.Vcc*(sys.R1/(sys.R1+R2_st));
    //step = false; // use in case of stepGenerator
    sys.newRef = false; // use in case of control
  }

  // Simulated voltage is given by the solution of the differential equation
  simV = vf - (vf-vi)*(exp(-(float)(micros()-ti)/tauModel_st));
  simR2 = sys.R1*(sys.Vcc/simV - 1);
  simLux = pow(10, (log10(simR2) - sys.b)/sys.m);
  return simLux;
}
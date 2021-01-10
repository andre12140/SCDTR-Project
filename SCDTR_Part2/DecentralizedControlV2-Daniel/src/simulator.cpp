#include <Arduino.h>
#include "system.h"
#include "simulator.h"

extern System sys;

float Simulation::luxSimulator(float _x_des)
{
  // Tau Experimenta lModel
  //Lux_st = sys.G*sys.PWM; // use in case of stepGenerator
  Lux_st = _x_des;
  if (Lux_st == 0)
  {
    tauModel_st = sys.tau_zero;
  }
  else
  {
    tauModel_st = -sys.tau_a * log(Lux_st) + sys.tau_b;
  }

  R2_st = sys.R2_base * pow(Lux_st, sys.R2_exp);

  // Record the time, initial voltage, and steady state voltage each step change
  if (sys.newRef)
  {
    ti = micros();
    vi = vf;
    vf = sys.Vcc * (sys.R1 / (sys.R1 + R2_st));
    //sys.step = false; // use in case of stepGenerator
    sys.newRef = false; // use in case of control
  }

  // Simulated voltage is given by the solution of the differential equation
  simV = vf - (vf - vi) * (exp(-(float)(micros() - ti) / tauModel_st));
  simR2 = sys.R1 * (sys.Vcc / simV - 1);
  simLux = pow(10, (log10(simR2) - sys.b) / sys.m);
  return simLux;
}
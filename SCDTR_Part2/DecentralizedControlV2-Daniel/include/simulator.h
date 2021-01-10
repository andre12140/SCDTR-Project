#ifndef _simulator_
#define _simulator_

class Simulation
{
private:
  float vf = 0;
  float vi = 0;
  float R2_st = 0;
  float Lux_st = 0;
  float tauModel_st = 0;
  float simV = 0;
  float simR2 = 0;

  unsigned long ti = 0;

public:
  float simLux = 0;
  float luxSimulator(float);
};

#endif
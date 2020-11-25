#ifndef _control_ 
#define _control_

class Control
{
private:
  const float T  = 0.01; // Sampling time
  int ui_before = 0;  // Previous integral control
  int ki = 65;  // Integral Gain
  float kp = 1; // Proportional Gain
  float y_pred = 0; // Prediction Obeservation (system simulation)

public:
  int uff = 0;  // Feedforward control term
  int ufb = 0;  // Feedback control term
  int ui = 0;   // Integral control term
  int up = 0;   // Proportional control term
  int u = 0;    // Control 
  float e = 0;  // Real error
  float e_sim = 0;  // Simulation error

  void feedbackControl();
  void feedforwardControl();
  void Decoupled_Fb_Ff_Control(float);
  void dataDisplay();
};

#endif 
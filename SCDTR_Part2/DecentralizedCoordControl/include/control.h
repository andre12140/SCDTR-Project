#ifndef _control_
#define _control_

class Control
{
private:
public:
  const float T = 0.01; // Sampling time
  int ui_before = 0;    // Previous integral control
  float y_pred = 0;     // Prediction Obeservation (system simulation)
  float ki = 200;       // Integral Gain
  float kp = 0;         // Proportional Gain
  int uff = 0;          // Feedforward control term
  int ufb = 0;          // Feedback control term
  int ui = 0;           // Integral control term
  int up = 0;           // Proportional control term
  int u = 0;            // Control
  float e = 0;          // Real error
  float e_sim = 0;      // Simulation error
  int *uff_PWM;         // All feedforward control intents

  void feedbackControl();
  void feedforwardControl();
  void Decoupled_Fb_Ff_Control(float);
  void decentralizedCoordControl();
  void dataDisplay();
};

#endif
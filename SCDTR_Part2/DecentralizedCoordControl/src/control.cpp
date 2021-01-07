#include <Arduino.h>
#include "system.h"
#include "control.h"
#include "state.h"

extern System sys;
extern upperLevel desk_occ;
extern lowLevel desk_free;

// Stand alone Feedforward Control
void Control::feedforwardControl()
{
  uff = (int)sys.x_des / sys.k[0];
  analogWrite(sys.analogOutPin, uff);
}

// Stand alone Feedback Control
void Control::feedbackControl()
{

  // Error to reference
  e = sys.x_des - sys.Lux;
  // Integral Part
  ui = ui_before + T * ki * e;
  // Proportional Part
  up = kp * e;
  // Control signal
  u = up + ui;
  // Saturate Integral Term
  if (u > 255)
  {
    ui = 255 - up;
    u = up + ui;
  }
  if (u < 0)
  {
    ui = 0 - up;
    u = up + ui;
  }
  // Send control signal to LED
  analogWrite(sys.analogOutPin, u);
  // Dead Zone
  if (!((e < 1.00) && (e > -1.00)))
  {
    ui_before = ui;
  }
}

// Decoupled Feedback plus Feedforward Control
void Control::Decoupled_Fb_Ff_Control(float _simLux)
{

  // Feedforward drives the system to the set point
  uff = (int)sys.x_des / sys.k[0];
  // Feedback attenuates disturbances and noise
  // error from simulation
  e_sim = _simLux - sys.Lux;
  // error to reference
  e = sys.x_des - sys.Lux;
  // Integral Part
  ui = ui_before + T * ki * e_sim;
  // Proportional Part
  up = kp * e_sim;
  // Control signal
  u = up + ui + uff;
  // Saturate Integral Term
  if (u > 255)
  {
    ui = 255 - uff - up;
    u = up + ui + uff;
  }
  if (u < 0)
  {
    ui = 0 - uff - up;
    u = up + ui + uff;
  }
  // Send control signal to LED
  analogWrite(sys.analogOutPin, u);
  // Dead Zone
  if (!((e < 1.00) && (e > -1.00)))
  {
    ui_before = ui;
  }
}

// Decentralized Coordinated Control
void Control::decentralizedCoordControl()
{
}

// Display control data
void Control::dataDisplay()
{

  Serial.print(millis());
  Serial.print(",     ");
  Serial.print(sys.x_des);
  Serial.print(",     ");
  Serial.print(sys.Lux);
  Serial.print(",     ");
  Serial.print(e);
  Serial.print(",     ");
  Serial.print(e_sim);
  Serial.print(",     ");
  Serial.print(u);
  Serial.print(",     ");
  Serial.print(uff);
  Serial.print(",     ");
  Serial.println(ui);
}
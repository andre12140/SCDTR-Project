#include <Arduino.h>
#include "system.h"
#include "control.h"
#include "state.h"
#include "node.h"
#include "simulator.h"

extern System sys;
extern upperLevel desk_occ;
extern lowLevel desk_free;
extern nodeL nodeList;
extern Simulation simulator;

void Control::initUff()
{
  // Initialize control signal with zero (feedforward)
  uff_PWM = (int *)malloc(sizeof(int) * nodeList.n_nodes);
  if (uff_PWM == NULL)
  {
    Serial.println("Error allocating memory.");
    exit(0);
  }
  memset(uff_PWM, 0, sizeof(int) * nodeList.n_nodes);
}

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
  // Control the system with the Decentralized Coordinated Algorithm with sequential updates
  //decentralizedCoordControl();
  can_frame frame;
  int count = 0;

  // Compute feedforward everytime there is a new reference to attain
  if (sys.newRef)
  {

    // Iterate 25 times
    for (int i = 0; i < 25; i++)
    {
      // Identify the node that is computing the feedforward control intent
      for (int n = 0; n < nodeList.n_nodes; n++)
      {
        if (nodeList.ID == nodeList.node_list[n])
        {

          // Waiting for messages from other nodes with their previous control intent (t-1)
          while (count != nodeList.n_nodes - 1)
          {
            Serial.print(" ");

            if (SPWM_flag)
            {
              SPWM_flag = false;
              count++;

              cli();
              cf_stream.get(frame);
              sei();

              int j = 0;
              // Establish the order to save the previous control intents in vector uff_PWM
              for (j = 0; j < nodeList.n_nodes; j++)
              {
                if (nodeList.node_list[j] == frame.can_id)
                  break;
              }
              memcpy(&(uff_PWM[j]), &(frame.data[2]), sizeof(int));
            }
          }
          count = 0;
          // Start computing the new control intent (t)
          uff_PWM[n] = sys.x_des - sys.o_node;

          for (int w = 0; w < nodeList.n_nodes; w++)
          {
            // Identify itself
            if (w == n)
            {
              continue;
            }
            // Subtracts the influence in terms of lux from the other nodes,
            // according to their control intents in previous time step
            uff_PWM[n] -= uff_PWM[w] * sys.k[w];
          }
          // Finalizes the computation by dividing the attained lux values by its own gain
          uff_PWM[n] = uff_PWM[n] / sys.k[n];
        }
        // To be executed by nodes that are not computing new control intents in the current time step
        else
        {

          // Prepares message with the command, node identifier and control intent
          char msg_sendPWM[4];
          msg_sendPWM[0] = send_PWM;
          msg_sendPWM[1] = nodeList.node_list[n];

          int j = 0;
          // Establish the right order to save control intents
          for (j = 0; j < nodeList.n_nodes; j++)
          {
            if (nodeList.node_list[j] == nodeList.ID)
              break;
          }
          memcpy(&msg_sendPWM[2], &(uff_PWM[j]), sizeof(int));
          comObj.write(nodeList.ID, msg_sendPWM, 4);
        }
      }
    }

    int j = 0;
    // Establish the order to save control intents in vector uff_PWM
    for (j = 0; j < nodeList.n_nodes; j++)
    {
      if (nodeList.node_list[j] == nodeList.ID)
        break;
    }
    // Saving the new feedforward component that accounts the accessible disturbances
    uff = uff_PWM[j];
  }

  // Feedback attenuates disturbances and noise
  // error from simulation
  e_sim = simulator.luxSimulator(sys.x_des) - sys.Lux;
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
  dataDisplay();
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
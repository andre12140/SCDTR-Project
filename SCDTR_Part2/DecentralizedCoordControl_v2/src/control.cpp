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
extern Control controller;

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
  CONTROL = true;
  Serial.println("Start of control routine");
  // Control the system with the Decentralized Coordinated Algorithm with sequential updates
  can_frame frame;
  int count = 0;

  // Compute feedforward everytime there is a new reference to attain
  if (sys.newRef)
  {
    simulator.luxSimulator(sys.x_des);
    // Iterate
    for (int i = 0; i < maxiterCoord; i++)
    {
      Serial.print("Iter: ");
      Serial.println(i);

      // Identify the node that is computing the feedforward control intent
      for (int n = 0; n < nodeList.n_nodes; n++)
      {
        Serial.print(" ");
        if (nodeList.ID == nodeList.node_list[n])
        {
          // Waiting for messages from other nodes with their previous control intent (t-1)
          while (count < (nodeList.n_nodes - 1))
          {
            Serial.print(" ");
            // Ready to receive the messages
            if (SPWM_flag)
            {

              bool has_data;
              cli();
              has_data = cf_stream.get(frame);
              sei();
              while (has_data)
              {
                Serial.println("has data");
                if ((frame.data[CMDm] == send_PWM) && (frame.data[IDm] == (nodeList.ID)))
                {

                  count++;

                  // Establish the order to save the previous control intents in vector uff_PWM
                  int j = nodeList.getNodeIDX(frame.can_id);
                  if (j == -1)
                  {
                    Serial.println("Index error");
                    exit(0);
                  }
                  // Saving in memory (vector uff_PWM) all control intents
                  memcpy(&(uff_PWM[j]), &(frame.data[Dm]), sizeof(int));
                }

                cli();
                has_data = cf_stream.get(frame);
                sei();
              }
              SPWM_flag = false;
            }
          }
          count = 0;
          // Start computing the new control intent (t)
          uff_PWM[n] = sys.x_des - sys.o_node;
          Serial.print("L-o = ");
          Serial.println(uff_PWM[n]);

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
            Serial.print("from node: ");
            Serial.println(w);
            Serial.println(uff_PWM[w]);
            Serial.print("Accum: ");
            Serial.println(uff_PWM[n]);
          }
          // Finalizes the computation by dividing the attained lux values by its own gain
          uff_PWM[n] = uff_PWM[n] / sys.k[n];

          Serial.print("Self control: ");
          Serial.println(uff_PWM[n]);
        }
        // To be executed by nodes that are not computing new control intents in the current time step
        else
        {
          Serial.println("Nodes outside");
          // Prepares message with the command, node identifier and control intent
          msg_sendPWM[CMDm] = send_PWM;
          msg_sendPWM[IDm] = nodeList.node_list[n];

          // Establish the right order to save control intents
          int j = nodeList.getNodeIDX(nodeList.ID);
          if (j == -1)
          {
            Serial.println("Index error");
            exit(0);
          }
          memcpy(&msg_sendPWM[Dm], &(uff_PWM[j]), sizeof(int));
          comObj.write(nodeList.ID, msg_sendPWM, 4);
        }
        delay(1);
      }
    }

    CONTROL = false;

    // Establish the order to save control intents in vector uff_PWM
    int j = nodeList.getNodeIDX(nodeList.ID);
    if (j == -1)
    {
      Serial.println("Index error");
      exit(0);
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

float Control::l2_norm(float *u, int n)
{
  float accum = 0.;
  for (int i = 0; i < n; ++i)
  {
    accum += u[i] * u[i];
  }
  return sqrt(accum);
}

float Control::evaluate_cost(float *c_node, float *y, float *d_avg, float *d, float rho)
{
  // gi = ci*di + yi*(di - di_avg) + (rho/2)*square_norm(di - di_avg)
  float accum = 0;
  float *aux = (float *)malloc(sizeof(float) * nodeList.n_nodes);

  if (aux == NULL)
  {
    Serial.println("Error allocating memory.");
    exit(0);
  }

  for (int i = 0; i < nodeList.n_nodes; i++)
  {
    accum += c_node[i] * d[i] + y[i] * (d[i] - d_avg[i]);
    aux[i] = d[i] - d_avg[i];
  }
  return accum + rho / 2 * pow(l2_norm((aux), 3), 2);
}

// bool Control::check_feasibility(float *d, float *k)
// {
//   bool check = true;
//   float tol = 0.001; // tolerance for rounding errors
//   float *lum = (float *)malloc(sizeof(float) * nodeList.n_nodes);
//   int idx = nodeList.getNodeIDX(nodeList.ID);

//   if (lum == NULL)
//   {
//     Serial.println("Error allocating memory.");
//     exit(0);
//   }
//   memset(lum, 0, sizeof(float) * nodeList.n_nodes);

//   // compute the illuminance for the luminaires
//   for (int i = 0; i < nodeList.n_nodes; i++)
//   {
//     lum[i] = d[i] * k[i]; // ORDEM CERTA??????
//   }
//   // violates minimum PWM value
//   if (d[idx] < 0 - tol)
//   {
//     check = false;
//   }
//   // violates maximum PWM value
//   if (d[idx] > 255 + tol)
//   {
//     check = false;
//   }
//   // violates the local illuminance lower bound
//   for (int i = 0; i < nodeList.n_nodes; i++)
//   {
//     if (lum[i] < sys.x_des - sys.o_node - tol)
//     {
//       check = false;
//     }
//   }
//   // Passed the feasibility check

//   return check;
// }

/* float *Control::consensus_iterate(float *c_node, float *d_avg, float *y, float aux_m, float rho, float o_, float L_, float *k, float k_norm)
{

  float *d_best, *z, *d_u, *d_bl, *d_b0, *d_b255, *d_l0, *d_l255, *d = (float *)malloc(sizeof(float) * nodeList.n_nodes);
  float *result = (float *)malloc(sizeof(float) * (nodeList.n_nodes + 1));
  int idx = nodeList.getNodeIDX(nodeList.ID);

  if (d_best == NULL)
  {
    Serial.println("Error allocating memory.");
    exit(0);
  }
  memset(d_best, -1, sizeof(float) * nodeList.n_nodes);
  memset(z, 0, sizeof(float) * nodeList.n_nodes);
  // unconstrained minimum
  memset(d_u, 0, sizeof(float) * nodeList.n_nodes);
  // minimum constrained to linear boundary
  memset(d_bl, 0, sizeof(float) * nodeList.n_nodes);
  // minimum constrained to 0 boundary
  memset(d_b0, 0, sizeof(float) * nodeList.n_nodes);
  // minimum constrained to 255 boundary
  memset(d_b255, 0, sizeof(float) * nodeList.n_nodes);
  // minimum constrained to linear and 0 boundary
  memset(d_l0, 0, sizeof(float) * nodeList.n_nodes);
  // minimum constrained to linear and 255 boundary
  memset(d_l255, 0, sizeof(float) * nodeList.n_nodes);
  memset(d, 0, sizeof(float) * nodeList.n_nodes);
  // result
  memset(result, 0, sizeof(float) * (nodeList.n_nodes + 1));

  // large number
  int cost_best = 1000000;

  bool sol_unconstrained = true;
  bool sol_boundary_linear = true;
  bool sol_boundary_0 = true;
  bool sol_boundary_255 = true;
  bool sol_linear_0 = true;
  bool sol_linear_255 = true;

  float cost_unconstrained = 0;
  float cost_boundary_linear = 0;
  float cost_boundary_0 = 0;
  float cost_boundary_255 = 0;
  float cost_linear_0 = 0;
  float cost_linear_255 = 0;
  float cost = 0;

  for (int i = 0; i < 3; i++)
  {
    // Verify ci cost
    z[i] = rho * d_avg[i] - c_node[i] - y[i];
  }

  // Solution in the interior

  // unconstrained minimum
  for (int i = 0; i < 3; i++)
  {
    d_u[i] = (1 / rho) * z[i];
  }

  sol_unconstrained = check_feasibility(d_u, k);
  if (sol_unconstrained)
  {
    // REVISE: IF UNCONSTRAINED SOLUTION EXISTS, THEN IT IS OPTIMAL
    // NO NEED TO COMPUTE THE OTHER
    cost_unconstrained = evaluate_cost(c_node, y, d_avg, d_u, rho);
    if (cost_unconstrained < cost_best)
    {
      for (int i = 0; i < 3; i++)
      {
        d_best[i] = d_u[i];
      }
      cost_best = cost_unconstrained;
    }
  }

  // compute minimum constrained to linear boundary
  for (int i = 0; i < 3; i++)
  {
    d_bl[i] = (1 / rho) * z[i] - k[i] / k_norm * (o_ - L_ + (1 / rho) * z[i] * k[i]);
  }
  // check feasibility of minimum constrained to linear boundary
  sol_boundary_linear = check_feasibility(d_bl, k);
  // compute cost and if best store new optimum
  if (sol_boundary_linear)
  {
    cost_boundary_linear = evaluate_cost(c_node, y, d_avg, d_bl, rho);
    if (cost_boundary_linear < cost_best)
    {
      for (int i = 0; i < 3; i++)
      {
        d_best[i] = d_bl[i];
      }
      cost_best = cost_boundary_linear;
    }
  }

  // compute minimum constrained to 0 boundary
  for (int i = 0; i < 3; i++)
  {
    if (i == idx)
    {
      d_b0[i] = 0;
    }
    else
    {
      d_b0[i] = (1 / rho) * z[i];
    }
  }

  // check feasibility of minimum constrained to 0 boundary
  sol_boundary_0 = check_feasibility(d_b0, k);
  // compute cost and if best store new optimum
  if (sol_boundary_0)
  {
    cost_boundary_0 = evaluate_cost(c_node, y, d_avg, d_b0, rho);
    if (cost_boundary_0 < cost_best)
    {
      for (int i = 0; i < 3; i++)
      {
        d_best[i] = d_b0[i];
      }
      cost_best = cost_boundary_0;
    }
  }

  // compute minimum constrained to 255 boundary
  for (int i = 0; i < 3; i++)
  {
    if (i == idx)
    {
      d_b255[i] = 255;
    }
    else
    {
      d_b255[i] = (1 / rho) * z[i];
    }
  }

  // check feasibility of minimum constrained to 255 boundary
  sol_boundary_255 = check_feasibility(d_b255, k);
  // compute cost and if best store new optimum
  if (sol_boundary_255)
  {
    cost_boundary_255 = evaluate_cost(c_node, y, d_avg, d_b255, rho);
    if (cost_boundary_255 < cost_best)
    {
      for (int i = 0; i < 3; i++)
      {
        d_best[i] = d_b255[i];
      }
      cost_best = cost_boundary_255;
    }
  }

  // compute minimum constrained to linear and 0 boundary
  for (int i = 0; i < 3; i++)
  {
    if (i == idx)
    {
      d_l0[i] = 0;
    }
    else
    {
      d_l0[i] = (1 / rho) * z[i] - (1 / aux_m) * k[i] * (o_ - L_) + (1 / rho / aux_m) * k[i] * (k[idx] * z[idx] - z[i] * k[i]);
    }
  }

  // check feasibility of minimum constrained to linear and 0 boundary
  sol_linear_0 = check_feasibility(d_l0, k);
  //  compute cost and if best store new optimum
  if (sol_linear_0)
  {
    cost_linear_0 = evaluate_cost(c_node, y, d_avg, d_l0, rho);
    if (cost_linear_0 < cost_best)
    {
      for (int i = 0; i < 3; i++)
      {
        d_best[i] = d_l0[i];
      }
      cost_best = cost_linear_0;
    }
  }

  // compute minimum constrained to linear and 255 boundary
  for (int i = 0; i < 3; i++)
  {
    if (i == idx)
    {
      d_l255[i] = 255;
    }
    else
    {
      d_l255[i] = (1 / rho) * z[i] - (1 / aux_m) * k[i] * (o_ - L_ + 255 * k[idx]) + (1 / rho / aux_m) * k[i] * (k[idx] * z[idx] - z[i] * k[i]);
    }
  }

  // check feasibility of minimum constrained to linear and 255 boundary
  sol_linear_0 = check_feasibility(d_l255, k);
  // compute cost and if best store new optimum
  if (sol_linear_0)
  {
    cost_linear_255 = evaluate_cost(c_node, y, d_avg, d_l255, rho);
    if (cost_linear_255 < cost_best)
    {
      for (int i = 0; i < 3; i++)
      {
        d_best[i] = d_l255[i];
      }
      cost_best = cost_linear_255;
    }
  }
  for (int i = 0; i < 3; i++)
  {
    d[i] = d_best[i];
  }
  cost = cost_best;

  for (int i = 0; i < 3; i++)
  {
    result[i] = d[i];
  }
  result[4] = cost;

  return result;
}
 */
// float *distributedOptimizationControl(float rho, int maxiter)
// {
//   float *result;

//   // Initial condition (iteration = 1)
//   /*     d11(1) = node1.d(1);
//     d12(1) = node1.d(2);
//     d21(1) = node2.d(1);
//     d22(1) = node2.d(2);
//     av1(1) = (d11(1)+d21(1))/2;
//     av2(1) = (d12(1)+d22(1))/2; */

//   // iterations
//   for (int i = 1; i <= maxiter; i++)
//   {
//     // COMPUTATION OF THE PRIMAL SOLUTIONS
//     result = controller.consensus_iterate(sys.c_node, sys.d_avg, sys.y, sys.aux_m, rho, sys.o_node, sys.x_des, sys.k, sys.k_norm);

//     for (int j = 0; j < 3; j++)
//     {

//       sys.d[j] = result[j];
//     }

//     // Nodes exchange their solutions
//     //

//     for (int j = 0; j < 3; j++)
//     {
//       // COMPUTATION OF THE AVERAGE
//       sys.d_avg[j] = (sys.d[j] + sys.d[j] + sys.d[j]) / 3;

//       // COMPUTATION OF THE LAGRANGIAN UPDATES
//       sys.y[j] = sys.y[j] + rho * (sys.d[j] - sys.d_avg[j]);

//       // SAVING DATA FOR PLOTS
//       /*     d11(i) = node1.d(1);
//             d12(i) = node1.d(2);
//             d21(i) = node2.d(1);
//             d22(i) = node2.d(2);
//             av1(i) = (d11(i)+d21(i))/2;
//             av2(i) = (d12(i)+d22(i))/2; */
//     }
//   }
//   //return d_avg;
// }

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
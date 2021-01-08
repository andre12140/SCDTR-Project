#include "system.h"
#include "canFrameStream.h"
#include "node.h"
#include "can_com.h"

extern nodeL nodeList;
extern Comunication comObj;

void System::readingsLDR()
{
  sensorValue = analogRead(analogInPin); // read the analog in value [0,1023]
  V = (float)(sensorValue * Vcc) / 1023; // Voltage [0,5]
  R2 = R1 * (Vcc / V - 1);               // R2 (voltage divider)
  Lux = pow(10, (log10(R2) - b) / m);
}

void System::initGainMatrix()
{
  k = (float *)malloc(sizeof(float) * nodeList.n_nodes);
  if (k == NULL)
  {
    Serial.println("Error allocating memory.");
    exit(0);
  }
  memset(k, 0, sizeof(float) * nodeList.n_nodes);
}

void System::get_o()
{
  // External disturbances when all LED's are turned off
  o_node = Lux;
  delay(500);
}

void System::calibration()
{
  CALIB = true;

  can_frame frame;
  int i = 0;
  int count = 0;
  initGainMatrix();
  nodeList.sortNodes();
  get_o();

  // Starts calibrating sequentially from list of available nodes
  for (i = 0; i < nodeList.n_nodes; i++)
  {
    /*     Serial.print("node list: ");
    Serial.println(nodeList.node_list[i]); */
    if (nodeList.ID == nodeList.node_list[i])
    {
      Serial.print("D Started calibrating node ");
      Serial.println(nodeList.ID);

      // Set LED to its maximum power to compute gains
      analogWrite(analogOutPin, 255);
      // Awaits stabilization of lux response
      delay(1000);
      // Computes and saves its gain
      k[i] = (Lux - o_node) / 255;
      // Informs other nodes to compute the coupling gains
      msgCCG[CMDm] = compute_CG;
      comObj.write(nodeList.ID, msgCCG, 1);
      // Collects the computed coupling gains from available nodes
      while (count < (nodeList.n_nodes - 1))
      {
        Serial.print(" ");
        // Ready to receive the messages
        if (SCG_flag)
        {
          /* Serial.println("SCG_flag"); */
          bool has_data;
          cli();
          has_data = cf_stream.get(frame);
          sei();
          while (has_data)
          {
            /*             for (int i = 0; i < 2; i++)
            {
              Serial.println("Msg received: ");
              Serial.println(frame.data[i], HEX);
            } */
            // Ensures the message is meant to be read
            if ((frame.data[CMDm] == send_CG) && (frame.data[IDm] == (nodeList.ID)))
            {
              /* Serial.println("has_data"); */
              count++;
              int j = nodeList.getNodeIDX(frame.can_id);
              Serial.println(j);
              if (j == -1)
              {
                Serial.println("Index error");
                exit(0);
              }
              // Saving in memory (vector k) the coupling gains
              memcpy(&k[j], &(frame.data[Dm]), sizeof(float));
            }

            cli();
            has_data = cf_stream.get(frame);
            sei();
          }
          SCG_flag = false;
        }
      }
      /* Serial.println("After while"); */
      analogWrite(analogOutPin, 0);
    }
    else
    {

      // If the node is not calibrating it always have LED turned off
      analogWrite(analogOutPin, 0);

      // Waits signaling to compute coupling gains
      while (!CCG_flag)
      {
        Serial.print(" ");
      }
      CCG_flag = false;

      cli();
      cf_stream.get(frame);
      sei();

      // Computation of coupling gain
      float cg = ((Lux - o_node) / 255) + 0.01;

      // Sends coupling gain info to calibrating node

      msg_sendCCG[CMDm] = send_CG;
      msg_sendCCG[IDm] = nodeList.node_list[i];
      memcpy(&msg_sendCCG[Dm], &(cg), sizeof(float));

      /*       for (int i = 0; i < 2; i++)
      {
        Serial.print("Node list: ");
        Serial.println(nodeList.node_list[i]);
        Serial.print(" Msg sent: ");
        Serial.println(msg_sendCCG[i], HEX);
      } */
      comObj.write(nodeList.ID, msg_sendCCG, 6);
    }
  }
  CALIB = false;
}

void System::configTimers()
{
  TCCR2B = (TCCR2B & B11111000) | B00000001; // for PWM frequency of 31372.55 Hz (Timer 2 | 8 bit)
  //Phase correct
  TCCR2A = (TCCR2A & B11111100) | B00000001;
  TCCR2B = TCCR2B & B11110111;
  //Normal port operation, OC2A disconnected. COM2A1,COM2A0 = 0
  TCCR2A = TCCR2A & B00111111;
  //Normal port operation, OC2B disconnected. COM2B1, COM2B0 = 0
  TCCR2A = TCCR2A & B11001111;
  //To ensure compatibility with future devices, this bit (FOC2A,FOC2B) must be set to zero when TCCR2B is written when operating in PWM mode.
  TCCR2B = TCCR2B & B00111111;

  cli();                   // disable interrupts
  TCCR1A = 0;              // clear register
  TCCR1B = 0;              // clear register
  TCNT1 = 0;               // reset counter
  OCR1A = 19999;           // Ensure 100 Hz sampling frequency (Ts = 10 ms)
  TCCR1B |= (1 << WGM12);  // CTC On
  TCCR1B |= (1 << CS01);   // Set prescaler for 8
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();                   //enable interrupts
}

// void System::stepGenerator()
// {

//   // Generating steps with incrementing PWM values 10 at a time
//   if (((millis() - curr_time) > 1000))
//   {
//     if (PWM == flag) // Anchoring the extreme values (0 or 255)
//     {
//       if (count < 128) // 1st threshold (half the scale)
//       {
//         PWM = count;
//         count += 10;
//         step = true;
//       }
//       else if (count == 130)
//       {
//         PWM = 0;
//         count += 10;
//         flag = 0;
//       }
//       else
//       {
//         flag = 0;
//         if (count < 265) // 2nd treshold (top of the scale) -> change anchor
//         {
//           PWM = count - 10;
//           count += 10;
//           step = true;
//         }
//         else
//         {
//           count = 0;
//           flag = 255;
//         } // reseting count and anchor
//       }
//     }
//     else
//     {
//       PWM = flag;
//       step = true;
//     }
//     // Update current time
//     curr_time = millis();
//   }
//   // Send PWM value to LED
//   analogWrite(analogOutPin, PWM);
// }

// void System::tauCalculator()
// {

//   if (stop == 1)
//   {

//     // Add new node to the list (Voltage, Time of acquisition)
//     list = addNode(list, V, micros());

//     if (list->next != NULL)
//     {
//       // Evaluate Steady State
//       if (abs(float((list->v - list->next->v))) <= 0.01)
//       {
//         count_st += 1;
//       }
//     }

//     // Steady state is reached
//     if (count_st == 30)
//     {

//       // Experimental tau
//       tau = searchClosestNode(list, (LastNode(list))->v, list->v);
//       // Freeing the allocated memory
//       list = freeList(list);
//       // Theoretical tau
//       theoreticalTau();
//       // Reseting the  counter
//       count_st = 0;
//       stop = 0;
//     }
//   }
// }

// void System::theoreticalTau()
// {
//   Req = (R1 * R2) / (R1 + R2);
//   theoTau = (Req * C1) * pow(10, 3); // milli
// }

// unsigned long System::searchClosestNode(System::v_node *list, float vMax, float vMin)
// {

//   System::v_node *temp = NULL;
//   unsigned long tau = 0, ti = (LastNode(list))->t;

//   if (flag == 0)
//   {
//     vTau = 0.63 * (vMax - vMin) + vMin;
//   } // Rising step
//   if (flag == 255)
//   {
//     vTau = 0.37 * (vMax - vMin) + vMin;
//   } // Falling step

//   temp = list;

//   while (temp->next != NULL)
//   {
//     if (abs(float(vTau - temp->v)) > abs(float(vTau - temp->next->v)))
//     {
//       tau = (temp->next->t - ti);
//     }

//     temp = temp->next;
//   }

//   return tau;
// }

// // Dinamically Allocate Memory for each node
// System::v_node *System::allocateNode(float volt, unsigned long time)
// {
//   System::v_node *temp = NULL;
//   temp = (System::v_node *)malloc(sizeof(System::v_node));
//   if (temp != NULL)
//   {
//     temp->v = volt;
//     temp->t = time;
//     temp->next = NULL;
//   }
//   return temp;
// }

// // Freeing the entire list from memory
// System::v_node *System::freeList(System::v_node *list)
// {

//   System::v_node *temp = NULL;

//   while (list != NULL)
//   {
//     temp = list;
//     list = list->next;
//     free(temp);
//   }
//   return NULL;
// }

// // Adding new node to list
// System::v_node *System::addNode(System::v_node *list, float v, unsigned long t)
// {

//   System::v_node *temp = NULL;

//   temp = allocateNode(v, t);
//   if (temp != NULL)
//   {
//     // Add head
//     if (list == NULL)
//     {
//       list = temp;
//       // Add to the list
//     }
//     else
//     {
//       temp->next = list;
//       list = temp;
//     }
//   }
//   return list;
// }

// // Gain acess to the last node of the list
// System::v_node *System::LastNode(System::v_node *list)
// {
//   System::v_node *temp = list;
//   while (temp->next != NULL)
//   {
//     temp = temp->next;
//   }
//   return temp;
// }

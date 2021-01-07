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

void System::get_o()
{
  // External disturbances when all LED's are turned off
  o_node = Lux;
  delay(500);
}

void System::calibration()
{
  char msgCCG[1];
  can_frame frame;
  int i = 0;
  int count = 0;

  nodeList.sortNodes();
  get_o();

  // Starts calibrating sequentially from list of available nodes
  for (i = 0; i < nodeList.n_nodes; i++)
  {
    if (nodeList.ID == nodeList.node_list[i])
    {

      Serial.print("Started Calib with node ");
      Serial.print(nodeList.ID);

      // Set LED to its maximum power to compute gains
      analogWrite(analogOutPin, 255);
      // Awaits stabilization of lux response
      delay(1000);
      // Computes its gain
      k[i] = (Lux - o_node) / 255;

      // Informs other nodes to compute the coupling gains
      msgCCG[0] = compute_CG;
      comObj.write(nodeList.ID, msgCCG, 1);

      // Collects the computed coupling gains from available nodes
      while (count != nodeList.n_nodes)
      {
        Serial.print(" ");
        // Ready to receive the messages
        if (SCG_flag)
        {
          count++;

          cli();
          cf_stream.get(frame);
          sei();

          int n = 0;
          // Establish the order to save coupling gains in vector k
          for (n = 0; n < nodeList.n_nodes; n++)
          {
            if (nodeList.node_list[n] == frame.can_id)
              break;
          }
          // Saving in memory the coupling gains
          memcpy(&k[n], &(frame.data[1]), sizeof(float));
        }
      }
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

      cli();
      cf_stream.get(frame);
      sei();

      // Compuation of coupling gain
      k[i] = (Lux - o_node) / 255;

      // Sends coupling gain info to calibrating node
      char msg_sendCCG[5];
      msg_sendCCG[0] = send_CG;
      memcpy(&msg_sendCCG[1], &(k[i]), sizeof(float));
      comObj.write(nodeList.ID, msg_sendCCG, 5);
    }
  }
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

void System::stepGenerator()
{

  // Generating steps with incrementing PWM values 10 at a time
  if (((millis() - curr_time) > 1000))
  {
    if (PWM == flag) // Anchoring the extreme values (0 or 255)
    {
      if (count < 128) // 1st threshold (half the scale)
      {
        PWM = count;
        count += 10;
        step = true;
      }
      else if (count == 130)
      {
        PWM = 0;
        count += 10;
        flag = 0;
      }
      else
      {
        flag = 0;
        if (count < 265) // 2nd treshold (top of the scale) -> change anchor
        {
          PWM = count - 10;
          count += 10;
          step = true;
        }
        else
        {
          count = 0;
          flag = 255;
        } // reseting count and anchor
      }
    }
    else
    {
      PWM = flag;
      step = true;
    }
    // Update current time
    curr_time = millis();
  }
  // Send PWM value to LED
  analogWrite(analogOutPin, PWM);
}

void System::tauCalculator()
{

  if (stop == 1)
  {

    // Add new node to the list (Voltage, Time of acquisition)
    list = addNode(list, V, micros());

    if (list->next != NULL)
    {
      // Evaluate Steady State
      if (abs(float((list->v - list->next->v))) <= 0.01)
      {
        count_st += 1;
      }
    }

    // Steady state is reached
    if (count_st == 30)
    {

      // Experimental tau
      tau = searchClosestNode(list, (LastNode(list))->v, list->v);
      // Freeing the allocated memory
      list = freeList(list);
      // Theoretical tau
      theoreticalTau();
      // Reseting the  counter
      count_st = 0;
      stop = 0;
    }
  }
}

void System::thetaCalculator()
{

  // New step, store ti
  if (stepChange == 1)
  {
    tiTheta = micros();
    vAux = V;
    stepChange = 0;
  }

  // Dead Time
  if ((abs(float((V - vAux))) != 0) && (tiTheta != 0))
  {
    theta = micros() - tiTheta;
    tiTheta = 0;
    stop = 0;
  }
  else
  {
    vAux = V;
  }
}

void System::theoreticalTau()
{
  Req = (R1 * R2) / (R1 + R2);
  theoTau = (Req * C1) * pow(10, 3); // milli
}

unsigned long System::searchClosestNode(System::v_node *list, float vMax, float vMin)
{

  System::v_node *temp = NULL;
  unsigned long tau = 0, ti = (LastNode(list))->t;

  if (flag == 0)
  {
    vTau = 0.63 * (vMax - vMin) + vMin;
  } // Rising step
  if (flag == 255)
  {
    vTau = 0.37 * (vMax - vMin) + vMin;
  } // Falling step

  temp = list;

  while (temp->next != NULL)
  {
    if (abs(float(vTau - temp->v)) > abs(float(vTau - temp->next->v)))
    {
      tau = (temp->next->t - ti);
    }

    temp = temp->next;
  }

  return tau;
}

// Dinamically Allocate Memory for each node
System::v_node *System::allocateNode(float volt, unsigned long time)
{
  System::v_node *temp = NULL;
  temp = (System::v_node *)malloc(sizeof(System::v_node));
  if (temp != NULL)
  {
    temp->v = volt;
    temp->t = time;
    temp->next = NULL;
  }
  return temp;
}

// Freeing the entire list from memory
System::v_node *System::freeList(System::v_node *list)
{

  System::v_node *temp = NULL;

  while (list != NULL)
  {
    temp = list;
    list = list->next;
    free(temp);
  }
  return NULL;
}

// Adding new node to list
System::v_node *System::addNode(System::v_node *list, float v, unsigned long t)
{

  System::v_node *temp = NULL;

  temp = allocateNode(v, t);
  if (temp != NULL)
  {
    // Add head
    if (list == NULL)
    {
      list = temp;
      // Add to the list
    }
    else
    {
      temp->next = list;
      list = temp;
    }
  }
  return list;
}

// Gain acess to the last node of the list
System::v_node *System::LastNode(System::v_node *list)
{
  System::v_node *temp = list;
  while (temp->next != NULL)
  {
    temp = temp->next;
  }
  return temp;
}

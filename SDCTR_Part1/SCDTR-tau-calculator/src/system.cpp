#include <Arduino.h>
#include "system.h"

void System::calibrationG(){
  // Turn LED ON (max value)
  analogWrite(analogOutPin, 255);
  delay(900);
  G = Lux/255;
  // Turn LED OFF
  analogWrite(analogOutPin, 0);
  delay(900);
  newRef = true;
}

void System::readingsLDR(){
  sensorValue = analogRead(analogInPin); // read the analog in value [0,1023]
  V = (float)(sensorValue*Vcc)/1023; // Voltage [0,5]
  R2 = R1*(Vcc/V - 1); // R2 (voltage divider)
  Lux = pow(10, (log10(R2) - b)/m);
}

int prev_count =0;
void System::stepGenerator(){

// Generating steps with incrementing PWM values 10 at a time
  if (((millis() - curr_time) > 1000))
  {    
    //Serial.println("stepGenerator");
    if(PWM == flag) // Anchoring the extreme values (0 or 255)
    { 
/*       if (prev_count != count){
        prev_count = count;
        Serial.print("PWM:");
        Serial.println(count);
      } */
      if(count<128) // 1st threshold (half the scale)
      {
        //Serial.println(count);
        PWM = count;
        count += 10;
        step = true;

      } 
      else if (count == 130)
      {
        PWM = 0;
        count += 10;
        flag = 0;
        //step = true; // VERIF
      }
      else
      {
        flag = 0;
        if (count<265) // 2nd treshold (top of the scale) -> change anchor
        {
          PWM = count - 10;  
          count += 10;
          step = true;
        } 
        else {count = 0; flag = 255;} // reseting count and anchor
      }
    }  
    else {PWM = flag; /* step = true; */}
  // Update current time
  curr_time = millis();
  }
  // Send PWM value to LED
  analogWrite(analogOutPin, PWM);
}

void System::tauCalculator(){
  
  if(step) {

    // Add new node to the list (Voltage, Time of acquisition)
    list = addNode(list, V, millis());

    if (list->next != NULL)
    {
      // Evaluate Steady State
      if (abs(float((list->v - list->next->v))) <= 0.01)
      {
          count_st += 1;
      } 
    }

    // Steady state is reached
    if (count_st == 40){

      // Experimental tau
      tau = searchClosestNode(list,(LastNode(list))->v, list->v);
/*       Serial.print("Exp Tau:");
      Serial.print(tau); */
      
      // Freeing the allocated memory
      list = freeList(list);
      // Theoretical tau
      theoreticalTau();

      Serial.print(PWM);
      Serial.print(",   ");
      Serial.print(Lux);
      Serial.print(",   ");
      Serial.print(tau);
      Serial.print(",   ");
      Serial.println(theoTau);

      // Reseting the  counter
      count_st = 0;
      step = false;
    }
  }
}

void System::thetaCalculator(){

  // New step, store ti
  if(stepChange == 1) 
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
  } else {vAux = V;}
}

void System::theoreticalTau(){
  Req = (R1*R2)/(R1+R2);
  theoTau = (Req*C1)*pow(10,3); // milli 
/*   Serial.print("\tTh Tau:");
  Serial.println(theoTau); */
}

unsigned long System::searchClosestNode(System::v_node *list, float vMax, float vMin){
  
  System::v_node *temp = NULL;
  unsigned long tau = 0, ti = (LastNode(list))->t;

  if (flag == 0){vTau = 0.63*(vMax-vMin) + vMin;} // Rising step
  if (flag == 255){vTau = 0.37*(vMax-vMin) + vMin;} // Falling step

  temp = list;

  while(temp->next != NULL){
    if( abs(float(vTau - temp->v)) > abs(float(vTau - temp->next->v))){
      tau = (temp->next->t - ti); 
    } 

    temp = temp->next;
  }

  return tau;
}

// Dinamically Allocate Memory for each node
System::v_node* System::allocateNode(float volt, unsigned long time){
  System::v_node *temp = NULL;
  temp = (System::v_node*)malloc(sizeof(System::v_node));
  if (temp != NULL) {
    temp->v = volt;
    temp->t = time;
    temp->next = NULL;
  } 
  return temp;
}

// Freeing the entire list from memory
System::v_node* System::freeList(System::v_node *list){

  System::v_node *temp = NULL;

  while(list != NULL){
    temp = list;
    list = list->next;
    free(temp);
  }
  return NULL;
}

// Adding new node to list
System::v_node* System::addNode(System::v_node *list, float v, unsigned long t){
  
  System::v_node *temp = NULL;

  temp = allocateNode(v,t);
  if(temp != NULL){
    // Add head
    if (list == NULL){
      list = temp;
    // Add to the list
    } else {
      temp->next = list;
      list = temp;
    }
  }
  return list;
}

// Gain acess to the last node of the list
System::v_node* System::LastNode(System::v_node *list){
  System::v_node *temp = list;
  while(temp->next != NULL){
    temp = temp->next;
  }
  return temp;
}


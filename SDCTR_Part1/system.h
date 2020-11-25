#ifndef _system_ 
#define _system_

class System
{
private:
  // Flags and counters 
  int flag = 255;
  int count = 0;
  int count_st = 0; // Evaluate steady state
  int stop = 0;
  int stepChange = 0;
  unsigned long curr_time = 0; 
  // Variables for tau calculation  
  float vTau = 0;
  float vAfter = 0;
  float vBefore = 0;         
  float tauModel = 0;
  float tauModel_st = 0;
  float theoTau = 0;  
  unsigned long tau = 0;  
  // Variable for Theta calculation
  float vAux = 0;     
  float Req = 0;
  float C1 = pow(10,-6);
  unsigned long tiTheta = 0;
  unsigned long theta = 0; //80768
  
  // Node definition with voltage and time
  typedef struct _v_node
  {
    float v;
    unsigned long t;
    struct _v_node *next; 
  } v_node;

  // Pointers to create lists with input data
  float *ptr_v = 0;
  unsigned long *ptr_t = 0;
  v_node *list = NULL;

public:
  const int analogInPin = A0; // Analog input pin that the potentiometer is attached to
  const int analogOutPin = 3; // Analog output pin that the LED is attached to
  int sensorValue = 0; // value read from the pot
  int PWM = 0; // value output to the PWM (analog out) [0,255]
  const float m = -0.7022;  // Slope
  const float b = 4.8134;  // Intersection
  const int Vcc = 5;
  const int R1 = 10000; 
  float V = 0; 
  float R2 = 0;
  float Lux = 0; 
  float G = 0;  // Gain 
  int x_des = 0; // Desired State (Lux)

  // Flags and Counters
  bool newRef = false; 
  bool step = false;
  volatile bool samp = false;

  void calibrationG();
  void stepGenerator();
  void readingsLDR();
  void tauCalculator();
  void thetaCalculator();
  void theoreticalTau();
  v_node* allocateNode(float volt, unsigned long time);
  v_node* freeList(v_node *list);
  v_node* addNode(v_node *list, float v, unsigned long t);
  unsigned long searchClosestNode(v_node *list, float vMax, float vMin);
  v_node* LastNode(v_node *list);
};

#endif 
#ifndef _state_ 
#define _state_

// Define state occupied 
class upperLevel
{
public:
  bool flag = false;
  int upperRef = 70;
};

// Define state free (default)
class lowLevel
{
public:
  bool flag = true;
  int lowRef = 20;
};

#endif 
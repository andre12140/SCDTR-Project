#ifndef _state_
#define _state_

// Define state occupied
class upperLevel
{
public:
  bool flag = false;
  int upperRef = 90;
};

// Define state free (default)
class lowLevel
{
public:
  bool flag = true;
  int lowRef = 40;
};

#endif
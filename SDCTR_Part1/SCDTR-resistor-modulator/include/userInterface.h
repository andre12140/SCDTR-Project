#ifndef _userInterface_ 
#define _userInterface_

class UserInterface
{
private:
  String readString;
  bool setLux = false;
  bool setUpper = false;
  bool setLower = false;
  bool setDSKO = false;
  
public:
  bool setPWM = false;
  void newInput();
};

#endif 
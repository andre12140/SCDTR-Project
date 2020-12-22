#include <Arduino.h>
#include "system.h"
#include "userInterface.h"
#include "state.h"

extern System sys;
extern upperLevel desk_occ;
extern lowLevel desk_free;

void UserInterface::newInput(){
  if (Serial.available())  
  {
    char c = Serial.read();  //gets one byte from serial buffer
    if (c == ',') 
    {
      if (readString.length() > 1) 
      {
        // change the desired lux value 
        if(setLux)
        {
          // Turn off occupied or free state
          desk_occ.flag = false;
          desk_free.flag = false;
          // Turn off PWM mode
          setPWM = false;

          sys.x_des = readString.toInt(); 
          setLux = false;
          sys.newRef = true;
        }
        // change the desired PWM value 
        else if (setPWM)
        {
          sys.PWM = readString.toInt();
        }
        // change the upper reference lux value (occupied) 
        else if (setUpper)
        {
          desk_occ.upperRef = readString.toInt();
          setUpper = false;
        }
        // change the lower reference lux value (f) 
        else if (setLower)
        {
          desk_free.lowRef = readString.toInt();
          setLower = false;
        }
        // change the desk occupancy state
        else if (setDSKO)
        {
          if(readString == "FREE"){setPWM = false; desk_occ.flag = false; desk_free.flag = true;}
          if(readString == "OCC"){setPWM = false; desk_free.flag = false; desk_occ.flag = true;}
          setDSKO = false;
        }
        
        // Command that flags a request to change the desired lux value 
        if(readString == "LUX"){setLux = true;} 
        // Command that flags a request to change the PWM value 
        else if (readString == "PWM"){setPWM = true;}
        // Command that flags a request to change the upper reference lux value (occupied) 
        else if (readString == "UPP"){setUpper = true;}
        // Command that flags a request to change the lower reference lux value (free) 
        else if (readString == "LW"){setLower = true;}
        // Command that flags a request to change the desk occupancy state
        else if (readString == "DSKO"){setDSKO = true;}
       
        readString = ""; //clears variable for new input
      }
    } 
    else 
    {     
    readString += c; //makes the string readString
    }
  }
}
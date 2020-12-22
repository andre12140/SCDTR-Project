## SDCTR Project Delivery 1 - Identification, Modelling & Control
## Luísa Nogueira 87060

This folder contains one Excel book, five c++ files and five header files.

## Excel ##
In the Excel book, one can visualize the complete functioning of the system, from the steps taken in identification,
modelling and then in control.
The book contains the following leaves:  
 - Identification: StaticAnalysis, G, R2, Tau.
 - Modelling: simFallingStep, simRisingStep.
 - Control: ControlSpeed, ControlExternalDisturbances, ControlSaturationUmin, ControlSaturationUmax.

## Code ##
# system.h and respective system.cpp
The header file contains the declaration of the class System, including all the variables and functions related to the
system functioning, identification and modelling. Regarding the functions, we have the necessary code to compute tau and theta,
calibrate the system, generate steps that maximize noise to signal ratio and the conversion to lux of the readings from the LDR.

# simulator.h and respective simulator.cpp
The header file contains the declaration of the class Simulation, with the variables regarding the simulation of lux values and the
respective function that performs the calculation. I would also like to refer that the simulator implemented uses a model of
R2 (see Excell leave R2) to compute R2 in steady-state, not needing to make a temporal shift of theta. 

# control.h and respective control.cpp
The header file contains the declaration of the class Control, with the variables and functions regarding the control of the system. Besides, 
it has a function that displays relevant data to the user, e.g. control signals and gains, present state (occupied or free), desired lux value, PWM value
and the real error and the error from the simulation. Regarding control three functions were built, one that performs stand-alone feedforward
control, another that performs stand-alone feedback control and finally the one that combines the two in a decoupled feedforward plus feedback control.

# state.h
This header file declares two classes, upperLevel and lowLevel, that define the state of the system as occupied or free.
The default setting for the system state is free with a desired reference of 20 lux. The occupied state is initiated with a
reference of 70 lux.

# userInterface.h and respective userInterface.cpp
The header file contains the declaration of the class UserInterface, with the variables regarding the user interaction and the respective function that
receives new inputs form the keyboard.
The user has five main command options:
 - "LUX": to define the desired lux reference.
 - "PWM": to send a fixed PWM value to the LDR.
 - "UPP": to change the upper reference lux value in the occupied state.
 - "LW": to change the lower reference lux value in the free state.
 - "DSKO": to change the desk occupancy state, here the user should also type "FREE" or "OCC" to select the state. 
After typing the command, the user needs to press "," as a separator and then the respective numeric value, followed again by the comma. 

# main.cpp
This file contains the core of the program. All classes are initialized, the sampling period is guaranteed through an 
interruption routine (sampling period equals 10 ms), the PWM frequency is changed to 31372.55 Hz trough timer two, and a 
phase-correct is implemented. In the loop() after the calibration and at each sampling time the control is applied. 


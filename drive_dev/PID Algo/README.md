## Electrical - PID Control System

This is a guide for Rover Electrical members (working on the firmware/hardware/testing for either the drive or arm) on PID Control files to be used to control wheel velocity or arm position. 
 
The C code written (pid.c) applies a discrete-time PID control algorithm that takes a system input and gives an output for a given time sample.
 
The parameters (for now) are set_point, Ts (which are to be chosen according to the controlled system's specifications Kp, Ki, Kd, (which are to be tuned by the testing team, a random example is in the main.c file), and errorThreshold (which decides when to apply PD or PID Control, this could also be decided and tuned by the testing team). 
 
The main.c file is just an example on how to call the PID Controller on whatever kind of firware/hardware the controller will be tested on. The void main method starts with parameter parametrization and an infinite while loop (which could be adjusted if the process needs to be stopped), this is where the PID Controller is called. The rest of the file is just automatically generated code by the STM32 IDE for implementation on a NUCLEO-L433RC-P ACTIVE STM32 Nucleo-64 development board; don't worry about it.
 
Notes:
- The intgrlTerm and deriTerm may have to be respectively multiplied and divided by the sampling time Ts!
- You may instantiate the method with outMin and outMax parameters if you want your system output to always be bounded below and above some values. You can see some parameter initialization for it in main.c, and some code for it at the end of pid.c (which is all commented out for now).

For any issues concerning the theory, have a look there: https://embeddedexpert.io/?p=960 (this is part of the resources I used to implement the PID Control files, all the sources are in the file's comments), or send me a message on Teams!
 
Written by: Mohamed-Amine Azzouz

#ifndef arduino_msg_H_
#define arduino_msg_H_

// Arduino Comunication

#define NID 1 // New ID received

// Message Protocol
#define CMDm 0
#define IDm 1
#define Dm 2

// Comand codes - 6 bits
// Command code

#define gl 0x01
#define gd 0x02
#define go 0x03
#define gO 0x04
#define o 0x1A
#define O 0x05
#define gU 0x06
#define U 0x07
#define gL 0x08
#define gx 0x09
#define gr 0x0A
#define gc 0x0B
#define c 0x0C
#define gp 0x0D
#define gpT 0x0E
#define gt 0x0F
#define ge 0x10
#define geT 0x11
#define gv 0x12
#define gvT 0x13
#define gf 0x14
#define gfT 0x15
#define r 0x16

#define b 0x17
#define ss 0x18 //start/stop (toggle)


#endif

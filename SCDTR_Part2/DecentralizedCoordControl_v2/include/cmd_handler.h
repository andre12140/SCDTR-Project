#ifndef CMD_HANDLER_H_
#define CMD_HANDLER_H_

#include <Arduino.h>
#include "arduino_msg.h"
#include "can_com.h"

// class cmdHandler
// {
// public:
//     // FUNCTION AUXILIARY VARIABLES
//     float PJ = 0.05; //maximum power

//     float system_cmd_total = 0; // Sum of values of each node in the network

//     bool check_cmd_ID(uint8_t cmd, uint8_t id_byte, uint8_t n_nodes, uint8_t *node_list, uint8_t ID_)
//     {
//         cmd = cmd & 0x3F; // Clears 1st two bits

//         if (cmd == gpT)
//         {
//             system_cmd_total += 66.55; // [MUDAR] ver diretamente a variável!
//             return cCom.broadcast_cmd(gp, id_byte, n_nodes, node_list, ID_);
//         }

//         else if (cmd == geT)
//         {
//             system_cmd_total += 44.55; // [MUDAR] ver diretamente a variável!
//             return cCom.broadcast_cmd(ge, id_byte, n_nodes, node_list, ID_);
//         }

//         else if (cmd == gvT)
//         {
//             system_cmd_total += 33.55; // [MUDAR] ver diretamente a variável!
//             return cCom.broadcast_cmd(gv, id_byte, n_nodes, node_list, ID_);
//         }

//         else if (cmd == gfT)
//         {
//             system_cmd_total += 22.55; // [MUDAR] ver diretamente a variável!
//             return cCom.broadcast_cmd(gf, id_byte, n_nodes, node_list, ID_);
//         }

//         else if (cmd == r)
//         {
//             system_cmd_total += 11.55; // [MUDAR] ver diretamente a variável!
//             return cCom.broadcast_cmd(r, id_byte, n_nodes, node_list, ID_);
//         }
//         return false;
//     }

//     String processes_cmd(String serverMessage)
//     {
//         uint8_t arduino_id = serverMessage[IDm] & 0x1F; // Uses only lowe part of byte

//         uint8_t client_id = (serverMessage[IDm] >> 5) & 0x07;
//         String return_msg;
//         if ((serverMessage[CMDm] & 0x3F) == gl)
//         { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));

//             return_msg = String(client_id) + "l " + String(arduino_id) + String(' ') + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == gd)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "d " + String(arduino_id) + String(' ') + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == go)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             return_msg = String(client_id) + "o " + String(arduino_id) + String(' ') + String(uint8_t(serverMessage[Dm]));
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == gO)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "O " + String(arduino_id) + String(' ') + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == gU)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "U " + String(arduino_id) + String(' ') + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == gL)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "L " + String(arduino_id) + String(' ') + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == gx)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "x " + String(arduino_id) + String(' ') + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == gr)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "r " + String(arduino_id) + String(' ') + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == gc)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "c " + String(arduino_id) + String(' ') + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == gp)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "p " + String(arduino_id) + String(' ') + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == gpT)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "p T " + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == gt)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "t " + String(arduino_id) + String(' ') + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == ge)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "e " + String(arduino_id) + String(' ') + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == geT)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "e T " + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == gv)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "v " + String(arduino_id) + String(' ') + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == gvT)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "v T " + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == gf)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "f " + String(arduino_id) + String(' ') + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == gfT)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             float aux;
//             memcpy(&aux, &serverMessage[Dm], sizeof(float));
//             return_msg = String(client_id) + "f T " + String(aux);
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == o)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             if (serverMessage[Dm] == 1)
//             {
//                 return_msg = String(client_id) + "ack";
//             }
//             else
//             {
//                 return_msg = String(client_id) + "err";
//             }
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == O)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             if (serverMessage[Dm] == 1)
//             {
//                 return_msg = String(client_id) + "ack";
//             }
//             else
//             {
//                 return_msg = String(client_id) + "err";
//             }
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == U)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             if (serverMessage[Dm] == 1)
//             {
//                 return_msg = String(client_id) + "ack";
//             }
//             else
//             {
//                 return_msg = String(client_id) + "err";
//             }
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == c)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             if (serverMessage[Dm] == 1)
//             {
//                 return_msg = String(client_id) + "ack";
//             }
//             else
//             {
//                 return_msg = String(client_id) + "err";
//             }
//             return return_msg;
//         }

//         else if ((serverMessage[CMDm] & 0x3F) == r)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             if (serverMessage[Dm] == 1)
//             {
//                 return_msg = String(client_id) + "ack";
//             }
//             else
//             {
//                 return_msg = String(client_id) + "err";
//             }
//             return return_msg;
//         }
//         else if ((serverMessage[CMDm] & 0x3F) == b_changed)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             return_msg = String(client_id) + "Get last minute buffer of variable [IMPLEMENTAR]";
//             return return_msg;
//         }
//         else if ((serverMessage[CMDm] & 0x3F) == ss)
//         { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
//             return_msg = String(client_id) + "Start/stop  [IMPLEMENTAR]";
//             return return_msg;
//         }

//         else
//         {
//             return String(client_id) + "ERROR";
//         }
//     }

//     uint8_t executes(char *cmd) // Executes the function associated with the requested command. Returns nomber of bytes of the return msg.
//     {
//         if ((cmd[CMDm] & 0x3F) == gl)
//         { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             float a = 12.34;
//             memcpy(&cmd[Dm], &a, sizeof(float));
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == gd)
//         { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             float a = 23.45;
//             memcpy(&cmd[Dm], &a, sizeof(float));
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == go)
//         { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             bool a = true;
//             memcpy(&cmd[Dm], &a, sizeof(bool));
//             return 3; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == gO)
//         { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             float a = 52.34;
//             memcpy(&cmd[Dm], &a, sizeof(float));
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == gU)
//         { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             float a = 37.34;
//             memcpy(&cmd[Dm], &a, sizeof(float));
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == gL)
//         { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             float a = 12.92;
//             memcpy(&cmd[Dm], &a, sizeof(float));
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == gx)
//         { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             float a = 65.34;
//             memcpy(&cmd[Dm], &a, sizeof(float));
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == gr)
//         { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             float a = 61.34;
//             memcpy(&cmd[Dm], &a, sizeof(float));
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == gc)
//         {                  // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             float a = 255; // DUTY CYCLE

//             float e = PJ * (a / 255) * 0.01; // Current energy consumption
//             memcpy(&cmd[Dm], &e, sizeof(float));
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == gp)
//         { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             float a = 1234.01;
//             memcpy(&cmd[Dm], &a, sizeof(float));
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == gt)
//         { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             float a = (float)millis() + 0.01;
//             memcpy(&cmd[Dm], &a, sizeof(float));
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == ge)
//         { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             float a = 123.34;
//             memcpy(&cmd[Dm], &a, sizeof(float));
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == gv)
//         { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             float a = 13.34;
//             memcpy(&cmd[Dm], &a, sizeof(float));
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == gf)
//         { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             float a = 123.34;
//             memcpy(&cmd[Dm], &a, sizeof(float));
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == o)
//         {             // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == O)
//         {
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == U)
//         {
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == c)
//         {
//             float e = 1; // DUTY CYCLE

//             float a = e * (255 / (PJ * 0.01)); // Current energy consumption
//             // set PWM with 'a' [TO DO!]
//             //bool a = true;
//             memcpy(&cmd[Dm], &a, sizeof(bool));
//             return 6; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == r)
//         { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
//             // Resets all nodes. Wait for ACK from all the network, then resets himself
//             bool a = true;
//             memcpy(&cmd[Dm], &a, sizeof(bool));
//             return 3; // returns number of bytes
//         }

//         else if ((cmd[CMDm] & 0x3F) == b_changed)
//         {
//         }

//         else if ((cmd[CMDm] & 0x3F) == ss)
//         {
//         }
//         return 0;
//     }
// };

#endif
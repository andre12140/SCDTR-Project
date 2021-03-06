#include <Arduino.h>
#include <EEPROM.h>
#include "system.h"
#include "EEPROM_data.h"
#include "arduino_msg.h"
#include "control.h"
#include "simulator.h"
#include "state.h"
#include "can_com.h"
#include "node.h"

#define rho 0.07

MCP2515 mcp2515(10); //SS pin 10
can_frame_stream cf_stream;
Comunication comObj;
nodeL nodeList;
System sys;
Simulation simulator;
Control controller;
upperLevel desk_occ;
lowLevel desk_free;
String arduinoMessage;
String serverMessage;

ISR(TIMER1_COMPA_vect)
{
  sys.samp = true;
  // At each sampling time read from LDR
  sys.Lux_prev2 = sys.Lux_prev1; // LUX measurement of t-2
  sys.Lux_prev1 = sys.Lux;       // LUX measurement of t-1
  sys.readingsLDR();
}

void irqHandler()
{
  can_frame frm;
  uint8_t irq = mcp2515.getInterrupts();

  //check messages in buffer 0
  if (irq & MCP2515::CANINTF_RX0IF)
  {
    mcp2515.readMessage(MCP2515::RXB0, &frm);
    if (!cf_stream.put(frm)) //no space
      comObj.arduino_overflow = true;
  }
  if (irq & MCP2515::CANINTF_RX1IF)
  {
    mcp2515.readMessage(MCP2515::RXB1, &frm);
    if (!cf_stream.put(frm)) //no space
      comObj.arduino_overflow = true;
  }
  irq = mcp2515.getErrorFlags(); //read EFLG
  if ((irq & MCP2515::EFLG_RX0OVR) |
      (irq & MCP2515::EFLG_RX1OVR))
  {
    comObj.mcp2515_overflow = true;
    mcp2515.clearRXnOVRFlags();
  }
  mcp2515.clearInterrupts();

  /*   Serial.print("\n Msg from ");
  Serial.println(frm.can_id);
  Serial.println("--");
  Serial.println((int)frm.data[IDm]);
  Serial.println(nodeList.ID); */

  if (nodeList.initializing_network && frm.data[0] == NID)
  {
    /* Serial.println("Received NID"); */
    nodeList.new_node = true;
    nodeList.EONI_flag = true;
  }
  // (Calibration) Nodes are ready to compute coupling gains
  else if ((frm.data[CMDm] == compute_CG) && sys.CALIB)
  {
    sys.CCG_flag = true;
  }
  // (Calibration) Nodes are ready to send coupling gains
  else if ((frm.data[CMDm] == send_CG) && sys.CALIB && ((uint8_t)(frm.data[IDm]) == nodeList.ID))
  {
    sys.SCG_flag = true;
  }
  // (Controller) Nodes share their control intent
  else if ((frm.data[CMDm] == send_PWM) && (controller.CONTROL) && ((uint8_t)(frm.data[IDm]) == nodeList.ID))
  {
    controller.SPWM_flag = true;
    /*     Serial.print(frm.can_id);
    Serial.println(" - [SPWM] "); */
  }
  else if (!(controller.CONTROL) && (!(nodeList.initializing_network)) && (!sys.CALIB))
  {
    /*     Serial.println("HUB");
    Serial.println(frm.data[IDm]);
    Serial.println(nodeList.ID); */
    comObj.interruptCanMsg = true; //notify loop()
  }
} //end irqHandler()

void initCanCom()
{
  SPI.begin();
  attachInterrupt(0, irqHandler, FALLING); //use interrupt at pin 2
  //Must tell SPI lib that ISR for interrupt vector zero will be using SPI
  SPI.usingInterrupt(0);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();
  //mcp2515.setLoopbackMode(); //for local testing
}

void readEEPROM()
{
  EEPROM.get(ID_ADDR, nodeList.ID);
  EEPROM.get(M_ADDR, sys.m);
  EEPROM.get(B_ADDR, sys.b);
  EEPROM.get(R2_BASE_ADDR, sys.R2_base);
  EEPROM.get(R2_EXP_ADDR, sys.R2_exp);
  EEPROM.get(TAU_A_ADDR, sys.tau_a);
  EEPROM.get(TAU_B_ADDR, sys.tau_b);
  EEPROM.get(TAU0_ADDR, sys.tau_zero);
  EEPROM.get(KP_ADDR, controller.kp);
  EEPROM.get(KI_ADDR, controller.ki);

  /*   Serial.println(sys.m);
  Serial.println(sys.b);
  Serial.println(sys.R2_base);
  Serial.println(sys.R2_exp);
  Serial.println(sys.tau_a);
  Serial.println(sys.tau_b);
  Serial.println(sys.tau_zero);
  Serial.println(controller.kp);
  Serial.println(controller.ki); */

  EEPROM.get(CHECKSUM_ADDR, CHECKSUM);

  float cmp_checksum = (float)nodeList.ID + sys.m + sys.b + sys.R2_base + sys.R2_exp + sys.tau_a + sys.tau_b + sys.tau_zero + controller.kp + controller.ki;
  if (cmp_checksum != CHECKSUM)
  {
    Serial.println("DATA IS CORRUPTED!");
    delay(10);
    exit(0);
  }

  else
  {
    Serial.println("DATA IS UNCORRUPTED!");
  }
}

void receivedMsgSerial()
{
  Serial.print("D SA\n");
  serverMessage = Serial.readString();
  // Checks commands without ID (targeting all nodes)
  if (comObj.check_cmd_ID(serverMessage[CMDm], serverMessage[IDm], nodeList.n_nodes, nodeList.node_list, nodeList.ID))
  {
    comObj.system_cmd = true; // Indicates that a system cmd was sent; Meaning that must store all returned value from node network
    comObj.current_system_cmd = (serverMessage[CMDm] & 0x3F);
  }

  // Checks commands with an ID (targeting one node of network)
  else if (uint8_t(serverMessage[IDm] & 0x1F) == nodeList.ID) // If message received is for this arduino
  {
    Serial.print("D Hub arduino got the command and replies\n");
    // Meter todos os casos possiveis numa fun????o!
    char *aux_msg = (char *)(serverMessage.c_str());

    comObj.executes(aux_msg, nodeList.node_list, nodeList.n_nodes, nodeList.ID);
    // Processa a mensagem e retorna para o Server
    Serial.print("C");
    Serial.println(comObj.processes_cmd(serverMessage));
  }
  else // If message is for an arduino of the network
  {
    Serial.print("D BYTE IDM ");
    Serial.println(uint8_t(serverMessage[IDm] & 0x1F), HEX);

    if (nodeList.checkID(serverMessage[IDm] & 0x1F))
    {
      Serial.print("D HUB arduino relays message to network\n");

      const char *serverMessage_char = serverMessage.c_str();
      comObj.write(nodeList.ID, (char *)serverMessage_char, serverMessage.length());
      comObj.HUB_MODE = true;
    }    // ID of the message corresponds to ID of arduino in network
    else // Given ID in message has no arduino
    {
      Serial.print("C Error! ID sent on message doesn't correespond to arduino ID in the network\n");
    }
  }
}

void setup()
{
  Serial.begin(1000000);
  sys.configTimers();
  readEEPROM();
  initCanCom();

  nodeList.node_list[0] = nodeList.ID;
  nodeList.network_init(); // Listens CANbus for other arduinos until a timeout and defines the list of nodes

  Serial.print("D ANI!\n");

  delay(500);

  sys.calibration();

  controller.initUff();
}

unsigned long counter = 0;
uint8_t msg = 0;
unsigned long ts = millis();
void loop()
{
  if (millis() - ts >= 3500) // Debug
  {
    ts = millis();
    Serial.print("C IA!\n");
    Serial.print("D Visibility error ");
    Serial.println(sys.visibility_error);
  }

  if (comObj.interruptCanMsg)
  {
    comObj.interruptCanMsg = false;
    comObj.canMsgReceived(nodeList.n_nodes, nodeList.node_list, nodeList.ID);
  }

  if (Serial.available()) // Received message from server
  {
    receivedMsgSerial();
  }

  if (sys.samp)
  {
    // Total energy consumption
    sys.total_energy += (sys.PJ * controller.u * 0.01);
    // Total visibility error
    sys.visibility_error += ((0) >= ((controller.e)) ? (0) : (controller.e)) / (millis() / 10);

    // Total flicker error

    float aux1 = ((sys.Lux - sys.Lux_prev1)) < (0) ? -(sys.Lux - sys.Lux_prev1) : (sys.Lux - sys.Lux_prev1);                       // Abs value of (Lux - Prev_lux1)
    float aux2 = ((sys.Lux_prev1 - sys.Lux_prev2)) < (0) ? -((sys.Lux_prev1 - sys.Lux_prev2)) : ((sys.Lux_prev1 - sys.Lux_prev2)); // Abs value of (Prev_lux1 - Prev_lux2)
    float fi = (((sys.Lux - sys.Lux_prev1) * (sys.Lux_prev1 - sys.Lux_prev2)) >= ((0)) ? (0) : ((aux1 + aux2) / (2 * 0.01)));
    sys.total_flicker_error += fi / (millis() / 10);

    //controller.decentralizedCoordControl();
    sys.samp = false;
  }
}

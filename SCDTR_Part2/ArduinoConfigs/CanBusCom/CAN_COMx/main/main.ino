#include <Arduino.h>
#include "canFrameStream.h"
#include <EEPROM.h>

#include "EEPROM_data.h"
#include "arduino_msg.h"

MCP2515 mcp2515(10); //SS pin 10
//notification flag for ISR and loop()
volatile bool interrupt = false;
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;
volatile bool initializing_network = true;
volatile bool new_node = false; // When identified a new node in the ISR routine
volatile uint8_t n_nodes = 1;   // Number of network nodes
bool EONI_flag = false;
bool HUB_MODE = false;        // Indicates if this arduino is in HUB mode (responsible for realying msgs from Server to arduinos network)
bool system_cmd = false;      // Indicates that a system cmd was sent; Meaning that must store all returned value from node network
uint8_t system_cmd_reply = 0; // Number of node that replied to system cmd (expected to be number of nodes -1)
float system_cmd_total = 0;   // Sum of values of each node in the network
uint8_t current_system_cmd;   // System cmd that is being executed
String arduinoMessage;
bool cmd_r_ack = 1;

char new_str[8] = {1, 1, 1, 1, 1, 1, 1, 1};

// FUNCTION AUXILIARY VARIABLES
float PJ = 0.05; //maximum power

uint8_t *node_list = (uint8_t *)malloc(sizeof(uint8_t));
String serverMessage;

can_frame_stream cf_stream;

void (*resetFunc)(void) = 0; // Reset Function

bool checkID(uint8_t id)
{ // Checks if the given id is a member of the node network
  for (int i = 0; i < n_nodes; i++)
  {
    if (node_list[i] == id)
    { // id matches the list member
      return true;
    }
  }
  return false;
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
      arduino_overflow = true;
  }
  if (irq & MCP2515::CANINTF_RX1IF)
  {
    mcp2515.readMessage(MCP2515::RXB1, &frm);
    if (!cf_stream.put(frm)) //no space
      arduino_overflow = true;
  }
  irq = mcp2515.getErrorFlags(); //read EFLG
  if ((irq & MCP2515::EFLG_RX0OVR) |
      (irq & MCP2515::EFLG_RX1OVR))
  {
    mcp2515_overflow = true;
    mcp2515.clearRXnOVRFlags();
  }
  mcp2515.clearInterrupts();
  //Serial.print("Msg Received from ");
  //Serial.println(frm.can_id);

  if (initializing_network && frm.data[0] == NID)
  {
    //Serial.println("Received NID");
    new_node = true;
    EONI_flag = true;
  }
  else
  {
    //Serial.println("INTERRUPt = TRUE");
    interrupt = true; //notify loop()
  }
  Serial.print("INTERRUP");
  Serial.print(" from ID ");
  Serial.println(frm.can_id);
} //end irqHandler()

union my_float
{
  float value;
  unsigned char bytes[4];
};

MCP2515::ERROR write(uint32_t id, char *val, uint8_t n)
{
  can_frame frame;
  frame.can_id = id;
  frame.can_dlc = n;
  for (int i = 0; i < n; i++) //prepare can message
    frame.data[i] = val[i];
  return mcp2515.sendMessage(&frame);
}
MCP2515::ERROR write_byte(uint32_t id, uint8_t val)
{
  can_frame frame;
  frame.can_id = id;
  frame.can_dlc = 1;
  frame.data[0] = val;
  //send data
  return mcp2515.sendMessage(&frame);
}

bool broadcast_cmd(uint8_t cmd_code, uint8_t id_byte)
{
  int i;
  for (i = 1; i < n_nodes; i++) // starts in second position. (1st one is it's own ID)
  {
    char node_msg[2];
    node_msg[IDm] = (node_list[i] & 0x1F) | (id_byte & 0xE0); // Adds client ID to all nodes
    node_msg[CMDm] = cmd_code & 0x3F;                         // Sets signal bits to zero (Sv = 0, R = 0)
    write(ID, node_msg, 2);
  }
  return i;
}

bool check_cmd_ID(uint8_t cmd, uint8_t id_byte)
{
  cmd = cmd & 0x3F; // Clears 1st two bits

  if (cmd == gpT)
  {

    system_cmd_total += 66.55; // [MUDAR] ver diretamente a variável!
    return broadcast_cmd(gp, id_byte);
  }

  else if (cmd == geT)
  {
    system_cmd_total += 44.55; // [MUDAR] ver diretamente a variável!
    return broadcast_cmd(ge, id_byte);
  }

  else if (cmd == gvT)
  {
    system_cmd_total += 33.55; // [MUDAR] ver diretamente a variável!
    return broadcast_cmd(gv, id_byte);
  }

  else if (cmd == gfT)
  {
    system_cmd_total += 22.55; // [MUDAR] ver diretamente a variável!
    return broadcast_cmd(gf, id_byte);
  }

  else if (cmd == r)
  {
    system_cmd_total += 11.55; // [MUDAR] ver diretamente a variável!
    return broadcast_cmd(r, id_byte);
  }
  return false;
}

unsigned long t = 0; // aux DEBUG
void network_init()
{
  can_frame frame;
  Serial.print(ID);
  Serial.println("Intializing network...");
  unsigned long time_ref = millis(); // Initial time stamp
                                     // Flag to indentify End Of Network Identification cycle
  while ((millis() - time_ref < 20000))
  {
    if (EONI_flag == true)
    {
      new_node = false;
      cli();
      cf_stream.get(frame);
      sei();
      //Serial.println(frame.data[0]);
      n_nodes++; // incrementes number of nodes
      realloc(node_list, n_nodes);
      node_list[n_nodes - 1] = frame.can_id; // Adds new node ID to list of nodes
      Serial.print("Identified node ID: ");
      Serial.println(frame.can_id);
      break;
    }
    if ((millis() - t) >= 1000)
    {
      Serial.println((t++) / 1000);
      t = millis();
    }
  }
  //Serial.print("D 20s\n");
  Serial.println("20s passed OR EONI Flag received!");
  //Serial.println(millis() - time_ref);

  write_byte(ID, NID);               // Broadcasts own ID
  time_ref = millis();               // New time stamp
  while (millis() - time_ref < 5000) // Listens for other nodes
  {
    if (new_node)
    {
      new_node = false;

      cli();
      cf_stream.get(frame);
      sei();
      //Serial.println(frame.data[0]);
      n_nodes++; // incrementes number of nodes
      realloc(node_list, n_nodes);
      node_list[n_nodes - 1] = frame.can_id; // Adds new node ID to list of nodes
      Serial.print("Identified node ID: ");
      Serial.println(frame.can_id);
    }
  }

  initializing_network = false;
  //Serial.print(ID);
  //Serial.println("\tEnd of network initialization!");
  // while (1)
  // {
  //   write_byte(ID, NID);
  //   delay(100);
  // }
}

void setup()
{
  Serial.begin(1000000);

  EEPROM.get(ID_ADDR, ID);
  EEPROM.get(M_ADDR, M);
  EEPROM.get(B_ADDR, B);
  EEPROM.get(R2_BASE_ADDR, R2_BASE);
  EEPROM.get(R2_EXP_ADDR, R2_EXP);
  EEPROM.get(TAU_A_ADDR, TAU_A);
  EEPROM.get(TAU_B_ADDR, TAU_B);
  EEPROM.get(TAU0_ADDR, TAU0);
  EEPROM.get(KP_ADDR, KP);
  EEPROM.get(KI_ADDR, KI);
  EEPROM.get(CHECKSUM_ADDR, CHECKSUM);

  float cmp_checksum = (float)ID + M + B + R2_BASE + R2_EXP + TAU_A + TAU_B + TAU0 + KP + KI;
  if (cmp_checksum != CHECKSUM)
  {
    //Serial.println("DATA IS CORRUPTED!");
    delay(10);
    exit(0);
  }

  else
  {
    //Serial.println("DATA IS UNCORRUPTED!");
  }

  node_list[0] = ID;

  SPI.begin();
  attachInterrupt(0, irqHandler, FALLING); //use interrupt at pin 2
  //Must tell SPI lib that ISR for interrupt vector zero will be using SPI
  SPI.usingInterrupt(0);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();
  //mcp2515.setLoopbackMode(); //for local testing

  network_init(); // Listens CANbus for other arduinos until a timeout and defines the list of nodes
  Serial.print("D ANI!\n");
}

String processes_cmd(String serverMessage)
{
  uint8_t arduino_id = serverMessage[IDm] & 0x1F; // Uses only lowe part of byte

  uint8_t client_id = (serverMessage[IDm] >> 5) & 0x07;
  String return_msg;
  if ((serverMessage[CMDm] & 0x3F) == gl)
  { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));

    return_msg = String(client_id) + "l " + String(arduino_id) + String(' ') + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == gd)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "d " + String(arduino_id) + String(' ') + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == go)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    return_msg = String(client_id) + "o " + String(arduino_id) + String(' ') + String(uint8_t(serverMessage[Dm]));
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == gO)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "O " + String(arduino_id) + String(' ') + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == gU)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "U " + String(arduino_id) + String(' ') + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == gL)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "L " + String(arduino_id) + String(' ') + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == gx)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "x " + String(arduino_id) + String(' ') + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == gr)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "r " + String(arduino_id) + String(' ') + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == gc)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "c " + String(arduino_id) + String(' ') + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == gp)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "p " + String(arduino_id) + String(' ') + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == gpT)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "p T " + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == gt)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "t " + String(arduino_id) + String(' ') + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == ge)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "e " + String(arduino_id) + String(' ') + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == geT)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "e T " + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == gv)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "v " + String(arduino_id) + String(' ') + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == gvT)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "v T " + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == gf)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "f " + String(arduino_id) + String(' ') + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == gfT)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    float aux;
    memcpy(&aux, &serverMessage[Dm], sizeof(float));
    return_msg = String(client_id) + "f T " + String(aux);
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == o)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    if (serverMessage[Dm] == 1)
    {
      return_msg = String(client_id) + "ack";
    }
    else
    {
      return_msg = String(client_id) + "err";
    }
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == O)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    if (serverMessage[Dm] == 1)
    {
      return_msg = String(client_id) + "ack";
    }
    else
    {
      return_msg = String(client_id) + "err";
    }
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == U)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    if (serverMessage[Dm] == 1)
    {
      return_msg = String(client_id) + "ack";
    }
    else
    {
      return_msg = String(client_id) + "err";
    }
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == c)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    if (serverMessage[Dm] == 1)
    {
      return_msg = String(client_id) + "ack";
    }
    else
    {
      return_msg = String(client_id) + "err";
    }
    return return_msg;
  }

  else if ((serverMessage[CMDm] & 0x3F) == r)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    if (serverMessage[Dm] == 1)
    {
      return_msg = String(client_id) + "ack";
    }
    else
    {
      return_msg = String(client_id) + "err";
    }
    return return_msg;
  }
  else if ((serverMessage[CMDm] & 0x3F) == b)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    return_msg = String(client_id) + "Get last minute buffer of variable [IMPLEMENTAR]";
    return return_msg;
  }
  else if ((serverMessage[CMDm] & 0x3F) == ss)
  { // // Get current duty cycle at luminaire i   (AND 0x3F to clear signal bits)
    return_msg = String(client_id) + "Start/stop  [IMPLEMENTAR]";
    return return_msg;
  }

  else
  {
    return String(client_id) + "ERROR";
  }
}

uint8_t executes(char *cmd) // Executes the function associated with the requested command. Returns nomber of bytes of the return msg.
{
  if ((cmd[CMDm] & 0x3F) == gl)
  { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    float a = 12.34;
    memcpy(&cmd[Dm], &a, sizeof(float));
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == gd)
  { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    float a = 23.45;
    memcpy(&cmd[Dm], &a, sizeof(float));
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == go)
  { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    bool a = true;
    memcpy(&cmd[Dm], &a, sizeof(bool));
    return 3; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == gO)
  { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    float a = 52.34;
    memcpy(&cmd[Dm], &a, sizeof(float));
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == gU)
  { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    float a = 37.34;
    memcpy(&cmd[Dm], &a, sizeof(float));
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == gL)
  { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    float a = 12.92;
    memcpy(&cmd[Dm], &a, sizeof(float));
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == gx)
  { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    float a = 65.34;
    memcpy(&cmd[Dm], &a, sizeof(float));
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == gr)
  { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    float a = 61.34;
    memcpy(&cmd[Dm], &a, sizeof(float));
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == gc)
  {                // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    float a = 255; // DUTY CYCLE

    float e = PJ * (a / 255) * 0.01; // Current energy consumption
    memcpy(&cmd[Dm], &e, sizeof(float));
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == gp)
  { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    float a = 1234.01;
    memcpy(&cmd[Dm], &a, sizeof(float));
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == gt)
  { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    float a = (float)millis() + 0.01;
    memcpy(&cmd[Dm], &a, sizeof(float));
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == ge)
  { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    float a = 123.34;
    memcpy(&cmd[Dm], &a, sizeof(float));
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == gv)
  { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    float a = 13.34;
    memcpy(&cmd[Dm], &a, sizeof(float));
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == gf)
  { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    float a = 123.34;
    memcpy(&cmd[Dm], &a, sizeof(float));
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == o)
  {           // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == O)
  {
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == U)
  {
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == c)
  {
    float e = 1; // DUTY CYCLE

    float a = e * (255 / (PJ * 0.01)); // Current energy consumption
    // set PWM with 'a' [TO DO!]
    //bool a = true;
    memcpy(&cmd[Dm], &a, sizeof(bool));
    return 6; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == r)
  { // Get current measured illuminance at desk <i>   (AND 0x3F to clear signal bits)
    // Resets all nodes. Wait for ACK from all the network, then resets himself
    bool a = true;
    memcpy(&cmd[Dm], &a, sizeof(bool));
    return 3; // returns number of bytes
  }

  else if ((cmd[CMDm] & 0x3F) == b)
  {
  }

  else if ((cmd[CMDm] & 0x3F) == ss)
  {
  }
  return 0;
}

unsigned long counter = 0;
uint8_t msg = 0;
unsigned long ts = millis();
void loop()
{
  if (millis() - ts >= 3500) // Debug
  {
    ts = millis();
    Serial.print("C0 IA!\n");
  }

  if (interrupt)
  {
    interrupt = false;
    if (mcp2515_overflow)
    {
      Serial.println("D MCP2516 RX Buf Overflow");
      mcp2515_overflow = false;
    }
    if (arduino_overflow)
    {
      Serial.println("D Arduino Buffers Overflow");
      arduino_overflow = false;
    }
    can_frame frame;
    bool has_data;
    cli();
    has_data = cf_stream.get(frame);
    sei();

    while (has_data)
    {

      if ((frame.data[IDm] & 0x1F) == ID && frame.can_id != ID) // Message for this node
      {
        Serial.println("D Message for this node");
        if (HUB_MODE && (((frame.data[CMDm] >> 7) & 0x01) == 1)) // Checks SV bit (if =1 rplies to server)
        {
          Serial.print("D Relayed message from another arduino to server\n");

          frame.data[IDm] = (frame.data[IDm] & 0xE0) | frame.can_id; // Clear Least 5 significant bits
          //memcpy(&(frame.data[IDm]), &(frame.can_id), 1);
          Serial.print("C ");
          Serial.println(processes_cmd((char *)(frame.data))); // Sends to Server formated return of the requested cmd

          HUB_MODE = false;
        }
        else if (((frame.data[CMDm] >> 6) & 0x01) == 0) // Evaluates response bit (if=0 replies)
        {
          //char aux_msg[8];
          // Processar comando
          //Enviar return

          Serial.print("D Received message from arduino HUB. Replies to him...\n");
          new_str[CMDm] = (char)(frame.data[CMDm] | 0x40);                 // Set response bit to one
          new_str[IDm] = (char)frame.can_id;                               // Adds arduino ID
          new_str[IDm] = (new_str[IDm] & 0x1F) | (frame.data[IDm] & 0xE0); // Adds client ID to high part of byte

          uint8_t n = executes(new_str); // return number of bytes of arduinoMessage
          write(ID, new_str, n);
          if ((new_str[CMDm] & 0x3F) == r) // if reset command
          {
            resetFunc();
          }
        }
        else if (((frame.data[CMDm] >> 6) & 0x01) == 1) // response bit = 1 > return from this arduino request
        {
          Serial.print("D This arduino got a response from it's request\n");
          if (system_cmd)
          {
            if (current_system_cmd == r) // reset command
            {
              cmd_r_ack = (cmd_r_ack & frame.data[Dm]);
              system_cmd_reply++;
              if (system_cmd_reply == (n_nodes - 1)) // all nodes replied
              {
                char cmd_proc[3];
                cmd_proc[CMDm] = current_system_cmd;
                cmd_proc[Dm] = cmd_r_ack;

                Serial.print("C ");
                Serial.println(processes_cmd(cmd_proc));
                if (cmd_r_ack) // All nodes reseted correctly
                {
                  resetFunc(); // resets arduino
                }
                else
                {
                  cmd_r_ack = 1; // resets flag
                }
              }
            }
            else // All broadcast commands (except reset)
            {
              float aux = 0;
              memcpy(&aux, &frame.data[Dm], sizeof(float)); // gets value return from cmd
              system_cmd_total += aux;

              system_cmd_reply++;
              if (system_cmd_reply == (n_nodes - 1)) // all nodes replied
              {
                char cmd_proc[6];
                cmd_proc[CMDm] = current_system_cmd;
                memcpy(&cmd_proc[Dm], &system_cmd_total, sizeof(float));

                Serial.print("C ");
                Serial.println(processes_cmd(cmd_proc));
                // Reset values for next system cmd call
                system_cmd_total = 0;
                system_cmd_reply = 0;
                system_cmd = false; // End system cmd cycle
              }
            }
          }
          //var = return(comando) // Saves local variable for internal use
        }
        else
        {
          Serial.println("D Else");
        }
        // for (int i = 0; i < frame.can_dlc; i++)
        //   msg.bytes[i] = frame.data[i];
        // Serial.print("\t\t");

        // Serial.print("Receiving: ");
        // Serial.print(frame.can_id);
        // Serial.print(" :");
        // for (int i = 0; i < frame.can_dlc; i++)
        //   Serial.println(msg.bytes[i]);
      }

      cli();
      has_data = cf_stream.get(frame);
      sei();
    }
  }

  if (Serial.available()) // Received message from server
  {
    Serial.print("D SA\n");
    serverMessage = Serial.readString();
    // Checks commands without ID (targeting all nodes)
    if (check_cmd_ID(serverMessage[CMDm], serverMessage[IDm]))
    {
      system_cmd = true; // Indicates that a system cmd was sent; Meaning that must store all returned value from node network
      current_system_cmd = (serverMessage[CMDm] & 0x3F);
    }

    // Checks commands with an ID (targeting one node of network)
    else if (uint8_t(serverMessage[IDm] & 0x1F) == ID) // If message received is for this arduino
    {
      Serial.print("D Hub arduino got the command and replies\n");
      // Meter todos os casos possiveis numa função!

      executes((char *)(serverMessage.c_str()));
      Serial.print("D serverMessage[Dm] ");
      Serial.println(serverMessage[Dm]);
      // Processa a mensagem e retorna para o Server
      Serial.print("C");
      Serial.println(processes_cmd(serverMessage));
    }
    else // If message is for an arduino of the network
    {
      if (checkID(serverMessage[IDm] & 0x1F))
      {
        Serial.print("D HUB arduino relays message to network\n");

        const char *serverMessage_char = serverMessage.c_str();
        write(ID, (char *)serverMessage_char, serverMessage.length());
        HUB_MODE = true;
      }    // ID of the message corresponds to ID of arduino in network
      else // Given ID in message has no arduino
      {

        Serial.print("C Error! ID sent on message doesn't correespond to arduino ID in the network\n");
      }
    }
  }
}

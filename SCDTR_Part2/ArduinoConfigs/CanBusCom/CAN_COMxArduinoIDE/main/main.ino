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
uint8_t *node_list = (uint8_t *)malloc(sizeof(uint8_t));

can_frame_stream cf_stream;

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
  Serial.println("Msg Received");

  if (initializing_network && frm.data[0] == NID)
  {
    Serial.println("Received NID");
    new_node = true;
    EONI_flag = true;
  }
  else
  {
    Serial.println("INTERRUPt = TRUE");
    interrupt = true; //notify loop()
  }

} //end irqHandler()

union my_can_msg
{
  unsigned long value;
  unsigned char bytes[8];
};

MCP2515::ERROR write(uint32_t id, uint32_t val)
{
  can_frame frame;
  frame.can_id = id;
  frame.can_dlc = 4;
  my_can_msg msg;
  msg.value = val;            //pack data
  for (int i = 0; i < 4; i++) //prepare can message
    frame.data[i] = msg.bytes[i];
  //send data
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
      Serial.println(frame.data[0]);
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
  Serial.println("20s passed OR EONI Flag received!");
  Serial.println(millis() - time_ref);

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
      Serial.println(frame.data[0]);
      n_nodes++; // incrementes number of nodes
      realloc(node_list, n_nodes);
      node_list[n_nodes - 1] = frame.can_id; // Adds new node ID to list of nodes
      Serial.print("Identified node ID: ");
      Serial.println(frame.can_id);
    }
  }

  initializing_network = false;
  Serial.print(ID);
  Serial.println("\tEnd of network initialization!");
}

void setup()
{
  Serial.begin(500000);

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
    Serial.println("DATA IS CORRUPTED!");
    delay(10);
    exit(0);
  }

  else
  {
    Serial.println("DATA IS UNCORRUPTED!");
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
}

unsigned long counter = 0;
uint8_t msg = 0;
unsigned long ts = millis();
void loop()
{
  //send a few msgs in a burst
  // for (int i = 0; i < 4; i++)
  // {
  //   Serial.print(ID);
  //   Serial.print("Sending: ");
  //   Serial.println(counter); ///////////// -------- ID ------------
  //   if (write(ID, counter++) != MCP2515::ERROR_OK)
  //     Serial.println("\t\t\t\tMCP2515 TX Buf Full");
  // }

  if (millis() - ts >= 3500)
  {
    my_can_msg msg_c;
    msg_c.bytes[0] = node_list[1];
    msg_c.bytes[1] = ++msg;
    write(ID, msg_c.value);
    ts = millis();
  }

  if (interrupt)
  {
    interrupt = false;
    if (mcp2515_overflow)
    {
      Serial.println("\t\t\t\tMCP2516 RX Buf Overflow");
      mcp2515_overflow = false;
    }
    if (arduino_overflow)
    {
      Serial.println("\t\t\t\tArduino Buffers Overflow");
      arduino_overflow = false;
    }
    can_frame frame;
    bool has_data;
    cli();
    has_data = cf_stream.get(frame);
    sei();
    my_can_msg msg;
    while (has_data)
    {
      if (frame.data[0] == ID)
      { // Message for this node

        for (int i = 0; i < frame.can_dlc; i++)
          msg.bytes[i] = frame.data[i];
        Serial.print("\t\t");

        Serial.print("Receiving: ");
        Serial.print(frame.can_id);
        Serial.print(" :");
        for (int i = 0; i < frame.can_dlc; i++)
          Serial.println(msg.bytes[i]);
      }

      cli();
      has_data = cf_stream.get(frame);
      sei();
    }
  }
  delay(500); //some time to breath
}

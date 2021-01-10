#include <Arduino.h>
#include <canFrameStream.h>

MCP2515 mcp2515(10); //SS pin 10
//notification flag for ISR and loop()
volatile bool interrupt = false;
volatile bool mcp2515_overflow = false;
volatile bool arduino_overflow = false;

can_frame_stream cf_stream;

void irqHandler()
{	
toc= millis();
Serial.print("Received msg in ");
Serial.println(toc-tic);

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
  interrupt = true; //notify loop()
} //end irqHandler()

union my_can_msg
{
  unsigned long value;
  unsigned char bytes[4];
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

void setup()
{
  Serial.begin(500000);
  SPI.begin();
  attachInterrupt(0, irqHandler, FALLING); //use interrupt at pin 2
  //Must tell SPI lib that ISR for interrupt vector zero will be using SPI
  SPI.usingInterrupt(0);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_16MHZ);
  mcp2515.setNormalMode();
  //mcp2515.setLoopbackMode(); //for local testing
}

unsigned long counter = 0;

unsigned long t_fixed = 0;
unsigned long tic = 0;
unsigned long toc = 0;
void loop()
{
  //Passed 1 second
  if (millis()-t_fixed > 1000){
	  tic = millis();
	  write(1, counter, sizeof(unsigned long));
	  t_fixed = millis();
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
    while (has_data)
    {
      
      cli();
      has_data = cf_stream.get(frame);
      sei();
    }
  }

}
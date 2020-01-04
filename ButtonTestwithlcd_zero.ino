#include "Arduino.h"
#include "SamNonDuePin.h"
#include "variant.h"
#include <due_can.h>
#include <mcp_can.h>
#include <SPI.h>

#define TEST1_CAN_TRANSFER_ID    0x7E1 //random 29 bits
#define TEST1_CAN0_TX_PRIO       1
#define CAN_MSG_DUMMY_DATA       0x00000000

// CAN frame max data length
#define MAX_CAN_FRAME_DATA_LEN   8

const int SPI_CS_PIN = 78; 
SWcan SW_CAN(SPI_CS_PIN);   // Set CS pin

// Message variable to be send
uint32_t CAN_MSG_1 = 0;


// constants won't change. They're used here to
// set pin numbers:
const int SW1 = X1;                // Pushbutton SW1
const int SW2 = PIN_EMAC_ERX1;     // Pushbutton SW2

const int Red =  32;       
const int Yellow3 =  X0;     
const int Yellow2 =  27;  
const int Yellow1 =  24;    
const int Green =  23; 
const int Blue =  5;         // the number of the LED pin
const int Red2 =  11;   

// others are: 32(RED), X0(YELLOW), 27(YELLOW), 24(YELLOW), 23(GREEN), 12(RGB_GREEN), 5(RGB_BLUE), 11(RGB_RED)

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int buttonState2 = 0;        // variable for reading the pushbutton status
int incomingByte;      // a variable to read incoming serial data into
int arm = 0;           // variable for keep alive enable
int arm2 = 0;         // variable for keep alive enable 2
float soc = 0;        // variable for state of charge
float fuel = 0;       // variable for fuel
int rpm0 = 1000;      // variable for rpm
int rpm1 = 0;         // variable for rpm
int rpm2 = 0;         // variable for rpm
int wheeldata;        // variable for incoming steering wheel data

void setup() {
 //set pins as output
  pinMode(Green, OUTPUT);
  pinMode(Yellow1, OUTPUT);
  pinMode(Red, OUTPUT);
   pinMode(Blue, OUTPUT);
   pinMode(Red2, OUTPUT);
//set pins as inputs
  pinModeNonDue(SW1, INPUT);
  pinModeNonDue(SW2, INPUT);
//set outputs high or low
  digitalWrite(Green, HIGH);
  digitalWrite(Yellow1, HIGH);
  digitalWrite(Red, HIGH);
    digitalWrite(Blue, LOW);
    digitalWrite(Red2, LOW);

//start CAN0 and filters
 CAN.init(CAN_BPS_500K);
 Can0.begin(CAN_BPS_500K);

 //By default there are 7 mailboxes for each device that are RX boxes
  //This sets each mailbox to have an open filter that will accept standard frames
  //standard
  Can0.setRXFilter(0, 0x1EF, 0x7ff, false); //set filter for mailbox 0 this is for rpm messeges
  Can0.setRXFilter(1, 0x7E8, 0x7ff, false); //set filter for mailbox 1 this is for diag replies
  CAN.setCallback(1, displaylcd);           //set callback pointer for mailbox 1


 //start swcan and filters
 SW_CAN.setupSW(0x00);
  delay(100);
  SW_CAN.init_Mask(0, 1, 0x1FFFFFFF);                         // there are 2 mask in mcp2515, you need to set both of them
  SW_CAN.init_Mask(1, 1, 0x1FFFFFFF);
  SW_CAN.init_Filt(0, 1, 0x10758040);                          // there are 6 filter in mcp2515
  SW_CAN.init_Filt(1, 1, 0x10758040);                          // there are 6 filter in mcp2515
  SW_CAN.mode(3); // Go to normal mode. 0 - Sleep, 1 - High Speed, 2 - High Voltage Wake-Up, 3 - Normal
   
// start serial usb and serial3 (lcd)
  Serial3.begin(9600);  //lcd serial
  SerialUSB.begin(115200); //usb diag port serial
 
}


void displaylcd(CAN_FRAME *frame1)    // 
{
  if(frame1->data.bytes[3] == 91)//5B  //cal SOC then send to display
  {
  soc=frame1->data.bytes[4];
  soc=soc*100/255;
  Serial3.print("t0.txt=");
  Serial3.write(0x22);
   Serial3.print(soc);
    Serial3.print("%");
    Serial3.write(0x22);
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
  }
  
if(frame1->data.bytes[3] == 47)//2F //cal fuel then send to display
  {
  fuel=frame1->data.bytes[4];
  fuel=fuel/255;
  fuel=fuel*9.3122;
  Serial3.print("t2.txt=");
  Serial3.write(0x22);
   Serial3.print(fuel);
    Serial3.print("Gal");
    Serial3.write(0x22);
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
  }
  
}


void read() // send obd2 diag request data SOC
{
 CAN_FRAME frame1;
 frame1.id = 0x7E0;
   frame1.length = 4;
   //Priority
   frame1.priority = 0;
   frame1.rtr = 1;
   //Below we set the 8 data bytes in 32 bit (4 byte) chunks
   //Bytes can be set individually with frame1.data.bytes[which] = something
   frame1.data.low = 0x5B002203;
   frame1.data.high = CAN_MSG_DUMMY_DATA;
   //We are using extended frames so mark that here. Otherwise it will just use
   //the first 11 bits of the ID set
   frame1.extended = 0;
   CAN.sendFrame(frame1);
   delay(100);
}

void read2() // send obd2 diag request data fuel
{
 CAN_FRAME frame1;
 frame1.id = 0x7E0;
   frame1.length = 4;
   //Priority
   frame1.priority = 0;
   frame1.rtr = 1;
   //Below we set the 8 data bytes in 32 bit (4 byte) chunks
   //Bytes can be set individually with frame1.data.bytes[which] = something
   frame1.data.low = 0x2F002203;
   frame1.data.high = CAN_MSG_DUMMY_DATA;
   //We are using extended frames so mark that here. Otherwise it will just use
   //the first 11 bits of the ID set
   frame1.extended = 0;
   CAN.sendFrame(frame1);
   delay(100);
}


void keepalive()    //send keep alive for engine
{
 CAN_FRAME frame1;
 frame1.id = TEST1_CAN_TRANSFER_ID;
   frame1.length = MAX_CAN_FRAME_DATA_LEN;
   //Priority
   frame1.priority = 0;
   frame1.rtr = 1;
   //Below we set the 8 data bytes in 32 bit (4 byte) chunks
   //Bytes can be set individually with frame1.data.bytes[which] = something
   frame1.data.low = 0x00003E01;
   frame1.data.high = CAN_MSG_DUMMY_DATA;
   //We are using extended frames so mark that here. Otherwise it will just use
   //the first 11 bits of the ID set
   frame1.extended = 0;
   CAN.sendFrame(frame1);
   delay(100);
}

void keepalive2()         //send keep alive for body control module (daytime LED)
{
 CAN_FRAME frame1;
 frame1.id = 0x241;
   frame1.length = MAX_CAN_FRAME_DATA_LEN;
   //Priority
   frame1.priority = 0;
   frame1.rtr = 1;
   //Below we set the 8 data bytes in 32 bit (4 byte) chunks
   //Bytes can be set individually with frame1.data.bytes[which] = something
   frame1.data.low = 0x00003E01;
   frame1.data.high = CAN_MSG_DUMMY_DATA;
   //We are using extended frames so mark that here. Otherwise it will just use
   //the first 11 bits of the ID set
   frame1.extended = 0;
   CAN.sendFrame(frame1);
   delay(100);
}


void on()             //send request to turn on engine
{
 CAN_FRAME frame1;
 frame1.id = TEST1_CAN_TRANSFER_ID;
   frame1.length = MAX_CAN_FRAME_DATA_LEN;
   //Priority
   frame1.priority = 0;
   frame1.rtr = 1;
   //Below we set the 8 data bytes in 32 bit (4 byte) chunks
   //Bytes can be set individually with frame1.data.bytes[which] = something
   frame1.data.low = 0x0631AE07;
   frame1.data.high = CAN_MSG_DUMMY_DATA;
   //We are using extended frames so mark that here. Otherwise it will just use
   //the first 11 bits of the ID set
   frame1.extended = 0;
   CAN.sendFrame(frame1);
    delay(100);
}



void off()          //send request to turn off engine
{
    CAN_FRAME frame1;
  frame1.id = TEST1_CAN_TRANSFER_ID;
   frame1.length = MAX_CAN_FRAME_DATA_LEN;
   //Priority
   frame1.priority = 0;
   frame1.rtr = 1;
   //Below we set the 8 data bytes in 32 bit (4 byte) chunks
   //Bytes can be set individually with frame1.data.bytes[which] = something
   frame1.data.low = 0x0531AE07;
   frame1.data.high = CAN_MSG_DUMMY_DATA;
   //We are using extended frames so mark that here. Otherwise it will just use
   //the first 11 bits of the ID set
   frame1.extended = 0;

   CAN.sendFrame(frame1);
    delay(100);
}


void stop()         //send request to release control of engine , disabled arm and arm2
{
 CAN_FRAME frame1;
 frame1.id = TEST1_CAN_TRANSFER_ID;
   frame1.length = MAX_CAN_FRAME_DATA_LEN;
   //Priority
   frame1.priority = 0;
   frame1.rtr = 1;
   //Below we set the 8 data bytes in 32 bit (4 byte) chunks
   //Bytes can be set individually with frame1.data.bytes[which] = something
   frame1.data.low = 0x0000AE02;
   frame1.data.high = CAN_MSG_DUMMY_DATA;
   //We are using extended frames so mark that here. Otherwise it will just use
   //the first 11 bits of the ID set
   frame1.extended = 0;
   arm=0;           //set arm status to disabled
   arm2=0;          //set arm2 status to disabled
   CAN.sendFrame(frame1);
   delay(100);
}



void printFrame(CAN_FRAME &frame) {  //cal rpm data and send to display
  if (frame.id == 495) //1EF
  {
  rpm1 = (int) frame.data.bytes[2];
  rpm2 = (int) frame.data.bytes[3];

  rpm1 = rpm1*256;
  rpm0 = rpm1+rpm2/4;
  rpm0 = rpm0+600;
  
  //send CAN Message to serial3 (LCD)
  Serial3.print("t1.txt=");
  Serial3.write(0x22);
   Serial3.print(rpm0);
  Serial3.print("RPM");
    Serial3.write(0x22);
    Serial3.write(0xff);
    Serial3.write(0xff);
    Serial3.write(0xff);
  }
  
 // if (frame.id == 2024) //7E8
 // {
 // send CAN Message to serial3 (LCD)
 // Serial3.print("t0.txt=");
 // Serial3.write(0x22);
 //  Serial3.print(frame.data.bytes[2], DEC);
 //  Serial3.print(frame.data.bytes[3], DEC);
 //   Serial3.write(0x22);
 //   Serial3.write(0xff);
 //   Serial3.write(0xff);
 //   Serial3.write(0xff);
    
 // }

  
//send CAN ID, length, and data to USB serial output
 //  SerialUSB.print("ID:");
  // SerialUSB.print(frame.id, HEX);
 //  SerialUSB.print(" Len:");
 //  SerialUSB.print(frame.length);
 //  SerialUSB.print(" Data:0x");
//   for (int count = 0; count < frame.length; count++) {
//      SerialUSB.print(frame.data.bytes[count], HEX);
//      SerialUSB.print(" ");
 //  }
 //    SerialUSB.print("\r\n");
}


void loop() { 
  
  static unsigned long lastTime = 0;  
  
  // see if there's incoming serial data:
  if (Serial3.available() > 0) {
    
    // read the oldest byte in the serial buffer:
    incomingByte = Serial3.read();
    
 //   SerialUSB.println(incomingByte);
    
    if (incomingByte == 's') {
      digitalWrite(Yellow1, LOW);
      stop();
      arm = 0;
    }
    if (incomingByte == 'o') {
      digitalWrite(Green, LOW);
      arm = 1;
      on();
    }
    if (incomingByte == 'f') {
      digitalWrite(Red, LOW);
      arm = 1;
      off();
    }
    if (incomingByte == 'z') {
      digitalWrite(Red, HIGH);
      digitalWrite(Green, HIGH);
      digitalWrite(Yellow1, HIGH);
    }
  }

//swcan receive
  unsigned char len = 0;
  unsigned char buf[8];

  if (CAN_MSGAVAIL == SW_CAN.checkReceive())           // check if data coming
  {
    SW_CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

    wheeldata = (int) buf[1];
    unsigned int canId = SW_CAN.getCanId();

    if(wheeldata == 104) //68     // if messege is button up
    {
       arm = 1;          //set arm status to enabled
      on();             // go to on routine
     }
   
  if(wheeldata == 84)//54          // if messege is button down
    {
      arm = 1;         //set arm status to enabled
      off();          // go to off routine
     }
  
   }




//receive a CAN message
  CAN_FRAME incoming;
  if (Can0.available() > 0) {
  Can0.read(incoming); 
  printFrame(incoming);
  }
  
  
  buttonState = digitalReadNonDue(SW1);
  if (buttonState == LOW) {  // KEY pressed turn on daytime LED
    CAN_FRAME frame1;
 frame1.id = 0x241;
   frame1.length = 8;
   //Priority
   frame1.priority = 0;
   frame1.rtr = 1;
   //Below we set the 8 data bytes in 32 bit (4 byte) chunks
   //Bytes can be set individually with frame1.data.bytes[which] = something
   frame1.data.low = 0xF00FAE07;
   frame1.data.high =0x000000F0;
   //We are using extended frames so mark that here. Otherwise it will just use
   //the first 11 bits of the ID set
   frame1.extended = 0;
   arm2=1;  //set arm2 status to enabled
   CAN.sendFrame(frame1);
   delay(100);   
  }
 

  buttonState2 = digitalReadNonDue(SW2);
  if (buttonState2 == LOW) {     // KEY pressed turn off daytime LED
     CAN_FRAME frame1;
    frame1.id = 0x241;
   frame1.length = 8;
   //Priority
   frame1.priority = 0;
   frame1.rtr = 1;
   //Below we set the 8 data bytes in 32 bit (4 byte) chunks
   //Bytes can be set individually with frame1.data.bytes[which] = something
   frame1.data.low = 0xF00FAE07;
   frame1.data.high =CAN_MSG_DUMMY_DATA;
   //We are using extended frames so mark that here. Otherwise it will just use
   //the first 11 bits of the ID set
   frame1.extended = 0;

   CAN.sendFrame(frame1);
   arm2=1;    //set arm2 status to enabled
   delay(100);
  }


 if ((millis() - lastTime) > 2500)  // keep alive timer 1 & 2
  { 
    lastTime = millis();
    digitalWrite(Red2, LOW);     // turn red LED ON:
     read(); 
     read2(); 
    if (arm==1)
    {  
      keepalive();      // go to keepalive routine
     }
     
     if (arm2==1)
    {  
      keepalive2();      // go to keepalive2 routine 
     }
    }
  else {
    digitalWrite(Red2, HIGH);     // turn red LED OFF: 
  }
 
}

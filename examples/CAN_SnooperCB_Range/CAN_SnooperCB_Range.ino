/*
Arduino Due - Displays frames that fall between a range of addresses.
Modified from some of the other examples - notably CAN_SnooperCallBack
The difference here is that we only allow a range of ids. This demonstrates
the new watchForRange function as well as per-mailbox and general callback
functionality

By Thibaut Viard/Wilfredo Molina/Collin Kidder 2013-2014
//  Modified by Al Thomason for ATmegaxxM1 avr_CAN demo 2016

*/

#include <avr_can.h>


void printFrame(CAN_FRAME *frame, int filter) {
   Serial.print("Fltr: ");
   if (filter > -1) Serial.print(filter);
   else Serial.print("???");
   Serial.print(" ID: 0x");
   Serial.print(frame->id, HEX);
   Serial.print(" Len: ");
   Serial.print(frame->length);
   Serial.print(" Data: 0x");
   for (int count = 0; count < frame->length; count++) {
       Serial.print(frame->data.bytes[count], HEX);
       Serial.print(" ");
   }
   Serial.print("\r\n");
}

void gotFrameMB0(CAN_FRAME *frame) 
{
  printFrame(frame, 0);
}

void gotFrameMB1(CAN_FRAME *frame) 
{
  printFrame(frame, 1);
}

void gotFrameMB3(CAN_FRAME *frame) 
{
  printFrame(frame, 3);
}

void gotFrameMB4(CAN_FRAME *frame) 
{
  printFrame(frame, 4);
}

void gotFrame(CAN_FRAME *frame) 
{
  printFrame(frame, -1);
}

void setup()
{

  Serial.begin(115200);
  
  // Initialize CAN0, Set the proper baud rates here
  Can0.init(CAN_BPS_250K);
  Can0.setNumTXBoxes(0);                                             // Use all MOb (only 6x on the ATmegaxxM1 series) for receiving.
  
  //extended
  Can0.watchForRange(0x10000000, 0x10004000);
  Can0.watchForRange(0x10050000, 0x100500FF);
  Can0.setRXFilter(0, 0, true);                                     //catch all mailbox
  
  //standard  
  Can0.watchForRange(0x400, 0x470);
  Can0.watchForRange(0x500, 0x5FF);
  Can0.setRXFilter(0, 0, false);                                    //catch all mailbox
  
  //now register all of the callback functions.
  Can0.attachCANInterrupt(0, gotFrameMB0);
  Can0.attachCANInterrupt(1, gotFrameMB1);
  Can0.attachCANInterrupt(2, gotFrameMB1);
  Can0.attachCANInterrupt(3, gotFrameMB3);
  Can0.attachCANInterrupt(4, gotFrameMB4);
  //this function will get a callback for any mailbox that doesn't have a registered callback from above
  Can0.attachCANInterrupt(gotFrame);
  
}

void loop(){ //note the empty loop here. All work is done via callback as frames come in - no need to poll for them
}



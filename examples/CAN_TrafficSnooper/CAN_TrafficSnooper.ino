// Arduino ATmegaxxM1 - Displays all traffic found on canbus port
// By Thibaut Viard/Wilfredo Molina/Collin Kidder 2013-2014
//  Modified by Al Thomason for ATmegaxxM1 avr_CAN demo 2016


// Required libraries
#include <avr_can.h>

void setup()
{

  Serial.begin(115200);
  Serial.println("Ready");
    
  
  // Initialize CAN0 and CAN1, Set the proper baud rates here
  Can0.begin(CAN_BPS_250K);
  
  
  //This sets each mailbox to have an open filter that will accept extended
  //or standard frames
  Can0.setNumTXBoxes(0);                                    // Use all the mailboxes for receiving.
    
  int filter;

  for (filter = 0; filter < 4; filter++) {                  //Set up 4 of the boxes for extended
	Can0.setRXFilter(filter, 0, 0, true);
    }  

   while (Can0.setRXFilter(0, 0, false) > 0) ;              // Set up the remaining MObs for standard messages.

}

void printFrame(CAN_FRAME &frame) {
   Serial.print((int) (millis()/1000));
   Serial.print("  ID: 0x");
   Serial.print(frame.id, HEX);
   Serial.print(" Len: ");
   Serial.print(frame.length);
   Serial.print(" Data: 0x");
   for (int count = 0; count < frame.length; count++) {
       Serial.print(frame.data.bytes[count], HEX);
       Serial.print(" ");
   }
   Serial.print("\r\n");
}

void loop(){
  CAN_FRAME incoming;

 if (Can0.rx_avail()) {
      	if (Can0.read(incoming)) 
            printFrame(incoming);
         else 
            Serial.print(" -- FAILED");
  }
}




// Arduino Due - Displays all traffic found on  canbus port
//Modified version of the SnooperCallback sketch. This one illustrates
//how to use a class object to receive callbacks. It's a little different
//from the non-OO approach.

// By Thibaut Viard/Wilfredo Molina/Collin Kidder 2013-2015
//  Modified by Al Thomason for ATmegaxxM1 avr_CAN demo 2016


#include <avr_can.h>


class ExampleClass : public CANListener //CANListener provides an interface to get callbacks on this class
{
  public:
  void printFrame(CAN_FRAME *frame, int mailbox);
  void gotFrame(CAN_FRAME *frame, int mailbox); //overrides the parent version so we can actually do something
};

//Prints out the most useful information about the incoming frame.
void ExampleClass::printFrame(CAN_FRAME *frame, int mailbox)
{
   Serial.print("MB: ");
   if (mailbox > -1) Serial.print(mailbox);
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

//Classes register just one method that receives all callbacks. If a frame didn't match any specific mailbox
//callback but the general callback was registered then the mailbox will be set as -1. Otherwise it is the mailbox
//with the matching filter for this frame.
void ExampleClass::gotFrame(CAN_FRAME* frame, int mailbox)
{
    this->printFrame(frame, mailbox);
}

ExampleClass myClass; //initialize the class global so the reference to it can be picked up anywhere

void setup()
{

  Serial.begin(115200);
  
   // Initialize CAN0, Set the proper baud rates here
   Can0.init(CAN_BPS_250K);
   Can0.setNumTXBoxes(0);                                               // Use all MOb (only 6x on the ATmegaxxM1 series) for receiving.
 

  //extended
  Can0.setRXFilter(0, 0x2FF00, 0x1FF2FF00, true);
  Can0.setRXFilter(1, 0x1F0000, 0x1F1F0000, true);
  Can0.setRXFilter(2, 0, 0, true);                                      //catch all mailbox
  
  //standard  
  Can0.setRXFilter(3, 0x40F, 0x7FF, false);
  Can0.setRXFilter(4, 0x310, 0x7F0, false);
  Can0.setRXFilter(5, 0, 0, false);                                     //catch all mailbox
  
  Can0.attachObj(&myClass);
  //let library know we want to receive callbacks for the following mailboxes
  //once we attach above the canbus object knows about us. The actual functions
  //to attach are members of CANListener so use your class name
  myClass.attachMBHandler(0);
  myClass.attachMBHandler(1);
  myClass.attachMBHandler(3);
  myClass.attachMBHandler(4);

  
  //set to get a callback for any other mailboxes not already covered above
  myClass.attachGeneralHandler();
}

void loop(){ //All work is done via callback as frames come in - no need to poll for them. SO, just print periodic message to show we're alive
	delay(5000);
	Serial.println("Still listening");
}



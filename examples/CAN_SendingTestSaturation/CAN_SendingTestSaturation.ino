//Sends out a sample CAN message as fast as can, exersizes Tx buffers 
//  Modified by Al Thomason for ATmegaxxM1 avr_CAN demo 2016

#include <avr_can.h>

void setup()
{

  Serial.begin(115200);
  
  // Initialize CAN0 -  Set the proper baud rate here
  
    if (Can0.begin(CAN_BPS_250K)) 
          Serial.println("Can Init OK");
    else  Serial.println("Can Init FAILED");

    
     Serial.print(Can0.setNumTXBoxes(99));              // Setup all the MObs for transmitting. 
     Serial.println(" Tx mailboxes setup.");
     
     
}

bool sendData()
{
    CAN_FRAME outgoing;
    outgoing.id = 0x400;
    outgoing.extended = false;
    outgoing.priority = 4;                              //0-15 lower is higher priority
    outgoing.length   = 8;
    
    outgoing.data.s0 = 0xFEED;
    outgoing.data.byte[2] = 0xDD;
    outgoing.data.byte[3] = 0x55;
    outgoing.data.high = millis();                      // Show something changing on the receiving end.
    return(Can0.sendFrame(outgoing));
}

void loop(){
  while (true)  {                                       // Don't even go back into Arduino IDE, just fling them out as fast as one can
  
    if (!sendData()) {                                  // Transmission request    
        Serial.print(" Retrying");                      // failed - perhaps Tx buffers are full, try delaying just a bit
        if (sendData())                                 // Then try again before really giving up.
            Serial.println(" - OK");
        else
            Serial.println(" - Failed");
    }
  }                                             
}


//Sends a sample CAN message out. 
//  Modified by Al Thomason for ATmegaxxM1 avr_CAN demo 2016

#include <avr_can.h>

void setup()
{

  Serial.begin(115200);
  
  // Initialize CAN0 Set the proper baud rates here
  
    if (Can0.begin(CAN_BPS_250K)) 
          Serial.println("Can Init OK");
    else  Serial.println("Can Init FAILED");

}

bool sendData()
{
    CAN_FRAME outgoing;
    outgoing.id = 0x400;
    outgoing.extended = false;
    outgoing.priority = 4;                            //0-15 lower is higher priority
    outgoing.length   = 8;
    
    outgoing.data.s0 = 0xFEED;
    outgoing.data.byte[2] = 0xDD;
    outgoing.data.byte[3] = 0x55;
    outgoing.data.high = millis();                  // Include a little changing data in each packet.
    return(Can0.sendFrame(outgoing));               // Take note:  sendFrame() really only places the message into the outgoing queue,
                                                    //             and will return TRUE if is succeeds.  This does NOT mean that the 
                                                    //             message was actually  successfully sent at a later time.
}

void loop(){
  static unsigned long lastTime = 0;

  if ((millis() - lastTime) > 500)                  // Send something out every 500mS
  {
     lastTime = millis();
          
     Serial.print((int) (millis()/1000));
     Serial.print(" - Message sent");   

    if (sendData())
        Serial.println(" - OK");
      else
        Serial.println(" - Failed");
  }
}



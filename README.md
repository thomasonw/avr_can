# avr_can
Object oriented canbus library for Atmel AVR CPUs used with the Arduino IDE.

https://github.com/thomasonw/avr_can



This library supports the built in CAN hardware in some Atmet AVR chips.  It was ported from the existing due_can library and follows those APIs.  In this way applications within the Arduino IDE can be more universally written changing only the underlying library.

Example CPU chips which are supported include:
* ATmega16M1, ATmega32M1, ATmega64M1
* ATmega32C1, ATmega64C1
* AT90CAN32, AT90CAN64, AT90CAN128

Note:  To date only limited testing has been performed using the ATmega64M1 controller.

Reference:  This is a porting of the CAN_DUE library at: 
  https://github.com/collin80/due_can
  
  
  =============================================
  
  
NOTE:  The current version of this lib has been made generic by removing all Arduino specific code.  As a result, this lib no longer manages the Transceiver Enable/Stby port, you will need to do that directly (if used).  

  A snap-shot of the Arduino version with the Enable/stby pin support may be found under the RELEASED tab.



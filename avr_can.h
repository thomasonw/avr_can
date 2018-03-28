
/*
  Copyright (c) 2013 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  
  Modifed by Al Thomason to support CAN enabled AVR CPUs:
    ATmegaxxM1, ATmegaxxC, AT90CANxxx  
    See https://github.com/thomasonw/avr_can

  Based off of due_can library:  https://github.com/collin80/due_can
  
  
  
  
  !!!!!!!!!!!!!!!
  Add something to communicate back the actual number of physical mailboxes?
  Or do we expect end user to just keep pooling for one, and when they get an out-of bounds error, that is how they will know???
  Due has 8,  ATmega has 6, AT90CAN as 15. . . . .
  
  What to do about all these functions which are not supported on the AVR CAN?  e.g., Low-power mode, and auto-repeate disable in arbit loss?
  
  
    
*/

#ifndef _CAN_LIBRARY_
#define _CAN_LIBRARY_

#include <avr/pgmspace.h>

#define CAN		Can0
#define NULL    0

/**  Some CPU specific selections, device at compile time based on targeted CPU  */
/** How many mailboxes are contained in the hardware?  (due had 8x) */ 
#ifdef    CANMB_QUANTITY
   #undef CANMB_QUANTITY
   #endif
#ifdef    CANMB_MASK
   #undef CANMB_MASK
   #endif

 

#if defined(__AVR_AT90CAN32__) || \
    defined(__AVR_AT90CAN64__) || \
    defined(__AVR_AT90CAN128__)
        #define  CANMB_QUANTITY     15                       // AT90CAN's contain 15 mailboxes
        #define  CANMB_MASK       0x0F
        
#elif defined(__AVR_ATmega32C1__) || \
      defined(__AVR_ATmega64C1__) || \
      defined(__AVR_ATmega16M1__) || \
      defined(__AVR_ATmega32M1__) || \
      defined(__AVR_ATmega64M1__)
        #define  CANMB_QUANTITY     6                       // ATmegaxxM1's contain 6 mailboxes
        #define  CANMB_MASK       0x07
#else
    
        #error Unsupported CPU in avr_can.h
#endif 


    
   
   // ----------
                //!< STD ID TAG Reading
#define Can_get_std_id(identifier)  { *((uint8_t *)(&(identifier))+3) = 0                        ; \
                                      *((uint8_t *)(&(identifier))+2) = 0                        ; \
                                      *((uint8_t *)(&(identifier))+1) =  CANIDT1>>5              ; \
                                      *((uint8_t *)(&(identifier))  ) = (CANIDT2>>5)+(CANIDT1<<3); }
    // ----------
                //!< EXT ID TAG Reading
#define Can_get_ext_id(identifier)  { *((uint8_t *)(&(identifier))+3) =  CANIDT1>>3              ; \
                                      *((uint8_t *)(&(identifier))+2) = (CANIDT2>>3)+(CANIDT1<<5); \
                                      *((uint8_t *)(&(identifier))+1) = (CANIDT3>>3)+(CANIDT2<<5); \
                                      *((uint8_t *)(&(identifier))  ) = (CANIDT4>>3)+(CANIDT3<<5); }
    // ----------
                //!< STD ID Construction
#define CAN_SET_STD_ID_10_4(identifier)  (((*((uint8_t *)(&(identifier))+1))<<5)+((* (uint8_t *)(&(identifier)))>>3))
#define CAN_SET_STD_ID_3_0( identifier)  (( * (uint8_t *)(&(identifier))   )<<5)
    // ----------
                //!< STD ID TAG writing
#define Can_set_std_id(identifier)  { CANIDT1   = CAN_SET_STD_ID_10_4(identifier); \
                                      CANIDT2   = CAN_SET_STD_ID_3_0( identifier); \
                                      CANIDT3   = 0;                               \  
                                      CANIDT4   = 0;                               \  
                                      CANCDMOB &= (~(1<<IDE));                     }     // Make sure to initialize the RTRTAG & RB0TAG bits (CANIDT4)
    // ----------
                //!< STD ID MASK writing
#define Can_set_std_msk(mask)       { CANIDM1   = CAN_SET_STD_ID_10_4(mask);      \
                                      CANIDM2   = CAN_SET_STD_ID_3_0( mask);      \
                                      CANIDM3   = 0;                              \
                                      CANIDM4   = 0;}                                   // Make sure to initialize the RTRMSK & IDEMSK bits
    // ----------
                //!< EXT ID Construction
#define CAN_SET_EXT_ID_28_21(identifier)  (((*((uint8_t *)(&(identifier))+3))<<3)+((*((uint8_t *)(&(identifier))+2))>>5))
#define CAN_SET_EXT_ID_20_13(identifier)  (((*((uint8_t *)(&(identifier))+2))<<3)+((*((uint8_t *)(&(identifier))+1))>>5))
#define CAN_SET_EXT_ID_12_5( identifier)  (((*((uint8_t *)(&(identifier))+1))<<3)+((* (uint8_t *)(&(identifier))   )>>5))
#define CAN_SET_EXT_ID_4_0(  identifier)   ((* (uint8_t *)(&(identifier))   )<<3)
    // ----------
                //!< EXT ID TAG writing
#define Can_set_ext_id(identifier)  { CANIDT1   = CAN_SET_EXT_ID_28_21(identifier); \
                                      CANIDT2   = CAN_SET_EXT_ID_20_13(identifier); \
                                      CANIDT3   = CAN_SET_EXT_ID_12_5( identifier); \
                                      CANIDT4   = CAN_SET_EXT_ID_4_0(  identifier); \
                                      CANCDMOB |= (1<<IDE);                         }
                                      
                                      
    // ----------
                //!< EXT ID MASK writing
#define Can_set_ext_msk(mask)       { CANIDM1   = CAN_SET_EXT_ID_28_21(mask); \
                                      CANIDM2   = CAN_SET_EXT_ID_20_13(mask); \
                                      CANIDM3   = CAN_SET_EXT_ID_12_5( mask); \
                                      CANIDM4   = CAN_SET_EXT_ID_4_0(  mask) + (1<<IDEMSK); }
 
 
 
#define MOB_Tx_ENA  1
#define MOB_Rx_ENA  2
#define MOB_Rx_BENA 3

 /** Define CAN mailbox transfer status code. */
#define CAN_MAILBOX_TRANSFER_OK       0     //! Read from or write into mailbox successfully.
#define CAN_MAILBOX_NOT_READY         0x01  //! Receiver is empty or transmitter is busy.
#define CAN_MAILBOX_RX_OVER           0x02  //! Message overwriting happens or there're messages lost in different receive modes.
#define CAN_MAILBOX_RX_NEED_RD_AGAIN  0x04  //! Application needs to re-read the data register in Receive with Overwrite mode.


#define SIZE_RX_BUFFER	16 //RX incoming ring buffer is this big  (due had 32)
#define SIZE_TX_BUFFER	8  //TX ring buffer is this big           (due had 16)
#define SIZE_LISTENERS	4  //number of classes that can register as listeners with this class

	/** Define the timemark mask. */
#define TIMEMARK_MASK              0x0000ffff

/* CAN timeout for synchronization. */
#define CAN_TIMEOUT                100000



/** Define the typical baudrate for CAN communication in KHz. */

//#define CAN_BPS_5K                    -
//#define CAN_BPS_10K                   - 
//#define CAN_BPS_25K                   - 
//#define CAN_BPS_33333				    -
//#define CAN_BPS_50K                   - 
  #define CAN_BPS_100K                  0
  #define CAN_BPS_125K                  1
  #define CAN_BPS_200K                  2
  #define CAN_BPS_250K                  3
  #define CAN_BPS_500K                  4
//#define CAN_BPS_800K                  - 
  #define CAN_BPS_1000K                 5
  #define CAN_BPS_MAX       CAN_BPS_1000K 
  
#define CAN_DEFAULT_BAUD	CAN_BPS_250K

//  Note:  Am not really happy about how this set_baud_rate function is configured, with this large table..  
//         But it is quick-n-dirty for now.

typedef struct {
    uint32_t  baudrate;
	uint8_t   canbt1;                                 // Just hold the pre-calcualted register values.  See ATmegaxxM1 datasheet.
	uint8_t   canbt2; 
	uint8_t   canbt3;  
} can_bit_timing_t;



/** Values of bit time register for different baudrates and different CPU frequencies.  */

const uint8_t can_bit_time[CAN_BPS_MAX+1][3] PROGMEM  = {
    #if F_CPU == 16000000                 //< Fclkio = 16 MHz, Tclkio = 62.5 ns
        {0x12, 0x0C, 0x37},                //< -- 100Kb/s, 16x Tscl, sampling at 75%
        {0x0E, 0x0C, 0x37},                //< -- 125Kb/s, 16x Tscl, sampling at 75%
        {0X08, 0x0C, 0x37},                //< -- 200Kb/s, 16x Tscl, sampling at 75%
        {0x06, 0x0C, 0x37},                //< -- 250Kb/s, 16x Tscl, sampling at 75%
        {0x06, 0x04, 0x13},                //< -- 500Kb/s,  8x Tscl, sampling at 75%
        {0x02, 0x04, 0x13}                 //< --  1 Mb/s,  8x Tscl, sampling at 75%   
        
    #elif F_CPU == 12000000              //< Fclkio = 12 MHz, Tclkio = 83.333 ns
        {0x0A, 0x0E, 0x4B},                //< -- 100Kb/s, 16x Tscl, sampling at 75%
        {0x0A, 0x0C, 0x37},                //< -- 125Kb/s, 16x Tscl, sampling at 75%
        {0X04, 0x0E, 0x4B},                //< -- 200Kb/s, 16x Tscl, sampling at 75%
        {0x04, 0x0C, 0x37},                //< -- 250Kb/s, 16x Tscl, sampling at 75%
        {0x02, 0x08, 0x25},                //< -- 500Kb/s,  8x Tscl, sampling at 75%
        {0x00, 0x08, 0x25}                 //< --  1 Mb/s,  8x Tscl, sampling at 75%   
        
       #elif F_CPU == 8000000              //< Fclkio = 8 MHz, Tclkio = 125 ns
        {0x08, 0x0C, 0x37},                //< -- 100Kb/s, 16x Tscl, sampling at 75%
        {0x06, 0x0C, 0x37},                //< -- 125Kb/s, 16x Tscl, sampling at 75%
        {0X02, 0x0E, 0x4B},                //< -- 200Kb/s, 16x Tscl, sampling at 75%
        {0x02, 0x0C, 0x37},                //< -- 250Kb/s, 16x Tscl, sampling at 75%
        {0x02, 0x04, 0x13},                //< -- 500Kb/s,  8x Tscl, sampling at 75%
        {0x00, 0x04, 0x12}                 //< --  1 Mb/s,  8x Tscl, sampling at 75%   
        
    #else      
        #error CPU Frequency F_CPU value not supported in avr_can.h
    #endif
};


//This is architecture specific. DO NOT USE THIS UNION ON ANYTHING OTHER THAN THE ATMEL AVR - UNLESS YOU DOUBLE CHECK THINGS!
//  note:  This structure has the same format as the prior "CORTEX M3 / Arduino Due" order - tests as a match for 8-bit AVR CPUs...
//
typedef union {
    uint64_t value;
	struct {
		uint32_t low;
		uint32_t high;
	};
	struct {
		uint16_t s0;
		uint16_t s1;
		uint16_t s2;
		uint16_t s3;
    };
	uint8_t bytes[8];
	uint8_t byte[8]; //alternate name so you can omit the s if you feel it makes more sense
} BytesUnion;

typedef struct
{
	uint32_t id;		// EID if ide set, SID otherwise
	uint8_t  rtr;		// Remote Transmission Request
   	uint8_t  priority;	// Priority but only important for TX frames and then only for special uses.
	uint8_t  extended;	// Extended ID flag
    uint16_t time;      // CAN timer value when mailbox message was received.
	uint8_t  length;	// Number of data bytes
	BytesUnion data;	// 64 bits - lots of ways to access it.
} CAN_FRAME;

class CANListener
{
public:
	CANListener();
	
	virtual void gotFrame(CAN_FRAME *frame, int mailbox);

  	void attachMBHandler(uint8_t mailBox);
	void detachMBHandler(uint8_t mailBox);
	void attachGeneralHandler();
	void detachGeneralHandler();

    
private:
	int callbacksActive; //bitfield letting the code know which callbacks to actually try to use (for object oriented callbacks only)
	
	friend class CANRaw; //class has to have access to the the guts of this one 
    
    
};

class CANRaw
{
  protected:
	uint8_t numTXBoxes;  //Contains number of Mobs we have been asked to use for TX.  
	
  private:
	/* CAN peripheral, set by constructor */
	volatile CAN_FRAME rx_frame_buff[SIZE_RX_BUFFER];
	volatile CAN_FRAME tx_frame_buff[SIZE_TX_BUFFER];

	volatile uint8_t rx_buffer_head, rx_buffer_tail;
    volatile uint8_t tx_buffer_head, tx_buffer_tail;
    
	void mailbox_int_handler(uint8_t mb);

	uint8_t busSpeed;                                                   //what speed is the bus currently initialized at? 0 if it is off right now
	
	uint32_t write_id;                                                  //public storage for an id. Will be used by the write function to set which ID to send to.
	bool bigEndian; 
    
    uint32_t RXIDFilterSave[CANMB_QUANTITY];                            // CAN ID Mask registers are overwritten with incomming message IDs, need to save values to reinitialize

	void (*cbCANFrame[CANMB_QUANTITY+1])(CAN_FRAME *);                  //Call-Back function pointer array - max mailboxes plus an optional catch all
	CANListener *listener[SIZE_LISTENERS];	

    void mailbox_set_MOb_index(uint8_t uc_index);                       // Sets internal Mob pointer to uc_index
    
     
    void enable_interrupt(uint8_t mb);                                  // NOTE that the passed parm is different from Due port.
	void disable_interrupt(uint8_t mb);    
 

    
    
    
    
    

  public:

    // Constructor
    CANRaw();

    
    uint8_t begin();
	uint8_t begin(uint8_t ul_baudrate);
	uint8_t init (uint8_t ul_baudrate);

 	void enable(); 
	void disable();
    
	uint8_t set_baudrate(uint8_t ul_baudrate);
	uint8_t getBusSpeed();

	void reset_all_mailbox();

    uint8_t setNumTXBoxes(uint8_t txboxes);

	int watchFor();                                 //allow anything through
	int watchFor(uint32_t id);                      //allow just this ID through (automatic determination of extended status)
	int watchFor(uint32_t id, uint32_t mask);       //allow a range of ids through
	int watchForRange(uint32_t id1, uint32_t id2);  //try to allow the range from id1 to id2 - automatically determine base ID and mask
	int setRXFilter(uint32_t id, uint32_t mask, bool extended);
	int setRXFilter(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended);

	int findFreeRXMailbox();

	bool rx_avail();
	int available();                                                //like rx_avail but returns the number of waiting frames
	uint8_t get_rx_buff(CAN_FRAME &);
	uint8_t read(CAN_FRAME &);
	bool sendFrame(CAN_FRAME& txFrame);
    
 	uint8_t  get_tx_error_cnt();
	uint8_t  get_rx_error_cnt(); 
    uint16_t get_internal_timer_value();
    uint16_t get_timestamp_value();
    
    void disable_overload_frame();
	void enable_overload_frame();
    void disable_time_triggered_mode();
	void enable_time_triggered_mode();  
	void setBigEndian(bool);
    
	

	template <typename t> void write(t inputValue);                  //write a variable # of bytes out in a frame. Uses id as the ID.
	void setWriteID(uint32_t id);
    
    
	void setCallback(int mailbox, void (*cb)(CAN_FRAME *));         // Not sure why two names for the same capability, but it was in the origional due lib so leaving it...
	void setGeneralCallback(void (*cb)(CAN_FRAME *));
	//note that these below versions still use mailbox number. There isn't a good way around this. 
	void attachCANInterrupt(void (*cb)(CAN_FRAME *));                //alternative callname for setGeneralCallback
	void attachCANInterrupt(uint8_t mailBox, void (*cb)(CAN_FRAME *));
	void detachCANInterrupt(uint8_t mailBox);
	
	//now, object oriented versions to make OO projects easier
	bool attachObj(CANListener *listener);
	bool detachObj(CANListener *listener);

    
    void interruptHandler();
    
	
   
 


    
	//misc old cruft kept around just in case anyone actually used any of it in older code.
	//some are used within the functions above. Unless you really know of a good reason to use
	//any of these you probably should steer clear of them.
    
    
    void mailbox_set_id(uint8_t uc_index, uint32_t id, bool extended);
	void mailbox_set_accept_mask(uint8_t uc_index, uint32_t mask, bool ext);
    void mailbox_set_databyte(uint8_t uc_index, uint8_t bytepos, uint8_t val);
    
    uint32_t mailbox_get_id(uint8_t uc_index);
   

    void mailbox_send_abort_cmd(uint8_t uc_index);
    void set_timestamp_capture_point(uint32_t ul_flag);
    void mailbox_init(uint8_t uc_index); 

    uint8_t mailbox_tx_frame(uint8_t uc_index);
    uint8_t mailbox_read(uint8_t uc_index, volatile CAN_FRAME *rxframe);
     
       
       
       
       
       
    /*********************   NOT CARRIED FORWARD INTO AVR PORT **************************************************
                             THESE ARE MUCH TOO CHIP SPECIFIC, NO REAL WAY TO TRANSLATE
                             IF USED INTERNALY, HAVE BEEN ADJUSTED.
	void disable_low_power_mode();
	void enable_low_power_mode();
	void disable_autobaud_listen_mode();
	void enable_autobaud_listen_mode();
	
	void set_timestamp_capture_point(uint32_t ul_flag);
	void enable_time_triggered_mode();
	void disable_timer_freeze();
	void enable_timer_freeze();
	void disable_tx_repeat();
	void enable_tx_repeat();
	void set_rx_sync_stage(uint32_t ul_stage);
	void enable_interrupt(uint32_t dw_mask);
	void disable_interrupt(uint32_t dw_mask);
	uint32_t get_interrupt_mask();
	uint32_t get_status();
    
    uint8_t begin(uint8_t ul_baudrate, uint8_t enablePin);

	void reset_internal_timer();
    	uint32_t getMailboxIer(int8_t mailbox);
	void global_send_transfer_cmd(uint8_t uc_mask);
 // void global_send_abort_cmd(uint8_t uc_mask);                        // REDACTED
	void mailbox_set_timemark(uint8_t uc_index, uint16_t us_cnt);
 // int32_t mailbox_get_status(uint8_t uc_index);                       // REDACTED
	void mailbox_send_transfer_cmd(uint8_t uc_index);
	void mailbox_send_abort_cmd(uint8_t uc_index);
	void mailbox_init(uint8_t uc_index);
	void mailbox_set_id(uint8_t uc_index, uint32_t id, bool extended);
  // void mailbox_set_priority(uint8_t uc_index, uint8_t pri);          // REDACTED

	
	uint8_t  mailbox_get_mode(uint8_t uc_index);
	void mailbox_set_datalen(uint8_t uc_index, uint8_t dlen);
	void mailbox_set_datal(uint8_t uc_index, uint32_t val);
	void mailbox_set_datah(uint8_t uc_index, uint32_t val);
    
  
    
        void global_send_abort_cmd(uint8_t uc_mask);
        
      

   
   

    void mailbox_send_transfer_cmd(uint8_t uc_index);
	void mailbox_send_abort_cmd(uint8_t uc_index);
	void mailbox_init(uint8_t uc_index);

	
	void mailbox_set_databyte(uint8_t uc_index, uint8_t bytepos, uint8_t val);

	void mailbox_set_mode(uint8_t uc_index, uint8_t mode);
	void mailbox_set_datal(uint8_t uc_index, uint32_t val);
	void mailbox_set_datah(uint8_t uc_index, uint32_t val);
	void mailbox_set_timemark(uint8_t uc_index, uint16_t us_cnt);

        
	void enable_interrupt(uint32_t dw_mask);
	void disable_interrupt(uint32_t dw_mask);
    
    *********************   NOT CARRIED FORWARD INTO AVR PORT **************************************************/
    
};

extern CANRaw Can0;

#endif // _CAN_LIBRARY_

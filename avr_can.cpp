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
  
  
   // ??  No handeling of Timer Overflow at this point???  (See end of source)
   
*/


#include "avr_can.h"
#include <avr/interrupt.h>
#include <string.h>
  
    

/**
* \brief constructor for the class
*
*/
CANRaw::CANRaw() {
	bigEndian = false;
	busSpeed = 0;
	
	for (int i = 0; i < SIZE_LISTENERS; i++) listener[i] = NULL;
}

/**
 * \brief Configure CAN baudrate.
 *
 * \param baudrate Baudrate value (kB/s), allowed values:  enum - can_BPS_t
 *
 * \retval Set the baudrate successfully or not.
 */
uint8_t CANRaw::set_baudrate(uint8_t ub_baudrate)
{ 
    if (ub_baudrate > CAN_BPS_MAX)
        return 0;
    
    busSpeed = ub_baudrate;
    
    CANBT1 = pgm_read_byte(&can_bit_time[ub_baudrate][0]);                  
    CANBT2 = pgm_read_byte(&can_bit_time[ub_baudrate][1]);  
    CANBT3 = pgm_read_byte(&can_bit_time[ub_baudrate][2]);  
    return 1;
   
}


uint8_t CANRaw::begin()
{
	return init(CAN_DEFAULT_BAUD);
}

uint8_t CANRaw::begin(uint8_t ul_baudrate) 
{
	return init(ul_baudrate);
}

uint8_t CANRaw::getBusSpeed()
{
	return busSpeed;
}

/**
 * \brief Initialize CAN controller.
 *
 * \param ul_mck CAN module input clock.
 * \param ul_baudrate CAN communication baudrate in Hz.
 *
 * \retval 0 If failed to initialize the CAN module; otherwise successful.
 *
 * \note PMC clock for CAN peripheral should be enabled before calling this function.
 */
uint8_t CANRaw::init(uint8_t ub_baudrate)
{
	uint8_t  ub_flag;

	//there used to be code here to cause this function to not run if the hardware is already initialized. But,
	//it can be helpful to reinitialize. For instance, if the bus rate was set improperly you might need to
	//go back through this code to reset the hardware.

	//initialize all function pointers to null
	for (int i = 0; i < CANMB_QUANTITY+1; i++) cbCANFrame[i] = 0;


		/* Initialize the baudrate for CAN module. */
	ub_flag = set_baudrate(ub_baudrate);
	if (ub_flag == 0) {
		return 0;
	}

	/* Reset the CAN eight message mailbox. */
	reset_all_mailbox();

	//Also disable all interrupts by default
     CANIE2 = 0x00;                                                 // Disable  all individual MOb interupts
     CANIE1 = 0x00;
     CANGIE = 0x00;                                                 // As well as master CAN controller interupts.

	//By default use one mailbox for TX 
	setNumTXBoxes(1);

    /* Enable the CAN controller. */
	enable();
	return ub_flag;
	
}

 /* \brief Initializes mailboxes to the requested mix of RX and TX boxes
 *
 * \param txboxes How many of the available boxes should be used for TX
 *
 * \retval number of tx boxes set.
 *
 */
uint8_t CANRaw::setNumTXBoxes(uint8_t txboxes) {
	uint8_t c;

    c = txboxes;
	if (txboxes > CANMB_QUANTITY) c = CANMB_QUANTITY;
	if (txboxes < 0)              c = 0;
    
    numTXBoxes = c;
    
    
	//Inialize remaining Mob as RX boxes
	for (c = 0; c < CANMB_QUANTITY - numTXBoxes; c++) {
		mailbox_set_id(c, 0x0, false);
		mailbox_set_accept_mask(c, 0x7FF, false);
	}

	//Initialize TX boxes
	for (c = CANMB_QUANTITY - numTXBoxes; c < CANMB_QUANTITY; c++) {
		mailbox_set_accept_mask(c, 0x7FF, false);
	}
    
    return (numTXBoxes);
}

/**
 * \brief Set up a callback function for given mailbox
 *
 * \param mailbox Which mailbox (0-max) to assign callback to.
 * \param cb A function pointer to a function with prototype "void functionname(CAN_FRAME *frame);"
 *
 */
void CANRaw::setCallback(int mailbox, void (*cb)(CAN_FRAME *))
{
	if ((mailbox < 0) || (mailbox > CANMB_QUANTITY-1)) return;
	cbCANFrame[mailbox] = cb;
}

/**
 * \brief Set up a general callback that will be used if no callback was registered for receiving mailbox
 *
 * \param cb A function pointer to a function with prototype "void functionname(CAN_FRAME *frame);"
 *
 * \note If this function is used to set up a callback then no buffering of frames will ever take place.
 */
void CANRaw::setGeneralCallback(void (*cb)(CAN_FRAME *))
{
	cbCANFrame[CANMB_QUANTITY] = cb;
}

void CANRaw::attachCANInterrupt(void (*cb)(CAN_FRAME *)) 
{
	setGeneralCallback(cb);
}

void CANRaw::attachCANInterrupt(uint8_t mailBox, void (*cb)(CAN_FRAME *)) 
{
	setCallback(mailBox, cb);
}

void CANRaw::detachCANInterrupt(uint8_t mailBox)
{
	if ((mailBox < 0) || (mailBox > CANMB_QUANTITY-1)) return;
	cbCANFrame[mailBox] = 0;
}

bool CANRaw::attachObj(CANListener *listener)
{
	for (int i = 0; i < SIZE_LISTENERS; i++)
	{
		if (this->listener[i] == NULL)
		{
			this->listener[i] = listener;
			listener->callbacksActive = 0;
			return true;			
		}
	}
	return false;
}

bool CANRaw::detachObj(CANListener *listener)
{
	for (int i = 0; i < SIZE_LISTENERS; i++)
	{
		if (this->listener[i] == listener)
		{
			this->listener[i] = NULL;			
			return true;			
		}
	}
	return false;  
}


/**
 * \brief Enable CAN Controller.
 *
 */
void CANRaw::enable()
{
	CANGCON |=  (1<<ENASTB);                                //Start up the CAN module
}

/**
 * \brief Disable CAN Controller.
 *
 */
void CANRaw::disable()
{                                                           //review --> Should I also set the ABRQ bit?
	CANGCON &= ~(1<<ENASTB);                                //was --> m_pCan->CAN_MR &= ~CAN_MR_CANEN;
}


/**
 * \brief CAN Controller won't generate overload frame.
 *
 */
void CANRaw::disable_overload_frame()
{                                                           //review --> AVR may not have same mode as SAM chip.
	CANGCON &= ~(1<<OVRQ);                                  //was -->m_pCan->CAN_MR &= ~CAN_MR_OVL;
}

/**
 * \brief CAN Controller will generate an overload frame after each successful
 * reception for mailboxes configured in Receive mode, Producer and Consumer.
 *
 */
void CANRaw::enable_overload_frame()
{                                                           //review --> AVR may not have same mode as SAM chip.
	CANGCON |= (1<<OVRQ);                                   //was -->m_pCan->CAN_MR |= CAN_MR_OVL;
}

/**
 * \brief Configure the timestamp capture point, at the start or the end of frame.
 *
 * \param m_pCan   Pointer to a CAN peripheral instance.
 * \param ul_flag 0: Timestamp is captured at each start of frame;
 *                1: Timestamp is captured at each end of frame.
 */
void CANRaw::set_timestamp_capture_point(uint32_t ul_flag)
{
	if (ul_flag) {
	CANGCON |= (1<<SYNTTC);                                   //was -->	m_pCan->CAN_MR |= CAN_MR_TEOF;
	} else {
	CANGCON &= ~(1<<SYNTTC);                                  //was -->	m_pCan->CAN_MR &= ~CAN_MR_TEOF;
	}
}

/**
 * \brief Disable CAN Controller time triggered mode.
 *
 */
void CANRaw::disable_time_triggered_mode()
{
	CANGCON &= ~(1<<TTC);                                            //was -->	m_pCan->CAN_MR &= ~CAN_MR_TTM;
}

/**
 * \brief Enable CAN Controller time triggered mode.
 *
 */
void CANRaw::enable_time_triggered_mode()
{
	CANGCON |= (1<<TTC);                                         //was --> m_pCan->CAN_MR |= CAN_MR_TTM;
}


/**
 * \brief Enable CAN interrupt.
 *
 * \param mb  MOb Interrupt to be enabled.
 */
void CANRaw::enable_interrupt(uint8_t mb)
{          
     if (mb < 8)    CANIE2 |= (1<<mb);                           //was -->m_pCan->CAN_IDR = dw_mask;
        else        CANIE1 |= (1<<(mb-8));                                                 

     CANGIE = 0xFE;                                              // Enable all CAN interupts, except the Overrun..
                                                                 //  (But we will only service  the Tx/Rx ones)

}


/**
 * \brief Disable CAN interrupt.
 *
 * \param mb  MOb Interrupt to be disabled.
 */
void CANRaw::disable_interrupt(uint8_t mb)
{                                                           
     if (mb < 8)    CANIE2 &= ~(1<<mb);                           //was -->m_pCan->CAN_IDR = dw_mask;
        else        CANIE1 &= ~(1<<(mb-8));
     
     if((CANIE1 == 0) && (CANIE2 == 0))  CANGIE = 0x00;                      // If no MOb are enabled, turn off the CAN IRQs altogether.
     
}


/**
 * \brief Get the 16-bit free-running internal timer count.
 *
 *
 * \retval The internal CAN free-running timer counter.
 */
uint16_t CANRaw::get_internal_timer_value()
{
	   int t = CANTIML;                         //was --> return (m_pCan->CAN_TIM);
	   t += (CANTIMH<<8);                       // Always read LOW then HIGH values of 16bit registers from AVR CPUs.
       return(t);
}

/**
 * \brief Get CAN timestamp register value.
 *
 *
 * \retval The timestamp value.
 */
uint16_t CANRaw::get_timestamp_value()
{
       int t = CANTTCL;                         //was --> 	return (m_pCan->CAN_TIMESTP);
	   t += (CANTTCH<<8);                       // Always read LOW then HIGH values of 16bit registers from AVR CPUs.
       return(t);
}

/**
 * \brief Get CAN transmit error counter.
 *
 *
 * \retval Transmit error counter.
 */
uint8_t CANRaw::get_tx_error_cnt()
{
	return (uint8_t) (CANTEC);                              //was --> (m_pCan->CAN_ECR >> CAN_ECR_TEC_Pos);
}

/**
 * \brief Get CAN receive error counter.
 *
 *
 * \retval Receive error counter.
 */
uint8_t CANRaw::get_rx_error_cnt()
{
	return (uint8_t) (CANREC);                              //was --> (m_pCan->CAN_ECR >> CAN_ECR_REC_Pos);
}


/**
 * \brief Send single mailbox abort request.
 *
 * \param uc_index Indicate which mailbox is to be configured.
 */
void CANRaw::mailbox_send_abort_cmd(uint8_t uc_index)
{
	mailbox_set_MOb_index(uc_index);                                        // Select Mob, set data index = 0 w/auto increment of message reg pointer..
                                                                           //review --> AVR may not have same mode as SAM chip.
    CANCDMOB &= ~((1<<CONMOB1)|(1<<CONMOB0));                              //was -->  m_pCan->CAN_MB[uc_index].CAN_MCR |= CAN_MCR_MACR;
}

/**
 * \brief Initialize the mailbox to a default, known state.
 *
 * \param p_mailbox Pointer to a CAN mailbox instance.
 */
void CANRaw::mailbox_init(uint8_t uc_index)
{
	mailbox_set_MOb_index(uc_index);                                        // Select Mob, set data index = 0 w/auto increment of message reg pointer..

     CANCDMOB = 0;                                                      //was -->   m_pCan->CAN_MB[uc_index].CAN_MMR = 0;
     CANIDM1  = 0;                                                      //was --> 	m_pCan->CAN_MB[uc_index].CAN_MAM = 0;
     CANIDM2  = 0;
     CANIDM3  = 0;
     CANIDM4  = 0;
 
     for (uint8_t cnt = 0; cnt < 8; cnt++)                              // Need to send out 0 8x times to clear all of the data registers for the Mob
         CANMSG = 0;                                                    // Note Auto-increment of data pointer was set by mailbox_set_MOb_index
     
    CANSTMOB  = 0;                                                      //was --> 	m_pCan->CAN_MB[uc_index].CAN_MCR = 0;
    
    
}
/**
* \brief - Internal use: sets the MOB register pointer to the correct instance.
 *          Also initializes data index to 0 with auto increment.
 *
 * \param p_mailbox Pointer to a CAN mailbox instance.
 */
void CANRaw::mailbox_set_MOb_index(uint8_t uc_index)
{
    
 if (uc_index > (CANMB_QUANTITY-1)) uc_index = CANMB_QUANTITY-1;
    CANPAGE = (uc_index & CANMB_MASK)  << 4;                            // Select Mob, set data index = 0 w/auto increment of message reg pointer..

   
}




/**
 * \brief Reset the mailboxes.
 *
 * \param m_pCan Pointer to a CAN peripheral instance.
 */
void CANRaw::reset_all_mailbox()
{
	for (uint8_t i = 0; i < CANMB_QUANTITY; i++) {		
		mailbox_init(i);
	}
    
}

void CANRaw::setBigEndian(bool end)  
{
	bigEndian = end;
}

void CANRaw::setWriteID(uint32_t id)
{
	write_id = id;
}

template <typename t> void CANRaw::write(t inputValue)
{
	CAN_FRAME tempFrame;
	uint8_t *buff = (uint8_t *)inputValue;
	int thisSize = sizeof(t);
	if (thisSize > 8) thisSize = 8;
	if (!bigEndian) {
		for (int i = 0; i < thisSize; i++) 
		{
			tempFrame.data.bytes[i] = buff[i];
		}
	}
	else //reverse byte order. The M3 is in little endian so this causes big endian order
	{
		for (int i = 0; i < thisSize; i++) 
		{
			tempFrame.data.bytes[i] = buff[thisSize - i - 1];
		}
	}

	tempFrame.id = this->write_id;
	tempFrame.length = thisSize;
	if (this->write_id > 0x7FF) tempFrame.extended = true;
	else tempFrame.extended = false;
	sendFrame(tempFrame);
}

/**
 * \brief Send a frame out of this canbus port
 *
 * \param txFrame The filled out frame structure to use for sending
 *
 * \note Will do one of two things - 1. Send the given frame out of the first available mailbox
 * or 2. queue the frame for sending later via interrupt. Automatically turns on TX interrupt
 * if necessary.
 * 
 * Returns whether sending/queueing succeeded. Will not smash the queue if it gets full.
 */
bool CANRaw::sendFrame(CAN_FRAME& txFrame) 
{
	for (int i = (CANMB_QUANTITY - numTXBoxes); i < CANMB_QUANTITY; i++) {          // Search the Tx MObs, looking for one that is not currently busy.
       	   mailbox_set_MOb_index(i);                                                //    Select a Mob
              if ( (i < 8)   && !(CANEN2 & (1<<i))        ||                        //    1st 8 read-bits in CANEN2.  bit=1 = in use.
                  ((i >= 8)  && !(CANEN1 & (1<<(i-8)))))                            //    Next 8 in CANEN1
			{                                                                       //is it available (not sending anything?)
				mailbox_set_id(i, txFrame.id, txFrame.extended);
                CANCDMOB = (txFrame.length & 0x0F);                                 // Set the data length
                if (txFrame.extended) 
                    CANCDMOB |= 1<<IDE;                                             // And if it is a standard or extended frame.
				for (uint8_t cnt = 0; cnt < 8; cnt++)
				{    
					CANMSG = txFrame.data.bytes[cnt];                               // Push data out to MOb.  Datapointer will increment with each write to CANMSG reg
				}  
				enable_interrupt(i);                                                //enable the TX interrupt for this box
                return(mailbox_tx_frame(i) == CAN_MAILBOX_TRANSFER_OK);             //we've sent it. send back if it worked..


			}
    }
	
    //if execution got to this point then no free mailbox was found above
    //so, queue the frame if possible. But, don't increment the 
	//tail if it would smash into the head and kill the queue.
	uint8_t temp;
	temp = (tx_buffer_tail + 1) % SIZE_TX_BUFFER;
	if (temp == tx_buffer_head) return false;
    tx_frame_buff[tx_buffer_tail].id = txFrame.id;
    tx_frame_buff[tx_buffer_tail].extended = txFrame.extended;
    tx_frame_buff[tx_buffer_tail].length = txFrame.length;
    tx_frame_buff[tx_buffer_tail].data.value = txFrame.data.value;
    tx_buffer_tail = temp;
	return true;
}

  

/**
 * \brief Read a frame from out of the mailbox and into a software buffer
 *
 * \param uc_index which mailbox to read
 * \param rxframe Pointer to a receive frame structure which we'll fill out
 *
 * \retval Different CAN mailbox transfer status.
 *
 */
uint8_t CANRaw::mailbox_read(uint8_t uc_index, volatile CAN_FRAME *rxframe)
{
    uint32_t ul_id;
    
	mailbox_set_MOb_index(uc_index);                                        // Select Mob, set data index = 0 w/auto increment of message reg pointer..

    if (CANCDMOB & (1<<IDE)) {
		Can_get_ext_id(ul_id);
        rxframe->extended = true;
	}
	else {
		Can_get_std_id(ul_id);
        rxframe->extended = false;
	}
    rxframe->id = ul_id;
    rxframe->length = CANCDMOB & 0x0F;
    rxframe->time   = CANSTML;
    rxframe->time   += (CANSTMH<<8);                                     // Always read LOW then HIGH values of 16bit registers from AVR CPUs.
    
                 
    for (uint8_t cnt = 0; cnt < 8; cnt++)                                // Fetch out all 8 bytes.  CANMSG index will auto-advance with each read.
    {    
        rxframe->data.bytes[cnt] = CANMSG;
    } 

	return 0;
}

/**
 * \brief Sets the ID portion of the given mailbox
 *
 * \param uc_index The mailbox to set (0-7)
 * \param id The ID to set (11 or 29 bit)
 * \param extended bool indicating if this ID should be designated as extended
 *
 */
void CANRaw::mailbox_set_id(uint8_t uc_index, uint32_t id, bool extended) 
{
	mailbox_set_MOb_index(uc_index);                                        // Select Mob

	if (extended) {Can_set_ext_id(id);  }                               // Was-->  m_pCan->CAN_MB[uc_index].CAN_MID = id | CAN_MID_MIDE;
	else          {Can_set_std_id(id);  }                               // Was--> m_pCan->CAN_MB[uc_index].CAN_MID = CAN_MID_MIDvA(id);

    RXIDFilterSave[uc_index] = id;                                       // Remember value to allow resetting after MOd read.

}

/**
 * \brief Get ID currently associated with a given mailbox
 *
 * \param uc_index The mailbox to get the ID from (0-7)
 *
 * \retval The ID associated with the mailbox
 *
 */
uint32_t CANRaw::mailbox_get_id(uint8_t uc_index) {
	mailbox_set_MOb_index(uc_index);                                        // Select Mob, set data index = 0 w/auto increment of message reg pointer..
    
    uint32_t  ul_id;
    
	if (CANCDMOB & (1<<IDE)) {
		Can_get_ext_id(ul_id);
	}
	else {
		Can_get_std_id(ul_id);
	}
    
    
    return(ul_id);
}



/**
 * \brief Set mask for RX on the given mailbox
 *
 * \param uc_index The mailbox to use
 * \param mask The mask to set
 * \param ext Whether this should be an extended mask or not
 *
 */
void CANRaw::mailbox_set_accept_mask(uint8_t uc_index, uint32_t mask, bool extended)
{
	mailbox_set_MOb_index(uc_index);                                        // Select Mob, set data index = 0 w/auto increment of message reg pointer..

	if (extended) {Can_set_ext_msk(mask);  }  
	else          {Can_set_std_msk(mask);  }   

}



/**
 * \brief Command a mailbox to send the frame stored in it
 *
 * \param uc_index which mailbox to send frame. Load it up first
 *
 * \retval CAN_MAILBOX_NOT_READY: Failed because mailbox isn't ready for transmitting message.
 *         CAN_MAILBOX_TRANSFER_OK: Successfully send out a frame.
 */
uint8_t CANRaw::mailbox_tx_frame(uint8_t uc_index)
{
	mailbox_set_MOb_index(uc_index);                                        // Select Mob, set data index = 0 w/auto increment of message reg pointer..

    /* Read the mailbox status firstly to check whether the mailbox is ready or not. */
   if ( (uc_index < 8)   && (CANEN2 & (1<< uc_index))       ||              //    1st 8 read-bits in CANEN2,  next 8 in CANEN1.  bit=1 = in use.
       ((uc_index >= 8)  && (CANEN1 & (1<<(uc_index-8)))))      return CAN_MAILBOX_NOT_READY;  
	
    /* Set the MBx bit in the Transfer Command Register to send out the remote frame. */
    CANCDMOB &= ~((1<<CONMOB1)|(1<<CONMOB0));                               // Need to clear Mob status 1st 1st, 
    CANCDMOB |= (MOB_Tx_ENA  << CONMOB0);                                   // Then start it transmitting.
 
	return CAN_MAILBOX_TRANSFER_OK;
}



int CANRaw::available()
{
	int val;
	if (rx_avail()) 
	{ 	
        val = rx_buffer_head - rx_buffer_tail;
		//Now, because this is a cyclic buffer it is possible that the ordering was reversed
		//So, handle that case
		if (val < 0) val += SIZE_RX_BUFFER;
        return(val);
	}
	else return 0;
}


/**
* \brief Check whether there are received canbus frames in the buffer
*
* \retval true if there are frames waiting in buffer, otherwise false
*/
bool CANRaw::rx_avail() {
	return (rx_buffer_head != rx_buffer_tail)?true:false;
}

//wrapper for syntactic sugar reasons - could have just been a #define
uint8_t CANRaw::read(CAN_FRAME & buffer) 
{
	return get_rx_buff(buffer);
}

/**
 * \brief Retrieve a frame from the RX buffer
 *
 * \param buffer Reference to the frame structure to fill out
 *
 * \retval 0 no frames waiting to be received, 1 if a frame was returned
 */
uint8_t CANRaw::get_rx_buff(CAN_FRAME& buffer) {
	if (rx_buffer_head == rx_buffer_tail) return 0;
	buffer.id = rx_frame_buff[rx_buffer_tail].id;
	buffer.extended = rx_frame_buff[rx_buffer_tail].extended;
	buffer.length = rx_frame_buff[rx_buffer_tail].length;
	buffer.data.value = rx_frame_buff[rx_buffer_tail].data.value;
	rx_buffer_tail = (rx_buffer_tail + 1) % SIZE_RX_BUFFER;
	return 1;
}

/**
* \brief Handle all interrupt reasons
*/
void CANRaw::interruptHandler() {

	uint16_t ul_status;
    int i;
 
	ul_status = CANSIT2;                                //get status of MOb interrupts
	ul_status += (CANSIT1 << 8);                        //read 16bit regs low and then high in AVR CPUs.
    
    for (i=0; i<CANMB_QUANTITY; i++) {                      // Run through all Mailbox interupts looking for a bit that is set.
        if (ul_status & 0x01<<i)                            // If the mailbox has its IRQ bit set,
             mailbox_int_handler(i);                        //  . . . call the handler
    }

   
    /**********   Origional Due code follows - looks like they ignore any other error type outside of mail-boxes..
    if (ul_status & CAN_SR_ERRA) { //error active
	}
	if (ul_status & CAN_SR_WARN) { //warning limit
	}
	if (ul_status & CAN_SR_ERRP) { //error passive
	}
	if (ul_status & CAN_SR_BOFF) { //bus off
	}
	if (ul_status & CAN_SR_SLEEP) { //controller in sleep mode
	}
	if (ul_status & CAN_SR_WAKEUP) { //controller woke up
	}
	if (ul_status & CAN_SR_TOVF) { //timer overflow
	}
	if (ul_status & CAN_SR_TSTP) { //timestamp - start or end of frame
	}
	if (ul_status & CAN_SR_CERR) { //CRC error in mailbox
	}
	if (ul_status & CAN_SR_SERR) { //stuffing error in mailbox
	}
	if (ul_status & CAN_SR_AERR) { //ack error
	}
	if (ul_status & CAN_SR_FERR) { //form error
	} 
	if (ul_status & CAN_SR_BERR) { //bit error
	}  
    **************************  Will do the same ******************/
    
    CANGIT = 0xFF;                      // And clear all system level IRQ flags, even if the errors were not serviced...
 
}

/**
* \brief Find unused RX mailbox and return its number
*/
int CANRaw::findFreeRXMailbox() {
	for (int c = 0; c < CANMB_QUANTITY; c++) {
        mailbox_set_MOb_index(c);                                            // Select Mob,
		if ((CANCDMOB & (3<<CONMOB0)) == 0) {                                // Is this MOb disabled?
				return c;
		}
	}
	return -1;
}

/**
* \brief Set up an RX mailbox (first free) for the given parameters.
*
* \param id - the post mask ID to match against
* \param mask - the mask to use for this filter
* \param extended - whether to use 29 bit filter
*
* \ret number of mailbox we just used (or -1 if there are no free boxes to use)
*/
int CANRaw::setRXFilter(uint32_t id, uint32_t mask, bool extended) {
	int c = findFreeRXMailbox();
	if (c < 0) return -1;

    return (setRXFilter(c, id, mask, extended));

}

/**
* \brief Set up an RX mailbox (given MB number) filter
*

* \param mailbox Which mailbox to use (0-7)
* \param id The ID to match against
* \param mask The mask to apply before ID matching
* \param extended Whether this should be extended mask or not
*
* \retval Mailbox number if successful or -1 on failure
*/
int CANRaw::setRXFilter(uint8_t mailbox, uint32_t id, uint32_t mask, bool extended) {
	if (mailbox > (CANMB_QUANTITY-1 - numTXBoxes)) return -1;           // Is this a valid Rx MOb?

	mailbox_set_MOb_index(mailbox);                                     // Select Mob
    CANCDMOB &= ~((1<<CONMOB1)|(1<<CONMOB0));                           // Disable this MOb while we reconfigure it.
 
	mailbox_set_accept_mask(mailbox, mask, extended);
    mailbox_set_id(mailbox, id, extended);
 	enable_interrupt(mailbox);
    
    /* Set the MBx bit in the Transfer Command Register to start receiving. */
    CANCDMOB |= (MOB_Rx_ENA  << CONMOB0);              // Then start it receiving.
    
	return mailbox;
}

//set up to allow everything through. Sets up two mailboxes and returns the first one
//Not a terribly good idea to call because it totally ignores most everything and just
//quickly sets things up for all in. But, it's perfect for the novice getting started.
int CANRaw::watchFor() 
{
	int retVal = setRXFilter(0, 0, 0, false);
	             setRXFilter(1, 0, 0, true);

	return retVal;
}

//Let a single frame ID through. Automatic determination of extended. Also automatically sets mask
int CANRaw::watchFor(uint32_t id)
{
	if (id > 0x7FF) return setRXFilter(id, 0x1FFFFFFF, true);
	else return setRXFilter(id, 0x7FF, false);
}

//Allow a range of IDs through based on mask. Still auto determines extended.
int CANRaw::watchFor(uint32_t id, uint32_t mask)
{
	if (id > 0x7FF) return setRXFilter(id, mask, true);
	else return setRXFilter(id, mask, false);
}

//A bit more complicated. Makes sure that the range from id1 to id2 is let through. This might open
//the floodgates if you aren't careful.
//There are undoubtedly better ways to calculate the proper values for the filter but this way seems to work.
//It'll be kind of slow if you try to let a huge span through though.
int CANRaw::watchForRange(uint32_t id1, uint32_t id2)
{
	uint32_t id = 0;
	uint32_t mask = 0;
	uint32_t temp;

	if (id1 > id2) 
	{   //looks funny I know. In place swap with no temporary storage. Neato!
		id1 = id1 ^ id2;
		id2 = id1 ^ id2; //note difference here.
		id1 = id1 ^ id2;
	}

	id = id1;

	if (id2 <= 0x7FF) mask = 0x7FF;
	else mask = 0x1FFFFFFF;

	/* Here is a quick overview of the theory behind these calculations.
	   We start with mask set to 11 or 29 set bits (all 1's)
	   and id set to the lowest ID in the range.
	   From there we go through every single ID possible in the range. For each ID
	   we AND with the current ID. At the end only bits that never changed and were 1's
	   will still be 1's. This yields the ID we can match against to let these frames through
	   The mask is calculated by finding the bitfield difference between the lowest ID and
	   the current ID. This calculation will be 1 anywhere the bits were different. We invert
	   this so that it is 1 anywhere the bits where the same. Then we AND with the current Mask.
	   At the end the mask will be 1 anywhere the bits never changed. This is the perfect mask.
	*/
    
             
	for (uint32_t c = id1; c <= id2; c++)
	{
		id &= c;
		temp = (~(id1 ^ c)) & 0x1FFFFFFF;
		mask &= temp;
	}
    
                 
	//output of the above crazy loop is actually the end result.
	if (id > 0x7FF) return setRXFilter(id, mask, true);
	else return setRXFilter(id, mask, false);
}


/**
* \brief Handle a mailbox interrupt event
* \param mb which mailbox generated this event
* 
*/
void CANRaw::mailbox_int_handler(uint8_t mb) {
    
	CAN_FRAME tempFrame;
	bool caughtFrame = false;
	CANListener *thisListener;
	if (mb > (CANMB_QUANTITY-1)) mb = (CANMB_QUANTITY-1);

   
    mailbox_set_MOb_index(mb);                                                 // Select Mob, set data index = 0 w/auto increment of message reg pointer..
                                
    if (CANSTMOB & (1<<RXOK)) {                                              // Here bacuase of an Receive interupt?
           	mailbox_read(mb, &tempFrame);                                     // Yes, so go get it!

              // Reset this MOb to receive another message.
            mailbox_set_id(mb, RXIDFilterSave[mb],(CANCDMOB & (1<<IDE)));     // Restore the ID filter, with extended/standard flag.
            CANSTMOB &= ~(1<<RXOK);                                           // Clear the Rx interupt flag
            CANCDMOB &= ~((1<<CONMOB1)|(1<<CONMOB0));                         // And reset the command register to re-enable Rx.  Clear status 1st..
            CANCDMOB |= (MOB_Rx_ENA  << CONMOB0);                             // Then start it receiving.
 
 
             // Now that we have the frames data, lets see if anything special needs to happen.
			// First, if so configured - invoke the callback. If no callback registered then buffer the frame.
			if (cbCANFrame[mb])                                             // Specific call-back assigned to this MOb?
			{
				caughtFrame = true;
                (*cbCANFrame[mb])(&tempFrame);
 			}
			else if (cbCANFrame[CANMB_QUANTITY])                            // How about a 'catch-all' call back?
			{
				caughtFrame = true;
				(*cbCANFrame[CANMB_QUANTITY])(&tempFrame);
                 
			}
			else
			{
				for (int listenerPos = 0; listenerPos < SIZE_LISTENERS; listenerPos++)
				{
					thisListener = listener[listenerPos];
					if (thisListener != NULL)
					{
						if (thisListener->callbacksActive & (1 << mb)) 
						{
							caughtFrame = true;
							thisListener->gotFrame(&tempFrame, mb);
						}
						else if (thisListener->callbacksActive & 256) 
						{
							caughtFrame = true;
							thisListener->gotFrame(&tempFrame, -1);
						}
					}
				}
			}
			if (!caughtFrame) //if none of the callback types caught this frame then queue it in the buffer
			{
				uint8_t temp = (rx_buffer_head + 1) % SIZE_RX_BUFFER;
				if (temp != rx_buffer_tail) 
				{  
                    memcpy((void *)&rx_frame_buff[rx_buffer_head], &tempFrame, sizeof(CAN_FRAME));
					rx_buffer_head = temp;
				}
                   
			}
                         
    } else if (CANSTMOB & (1<<TXOK)) {                                                      // Something just transmitted.
               CANSTMOB &= ~(1<<TXOK);                                                       // Clear the Tx interupt flag
               CANCDMOB = 0;  								    //   ... and the controller reg.
         	if (tx_buffer_head != tx_buffer_tail) 
			{ //if there is a frame in the queue to send - refill this now empty MOb and start sending.
				mailbox_set_id(mb, tx_frame_buff[tx_buffer_head].id, tx_frame_buff[tx_buffer_head].extended);
                CANCDMOB = (tx_frame_buff[tx_buffer_head].length & 0x0F);
                if (tx_frame_buff[tx_buffer_head].extended)
                    CANCDMOB |= 1<<IDE;
				for (uint8_t cnt = 0; cnt < 8; cnt++)
				{    
					CANMSG = tx_frame_buff[tx_buffer_head].data.bytes[cnt];
				}       
				enable_interrupt(mb);                                                        //enable the TX interrupt for this MOb
				mailbox_tx_frame(mb);
				tx_buffer_head = (tx_buffer_head + 1) % SIZE_TX_BUFFER;
			}
			else {
				disable_interrupt(mb);                                                      // We are done with this MOb for now.
			}
    } else { 
                                                                                            // Some type of error in the MOb,
     //!       disable_interrupt(mb);                                                          // Due API does not report out errors, so just clear it here and free the MOb
       //!     CANSTMOB &= ~((1<<AERR)|(1<<FERR)|(1<<CERR)|(1<<SERR)|(1<<BERR)|(1<<DLCW));     // Clear all the 'other' interupt bits that we are ignoring.
      //!      CANCDMOB &= ~((1<<CONMOB1)|(1<<CONMOB0));                                       // And reset the command register to unused (idle) 
            uint8_t bCANCDMOB = CANCDMOB;                                                   //! Reset the Mob to continue on as it had before.
            CANCDMOB = 0;                                                                   //! Need to reset MOb to idle 1st
            CANSTMOB &= ~((1<<AERR)|(1<<FERR)|(1<<CERR)|(1<<SERR)|(1<<BERR)|(1<<DLCW));     //! Clear all the 'other' interupt bits that we are ignoring.
            CANCDMOB = bCANCDMOB;                                                           //! Now restart it as it was before.

    }

}

CANListener::CANListener()
{
	callbacksActive = 0; //none. Bitfield were bits 0-7 are the mailboxes and bit 8 is the general callback
}

//an empty version so that the linker doesn't complain that no implementation exists.
void CANListener::gotFrame(CAN_FRAME *frame, int mailbox)
{
  
}

void CANListener::attachMBHandler(uint8_t mailBox)
{
	if (mailBox >= 0 && mailBox < CANMB_QUANTITY)
	{
		callbacksActive |= (1<<mailBox);
	}
}

void CANListener::detachMBHandler(uint8_t mailBox)
{
	if (mailBox >= 0 && mailBox < CANMB_QUANTITY)
	{
		callbacksActive &= ~(1<<mailBox);
	}  
}

void CANListener::attachGeneralHandler()
{
	callbacksActive |= 256;
}

void CANListener::detachGeneralHandler()
{
	callbacksActive &= ~256;
}






/**
 * \brief Interrupt dispatcher - Never directly call these
 *
 * \note These function are needed because interrupt handlers cannot be part of a class
 */

#if defined(__AVR_AT90CAN32__) || \
    defined(__AVR_AT90CAN64__) || \
    defined(__AVR_AT90CAN128__)
	ISR(CANIT_vect)                                             // AT90CAN has slight delta in defs   
#else
	ISR(CAN_INT_vect)
#endif   
{
    uint8_t savedMOB;                                           // Save and restore the index value, in case interupt occured during an ongoing operation..

    savedMOB = CANPAGE;
    Can0.interruptHandler();
    CANPAGE = savedMOB;

} 


#if defined(__AVR_AT90CAN32__) || \
    defined(__AVR_AT90CAN64__) || \
    defined(__AVR_AT90CAN128__)
	ISR(OVRIT_vect)                                             // AT90CAN has slight delta in defs   
#else
	ISR(CAN_TOVF_vect)
#endif
{                                                               // Should never get here, as Overlow interupts are not enabled - -
        CANGIT  |= (1<<OVRTIM);                                 // But if we do, simply ignore this error and reset things.
}



/// instantiate the canbus adapter
CANRaw Can0;

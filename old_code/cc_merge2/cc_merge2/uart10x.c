/*************************************************************************
Title:    Interrupt UART library with receive/transmit circular buffers
Author:   Peter Fleury <pfleury@gmx.ch>   http://jump.to/fleury
File:     $Id: uart.c,v 1.6.2.1 2007/07/01 11:14:38 peter Exp $
Software: AVR-GCC 4.1, AVR Libc 1.4.6 or higher
Hardware: any AVR with built-in UART, 
License:  GNU General Public License 
          
DESCRIPTION:
    An interrupt is generated when the UART has finished transmitting or
    receiving a byte. The interrupt handling routines use circular buffers
    for buffering received and transmitted data.
    
    The UART_RX_BUFFER_SIZE and UART_TX_BUFFER_SIZE variables define
    the buffer size in bytes. Note that these variables must be a 
    power of 2.
    
USAGE:
    Refere to the header file uart.h for a description of the routines. 
    See also example test_uart.c.

NOTES:
    Based on Atmel Application Note AVR306
                    
LICENSE:
    Copyright (C) 2006 Peter Fleury

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
	

MODIFICATIONS:
	Aug 22 2011 - Code added and functions modified for dual serial ports - AK
                        
*************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/eeprom.h>
#include "uart.h"



/*
 *  constants and macros
 */
/* size of RX/TX buffers */



#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1)
#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1)

#define UART1_RX_BUFFER_MASK ( UART1_RX_BUFFER_SIZE - 1)
#define UART1_TX_BUFFER_MASK ( UART1_TX_BUFFER_SIZE - 1)

#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
#error RX buffer size is not a power of 2
#endif
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
#error TX buffer size is not a power of 2
#endif

#if ( UART1_RX_BUFFER_SIZE & UART1_RX_BUFFER_MASK )
#error RX1 buffer size is not a power of 2
#endif
#if ( UART1_TX_BUFFER_SIZE & UART1_TX_BUFFER_MASK )
#error TX1 buffer size is not a power of 2
#endif

#if defined(__AVR_AT90S2313__) \
 || defined(__AVR_AT90S4414__) || defined(__AVR_AT90S4434__) \
 || defined(__AVR_AT90S8515__) || defined(__AVR_AT90S8535__) \
 || defined(__AVR_ATmega103__)
 /* old AVR classic or ATmega103 with one UART */
 #define AT90_UART
 #define UART0_RECEIVE_INTERRUPT   SIG_UART_RECV
 #define UART0_TRANSMIT_INTERRUPT  SIG_UART_DATA
 #define UART0_STATUS   USR
 #define UART0_CONTROL  UCR
 #define UART0_DATA     UDR  
 #define UART0_UDRIE    UDRIE
#elif defined(__AVR_AT90S2333__) || defined(__AVR_AT90S4433__)
 /* old AVR classic with one UART */
 #define AT90_UART
 #define UART0_RECEIVE_INTERRUPT   SIG_UART_RECV
 #define UART0_TRANSMIT_INTERRUPT  SIG_UART_DATA
 #define UART0_STATUS   UCSRA
 #define UART0_CONTROL  UCSRB
 #define UART0_DATA     UDR 
 #define UART0_UDRIE    UDRIE
#elif  defined(__AVR_ATmega8__)  || defined(__AVR_ATmega16__) || defined(__AVR_ATmega32__) \
  || defined(__AVR_ATmega8515__) || defined(__AVR_ATmega8535__) \
  || defined(__AVR_ATmega323__)
  /* ATmega with one USART */
 #define ATMEGA_USART
 #define UART0_RECEIVE_INTERRUPT   SIG_UART_RECV
 #define UART0_TRANSMIT_INTERRUPT  SIG_UART_DATA
 #define UART0_STATUS   UCSRA
 #define UART0_CONTROL  UCSRB
 #define UART0_DATA     UDR
 #define UART0_UDRIE    UDRIE
#elif defined(__AVR_ATmega163__) 
  /* ATmega163 with one UART */
 #define ATMEGA_UART
 #define UART0_RECEIVE_INTERRUPT   SIG_UART_RECV
 #define UART0_TRANSMIT_INTERRUPT  SIG_UART_DATA
 #define UART0_STATUS   UCSRA
 #define UART0_CONTROL  UCSRB
 #define UART0_DATA     UDR
 #define UART0_UDRIE    UDRIE
#elif defined(__AVR_ATmega162__) 
 /* ATmega with two USART */
 #define ATMEGA_USART0
 #define ATMEGA_USART1
 #define UART0_RECEIVE_INTERRUPT   SIG_USART0_RECV
 #define UART1_RECEIVE_INTERRUPT   SIG_USART1_RECV
 #define UART0_TRANSMIT_INTERRUPT  SIG_USART0_DATA
 #define UART1_TRANSMIT_INTERRUPT  SIG_USART1_DATA
 #define UART0_STATUS   UCSR0A
 #define UART0_CONTROL  UCSR0B
 #define UART0_DATA     UDR0
 #define UART0_UDRIE    UDRIE0
 #define UART1_STATUS   UCSR1A
 #define UART1_CONTROL  UCSR1B
 #define UART1_DATA     UDR1
 #define UART1_UDRIE    UDRIE1
#elif defined(__AVR_ATmega64__) || defined(__AVR_ATmega128__) 
 /* ATmega with two USART */
 #define ATMEGA_USART0
 #define ATMEGA_USART1
 #define UART0_RECEIVE_INTERRUPT   SIG_UART0_RECV
 #define UART1_RECEIVE_INTERRUPT   SIG_UART1_RECV
 #define UART0_TRANSMIT_INTERRUPT  SIG_UART0_DATA
 #define UART1_TRANSMIT_INTERRUPT  SIG_UART1_DATA
 #define UART0_STATUS   UCSR0A
 #define UART0_CONTROL  UCSR0B
 #define UART0_DATA     UDR0
 #define UART0_UDRIE    UDRIE0
 #define UART1_STATUS   UCSR1A
 #define UART1_CONTROL  UCSR1B
 #define UART1_DATA     UDR1
 #define UART1_UDRIE    UDRIE1
#elif defined(__AVR_ATmega161__)
 /* ATmega with UART */
 #error "AVR ATmega161 currently not supported by this libaray !"
#elif defined(__AVR_ATmega169__) 
 /* ATmega with one USART */
 #define ATMEGA_USART
 #define UART0_RECEIVE_INTERRUPT   SIG_USART_RECV
 #define UART0_TRANSMIT_INTERRUPT  SIG_USART_DATA
  #define UART0_STATUS   UCSRA
 #define UART0_CONTROL  UCSRB
 #define UART0_DATA     UDR
 #define UART0_UDRIE    UDRIE
#elif defined(__AVR_ATmega48__) ||defined(__AVR_ATmega88__) || defined(__AVR_ATmega168__)
 /* ATmega with one USART */
 #define ATMEGA_USART0
 #define UART0_RECEIVE_INTERRUPT   SIG_USART_RECV
 #define UART0_TRANSMIT_INTERRUPT  SIG_USART_DATA
 #define UART0_STATUS   UCSR0A
 #define UART0_CONTROL  UCSR0B
 #define UART0_DATA     UDR0
 #define UART0_UDRIE    UDRIE0
#elif defined(__AVR_ATtiny2313__)
 #define ATMEGA_USART
 #define UART0_RECEIVE_INTERRUPT   SIG_USART0_RX 
 #define UART0_TRANSMIT_INTERRUPT  SIG_USART0_UDRE
 #define UART0_STATUS   UCSRA
 #define UART0_CONTROL  UCSRB
 #define UART0_DATA     UDR
 #define UART0_UDRIE    UDRIE
#elif defined(__AVR_ATmega329__) ||defined(__AVR_ATmega3290__) ||\
      defined(__AVR_ATmega649__) ||defined(__AVR_ATmega6490__) ||\
      defined(__AVR_ATmega325__) ||defined(__AVR_ATmega3250__) ||\
      defined(__AVR_ATmega645__) ||defined(__AVR_ATmega6450__)
  /* ATmega with one USART */
  #define ATMEGA_USART0
  #define UART0_RECEIVE_INTERRUPT   SIG_UART_RECV
  #define UART0_TRANSMIT_INTERRUPT  SIG_UART_DATA
  #define UART0_STATUS   UCSR0A
  #define UART0_CONTROL  UCSR0B
  #define UART0_DATA     UDR0
  #define UART0_UDRIE    UDRIE0
#elif defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega640__) || defined(__AVR_ATmega2561__) || defined(__AVR_ATmega1281__)
/* ATmega with two USART *///  This def is used in GIC fw for 2560. Uarts 0 and 2 in use.
  #define ATMEGA_USART0
  #define ATMEGA_USART1
  #define UART0_RECEIVE_INTERRUPT   USART0_RX_vect
  #define UART1_RECEIVE_INTERRUPT   USART1_RX_vect
  #define UART0_TRANSMIT_INTERRUPT  USART0_UDRE_vect
  #define UART1_TRANSMIT_INTERRUPT  USART1_UDRE_vect
  #define UART0_STATUS   UCSR0A
  #define UART0_CONTROL  UCSR0B
  #define UART0_DATA     UDR0
  #define UART0_UDRIE    UDRIE0
  #define UART1_STATUS   UCSR1A
  #define UART1_CONTROL  UCSR1B
  #define UART1_DATA     UDR1
  #define UART1_UDRIE    UDRIE1  
#elif defined(__AVR_ATmega644__)
 /* ATmega with one USART */
 #define ATMEGA_USART0
 #define UART0_RECEIVE_INTERRUPT   SIG_USART_RECV
 #define UART0_TRANSMIT_INTERRUPT  SIG_USART_DATA
 #define UART0_STATUS   UCSR0A
 #define UART0_CONTROL  UCSR0B
 #define UART0_DATA     UDR0
 #define UART0_UDRIE    UDRIE0
#elif defined(__AVR_ATmega164P__) || defined(__AVR_ATmega324P__) || defined(__AVR_ATmega644P__)
 /* ATmega with two USART */
 #define ATMEGA_USART0
 #define ATMEGA_USART1
 #define UART0_RECEIVE_INTERRUPT   SIG_USART_RECV
 #define UART1_RECEIVE_INTERRUPT   SIG_USART1_RECV
 #define UART0_TRANSMIT_INTERRUPT  SIG_USART_DATA
 #define UART1_TRANSMIT_INTERRUPT  SIG_USART1_DATA
 #define UART0_STATUS   UCSR0A
 #define UART0_CONTROL  UCSR0B
 #define UART0_DATA     UDR0
 #define UART0_UDRIE    UDRIE0
 #define UART1_STATUS   UCSR1A
 #define UART1_CONTROL  UCSR1B
 #define UART1_DATA     UDR1
 #define UART1_UDRIE    UDRIE1
#elif defined(__AVR_ATxmega128A1__)
/*ATxmegaA3 with 7 USARTS */
 #define ATxmega_USART0
 #define ATxmega_USART1
 #define ATxmega_USART2
 #define ATxmega_USART3
 #define ATxmega_USART4
 #define ATxmega_USART5
 #define ATxmega_USART6

#else
 #error "no UART definition for MCU available"
#endif


/*
 *  module global variables
 */
static volatile unsigned char UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART_RxBuf[UART_RX_BUFFER_SIZE];
static volatile unsigned char UART_TxHead;
static volatile unsigned char UART_TxTail;
static volatile unsigned char UART_RxHead;
static volatile unsigned char UART_RxTail;
static volatile unsigned char UART_LastRxError;

#if defined( ATMEGA_USART1 )
static volatile unsigned char UART1_TxBuf[UART_TX_BUFFER_SIZE];
static volatile unsigned char UART1_RxBuf[UART_RX_BUFFER_SIZE];
static volatile unsigned char UART1_TxHead;
static volatile unsigned char UART1_TxTail;
static volatile unsigned char UART1_RxHead;
static volatile unsigned char UART1_RxTail;
static volatile unsigned char UART1_LastRxError;
#endif



SIGNAL(UART0_RECEIVE_INTERRUPT)
//************************************************************************
//Function: UART Receive Complete interrupt
//Purpose:  called when the UART has received a character
//************************************************************************
{
    unsigned char tmphead;
    unsigned char data;
    unsigned char usr;
    unsigned char lastRxError;

		
 
    // read UART status register and UART data register
    usr  = UART0_STATUS;
    data = UART0_DATA;
    
 
#if defined( AT90_UART )
    lastRxError = (usr & (_BV(FE)|_BV(DOR)) );
#elif defined( ATMEGA_USART )
    lastRxError = (usr & (_BV(FE)|_BV(DOR)) );
#elif defined( ATMEGA_USART0 )
    lastRxError = (usr & (_BV(FE0)|_BV(DOR0)) );
#elif defined ( ATMEGA_UART )
    lastRxError = (usr & (_BV(FE)|_BV(DOR)) );
#endif
        
    // calculate buffer index 
    tmphead = ( UART_RxHead + 1) & UART_RX_BUFFER_MASK;
    
    if ( tmphead == UART_RxTail ) {
        // error: receive buffer overflow 
        lastRxError = UART_BUFFER_OVERFLOW >> 8;
    }else{
        // store new index 
        UART_RxHead = tmphead;
        // store received data in buffer 
       UART_RxBuf[tmphead] = data;
    }
    UART_LastRxError = lastRxError;   
	
}

SIGNAL(UART1_RECEIVE_INTERRUPT)
//************************************************************************
//Function: UART Receive Complete interrupt
//Purpose:  called when the UART has received a character
//************************************************************************
{
    unsigned char tmphead1;
    unsigned char data1;
    unsigned char usr1;
    unsigned char lastRxError1;

		
 
    // read UART status register and UART data register
    usr1  = UART1_STATUS;
    data1 = UART1_DATA;
    
 
#if defined( AT90_UART )
    lastRxError1 = (usr1 & (_BV(FE)|_BV(DOR)) );
#elif defined( ATMEGA_USART )
    lastRxError1 = (usr1 & (_BV(FE)|_BV(DOR)) );
#elif defined( ATMEGA_USART0 )
    lastRxError1 = (usr1 & (_BV(FE0)|_BV(DOR0)) );
#elif defined ( ATMEGA_UART )
    lastRxError1 = (usr1 & (_BV(FE)|_BV(DOR)) );
#endif
        
    // calculate buffer index 
    tmphead1 = ( UART1_RxHead + 1) & UART1_RX_BUFFER_MASK;
    
    if ( tmphead1 == UART1_RxTail ) {
        // error: receive buffer overflow 
        lastRxError1 = UART_BUFFER_OVERFLOW >> 8;
    }else{
        // store new index 
        UART1_RxHead = tmphead1;
        // store received data in buffer 
       UART1_RxBuf[tmphead1] = data1;
    }
    UART1_LastRxError = lastRxError1;   
	
}


SIGNAL(UART0_TRANSMIT_INTERRUPT)
/*************************************************************************
Function: UART Data Register Empty interrupt
Purpose:  called when the UART is ready to transmit the next byte
**************************************************************************/
{
    unsigned char tmptail;
    
    if ( UART_TxHead != UART_TxTail) {
        /* calculate and store new buffer index */
        tmptail = (UART_TxTail + 1) & UART_TX_BUFFER_MASK;
        UART_TxTail = tmptail;
        /* get one byte from buffer and write it to UART */
        UART0_DATA = UART_TxBuf[tmptail];  /* start transmission */
    }else{
        /* tx buffer empty, disable UDRE interrupt */
        UART0_CONTROL &= ~(1 << UART0_UDRIE);
    }
}

SIGNAL(UART1_TRANSMIT_INTERRUPT)
/*************************************************************************
Function: UART1 Data Register Empty interrupt
Purpose:  called when the UART1 is ready to transmit the next byte
**************************************************************************/
{
    unsigned char tmptail1;

    
    if ( UART1_TxHead != UART1_TxTail) {
        /* calculate and store new buffer index */
        tmptail1 = (UART1_TxTail + 1) & UART1_TX_BUFFER_MASK;
        UART1_TxTail = tmptail1;
        /* get one byte from buffer and write it to UART */
        UART1_DATA = UART1_TxBuf[tmptail1];  /* start transmission */
    }else{
        /* tx buffer empty, disable UDRE interrupt */
        UART1_CONTROL &= ~(1 << UART1_UDRIE);
    }
}

/*************************************************************************
Function: uart_init()
Purpose:  initialize UART and set baudrate
Input:    baudrate using macro UART_BAUD_SELECT()
Returns:  none
**************************************************************************/
void uart_init(unsigned int baudrate)
{
    UART_TxHead = 0;
    UART_TxTail = 0;
    UART_RxHead = 0;
    UART_RxTail = 0;
    
	CLKPR = (1<<CLKPCE);  	//enable clock prescaler change
	CLKPR = 0x00;			//clock divide = 1  so  16MHz/1 = 16MHz

  	UBRR0H = 0;
    UBRR0L = 103;	//19200 baud for laptop/LCD info panel

    /* Set frame format: asynchronous, 8data, no parity, 1stop bit */
	UCSR0A = 2;		//2x on
	UCSR0B = 24;	//tx and rx on  AND rxie on for testing
    UCSR0C = 6;		//8 - 1 - N

}/* uart_init */

void uart1_init(unsigned int baudrate)
{
	UART1_TxHead = 0;
    UART1_TxTail = 0;
    UART1_RxHead = 0;
    UART1_RxTail = 0;
	
	CLKPR = (1<<CLKPCE);  	//enable clock prescaler change
	CLKPR = 0x00;			//clock divide = 1  so  16MHz/1 = 16MHz
	
	UBRR1H = 0;
    UBRR1L = 103;	// now 250k for CAN  19200 for testing    //9600 baud for gsm modem (207 with 2x on)
    
    /* Set frame format: asynchronous, 8data, no parity, 1stop bit */
	UCSR1A = 2;		//2x on
	UCSR1B = 24;	//tx and rx on AND rxie on
    UCSR1C = 6;		//8 - 1 - N

}/* uart1_init */


/*************************************************************************
Function: uart_getc()
Purpose:  return byte from ringbuffer  
Returns:  lower byte:  received byte from ringbuffer
          higher byte: last receive error
**************************************************************************/
unsigned int uart_getc(void)
{    
    unsigned char tmptail;
    unsigned char data;


    if ( UART_RxHead == UART_RxTail ) {
        return UART_NO_DATA;   /* no data available */
    }
    
    /* calculate /store buffer index */
    tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;
    UART_RxTail = tmptail; 
    
    /* get data from receive buffer */
    data = UART_RxBuf[tmptail];
    
    return (UART_LastRxError << 8) + data;

}/* uart_getc */

unsigned int uart1_getc(void)
{    
    unsigned char tmptail1;
    unsigned char data1;


    if ( UART1_RxHead == UART1_RxTail ) {
        return UART_NO_DATA;   /* no data available */
    }
    
    /* calculate /store buffer index */
    tmptail1 = (UART1_RxTail + 1) & UART1_RX_BUFFER_MASK;
    UART1_RxTail = tmptail1; 
    
    /* get data from receive buffer */
    data1 = UART1_RxBuf[tmptail1];
    
    return (UART1_LastRxError << 8) + data1;

}/* uart1_getc */


/*************************************************************************
Function: uart_putc()
Purpose:  write byte to ringbuffer for transmitting via UART, choosing the
		  correct port to write to based on current traffic flow
Input:    byte to be transmitted
Returns:  none          
**************************************************************************/
void uart_putc(unsigned char data)
{	
	unsigned char tmphead;
	
	if (portKey == 0)
	{
		tmphead  = (UART_TxHead + 1) & UART_TX_BUFFER_MASK;
    
		while ( tmphead == UART_TxTail ){
			;// wait for free space in buffer 
		}
		
		UART_TxBuf[tmphead] = data;
		UART_TxHead = tmphead;

		// enable UDRE interrupt 
		UART0_CONTROL |= (1 << UART0_UDRIE);
		
	}
	
	else if (portKey == 1)
	{
		tmphead  = (UART1_TxHead + 1) & UART1_TX_BUFFER_MASK;
    
		while ( tmphead == UART1_TxTail ){
			;// wait for free space in buffer 		
		}
		//PORTH &= 251;
    
		UART1_TxBuf[tmphead] = data;
		UART1_TxHead = tmphead;

		// enable UDRE interrupt 
		UART1_CONTROL |= (1 << UART1_UDRIE);	
	}		
	
}/* uart_putc */

void uart1_putc(unsigned char data1)
{
	
	unsigned char tmphead;
	tmphead  = (UART1_TxHead + 1) & UART1_TX_BUFFER_MASK;
    
		while ( tmphead == UART1_TxTail ){
			;// wait for free space in buffer 		
		}
		//PORTH &= 251;
    
		UART1_TxBuf[tmphead] = data1;
		UART1_TxHead = tmphead;

		// enable UDRE interrupt 
		UART1_CONTROL |= (1 << UART1_UDRIE);	
	    	
}/* uart1_putc */


/*************************************************************************
Function: uart_puts()
Purpose:  transmit string to UART
Input:    string to be transmitted
Returns:  none          
**************************************************************************/
void uart_puts(const char *s )
{
    while (*s)
	{
	  
      uart_putc(*s++);
	  
	}
}/* uart_puts */

void uart1_puts(const char *t )
{
    while (*t) 
      uart1_putc(*t++);

}/* uart1_puts */


/*************************************************************************
Function: uart_puts_p()
Purpose:  transmit string from program memory to UART
Input:    program memory string to be transmitted
Returns:  none
**************************************************************************/
void uart_puts_p(const char *progmem_s )
{
    register char c;
    
    while ( (c = pgm_read_byte(progmem_s++)) ) 
      uart_putc(c);

}/* uart_puts_p */

void uart1_puts_p(const char *progmem_t )
{
    register char c;
    
    while ( (c = pgm_read_byte(progmem_t++)) ) 
      uart1_putc(c);

}/* uart1_puts_p */


// UART polled functions

char isCharAvailable()
{	
	// Does the RX0 bit of the USART Status and Control Register
	// indicate a char has been received?
	if ( (UCSR0A & (0x80)) ) return 1;
	else return 0;
}

char isCharAvailable_1()
{	
	// Does the RX0 bit of the USART Status and Control Register
	// indicate a char has been received?
	if ( (UCSR1A & (0x80)) ) return 1;
	else return 0;
}

char receiveChar()
{
	// Return the char in the UDR0 register
	return UDR0;
	
}

char receiveChar_1()
{
	// Return the char in the UDR2 register
	return UDR1;
	
}
#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#define F_CPU 16000000UL
#include <util/delay.h>

#define USART_BAUDRATE 38400
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)
//define max buffer size
#define FRAME_SIZE 20
#define ACTION_MAX 8
#define COMMAND_PARAMATER_SIZE 8
//type definition of buffer structure
volatile uint8_t PLC_ID=0;
volatile uint8_t packet_received=0;



typedef struct{
	//Array of chars
	uint8_t unit[COMMAND_PARAMATER_SIZE];
	uint8_t index;				//array element index
}u8actions;

typedef struct{
	uint8_t ID;					//Confirm if frame relevant to this unit.
	uint8_t frame[FRAME_SIZE];
	uint8_t index;				//array element index
	u8actions list[ACTION_MAX];
	uint8_t command;			//Command	
}u8frame;

//declare frame
u8frame rx_frame,tx_frame,status_frame;

//initialize frame
void FrameInit(u8frame *pdu)
{
	//set index to start of buffer
	pdu->index=0;
}
//write to frame routine

void FrameEmpty(u8frame *pdu)
{
	uint8_t count=0;
	UCSR0B |= (1<<TXEN0); // Enable transmit.

	for(count=0;count<pdu->index;count++)
	{
		while(!(UCSR0A & (1<<UDRE0)));
			UDR0=pdu->frame[count];
			//UDR0='A';
		while(!(TXC0));
	}
	//start over
	//reset frame
	pdu->index=0;
	//disable transmission and UDR0 empty interrupt
	UCSR0B &= ~((1<<TXEN0)|(1<<UDRIE0));
	//disable reception and RC complete interrupt
	//UCSR0B &= ~(1<<RXEN0)|(1<<RXCIE0);

}



uint8_t CharacterWrite(uint8_t u8data)
{	
	UCSR0B |= (1<<TXEN0); // Enable transmit.

	while(!(UCSR0A & (1<<UDRE0)));
	UDR0=u8data;
	while(!(TXC0));
	return 0;

	//disable transmission and UDR0 empty interrupt
	UCSR0B &= ~((1<<TXEN0)|(1<<UDRIE0));
}

uint8_t BufferWrite(u8frame *pdu, uint8_t u8data)
{
	if(pdu->index<FRAME_SIZE)
	{
		pdu->frame[pdu->index] = u8data;
		//increment buffer index
		pdu->index++;
		return 0;
	} 
	else return 1;
}
uint8_t FrameRead(u8frame *pdu, volatile uint8_t *u8data)
{
	//uint8_t count=0;
	if(pdu->index>0)
	{
	 pdu->index--;
	 *u8data=pdu->frame[pdu->index];
	
		//for (count=0;count<buf->index;count++)
		//{
		//	while(!(UCSR0A & (1<<UDRE0)));
		//	*u8data=buf->buffer[count];
		//	while(!(TXC0));
		//}
		return 0;
	}
	else return 1;
}
void USART0Init(void)
{
	// Set baud rate
	UBRR0H = (uint8_t)(UBRR_VALUE>>8);
	UBRR0L = (uint8_t)UBRR_VALUE;
	// Set frame format to 8 data bits, no parity, 1 stop bit
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	//enable reception and RC complete interrupt
	UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
}



//RX Complete interrupt service routine
ISR(USART_RX_vect)
{
	uint8_t u8temp;
	while(! (UCSR0A & (1 << RXC0))){};
	u8temp=UDR0;

	//check if period char or end of buffer
	if ((BufferWrite(&rx_frame, u8temp)==1)||(u8temp=='.'))
	{
		//disable reception and RX Complete interrupt
		UCSR0B &= ~((1<<RXEN0)|(1<<RXCIE0));
		//enable transmission and UDR0 empty interrupt
		//UCSR0B |= (1<<TXEN0)|(1<<UDRIE0);

		packet_received=1; //set packet received flag
	}
	
}

uint8_t RXFrameParse(u8frame *frame)
{
	// Check ID
	
	if(frame->ID != PLC_ID)
	{
		// Reinitialize all frames as message not for us.
		FrameInit(&rx_frame);
	}



	return 0;
}


int main()
{
	
	FrameInit(&rx_frame);
	FrameInit(&tx_frame);
	FrameInit(&status_frame);
	
	set_sleep_mode(SLEEP_MODE_IDLE);
	
	USART0Init();
	DDRD |= (1 << PD5);				// set Port D pin PC5 for output
	DDRD |= (1 << PD4);
	PORTD |= (1<< PD5);				// Set high to enable transmit.
	PORTD &= ~(1<< PD4);			// Set low to enable receive.

	//enable global interrupts
	sei();

	while(1)
	{
				       
		//sleep_mode();

		if(packet_received==1)
		{
			RXFrameParse(&rx_frame);			
			FrameEmpty(&rx_frame);
			FrameInit(&rx_frame);
			packet_received=0;
			//CharacterWrite(packet_received);
			//enable reception and RC complete interrupt
			UCSR0B |= (1<<RXEN0)|(1<<RXCIE0);
			//UCSR0B |= (1<<RXCIE0);

		}
		
	}
}

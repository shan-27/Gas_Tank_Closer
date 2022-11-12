/*
 * Automatic_Gas_Tank_Closer.c
 *
 * Created: 11/12/2022 2:48:12 PM
 * Author : MSI
 */ 

#define F_CPU 16000000UL
#define BAUD 9600
#define UBRR_VALUE ((F_CPU/16/BAUD) -1)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

int knob = 0 ; // 0 = closed , 1 = open

volatile int rxFlag =0;
volatile unsigned char rxData;
int A0,A1,A2,A3 =0;


int ADC_value = 0;

/*--------Servo Control-----------*/

void InitServo()
{
	DDRB |= (1<<PORTB1);	/* Make OC1A pin as output */
	TCNT1 = 0;				/* Set timer1 count zero */
	ICR1 = 2499;			/* Set TOP count for timer1 in ICR1 register */

	/* Set Fast PWM, TOP in ICR1, Clear OC1A on compare match, clock/64 */
	TCCR1A = (1<<WGM11)|(1<<COM1A1);
	TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS10)|(1<<CS11);
}

void RotateServo(int pos)
{
	_delay_ms(1500);
	OCR1A = pos;	/* Set servo shaft at given position */
	_delay_ms(1500);
	
}


/*--------Sending gas level to computer serial monitor using USART -----------*/

void initUSART(void)
{
	UBRR0H = (unsigned char) (UBRR_VALUE>>8) ;		//Set UBRR upper byte
	UBRR0L= (unsigned char) (UBRR_VALUE) ;			//Set UBRR lower byte
	UCSR0B =(1<<RXCIE0) | (1<<RXEN0) |(1<<TXEN0) ;	//Enable transmitter/receiver
	UCSR0C=(3<<UCSZ00);								//8-data bits
	sei ();											//Enable global interrupt
}

void transmitUSART (unsigned char data)
{
	while (! (UCSR0A & (1<<UDRE0))) ;		// wait till the transmit buffer is ready
	UDR0 = data;							// Put data to the transmit buffer
}

unsigned char receiveUSART(void)
{
	while(! (UCSR0A & (1<<RXC0))) ;			// wait till data received
	return UDR0;							// Receive data
}

/*--------ADC conversion part of the gas sensor readings -----------*/

void initADC(void){
	ADCSRA |= (1<<ADEN);
	ADCSRA |= 0B00000111;
	ADCSRA |= (1<<ADSC);
}

int readGasLevel()
{
	ADMUX = (ADMUX & 0xF0) | 0;				// Set input channel to 0
	ADMUX &= 0B00111111;					// Turn off the internal reference voltage
	ADCSRA |= (1<<ADSC);					// Start the conversion
	
	while(ADCSRA & (1<<ADSC));				// Wait till conversion completes
	return ADC;								// Returning the ADC value
}

/*-------- Printing the gas level -----------*/

void printGasLevel(int ADC_value)
{
	A0 = ADC_value%10;
	A1 = (ADC_value%100)/10;
	A2 = (ADC_value%1000)/100;
	A3 = (ADC_value%10000)/1000;

	rxFlag=0;
	rxData =0x30+A3;
	transmitUSART(rxData);
	
	rxData =0x30+A2;
	transmitUSART(rxData);
	
	rxData =0x30+A1;
	transmitUSART(rxData);
	
	rxData =0x30+A0;
	transmitUSART(rxData);
	
	transmitUSART(0x0A); //Line feed
	transmitUSART (0x0D);
	
	_delay_ms(500);
}




int main(void)
{
	/* Replace with your application code */
	
	/* Servo Positions */
	// pos = 120 (180 degree)
	// pos = 600 (0 degree)
	// pos = 360 (90 degree)
	
	int mute = 0 ;
	
	InitServo();			// Initialize the servo motor
	DDRD &= 0B10110011;
	DDRD |= 0B10110000;
	initUSART () ;			// Initialize the USART
	initADC();				// Initialize the ADC
	
	unsigned char rxByte;

	DDRB |= 0B00000100;
	
	PORTD|= 0B00100000;		// Notifying the user the device is on using turn on led (Green LED)
	
	PORTD|= 0B10000000;		// Notifying the user the device is on using buzzer
	_delay_ms(100);
	PORTD &= 0B01111111;
	_delay_ms(100);
	PORTD|= 0B10000000;
	_delay_ms(100);
	PORTD &= 0B01111111;
	
	
	while (1)
	{
		ADC_value = readGasLevel(); // Read the gas level and save in a variable
		printGasLevel(ADC_value);	// Print the gas level on a serial monitor (For debugging purposes)
		
		
		if (ADC_value>400)			// Checking gas level is greater than 400
		{
			PORTB |= 0B00000100;	// Turn on gas leak indicator light
			PORTD|= 0B10000000;		// Turn on the buzzer
			RotateServo(360);		// Turn off the gas regulator
			knob = 0;
			_delay_ms(100);
			
		}
		else
		{
			PORTB &= 0B11111011;	// Turn off gas leak indicator light
		}
		
		/*----- close button--------*/
		
		if ((PIND & 0B00000100)== 0) // Checking the closed button is pressed or not
		{
			
		}
		else
		{
			if (knob==1)
			{
				PORTD |= 0B00010000; // Turn on RED led (alert bulb) to notify closing operation is started
				_delay_ms(50);
				
				RotateServo(360);	 // Turn off the gas regulator
				knob = 0;
				
				PORTD |= 0B00010000; // Blinking RED led (alert bulb) to notify closing operation is successfully completed
				_delay_ms(100);
				PORTD &= 0B11101111;
				_delay_ms(100);
				PORTD |= 0B00010000;
				_delay_ms(100);
				PORTD &= 0B11101111;
				_delay_ms(100);
			}
			else
			{
				PORTD |= 0B00010000; // Blinking RED led (alert bulb) to notify already closed
				_delay_ms(100);
				PORTD &= 0B11101111;
				_delay_ms(100);
				PORTD |= 0B00010000;
				_delay_ms(100);
				PORTD &= 0B11101111;
				_delay_ms(100);
			}
			
		}
		
		/*------Mute Button---------*/
		
		if ((PIND & 0B01000000)== 0) // Checking the mute button is pressed or not
		{
			
			
		}
		else
		{
			
			PORTD |= 0B00010000;	// Turn on RED led (alert bulb) to notify mute operation is started
			_delay_ms(50);
			
			PORTD&= 0B01111111;		// Turn off the buzzer
			
			PORTD |= 0B00010000;	// Blinking RED led (alert bulb) to notify mute operation is successfully completed
			_delay_ms(100);
			PORTD &= 0B11101111;
			_delay_ms(100);
			PORTD |= 0B00010000;
			_delay_ms(100);
			PORTD &= 0B11101111;
			_delay_ms(100);
			
			
		}
		
		/*--------Open  button---------*/
		
		if ((PIND & 0B00001000)== 0) // Checking the closed button is pressed or not
		{
			
		}
		else
		{
			if (knob==0)
			{
				PORTD |= 0B00010000;	// Turn on RED led (alert bulb) to notify opening operation is started
				_delay_ms(50);
				
				RotateServo(120);		// Turn of the gas regulator
				knob = 1;
				
				PORTD |= 0B00010000;	// Blinking RED led (alert bulb) to notify opening operation is successfully completed
				_delay_ms(100);
				PORTD &= 0B11101111;
				_delay_ms(100);
				PORTD |= 0B00010000;
				_delay_ms(100);
				PORTD &= 0B11101111;
				_delay_ms(100);
			}
			else
			{
				PORTD |= 0B00010000;	// Blinking RED led (alert bulb) to notify already opened
				_delay_ms(100);
				PORTD &= 0B11101111;
				_delay_ms(100);
				PORTD |= 0B00010000;
				_delay_ms(100);
				PORTD &= 0B11101111;
				_delay_ms(100);
			}
			
		}
		
		
		
	}
}

ISR (USART0_RX_vect) // USART interrupt
{
	rxFlag=1;
	rxData=UDR0;
}

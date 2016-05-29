#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>

volatile int value = 0;

#define DS3232MADDR 0xD0

#define MCP23017ADDR	0x40
#define IODIRA			0x00
#define IODIRB			0x01
#define GPIOA			0x12
#define GPIOB			0x13

#define ERROR 1
#define SUCCESS 0

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

#define MODE_SHOW_TIME			0
#define MODE_SET_TEN_HOURS		1
#define MODE_SET_HOURS			2
#define MODE_SET_TEN_MINUTES	3
#define MODE_SET_MINUTES		4
#define MODE_SET_TEN_SECONDS	5
#define MODE_SET_SECONDS		6
#define MODE_LAST				7

volatile int seconds;
volatile int tenseconds;
volatile int minutes;
volatile int tenminutes;
volatile int hours;
volatile int tenhours;

volatile int mode = 0;
volatile int sqwold = 0;

void TWIInit(void)
{
	//set SCL to 400kHz
	TWSR = 0x00;
	TWBR = 0x0C;
	//enable TWI
	TWCR = (1<<TWEN);
}

void TWIStart(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
}
//send stop signal
void TWIStop(void)
{
	TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void TWIWrite(uint8_t u8data)
{
	TWDR = u8data;
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
}

uint8_t TWIReadACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWEA);
	while ((TWCR & (1<<TWINT)) == 0);
	return TWDR;
}
//read byte with NACK
uint8_t TWIReadNACK(void)
{
	TWCR = (1<<TWINT)|(1<<TWEN);
	while ((TWCR & (1<<TWINT)) == 0);
	return TWDR;
}

uint8_t TWIGetStatus(void)
{
	uint8_t status;
	//mask status
	status = TWSR & 0xF8;
	return status;
}

uint8_t EEWriteByte(uint8_t u16device, uint16_t u16addr, uint8_t u8data)
{
	TWIStart();
	if (TWIGetStatus() != 0x08)
	return ERROR;
	//select devise and send A2 A1 A0 address bits
	TWIWrite((u16device)|(uint8_t)((u16addr & 0x0700)>>7));
	if (TWIGetStatus() != 0x18)
	return ERROR;
	//send the rest of address
	TWIWrite((uint8_t)(u16addr));
	if (TWIGetStatus() != 0x28)
	return ERROR;
	//write byte to eeprom
	TWIWrite(u8data);
	if (TWIGetStatus() != 0x28)
	return ERROR;
	TWIStop();
	return SUCCESS;
}

uint8_t EEReadByte(uint8_t u16device, uint16_t u16addr, uint8_t *u8data)
{
	//uint8_t databyte;
	TWIStart();
	if (TWIGetStatus() != 0x08)
	return ERROR;
	//select devise and send A2 A1 A0 address bits
	TWIWrite((u16device)|((uint8_t)((u16addr & 0x0700)>>7)));
	if (TWIGetStatus() != 0x18)
	return ERROR;
	//send the rest of address
	TWIWrite((uint8_t)(u16addr));
	if (TWIGetStatus() != 0x28)
	return ERROR;
	//send start
	TWIStart();
	if (TWIGetStatus() != 0x10)
	return ERROR;
	//select devise and send read bit
	TWIWrite((u16device)|((uint8_t)((u16addr & 0x0700)>>7))|1);
	if (TWIGetStatus() != 0x40)
	return ERROR;
	*u8data = TWIReadNACK();
	if (TWIGetStatus() != 0x58)
	return ERROR;
	TWIStop();
	return SUCCESS;
}

inline void incordec(int mode, int inc)
{
	switch (mode)
	{
		case MODE_SET_TEN_HOURS:
		{
			tenhours += inc;
			tenhours = MIN(tenhours, 2);
			tenhours = MAX(tenhours, 0);
			break;
		}
		case MODE_SET_HOURS:
		{
			hours += inc;
			hours = MIN(hours, 9);
			hours = MAX(hours, 0);
			break;
		}
		case MODE_SET_TEN_MINUTES:
		{
			tenminutes += inc;
			tenminutes = MIN(tenminutes, 5);
			tenminutes = MAX(tenminutes, 0);
			break;
		}
		case MODE_SET_MINUTES:
		{
			minutes += inc;
			minutes = MIN(minutes, 9);
			minutes = MAX(minutes, 0);
			break;
		}
		case MODE_SET_TEN_SECONDS:
		{
			tenseconds += inc;
			tenseconds = MIN(tenseconds, 5);
			tenseconds = MAX(tenseconds, 0);
			break;
		}
		case MODE_SET_SECONDS:
		{
			seconds += inc;
			seconds = MIN(seconds, 9);
			seconds = MAX(seconds, 0);
			break;
		}
	}

	EEWriteByte(DS3232MADDR, 0x00, (tenseconds << 4) | seconds);
	EEWriteByte(DS3232MADDR, 0x01, (tenminutes << 4) | minutes);
	EEWriteByte(DS3232MADDR, 0x02, (tenhours << 4) | hours);
}

ISR(PCINT1_vect)
{
	static uint8_t old_AB = 3;  //lookup table index
	static int8_t encval = 0;   //encoder value
	static const int8_t enc_states [] PROGMEM =
	{0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};  //encoder lookup table
	/**/
	
	if (mode == MODE_SHOW_TIME)
	{
		if ((PINC & (1 << 2)) != sqwold)
		{
			sqwold = PINC & (1 << 2);
			if (sqwold)
			{
				seconds++;

				if (seconds == 10)
				{
					seconds = 0;
					tenseconds++;
				}
				if (tenseconds == 6)
				{
					tenseconds = 0;
					minutes++;
				}
				if (minutes == 10)
				{
					minutes = 0;
					tenminutes++;
				}
				if (tenminutes == 6)
				{
					tenminutes = 0;
					hours++;
				}

				if ((tenhours == 2) && (hours == 4))
				{
					hours = 0;
					tenhours = 0;
				}
				if (hours == 10)
				{
					hours = 0;
					tenhours++;
				}
			}
		}
	}
	else
	{
		old_AB <<=2;  //remember previous state
		old_AB |= ( PINC & 0x03 );
		encval += pgm_read_byte(&(enc_states[( old_AB & 0x0f )]));
		/* post "Navigation forward/reverse" event */
		if( encval > 3 ) {  //four steps forward
			incordec (mode, 1);
			encval = 0;
		}
		else if( encval < -3 ) {  //four steps backwards 
			incordec (mode, -1);
			encval = 0;
		}
	}
}

int main(void)
{
//	int blinking = 0;
	int blink_debounce = 0;
	uint8_t tmp;
	
	/* leds */
	DDRD = 0xFF;
	
	/* encoder */
	PORTC |= (1 << 0) | (1 << 1);
	PCMSK1 |= (1 << PCINT8) | (1 << PCINT9);

	/* pin change interrupts */
	PCICR |= (1 << PCIE1);
	
	/* button */
	PORTC |= (1 << 3);
	
	/* SQW pullup and interrupt*/
	PORTC |= (1 << 2);
	PCMSK1 |= (1 << PCINT10);

	
	/* start i2C */
	TWIInit();
	
	EEWriteByte(DS3232MADDR, 0x0E, 0b00100000); // Enable 1hz output
	
	EEReadByte(DS3232MADDR, 0x00, &tmp);
	seconds = tmp & 0x0F;
	tenseconds = (tmp & 0x70) >> 4;
	
	EEReadByte(DS3232MADDR, 0x01, &tmp);
	minutes = tmp & 0x0F;
	tenminutes = (tmp & 0x70) >> 4;

	EEReadByte(DS3232MADDR, 0x02, &tmp);
	hours = tmp & 0x0F;
	tenhours = (tmp & 0x30) >> 4;

	/* set all to outputs */
	EEWriteByte(MCP23017ADDR, IODIRA, 0x00);
	EEWriteByte(MCP23017ADDR, IODIRB, 0x00);
	
	/* set all high */
//	EEWriteByte(MCP23017ADDR, GPIOA, 0x0F);
	EEWriteByte(MCP23017ADDR, GPIOB, 0x00);

	/* turn on all interrupts */
	sei();
	
    while(1)
    {
		if (!(PINC & 1 << 3))
		{
			blink_debounce <<= 1;
			blink_debounce |= 0x1;
			if (blink_debounce == 0xFF)
			{
//				blinking = !blinking;
				blink_debounce = 0;
				
				mode++;
				if (mode == MODE_LAST)
				{
					mode = 0;
				}
				
//				if (blinking)
//				{
//					PCICR |= (1 << PCIE1);
//				}
//				else
//				{
//					PCICR &= ~(1 << PCIE1);
//				}
			}
		}
		
		PORTD = (tenseconds << 4) | seconds;
		EEWriteByte(MCP23017ADDR, GPIOB, (tenminutes << 4) | minutes);
		_delay_ms(50);

		switch (mode)
		{
			case MODE_SHOW_TIME:
			{
				_delay_ms(50);
				break;
			}
			case MODE_SET_TEN_MINUTES:
			{
				EEWriteByte(MCP23017ADDR, GPIOB, (0xF << 4) | minutes);
				_delay_ms(50);
				break;
			}
			case MODE_SET_MINUTES:
			{
				EEWriteByte(MCP23017ADDR, GPIOB, (tenminutes << 4) | 0xF);
				_delay_ms(50);
				break;
			}
			case MODE_SET_TEN_SECONDS:
			{
				PORTD = (0xF << 4) | seconds;
				_delay_ms(50);
				break;
			}
			case MODE_SET_SECONDS:
			{
				PORTD = (tenseconds << 4) | 0xF;
				_delay_ms(50);
				break;
			}
			default:
			{
				_delay_ms(50);
				break;
			}
		}
    }
}
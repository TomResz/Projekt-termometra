#include "I2c.h"
void TWI_start(void)
{
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
}
void TWI_init(void)
{
	TWBR = 92; // 72
	TWCR = (1<<TWEN); // wlaczenie tryb TWI
}
void TWI_stop(void)
{
	TWCR = (1<<TWINT) | (1<<TWSTO) | (1<<TWEN);
	while((TWCR & (1<<TWSTO)));
}

uint8_t TWI_readNak()
{
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

uint8_t TWI_readAck()
{
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}
void TWI_write(uint8_t bajt)
{
	TWDR = bajt;
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT)));
}
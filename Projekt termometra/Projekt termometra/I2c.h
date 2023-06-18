#ifndef I2C_H_
#define I2C_H_

#include <avr/io.h>

void TWI_init(void);
void TWI_start(void);
void TWI_stop(void);
void TWI_write(uint8_t bajt);

#endif /* I2C_H_ */
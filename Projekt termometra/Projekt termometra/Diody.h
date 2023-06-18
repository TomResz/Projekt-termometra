#ifndef DIODY_H_
#define DIODY_H_

#include <avr/io.h>

// porty diod
// fahrenheit to równoczesnie swieciace celsjusz i kelwin
#define diody PORTA
#define diodyRodzajTemperaturyDDR DDRA



#define diodaCelsjusz (1<<4)
#define diodaFahrenreit (1<<5)
#define diodaKelwin (1<<4)|(1<<5)

#define ustawPorty (1<<4)|(1<<5)|(1<<6)|(1<<7)
#define zapalWszystkieDiody ((1<<4)|(1<<5)|(1<<6)|(1<<7))
#define diodaMax ((1<<7))
#define diodaMin ((1<<6))


#define  wygasDiodyTemp (1<<4)|(1<<5)
#define wygasDiodyRodzaj ((1<<6)|(1<<7))
void czyscTempDiody(void);
void wygasWszystkieDiody(void);
void czyscRodzajDiody(void);

#endif /* DIODY_H_ */
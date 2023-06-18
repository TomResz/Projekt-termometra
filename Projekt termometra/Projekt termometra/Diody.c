#include "Diody.h"
void czyscTempDiody(void)
{
	diody |= wygasDiodyTemp;
}

void wygasWszystkieDiody(void)
{
	diody |= ustawPorty;
}
void czyscRodzajDiody(void)
{
	diody |= wygasDiodyRodzaj;
}
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "Diody.h"
#include "I2c.h"


#define segmentyDDR DDRB
#define segmenty PORTB
#define wyborDDR DDRD
#define wybor PORTD

#define przyciskiDDR DDRA
#define przyciski PORTA
#define przyciskiIN PINA
#define SKALA 0
#define MIN 1
#define MAX 2
#define RESET 3
/*
Podlaczenie segmentow:
A->D7
B->D6
C->D5
D->D4
E->D3
F->D2
G->D1
kropka ->D0
Podlaczenie wyboru:
B0 -> najbardziej z lewej cyfra
B3 -> najbardziej z prawej cyfra
*/

static const PROGMEM uint8_t seg7[]={0xFC,	0x60,	0xDA,	0xF2,	0x66,	0xB6,	0xBE,	0xE4,	0xFE,	0xF6 }; // do testow na symulatorze, aktywne 1

//static const PROGMEM uint8_t seg7[]={	// wlasciwy seg7 na projekt
//0x03,	// 0
//0x9F,	// 1
//0x25,	// 2
//0x0D,	// 3
//0x99,	// 4
//0x49,	// 5
//0x41,	// 6
//0x1B,	// 7
//0x01,	// 8
//0x09,	// 9}
//};
volatile uint8_t tryb_pomiaru_termometra=1; // 1- Celsjusz, 2-Kelwin, 3-Fahrenreit
volatile uint8_t ogolny_tryb_termometra=0; // 0 - skala/biezaca, 1 -min , 2 -max
volatile uint8_t tryb_skala=0,tryb_min=0,tryb_max=0,tryb_reset=0; // zmienne dla przyciskow

volatile uint8_t tryb_przed_resetem=0; 
//volatile uint8_t cejsluszZnak=0x63,fahrenreitZnak=0x71; // znak C i F
volatile uint8_t cejsluszZnak=0x9c,fahrenreitZnak=0x8e; // symulator
uint8_t temperatura_min[2]={120,0}; // tablica dla wynikow min
uint8_t temperatura_max[2]={0,0}; // tablica dla wynikow max

uint8_t pomiar_temperatury[2]={0,128}; // pierwsza liczba to temp calkowita, druga to temperatura po przecinku, gdzie 3 bity najstarsz symbolizuje 3 miejsca po przecinku
uint8_t wynik[]={0,0,0,0};
volatile uint8_t ilosc_pomiarow=0;

void pokaz_temperature_w_celsjuszach(uint8_t i,uint8_t temperatura_calkowita, uint8_t temperatura_po_po_przecinku)
{
	diody |= wygasDiodyTemp;
	diody &= ~diodaCelsjusz;
	wynik[0] = ((temperatura_calkowita % 1000) % 100) / 10;
	wynik[1] = (temperatura_calkowita % 10);
	wynik[2] = temperatura_po_po_przecinku %10;
	
	switch(i)
	{
		case 1:
		segmenty = pgm_read_byte(&(seg7[wynik[i]])) | 0x01;
		//segmenty = pgm_read_byte(&(seg7[wynik[i]])) & 0xfe;
		break;
		case 3:
		segmenty=cejsluszZnak;
		break;
		default:
		segmenty = pgm_read_byte(&(seg7[wynik[i]]));
		break;
	}
}
void pokaz_temperature_w_fahrenheitach(uint8_t i,uint8_t temperatura_calkowita, uint8_t temperatura_po_przecinku)
{
	diody |= wygasDiodyTemp;
	diody &= ~diodaFahrenreit;
	uint16_t temperaturaF=((temperatura_calkowita*10+temperatura_po_przecinku)*9)/5 +320;
	
	// konwersja wynikow na wartosci tablicowe
	wynik[0] = temperaturaF/ 100;
	wynik[1] = (temperaturaF %100)/10;
	wynik[2] = (temperaturaF % 100)%10;
	
	switch(i)
	{
		case 1:
		segmenty = pgm_read_byte(&(seg7[wynik[i]])) | 0x01;
		
		//segmenty = pgm_read_byte(&(seg7[wynik[i]])) & 0xfe;
		break;
		case 3:
		segmenty=fahrenreitZnak; // znak jednostki
		break;
		default:
		segmenty = pgm_read_byte(&(seg7[wynik[i]])); // domyslny symbol
		break;
	}
}
void pokaz_temperature_w_kelwinach(uint8_t i,uint8_t temperatura_calkowita, uint8_t temperatura_po_po_przecinku)
{
	diody |= wygasDiodyTemp;
	diody &= ~(diodaKelwin);
	uint16_t temperatura=temperatura_calkowita+273;
	temperatura_po_po_przecinku+=1.5;
	wynik[0] = temperatura / 100;
	wynik[1] = ((temperatura % 1000) % 100) / 10;
	wynik[2] = (temperatura % 10);
	wynik[3] = (uint16_t)temperatura_po_po_przecinku %10;
	
	if(i==2){
		segmenty=pgm_read_byte(&(seg7[wynik[i]])) | 0x01;
		//segmenty = pgm_read_byte(&(seg7[wynik[i]])) & 0xfe;
	}
	else{
		segmenty=pgm_read_byte(&(seg7[wynik[i]]));
	}
}
uint8_t oblicz_liczbe_po_przecinku(uint8_t liczba)
{
	uint8_t liczba_po_przecinku=0;
	if(liczba & (1<<7)) liczba_po_przecinku +=5;
	if(liczba & (1<<6)) liczba_po_przecinku +=2.5;
	if(liczba & (1<<5)) liczba_po_przecinku +=1.25;
	return liczba_po_przecinku;
}
void odswiezanieSegmentowWyswietlacza(void)
{
	static uint8_t i=4;
	wybor|=(1<<i);
	if(i++==7) i=4;
	uint8_t temperatura_calkowita=pomiar_temperatury[0];
	uint8_t temperatura_po_przecinku= oblicz_liczbe_po_przecinku(pomiar_temperatury[1]);
	if(ogolny_tryb_termometra==0)
	{
		czyscRodzajDiody();
		switch(tryb_pomiaru_termometra)
		{
			case 1:
			pokaz_temperature_w_celsjuszach(i-4,temperatura_calkowita,temperatura_po_przecinku);
			break;
			case 2:
			pokaz_temperature_w_kelwinach(i-4,temperatura_calkowita,temperatura_po_przecinku);
			break;
			case 3:
			pokaz_temperature_w_fahrenheitach(i-4,temperatura_calkowita,temperatura_po_przecinku);
			break;
		}
	}
	else if(ogolny_tryb_termometra==1) // tryb min
	{
		czyscRodzajDiody();
		diody &= ~diodaMin;
		uint8_t tempMin=temperatura_min[0];
		uint8_t tempMinPrzec=temperatura_min[1];
		tempMinPrzec=oblicz_liczbe_po_przecinku(tempMinPrzec);
		switch(tryb_pomiaru_termometra)
		{
			case 1:
			pokaz_temperature_w_celsjuszach(i-4,tempMin,tempMinPrzec);
			break;
			case 2:
			pokaz_temperature_w_kelwinach(i-4,tempMin,tempMinPrzec);
			break;
			case 3:
			pokaz_temperature_w_fahrenheitach(i-4,tempMin,tempMinPrzec);
			break;
		}
	}
	else if(ogolny_tryb_termometra==2) // wartosc MAX
	{
		czyscRodzajDiody();
		diody &= ~diodaMax;
		uint8_t tempMax=temperatura_max[0];
		uint8_t tempMaxPrzec=temperatura_max[1];
		tempMaxPrzec=oblicz_liczbe_po_przecinku(tempMaxPrzec);
		switch(tryb_pomiaru_termometra)
		{
			case 1:
			pokaz_temperature_w_celsjuszach(i-4,tempMax,tempMaxPrzec);
			break;
			case 2:
			pokaz_temperature_w_kelwinach(i-4,tempMax,tempMaxPrzec);
			break;
			case 3:
			pokaz_temperature_w_fahrenheitach(i-4,tempMax,tempMaxPrzec);
			break;
		}
	}
	else // RESET
	{
		static uint8_t licznikResetu=0;
		if(licznikResetu==0){
			wygasWszystkieDiody();
			diody &= ~zapalWszystkieDiody;
			licznikResetu++;
		}
		else if(licznikResetu++==100){ // ok 500 ms trwa flaga resetu
			licznikResetu=0;
			ogolny_tryb_termometra=tryb_przed_resetem;
		}
		
		
	}

	wybor&=~(1<<i);
}
void odswiez_temperature_MAX_i_MIN(void)
{
	if(pomiar_temperatury[0] > temperatura_max[0] ) // porownywanie po liczbach calkowitych
	{
		temperatura_max[0]=pomiar_temperatury[0];
		temperatura_max[1]=pomiar_temperatury[1];
	}
	else if(pomiar_temperatury[0] < temperatura_min[0] ) // porownywanie po liczbach calkowitych
	{
		temperatura_min[0]=pomiar_temperatury[0];
		temperatura_min[1]=pomiar_temperatury[1];
	}
	else if(pomiar_temperatury[0]==temperatura_max[0] && pomiar_temperatury[1] > temperatura_max[1]) // porownywanie po liczbach po przecinku
	{
		temperatura_max[0]=pomiar_temperatury[0];
		temperatura_max[1]=pomiar_temperatury[1];
	}
	else if(pomiar_temperatury[0] == temperatura_min[0] && pomiar_temperatury[1] < temperatura_min[1]) // porownywanie po liczbach po przecinku
	{
		temperatura_min[0]=pomiar_temperatury[0];
		temperatura_min[1]=pomiar_temperatury[1];
	}
}

ISR(TIMER0_OVF_vect)
{
	odswiezanieSegmentowWyswietlacza();
	//obsluga przyciskow
	if(!(przyciskiIN & (1<<SKALA)))
	{
		switch(tryb_skala)
		{
			case 0:
			tryb_skala=1;
			break;
			case 1:
			tryb_skala=2;
			break;
		}
	}
	else
	{
		switch(tryb_skala)
		{
			case 3:
			tryb_skala=4;
			break;
			case 4:
			tryb_skala=0;
			break;
		}
	}
	
	if(!(przyciskiIN & (1<<MIN)))
	{
		switch(tryb_min)
		{
			case 0:
			tryb_min=1;
			case 1:
			tryb_min=2;
			break;
		}
	}
	else
	{
		switch(tryb_min)
		{
			case 3:
			tryb_min=4;
			break;
			case 4:
			tryb_min=0;
			break;
		}
	}
	if(!(przyciskiIN & (1<<MAX)))
	{
		switch(tryb_max)
		{
			case 0:
			tryb_max=1;
			break;
			case 1:
			tryb_max=2;
			break;
			
		}
	}
	else
	{
		switch(tryb_max)
		{
			case 3:
			tryb_max=4;
			break;
			case 4:
			tryb_max=0;
			break;
		}
	}
	if(!(przyciskiIN & (1<<RESET)))
	{
		switch(tryb_reset)
		{
			case 0:
			tryb_reset=1;
			break;
			case 1:
			tryb_reset=2;
			break;
		}
	}
	else
	{
		switch(tryb_reset)
		{
			case 3:
			tryb_reset=4;
			break;
			case 4:
			tryb_reset=0;
			break;
		}
	}
}
ISR(TIMER1_COMPA_vect)
{
	// zczytywanie temperatury
	//TWI_start(); 
	//TWI_write(0x90 | 0x00); // adres czujnika LM75A + komendy zapisu i2c
	//TWI_write(0x00); // rejestr odczytu temperatury w LM75A
	//TWI_start();
	//TWI_write(0x90 | 0x01); // adres czujnika LM75A + komenda odczytu i2c
	//
	//pomiar_temperatury[0]=TWI_readAck(); // bajt temperatury calkowitej, z potwierdzenie
	//pomiar_temperatury[1]=TWI_readNak(); // bajt temperatury po przecinky, bez potwierdzenia
	//TWI_stop(); // koniec pomiaru
	
	//test wyswietlania
	if(pomiar_temperatury[0]++==36) pomiar_temperatury[0]=15;

	if(ilosc_pomiarow==0)
	{
		ilosc_pomiarow++;
		temperatura_max[0]=pomiar_temperatury[0];
		temperatura_max[1]=pomiar_temperatury[1];
		temperatura_min[0]=pomiar_temperatury[0];
		temperatura_max[1]=pomiar_temperatury[1];
	}
	odswiez_temperature_MAX_i_MIN(); // aktualizacja temperatury min i max
	
}
int main(void)
{
	TWI_init();
	segmentyDDR=0xff; // inicjacja segmentow
	wyborDDR=0xff; // inicjacja wyboru + portow dla diod trybu
	
	przyciskiDDR &= ~((1<<SKALA)|(1<<MIN)|(1<<MAX)|(1<<RESET)); // init przyciskow
	przyciski |= ((1<<SKALA)|(1<<MIN)|(1<<MAX)|(1<<RESET)); // pull up
	
	diodyRodzajTemperaturyDDR |= ustawPorty; // wy maskowanie portow dla diod
	wygasWszystkieDiody();
	
	// inicjacja timer'ów
	TCCR0 = (1<<CS02)|(0<<CS00); // ustawienie timera0 prescaler 256
	TCCR1B |=(1<<CS12) | (1<<WGM12); // tryb CTC , presclaer 256
	OCR1A=31250; // co pol sekundy przerwanie z timer1
	TIMSK |= (1<<TOIE0) | (1<<OCIE1A);
	sei(); // wlaczenie globalnych przerwan
	
	while (1)
	{
		// sprawdzanie aktywnych klawiszy
		if(tryb_skala==2)
		{
			if(++tryb_pomiaru_termometra==4) tryb_pomiaru_termometra=1; // kolejne zmiany trybu pomiaru temperatury - C, K , F , C ...
			ogolny_tryb_termometra=0;
			tryb_skala=3;
		}
		if(tryb_min==2)
		{
			ogolny_tryb_termometra=1;
			tryb_min=3;
		}
		if (tryb_max==2)
		{
			ogolny_tryb_termometra=2;
			tryb_max=3;
		}
		if(tryb_reset==2)
		{
			ilosc_pomiarow=0; // resetowanie pomiarow max i min
			tryb_przed_resetem=ogolny_tryb_termometra;
			ogolny_tryb_termometra=3;
			tryb_reset=3;
		}
	}
}
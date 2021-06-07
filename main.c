/*
 * Projekt-Name: Lichterkette
 *
 * Created (Datum): 23.12.2014
 *  Author: Nathanael
 */ 


#include <avr/io.h>
#define F_CPU 1200000UL  // 1,2 MHz
#include <util/delay.h>

#define led0on    PORTB |= (1<<PB0);
#define led0off   PORTB &= ~(1<<PB0);
#define led3on    PORTB |= (1<<PB3);
#define led3off   PORTB &= ~(1<<PB3);
#define led4on    PORTB |= (1<<PB4); 
#define led4off   PORTB &= ~(1<<PB4);

// ADC initialisieren
void ADC_Init(void)
{
  // Vcc als Referenzspannung benutzen
  ADMUX = (0<<REFS0);
  
  // Bit ADATE (ADTS2:0 in ADCSRB ist 0, was "free running" ergeben würde, wenn ADATE = 1 ist) in ADCSRA steht beim Einschalten
  // schon auf 0, also single conversion
  ADCSRA = (1<<ADPS1) | (1<<ADPS0);   // Frequenzvorteiler: 8
  ADCSRA |= (1<<ADEN);                // ADC aktivieren
  
  // "Dummy-Readout", um ADC warmlaufen zu lassen
  ADCSRA |= (1<<ADSC);                // eine ADC-Wandlung
  while(ADCSRA & (1<<ADSC))
  {
    // auf Abschluss der Konvertierung warten
    // da in ADCSRA das ADSC-Bit gesetzt ist und nach der Messung automatisch auf 0 gesetzt
    // wird, geht diese Schleife solang, bis das Bit "zurückspringt", also die Messung
    // beendet ist, da dann an dieser Stelle 0 (ADCSRA) und 1 (1<<ADSC) gegenüberstehen
  }
  (void) ADCW; // ADCW einmal lesen, sonst wird Ergebnis der nächsten Messung nicht übernommen
}

// ADC Einzelmessung
uint16_t ADC_Read(uint8_t channel)
{
  // Kanal wählen, ohne andere Bits zu beeinflussen
  ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F); // 0x1F = 31 = 0b00011111, d.h. die ersten drei Bits bleiben, wie sie sind, die letzten fünf werden null
  ADCSRA |= (1<<ADSC);                          // eine Wandlung "single conversion"
  while(ADCSRA & (1<<ADSC))
  {
    // Auf Abschluss der Konverstierung warten
    // da in ADCSRA das ADSC-Bit gesetzt ist und nach der Messung automatisch auf 0 gesetzt
    // wird, geht diese Schleife solang, bis das Bit "zurückspringt", also die Messung
    // beendet ist, da dann an dieser Stelle 0 (ADCSRA) und 1 (1<<ADSC) gegenüberstehen
  }
  return ADCW; // ADCW (auslesen und) zurückgeben
}

int main(void)
{
  // PB0, PB3 und PB4 ist Ausgang
  //DDRB = 25;
  //DDRB = 0b00011001;
  DDRB |= (1<<DDB0) | (1<<DDB3) | (1<<DDB4); // hier werden die Datenrichtungsbits einzeln reingeschoben
  
  PORTB = 0; // alle LEDs aus
  uint16_t adcval;    // ADC Value
  ADC_Init();
  while(1)
  {    
    uint8_t zufalls_port_alt = 3;
    // Beim ersten Mal soll nicht Port3 angesprochen werden (da der jedes 2. Mal an ist)
    uint8_t zufalls_port;
    for(uint8_t i = 0; i < 25; i++)
    {
      do
      {
        adcval = ADC_Read(1); // Channel 1, d.h. PB2 (=ADC1) wird als ADC benutzt
        zufalls_port = adcval % 3 + 3; // PB3 oder PB4 oder 5
        if(zufalls_port == 5)
        {
          // An PB5 ist keine LED (sondern RESET), deshalb wird daraus PB0
          zufalls_port = 0;
        }
      } while(zufalls_port == zufalls_port_alt);
      
      PORTB |= (1<<zufalls_port);
      _delay_ms(300);
      PORTB &= ~(1<<zufalls_port);
      zufalls_port_alt = zufalls_port;
    }
  }
  return 0;
}

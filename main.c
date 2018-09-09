/*
 * GccApplication1.c
 *
 * Created: 2018-07-23 19:10:12
 * Author : mysteq
 */

#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
//#include "dht.h"

#define BAUDRATE 9600
#define MY_UBRR 51

#define DHT_DDR DDRD
#define DHT_PORT PORTD
#define DHT_PIN PD2

uint8_t soil_humidity_raw = 0x00;
uint8_t soil_humidity = 0x00;
unsigned char last_state = 0;
unsigned char count_zeros = 0;
unsigned char count_ones = 0;
unsigned char dht_data[5];
unsigned char buff = 0x00;
unsigned char out_buff[128];
uint16_t changed_data = 0x0000;
int8_t  humidity = -2;
int8_t temperature = -2;

void init_USART(unsigned char ubrr){
    /* baud rate registers */
    UBRRL = (unsigned char)ubrr;
    UBRRH = (unsigned char)ubrr>>8;

    UCSRC = (1<<URSEL)|(1<<UPM1)|(1<<USBS)|(1<<UCSZ1)|(1<<UCSZ0); //even parity, 2 bit stop, 8 bit word
    UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);

}
/*
void init_TIMER2(){

    TCCR2 |= (1<<CS21); //clkT2S /8 (From prescaler)
    OCR2 = 10;
}
void stop_TIMER2()
{
    TCCR2 &= ~((1<<CS22)|(1<<CS21)|(1<<CS20));
}
 */

void init_INT0(){
    MCUCR |= (1<<ISC01); //INT0 failing edge
    GICR |=(1<<INT0);
}

void disable_INT0(){
    GICR &= ~(1<<INT0);
}
void start_TIMER0(){
    TCCR0 |= (1<<CS01); // clk/8
}
void stop_TIMER0(){
    TCCR0 |= 0x00;

}

void SEND_START(){
    DHT_DDR |= (1<<DHT_PIN);
    DHT_PORT &= ~(1<<DHT_PIN);
    _delay_ms(18);
    DHT_PORT |= (1<<DHT_PIN);
    DHT_DDR &= ~(1<<DHT_PIN);
}



void USART_send_char(char _x){
    while(!(UCSRA & (1<<UDRE)));
    UDR = _x;

}

void USART_send_string(){

    cli();
    for(unsigned char x = 0;x<128;++x){
        USART_send_char(out_buff[x]);
    }
    strncpy(out_buff,0x00,127);
    sei();
}


int main(void)
{
    DDRC|=1<<PORTC5;
    PORTC=0x00;

    init_USART(MY_UBRR);
    init_ADC();
    sei();
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wmissing-noreturn"
    while (1)
    {
        PORTC^=1<<PORTC5;
        ADCSRA |= (1<<ADSC);
        _delay_ms(3000);
/*
        DDRC|=(1<<PORTC4);
        PORTC |= (1<<PORTC4);
        _delay_ms(50);
        PORTC &= ~(1<<PORTC4);
        _delay_ms(25);
        PORTC|=(1<<PORTC4);
        _delay_us(35);
        PORTC &=~(1<<PORTC4);
        DDRC &= ~(1<<PORTC4);
*/
        snprintf(out_buff,127,"Hello world!\r\n");
        USART_send_string();

    }
#pragma clang diagnostic pop
}



ISR(USART_RXC_vect){
    //PORTC^=1<<PORTC5;

    buff = UDR;
    UDR = buff;
}

ISR(ADC_vect){
    //cli();
    PORTC^=1<<PORTC5;
    soil_humidity_raw = ADCH;
    //USART_send_string();
    //sei();
}

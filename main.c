/*
 * GccApplication1.c
 *
 * Created: 2018-07-23 19:10:12
 * Author : mysteq
 */

#define F_CPU 8000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include <util/atomic.h>
//#include "dht.h"


#define BAUDRATE 9600
#define MY_UBRR 51

#define DHT_DDR DDRD
#define DHT_PORT PORTD
#define DHT_PIN PD2
#define DDR_SPI     DDRB
#define DD_MISO     DDB4

#define DHT_START_TIME_L 130
#define DHT_START_TIME_H 140

#define DHT_BIT_ZERO_TIME_L 71
#define DHT_BIT_ZERO_TIME_H 81
#define DHT_BIT_ONE_TIME_L 118
#define DHT_BIT_ONE_TIME_H 128

#define FLAG_ERROR 0x02
#define FLAG_STARTED 0x01
#define FLAG_FINISHED 0x04
#define FLAG_SUMOK 0x08
#define SPI_BUFF_SIZE 64
#define DHT_DATA_SIZE 5
//#define USART_BUFF_SIZE 128
#define SPI_TRANS_ACTIVE 0x01
#define SPI_TRANS_INACTIVE 0x00
volatile unsigned char dht_data[DHT_DATA_SIZE];
volatile unsigned char buff = 0x00;
volatile unsigned char transmission_flags = 0x00;
volatile unsigned char bit_time = 0x00;
volatile unsigned char bits = 0x00;
volatile unsigned char sum = 0x00;
volatile unsigned char spi_byte_recv = 0x00;
unsigned char out_buff[SPI_BUFF_SIZE];
volatile unsigned char spi_copy_buff[SPI_BUFF_SIZE];
volatile unsigned char spi_buff_pos = 0;
volatile unsigned char spi_trans_status = 0x00;
uint64_t dht11_data = 0;
void init_USART(unsigned char ubrr){
    /* baud rate registers */
    UBRRL = (unsigned char)ubrr;
    UBRRH = (unsigned char)ubrr>>8;

    UCSRC = (1<<URSEL)|(1<<UPM1)|(1<<USBS)|(1<<UCSZ1)|(1<<UCSZ0); //even parity, 2 bit stop, 8 bit word
    UCSRB = (1<<RXCIE)|(1<<RXEN)|(1<<TXEN);

}

void init_INT0(){
    MCUCR |= (1<<ISC01); //INT0 failing edge
    GICR |=(1<<INT0);
}

void disable_INT0(){
    GICR &= ~(1<<INT0);
    TCNT0 = 0x00;
}
void start_TIMER0(){
    GICR |= (1<<INT0);
    TCCR0 |= (1<<CS01); // clk/8
}

void start_TIMER1(){
    TCCR1B |= (1<<CS12)|(1<<CS10);
    TCNT1 = 0;
}
void stop_TIMER1(){
    TCCR1B = 0x00;
}

void wait_TIMER1_20ms(){
    start_TIMER1();
    while(TCNT1L <= 156){}; //256*78 = 19968us
    stop_TIMER1();
}
void stop_TIMER0(){
    TCCR0 |= 0x00;
}
void parse_data(){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        dht_data[0] = ((unsigned char)(dht11_data));
        dht11_data >>= 8;
        dht_data[1] = ((unsigned char)(dht11_data));
        dht11_data >>= 8;
        dht_data[2] = ((unsigned char)(dht11_data)); //temp
        dht11_data >>= 8;
        dht_data[3] = ((unsigned char)(dht11_data));
        dht11_data >>= 8;
        dht_data[4] = ((unsigned char)(dht11_data)); //humidity
        dht11_data = 0;
    }
}
void abort_DHT(){
        stop_TIMER0();
        disable_INT0();
    //snprintf(out_buff,sizeof(out_buff),"time: %u\r\n",time);
}

void finish_DHT(){
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
        stop_TIMER0();
        disable_INT0();
        transmission_flags |= FLAG_FINISHED;
        parse_data();
        sum = dht_data[0];
        sum += dht_data[1];
        sum += dht_data[2];
        sum += dht_data[3];
        if(sum == dht_data[4]){
            transmission_flags |= FLAG_SUMOK;
        }
        else {
            transmission_flags |= FLAG_ERROR;
        }
    }
}

void start_DHT(){
        abort_DHT();
        dht_data[0] = 0x00;
        dht_data[1] = 0x00;
        dht_data[2] = 0x00;
        dht_data[3] = 0x00;
        dht_data[4] = 0x00;
        bits = 0x00;
        transmission_flags = 0x00;
        DHT_DDR |= (1 << DHT_PIN);
        DHT_PORT &= ~(1 << DHT_PIN);
        wait_TIMER1_20ms();
        DHT_DDR &= ~(1 << DHT_PIN);
        start_TIMER0();
        init_INT0();
}

void USART_send_char(char _x){
    while(!(UCSRA & (1<<UDRE)));
    UDR = _x;
}

void USART_send_string(){
    unsigned int l = (strlen(out_buff)+1 < SPI_BUFF_SIZE)?strlen(out_buff)+1: SPI_BUFF_SIZE;
    for(unsigned char x = 0;x <=l;++x){
        USART_send_char(out_buff[x]);
    }
}

void add_bit(unsigned char bit){
        dht11_data <<= 1;
        dht11_data |= bit;
        bits += 1;
}


void SPI_SlaveInit(void) {
/* Set MISO output, all others input */
    DDR_SPI = (1<<DD_MISO);
/* Enable SPI */
    SPCR = (1<<SPE);
    SPCR |= (1<<SPIE); //SPI slave interrupt enable
   // memset(spi_send_buff,0, sizeof(spi_send_buff));
}

int main(void)
{
    init_USART(MY_UBRR);
    sei();
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wmissing-noreturn"
    SPI_SlaveInit();
    for(uint8_t i = 0;i<200;i++){
        wait_TIMER1_20ms();
    }
    while (1)
    {
        start_DHT();
        for(uint8_t i= 0;i<200;i++){
            wait_TIMER1_20ms();
        }
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
            for(uint8_t i = 0; i< sizeof(spi_copy_buff);i++ ){
                spi_copy_buff[i] = out_buff[i];
            }
            if(transmission_flags & (FLAG_FINISHED|FLAG_SUMOK)){
                snprintf(out_buff, sizeof(out_buff),"TEMP %hhu HUM %hhu\n",dht_data[2],dht_data[4]);
            }else if(transmission_flags & (FLAG_FINISHED|FLAG_ERROR)){
                snprintf(out_buff, sizeof(out_buff),"ERROR\n");

            }
        }
        USART_send_string();
    }
#pragma clang diagnostic pop
}

ISR(USART_RXC_vect){
    buff = UDR;
    UDR = buff;
}

ISR(TIMER0_OVF_vect){
    transmission_flags |= FLAG_ERROR;
    abort_DHT();
}

ISR(INT0_vect){

    bit_time = TCNT0;
    TCNT0 = 0;
    if(transmission_flags & FLAG_ERROR){
        abort_DHT();
    }
    if(bit_time >= DHT_BIT_ZERO_TIME_L && bit_time <= DHT_BIT_ZERO_TIME_H){
        add_bit(0x00);
    }
    else if(bit_time >= DHT_BIT_ONE_TIME_L && bit_time <= DHT_BIT_ONE_TIME_H)
    {
        add_bit(0x01);
    }
    else if(bit_time >= DHT_START_TIME_L && bit_time<=DHT_START_TIME_H){
        transmission_flags = FLAG_STARTED;
    }
    else if(bit_time > (DHT_START_TIME_L+DHT_BIT_ZERO_TIME_L)){
        transmission_flags |= FLAG_ERROR;
        abort_DHT();
    }
    if(bits==40){
        finish_DHT();
    }


}

ISR(SPI_STC_vect){
    spi_byte_recv = SPDR;
        if(SPDR == 'S'){
            spi_buff_pos = 0x00;
        }
        if(spi_buff_pos >= sizeof(spi_copy_buff)){
            SPDR = 0x00;
        }
        else {
            SPDR = spi_copy_buff[spi_buff_pos%sizeof(spi_copy_buff)];
            spi_buff_pos = (spi_buff_pos +1)%sizeof(spi_copy_buff);
        }
}

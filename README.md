### DHT11 Protocol implementation
This implementation is not using "_delay_ms" nor "_delay_us" from delay.h library.

It's just TIMER0 and TIMER1, UART and INT0 

### HOWTO
Just clone and change PINs in #define section. It's tuned for 8MHz internal oscillator.

Times have been choosen during experiments and set arbitrally

### Example output
`TEMP 27 HUM 43`

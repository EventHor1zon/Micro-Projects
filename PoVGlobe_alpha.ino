/*
*   Program:  Persistence of Vision Globe
*   Version:  0.1
*   Date:     12/6/19
*   Author:   R Miller
*   Language: C++
*   Compiler: avrdude via Arduino IDE
*   IDE:      Atom IDE, Arduino IDE
*   Platform: ATMega328p @ 16mHz
*
*   Will use 16-bit Timer1 for timing events
*     Registers -  TCNT1, OCR1A, OCR1B, ICR1
*/

/*
 *  Notes:
 *        Got spi succesfully working with pointers. 
 *        Also got image stored and read from progmem succesfully
 *        made python program to turn .ppm files into the correct image type
 *        made test images & tested them
 *        
 *         STILL have problems with the START/END protocol of LEDs. First one always white with 'end', dark with 'start'
 *          
 * 
 */



#include <avr/io.h>
#include <avr/pgmspace.h>

#include <Arduino.h>

#include "image7.h"

#define NUMLEDS     9
#define T_MAX       0xFFFF



void spi_start();
void spi_end();
void spi_write(uint8_t *x);
void write_frame(uint32_t *data);

bool GO = 0;
bool SPI_W = 0;
uint8_t LED_START = 0x00;
uint8_t LED_END = 0xFF;
uint8_t GLOBAL = 0xE0;
uint8_t BRIGHTNESS = 0x1F;
uint32_t T_NOW = 0;
uint32_t T_LAST = 0;
uint32_t T_ROT = 0;
uint16_t FRAME_T=0;
uint16_t FRAME_C=0;
uint16_t FRAME_NO = 126;
uint16_t TEMP =0;
uint16_t img_size = FRAME_NO * NUMLEDS * 4;
void *image_ptr;


uint32_t data[NUMLEDS];

/***************   INTERRUPT FUNCTIONS *********************/


ISR(INT0_vect){                           //  External Interrupt Vector on PORTD[2] - nano pin D2, PCB pin 20
    // (if can't turn off interrupts during an interrupt, use flags instead...)
    unsigned char sreg = SREG;            // store interrupt settings
    cli();                                // clear interrupts
    T_NOW = TCNT1;                        // get time now - rem, triggers twice per spin!
    if(T_LAST > T_NOW){                   // if timer overflown, must deal with differently
      T_ROT=T_NOW+(T_MAX-T_LAST);           // delta_T = t_now + (difference in t_last and max)
    } else { T_ROT = T_NOW - T_LAST; }    // otherwise t_rot = t_now - t_last
    T_LAST=T_NOW;                         // set t_last for next use
    FRAME_T = T_ROT/(FRAME_NO/2);         // get frame_t - triggers twice so divide frame_no by 2
    if(FRAME_T < T_MAX) { 
        GO = 1;
      }
    else {
        GO = 0;
      }
    SREG=sreg;                            // restore interrupt settings
    PIND = (1 << PIND3);
  }                                       // end interrupt

ISR(TIM1_COMPA_vect){                     // interrupt on timer match; i.e FRAME_T reached
  // is this too much code for an interrupt? Could use flags instead.
  unsigned char sreg=SREG;                // store interrupt settings
  cli();                                  // clear interrupts
  FRAME_C++;                              // advance the frame counter
  if (FRAME_C > FRAME_NO) FRAME_C = 0;    // if frames completed, reset frame_c
  SPI_W=1;                                // set SPI write flag - don't write here, too CPU intensive
  TEMP=TCNT1+FRAME_T;                     // find value for next interrupt. In case of overflow, use temp buffer
  if(TEMP > T_MAX){                         // if higher than counter max value
    OCR1A = (uint16_t)(TEMP-T_MAX);         // next frame is t - max temp variable? May need to typecast from a larger val)
  } else { OCR1A = (uint16_t)TEMP; }      // otherwise set next interrupt to temp - may need to typecast?
  SREG=sreg;                              // restore interrupts
}


void spi_write(uint8_t *x){                // SPI write function ( uses uint8_t as 8-byte val)
    SPDR = *x;                             // pass x into the 8 bit spi data reg
    while ((SPSR & (1<<SPIF)) == 0);      // wait until SPI transaction concluded (compare SPI reg with SPI done flag)
}


void spi_start(){               // start frame 32 * 0 bits
  for(int i=0; i<4; i++){       // send 4 lots of START
    spi_write(&LED_START);           // (start is 0)
  }
}

void spi_end(){                 // end frame 32 * 1 bits
  for(int i=0; i<32; i++){       // send 4 lots of END - try more - see note
      spi_write(&LED_END);             // end = 1
  }
}

void write_frame(uint32_t *frame_ptr){                                        // takes pointer to data structure
                                                                              // if using progmem to hold data must retrieve with special functions first.
    memcpy_P((void *)&data, (const void *)frame_ptr, sizeof(uint32_t)* 9);    // retrieve frame data from progmemory
    uint8_t *ptr=(uint8_t *)data;                                             // a uint8_t pointer to address in data
    spi_start();                                                              // write startframe
    for(int i=0; i<NUMLEDS; i++){                                                             // for each LED
    spi_write(ptr);                        // write global
    spi_write(ptr+1);                   // next 8 bits for red
    spi_write(ptr+2);                // next 8 bits for green
    spi_write(ptr+3);                // next 8 for blue
    ptr += 4;                        //  uint8_t POINTERS MOVE IN size of uint8_ts! NOT BITS, FOO!!
    }
    spi_end();                                      // send end frame
}


void initSPI(){                           // sets up SPI interface
    PRR = (0 << PRSPI);                     // turns SPI on in Power Reduction registers
    DDRB = (1<<DDB2)|(1<<DDB3)|(1<<DDB5);     // init PortB pins as output - SS, MOSI, SCK
    SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR1)|(0<<SPR0);  // spi_enable | spi_master mode | spi speed setting
    SPSR = (1<<SPI2X);                      // set double speed on spi
    PORTB &= ~(1<<PB2);                     // set SS pin low - LEDs are dumb (technical term) but still need SS low in order to write out
}


void initTimer1(){

    TCCR1A = (0 << WGM11) | (0 << WGM10); // select normal mode in timer control register
    TCCR1B = (0 << WGM13) | (0 << WGM12) | (0 << CS12) | (1 << CS11) | (0 << CS10);   // set prescaler to 64
    TIMSK1 = (1 << OCIE1A);               // turn on interrupt if Output compare A = timer val
    TIFR1 = (1 << OCF1A);                 // generate flag on match ocA->timer, enables interrupt
    
  
  }

void initGPIO(){
    // use the int0 (D2) pin for the external interrupts
    // interrupt on falling edge of signal

    DDRD = (0 << DDD2) | (1 << DDD3);         // set d2 as input, d3 as output
    PORTD = (0 << PORTD0) | (1 << PORTD3);      // disable pullup resistor, put d3 out high
    EIMSK = (1 << INT0);                      // enable interrupt mask
    EICRA = (1 << ISC01) | (1 << ISC00);    // select  rising edge int
    EIFR = (1 << INTF0);                    // enable interrupt routine flag
  
  }

void setup() {
  // put your setup code here, to run once:

  initGPIO();
  //initTimer1();
  initSPI();

  image_ptr=&image[0];
}


void wait(int iters){
  for(int f= 0; f<iters; f++){
    for(int i=0; i<65535; i++){
        int a = 1000;
        int b = 1;
        for(int j=0;j<1000; j++){
            a-=b;
          }
      }
    }
  }

void loop() {
  // put your main code here, to run repeatedly:
  int sz = (NUMLEDS * sizeof(uint32_t));
  
  //Serial.println(buff);
  for(int i=0; i<FRAME_NO; i++){
        write_frame(image_ptr);
        image_ptr+=sz;      // increment pointer to next frame
  //    memset((void *)&buff, "\0", sizeof(buff));
  //    sprintf(buff, "pointer val: %p\n", image_ptr);
  //    Serial.println(buff);
        delay(10);
  }
  image_ptr=&image[0];
  delay(1000);
  
}


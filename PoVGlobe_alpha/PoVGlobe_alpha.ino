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
 *        [x] Still have problems with the START/END protocol of LEDs. First one always white with 'end', dark with 'start' ::: FIXED
 *         [+]Error was in the endedness of the data shifting.
 *        [x] Need to solve how to write LEDs as a single strip
 *          [+] Sorted with offset and a reverse array reading function, quite trivial
 *        [-] Problem with image converting script needs to swap blue/red pixel values
 *          
 * 
 */



#include <avr/io.h>
#include <avr/pgmspace.h>

#include <Arduino.h>

#include "image5.h"

#define NUMLEDS     9
#define T_MAX       0xFFFF


/************* Function Prototypes *****************/

void spi_start();
void spi_end();
void spi_write(uint8_t *x);
void write_frame(uint32_t *data);
void write_frame_fwd(uint32_t *data);
void write_frame_rev(uint32_t *data);


/************* VARIABLES ****************************/

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
int frame_size = (NUMLEDS * sizeof(uint32_t));
void *image_ptr;


uint32_t data[NUMLEDS];
uint32_t OFFSET = NUMLEDS * FRAME_NO;

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

    SREG=sreg;                            // restore interrupt settings
    PIND = (1 << PIND3);
  }                                       // end interrupt

ISR(TIM1_COMPA_vect){                     // interrupt on timer match; i.e FRAME_T reached
  // is this too much code for an interrupt? Could use flags instead.
  unsigned char sreg=SREG;                // store interrupt settings
  cli();                                  // clear interrupts
  FRAME_C++;                              // advance the frame counter
  if (FRAME_C > FRAME_NO) FRAME_C = 0;    // if frames completed, reset frame_c
//  SPI_W=1;                                // set SPI write flag - don't write here, too CPU intensive
  TEMP=TCNT1+FRAME_T;                     // find value for next interrupt. In case of overflow, use temp buffer
  if(TEMP > T_MAX){                         // if higher than counter max value
    OCR1A = (uint16_t)(TEMP-T_MAX);         // next frame is t - max temp variable? May need to typecast from a larger val)
  } else { OCR1A = (uint16_t)TEMP; }      // otherwise set next interrupt to temp - may need to typecast?
  SREG=sreg;                              // restore interrupts
}

/****************** SPI FUNCTIONS **********************/

void spi_write(uint8_t *x){                // SPI write function ( uses uint8_t as 8-byte val)
    SPDR = *x;                             // pass x into the 8 bit spi data reg
    while ((SPSR & (1<<SPIF)) == 0);      // wait until SPI transaction concluded (compare SPI reg with SPI done flag)
    return;
}


void spi_start(){               // start frame 32 * 0 bits
  for(int i=0; i<4; i++){       // send 4 lots of START
    SPDR = 0x00;                             // pass x into the 8 bit spi data reg
    while ((SPSR & (1<<SPIF)) == 0);      // wait until SPI transaction concluded (compare SPI reg with SPI done flag)
  }
}

void spi_end(){                 // end frame 32 * 1 bits
  for(int i=0; i<2; i++){       // send 4 lots of END - try more - see note
    SPDR = 0xFF;                             // pass x into the 8 bit spi data reg
    while ((SPSR & (1<<SPIF)) == 0);      // wait until SPI transaction concluded (compare SPI reg with SPI done flag)
  }
}

void write_frames(uint32_t *frame_ptr){                                        // takes pointer to data structure
                                                                              // if using progmem to hold data must retrieve with special functions first.
    //memcpy_P((void *)&data, (const void *)frame_ptr, sizeof(uint32_t)* 9);    // retrieve frame data from progmemory
    //uint8_t *ptr=(uint8_t *)&data[0];                                             // a uint8_t pointer to address in data
    spi_start();                                                              // write startframe
    write_frame_rev(frame_ptr);                                               // write 1st half of globe
    write_frame_fwd(frame_ptr+OFFSET);                                    // write 2nd half of globe -reverso- style
    spi_end();                                                                // send end frame
}

void write_frame_rev(uint32_t *frame_ptr){
    
    memcpy_P((void *)&data, (const void *)frame_ptr, sizeof(uint32_t) * NUMLEDS);
    uint8_t *ptr=(uint8_t *)&data[NUMLEDS-1];                                // for the 1st frame, write bottom to top
    for(int i=0; i<NUMLEDS; i++){ 
        spi_write(ptr+3);                                                    // write global
        spi_write(ptr+2);                                                    // next 8 bits for blue
        spi_write(ptr+1);                                                    // next 8 bits for green
        spi_write(ptr);                                                      // next 8 for red
        ptr -= 4;                                                            //  move backwards through the array
      }
  }

void write_frame_fwd(uint32_t *frame_ptr){

    memcpy_P((void *)&data, (const void *)frame_ptr, sizeof(uint32_t) * NUMLEDS);
    uint8_t *ptr=(uint8_t *)&data[0];                                        // for the 2nd frame, write top to bottom
    for(int i=0; i<NUMLEDS; i++){ 
        spi_write(ptr+3);                                                    // write global
        spi_write(ptr+2);                                                    // next 8 bits for blue
        spi_write(ptr+1);                                                    // next 8 bits for green
        spi_write(ptr);                                                      // next 8 for red
        ptr += 4;                                                            //  move forwards through the array 
      }
  }

/********************* INIT FUNCTIONS ****************************/

void initSPI(){                           // sets up SPI interface
    PRR = (0 << PRSPI);                     // turns SPI on in Power Reduction registers
    DDRB = (1<<DDB2)|(1<<DDB3)|(1<<DDB5);     // init PortB pins as output - SS, MOSI, SCK
    SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR1)|(0<<SPR0);  // spi_enable | spi_master mode | spi speed setting
    SPSR = (0<<SPI2X);                      // set double speed on spi
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
    PORTD = (1 << PORTD2) | (0 << PORTD3);      // disable pullup resistor, put d3 out high
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


/************ MAIN LOOP ******************/

void loop() {
  // put your main code here, to run repeatedly:
  
  
  
}


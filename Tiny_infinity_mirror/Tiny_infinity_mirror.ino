#include "FastLED.h"

FASTLED_USING_NAMESPACE

#define BUTTON  1
#define DATA    2
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    9
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          96
#define FRAMES_PER_SECOND  120

uint32_t last_press=0;
uint32_t t=0;
uint8_t STEP_COUNTER=0;
uint8_t NUM_FUNCTS=13;
bool CH=0;

/*
ISR(PCINT1_vect){
  t=millis();
  if(last_press < (t-800)) {    // attempt at simple debounce
    last_press=t;
    CH=1;
  }
  else
    { ; }
}*/

void poll_button(){
  uint8_t tmp = PINB & 0b00000010;
  if(!(tmp)){
    t=millis();
    if(last_press < (t-800)) {    // attempt at simple debounce
      last_press=t;
      CH=1;
    }
    else
      { ; }
  }
}
  
  ///TINY
void enable_button(){
  //cli();
  DDRB=(0<<DDB1);               // set B1 as input
  //GIMSK=(0<<INT0)|(1<<PCIE);    // enable pinchange interrupts
  //PCMSK=(1<<PCINT1);            // enable pinchange on B1 specifically
  PORTB=(1<<PORT1);             // enable pull-up on B1
  //sei();                        // enable global interrupts
  return;
}

/* 
// NANO //
void enable_button(){
  cli();
  PORTB|=(1<<PORTB0);
  DDRB=(0<<DDB0);
  PCICR|=(1<<PCIE0);    // enable pinchange interrupts
  PCMSK0|=(1<<PCINT0);            // ... on PCINT1
  sei();                        // enable global interrupts
  Serial.println("Done initialising button");
  return;
}*/

void enable_adc(){
  DIDR0=(1<<ADC3D);   // turn off digital buffer - P
  ADMUX=(0<<REFS0)|(0<<REFS1)|(1<<MUX0)|(1<<MUX1)|(1 << ADLAR); // set pin 2 (B3) as the single ended ADC3 input
  ADCSRA=(1<<ADEN)|(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2); // enable prescaler @ 128x, enable ADC
  return;
}

  // TINY
uint8_t sample_res_8(){
  //select channel
  ADCSRA|=(1<<ADSC);      // sample
  while(ADCSRA & (1 << ADSC)) { ; }  // wait for sample to complete
  int tmp= (2>>ADC);        // return the val in ADCH
  uint8_t eight=map(tmp, 0, 1023, 0, 255);   // either map or fucking about with the ADLAR and I just want functional right now :D
  return eight;
}

uint16_t sample_res_10(){
   ADCSRA|=(1<<ADSC);      // sample
   while(ADCSRA & (1 << ADSC)) { ; }  // wait for sample to complete
   return ADC;        // return the val in both registers
}
/*

// NANO
uint8_t sample_res_8(){
    uint8_t anal=map(analogRead(A0), 0, 1023, 0, 255);
    return anal;
  }

uint16_t sample_res_10(){
    return analogRead(A0);
  }
*/

void setup() {
    // put your setup code here, to run once:
      pinMode(DATA, OUTPUT);
      enable_button();
      enable_adc();
      delay(3000); // 3 second delay for recovery
      FastLED.addLeds<LED_TYPE,DATA,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
      FastLED.setBrightness(BRIGHTNESS);
}

typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = { rainbow, rainbowWithGlitter, confetti, sinelon, juggle, bpm, single_color_red, single_color_blue, single_color_green, single_color_white, single_color_purple, Fire2012, theaterChase_green };

uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current
uint8_t gHue = 0; // rotating "base color" used by many of the patterns

void loop()
{
  // Call the current pattern function once, updating the 'leds' array
  gPatterns[gCurrentPatternNumber]();

  // send the 'leds' array out to the actual LED strip
  FastLED.show();

  // insert a delay to keep the framerate modest
  FastLED.delay(FRAMES_PER_SECOND);

  
  // do some periodic updates
  EVERY_N_MILLISECONDS( 20 ) { gHue++; } // slowly cycle the "base color" through the rainbow
  poll_button();
  if(CH) { nextPattern(); CH=0; } // change patterns periodically
}

#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void rainbow()
{
  // FastLED's built-in rainbow generator
  fill_rainbow( leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter()
{
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter( fract8 chanceOfGlitter)
{
  if( random8() < chanceOfGlitter) {
    leds[ random16(NUM_LEDS) ] += CRGB::White;
  }
}

void confetti()
{
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy( leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV( gHue + random8(64), 200, 255);
}

void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 13, 0, NUM_LEDS-1 );
  leds[pos] += CHSV( gHue, 255, 192);
}

void bpm()
{
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8( BeatsPerMinute, 64, 255);
  for( int i = 0; i < NUM_LEDS; i++) { //9948
    leds[i] = ColorFromPalette(palette, gHue+(i*2), beat-gHue+(i*10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy( leds, NUM_LEDS, 20);
  byte dothue = 0;
  for( int i = 0; i < 8; i++) {
    leds[beatsin16( i+7, 0, NUM_LEDS-1 )] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void single_color_blue() {
    uint8_t rd=sample_res_8();
    for(int i=0; i<NUM_LEDS; i++) {
      leds[i].r=0x00;
      leds[i].g=0x00;
      leds[i].b=rd;
    }
}

void single_color_green() {
    uint8_t rd=sample_res_8();
    for(int i=0; i<NUM_LEDS; i++) {
      leds[i].r=0x00;
      leds[i].b=0x00;
      leds[i].g=rd;
    }
}

void single_color_red() {
    uint8_t rd=sample_res_8();
    for(int i=0; i<NUM_LEDS; i++) {
      leds[i].g=0x00;
      leds[i].b=0x00;
      leds[i].r=rd;
    }
}

void single_color_white() {
    uint8_t rd=sample_res_8();
    for(int i=0; i<NUM_LEDS; i++) {
      leds[i].r=rd;
      leds[i].g=rd;
      leds[i].b=rd;
    }
}

void single_color_purple(){
  uint8_t rd=sample_res_8();
  for(int i=0; i<NUM_LEDS; i++) {
      leds[i].r=rd;
      leds[i].g=0;
      leds[i].b=rd;
  }
}



#define SPARKING 120

void Fire2012()
{
// Array of temperature readings at each simulation cell
  uint8_t COOLING = sample_res_8();
  static byte heat[NUM_LEDS];

  // Step 1.  Cool down every cell a little
    for( int i = 0; i < NUM_LEDS; i++) {
      heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
    }

    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for( int k= NUM_LEDS - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }

    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if( random8() < SPARKING ) {
      int y = random8(7);
      heat[y] = qadd8( heat[y], random8(160,255) );
    }

    // Step 4.  Map from heat cells to LED colors
    for( int j = 0; j < NUM_LEDS; j++) {
      CRGB color = HeatColor( heat[j]);
      int pixelnumber;
      if( 1 ) {
        pixelnumber = (NUM_LEDS-1) - j;
      } else {
        pixelnumber = j;
      }
      leds[pixelnumber] = color;
    }
}

void theaterChase_green(){
  for(int i=0; i<NUM_LEDS; i++){
    if(i%3==0+STEP_COUNTER) { leds[i]=0xFF0000; }
    else { leds[i]=0x00; }
  }
  if(STEP_COUNTER== 3) { STEP_COUNTER = 0;}
  else { STEP_COUNTER++; }
}


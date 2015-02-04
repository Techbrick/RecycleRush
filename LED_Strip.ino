#include <Wire.h>
#include <FastLED.h>
#define NUM_LEDS 80 
#define NUM_CHASE 10
#define NUM_DANCES 8
int green[NUM_CHASE];

CRGB leds[NUM_LEDS];
volatile int dance = 0;
volatile int count = 0;

void setup()
{
  pinMode(13, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(2, INPUT);
  
  Wire.begin(5);
  Wire.onReceive(receiveEvent);
  
  //This is the chipset in the AM-2640 LED strip using the default Data/CLK pins
   FastLED.addLeds<WS2801, RGB>(leds, NUM_LEDS);
   for (int i = 0; i < NUM_CHASE; i++)
   {
     green[i] = NUM_LEDS/NUM_CHASE*i;
   }
   
   //attachInterrupt(0, nextDance, RISING);
}

void loop()
{
  switch (dance) {
    case 0:
      allRed();
      break;
    case 1:
      allGreen();
      break;
    case 2:
      rainbow();
      break;
    case 3:
      scrollIn();
      break;
    case 4:
      tyson();
      break;
    case 5:
      dillon();
      break;
    case 6:
      alexis();
      break;
     case 7:
       greenScroll();
       break;
  }
  
  FastLED.show();
}

void receiveEvent(int howMany)
{
  while (Wire.available()) {
    int c = Wire.read();
    if (c < NUM_DANCES) {
      if (c != dance) {
        dance = c;
        count = 0;
      }
    }
  }
}

void allRed()
{  
  /*if(count == 0) {
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i].setRGB((int)random(256),(int)random(256),(int)random(256));
    }
  }
  else
  {*/
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i].setRGB(255,0,0);
    } 
  //}
  
  delay(60);
  //count = (count + 1) % 2;
}

void allGreen()
{  
  for (int i = 0; i < NUM_LEDS; i++) {
      leds[i].setRGB(0,255,0);
  } 
  
  delay(60);
}

void greenScroll()
{
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].setRGB(50,0,0);
  }
  
  for (int i = 0; i < NUM_CHASE; i++)
  {
    green[i] = (NUM_LEDS / NUM_CHASE * i + count) % NUM_LEDS;
    leds[green[i]].setRGB(0,50,0);
  }
  
  count = (count + 1) % NUM_LEDS;
  delay(60);  
}

void rainbow()
{
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].setHSV((int)((256.0/(NUM_LEDS-1))*((i+count)%NUM_LEDS)),255,150);
  }
  count = (count + 1) % NUM_LEDS;
  delay(60);
}

void countBar()
{
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].setRGB(255,255,255);
  }
  
  if (count < NUM_LEDS)
  {
    for (int i = 0; i < count; i++)
    {
      leds[i].setRGB(0,0,255); 
    }
  }
  else
  {
    for (int i = 0; i < 2 * NUM_LEDS - count; i++)
    {
      leds[i].setRGB(0,0,255); 
    }
  }

  count = (count + 1) % (2 * NUM_LEDS - 1);
  delay(30);  
}

void tyson()
{
   for (int i = 0; i < NUM_LEDS; i++) {
    if (count == 0) {
     leds[i].setRGB(255,0,0);
    }
    else if (count == 1) {
     leds[i].setRGB(0,0,255);
    }
    else if (count == 2) {
     leds[i].setRGB(255,0,255);
    }
  }
  
  count = (count + 1) % 3;
  delay(500);
}

void dillon()
{
  for (int i = 0; i < NUM_LEDS/3; i++) {
    leds[(i+count)%NUM_LEDS].setRGB(255,255,0);
  }
  for (int i = NUM_LEDS/3; i < 2*NUM_LEDS/3; i++) {
    leds[(i+count)%NUM_LEDS].setRGB(0,100,0);
  }
  for (int i = 2*NUM_LEDS/3; i < NUM_LEDS; i++) {
    leds[(i+count)%NUM_LEDS].setRGB(0,0,100);
  }
  
  count = (count + 1) % NUM_LEDS;
  delay(30);
}

void alexis()
{
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].setRGB(0,0,0);
  }
  if (count < NUM_LEDS/2) {
    for (int i = 0; i < NUM_LEDS/2-count; i++) {
      leds[i].setRGB(255,0,255);
    }
    for (int i = NUM_LEDS/2+count; i < NUM_LEDS; i++) {
      leds[i].setRGB(0,0,100);
    }
  }
  else {
    for (int i = 0; i < count-NUM_LEDS/2; i++) {
      leds[i].setRGB(255,0,255);
    }
    for (int i = 3*NUM_LEDS/2-count; i < NUM_LEDS; i++) {
      leds[i].setRGB(0,0,100);
    }
  }
  
  count = (count + 1) % NUM_LEDS;
  delay(15);  
}

void scrollIn() {
  int space = 8;
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i].setRGB(0,0,0);
  }
  for (int i = count; i < NUM_LEDS/2; i += space) {
    leds[i].setRGB(0,0,255);
  }
  for (int i = NUM_LEDS-count; i > NUM_LEDS/2; i -= space) {
    leds[i].setRGB(0,0,255);
  }
  count = (count + 1) % space;
  delay(80);
}

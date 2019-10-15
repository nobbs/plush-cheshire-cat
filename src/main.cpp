#include <Arduino.h>
#include <FastLED.h>
#include <LowPower.h>



// interrupt pin
#define INT_PIN 2
#define INT_MODE CHANGE

// some led information
#define LED_PIN 6
#define LED_NUM 40
#define LED_CHIP WS2812B
#define LED_COLOR_ORDER GRB

#define SERIAL_BAUD 115200

CRGB eyes[LED_NUM];

// #define NIGHTFURY

#ifdef NIGHTFURY
  uint8_t hue = 90;
  uint8_t hue_from = 80;
  uint8_t hue_to = 100;
#else
  uint8_t hue = 145;
  uint8_t hue_from = 35;
  uint8_t hue_to = 215;
#endif

uint8_t bri = 150;
uint8_t sat = 255;
uint8_t value = 255;

uint8_t cache_bri = 0;

void wait_a_while() {
  unsigned long start = millis();
  while (millis () - start < 1000) {
    // noInterrupts();
    // interrupts();
  }
}

void shut_off_lights() {
  cache_bri = FastLED.getBrightness();
  FastLED.setBrightness(0);
  // FastLED.clear(true);
  FastLED.show();
  wait_a_while();
}

void turn_on_lights() {
  FastLED.setBrightness(cache_bri);
  FastLED.show();
  wait_a_while();
}

void wake_up();

void sleep();

void wake_up() {
  detachInterrupt(0);
  turn_on_lights();
  EIFR = bit (INTF0);  // clear flag for interrupt 0
  attachInterrupt(0, sleep, INT_MODE);
}

void sleep() {
  // FastLED.clear(true);
  // Serial.println("Sleep!");
  // wait_a_while();
  detachInterrupt(0);
  shut_off_lights();
  EIFR = bit (INTF0);  // clear flag for interrupt 0
  attachInterrupt(0, wake_up, INT_MODE);
  LowPower.idle(SLEEP_FOREVER, ADC_ON, TIMER2_ON, TIMER1_ON, TIMER0_ON, SPI_ON, USART0_ON, TWI_ON);
}

void setup() {
  // set up serial port
  Serial.begin(SERIAL_BAUD);

  while (!Serial) {
    delay(50);
  } // Wait

  Serial.println("Starting...");

  // set up interrupts
  // attachInterrupt(digitalPinToInterrupt(INT_PIN), enter_sleep, LOW);
  pinMode(INT_PIN, INPUT);
  // digitalWrite(INT_PIN, INT_MODE);
  // attachInterrupt(digitalPinToInterrupt(INT_PIN), go_sleep, LOW);

  FastLED.addLeds<LED_CHIP, LED_PIN, LED_COLOR_ORDER>(eyes, LED_NUM).setCorrection(TypicalPixelString);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 250);
  FastLED.setBrightness(bri);

  eyes[0].setHue(hue);
  eyes[1].setHue(hue);
  FastLED.show();
  Serial.println("Glowing!");


  delay(250);
  // attachInterrupt(0, wake_up, LOW);
  attachInterrupt(0, sleep, INT_MODE);  
}

void blink() {
  uint8_t before = FastLED.getBrightness();
  uint8_t factor = 10;
  uint16_t steps = (before - 0) / factor;
  uint16_t dur_step = 5;
  uint16_t dur_dark = 100;

  Serial.println(dur_step);

  for(uint16_t s = 0; s < steps; s++) {
      FastLED.setBrightness(before - factor * s);
      FastLED.show();
      delay(dur_step);
  }

  FastLED.setBrightness(0);
  FastLED.show();
  delay(dur_dark);

  for(uint16_t s = steps; s > 0; s--) {
      FastLED.setBrightness(before - factor * s);
      FastLED.show();
      delay(dur_step);
  }

  FastLED.setBrightness(before);
  FastLED.show();
}

void colorchange() {
  uint8_t cur_hue = hue;
  uint8_t next_hue = random(hue_from, hue_to);
  int8_t dir = (cur_hue > next_hue) ? -1 : 1;

  Serial.print(cur_hue);
  Serial.print(" to ");
  Serial.print(next_hue);
  Serial.println();

  for(cur_hue = hue; cur_hue != next_hue; cur_hue = cur_hue + dir) {
    for (uint8_t idx = 0; idx < LED_NUM; idx++)
      eyes[idx].setHue(cur_hue);
    FastLED.show();
    delay(25);
  }
  hue = cur_hue;
}

void loop() {
  // blink();
  for(size_t i = 0; i < 5; i++) {
    colorchange();
  }  
}
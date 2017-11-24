/**
 * Simple Injection Controller for CUB(JBH-AA01)
 * http://www.suke-blog.com
 */

#include <Arduino.h>
#include <stdio.h>

#define SERIAL_RATE 115200
#define SERIAL_TIMEOUT 1000

//PIN Config
#define PIN_ECU_INJECTOR_SIG 3
#define PIN_INJECTOR_OUT 8
#define PIN_MODE0 A0
#define PIN_MODE1 A1
#define PIN_MODE2 A2
#define PIN_LED 13

uint8_t g_mode = 0;
const uint32_t g_modeTable[] = {
   0,    //0
   1000,    //1
   2000,    //2
   3000,    //3
   4000,    //4
   5000,    //5
   6000,    //6
   7000     //7
};

// forDebug
static FILE uartout;
static int uart_putchar(char c, FILE *stream) {
  if(Serial.write(c) > 0) {
    return 0;
  } else {
    return -1;
  }
}

void setup(){
  //forDebug
  fdev_setup_stream(&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartout;
  // config serial
  Serial.begin(SERIAL_RATE);
  Serial.setTimeout(SERIAL_TIMEOUT);

  //GPIO setting
  pinMode(PIN_ECU_INJECTOR_SIG, INPUT_PULLUP);
  pinMode(PIN_INJECTOR_OUT, OUTPUT);
  pinMode(PIN_MODE0, INPUT_PULLUP);
  pinMode(PIN_MODE1, INPUT_PULLUP);
  pinMode(PIN_MODE2, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  //Injector off
  digitalWrite(PIN_INJECTOR_OUT, LOW);

  //debug led OFF
  digitalWrite(PIN_LED, LOW);

  //delay 100ms
  delay(100);

  //read ModePin
//  g_mode = digitalRead(PIN_MODE0) | digitalRead(PIN_MODE1) << 1 | digitalRead(PIN_MODE2) << 2;
  g_mode = PINC & 0x07;

  printf("g_mode=%d,\n", g_mode);
  printf("delay=%d,\n", g_modeTable[g_mode]);

  //MODE0: for debug
  if(g_mode != 0){
    cli();  //All interrupt disable
  }
}


void loop(){
  //wait for injector-on signal from ECU
  while(digitalRead(PIN_ECU_INJECTOR_SIG) != LOW){};

  //Injector ON
  digitalWrite(PIN_INJECTOR_OUT, HIGH);
  digitalWrite(PIN_LED, HIGH);

  //wait for injector-off signal from ECU
  while(digitalRead(PIN_ECU_INJECTOR_SIG) != HIGH){};

  //wait Increase time
  delayMicroseconds(g_modeTable[g_mode]);

  //Injector OFF
  digitalWrite(PIN_INJECTOR_OUT, LOW);
  digitalWrite(PIN_LED, LOW);
}

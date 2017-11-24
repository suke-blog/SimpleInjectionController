/**
 * Simple Injection Controller for CUB(JBH-AA01)
 * http://www.suke-blog.com
 *
 * @version 0.1a
 */

#include <Arduino.h>
#include <stdio.h>
#include <avr/wdt.h>

#define SERIAL_RATE 115200
#define SERIAL_TIMEOUT 1000

//PIN Config
#define PIN_ECU_INJECTOR_SIG 8
#define PIN_INJECTOR_OUT 11
#define PIN_ECU_SENSOR_O2 7
#define PIN_MODE0 A0
#define PIN_MODE1 A1
#define PIN_MODE2 A2
#define PIN_LED 13

uint8_t g_mode = 0;
uint32_t g_multiplier = 0;
const uint32_t g_modeTable[] = {
   0x000,    //0.00
   0x019,    //0.10
   0x033,    //0.20
   0x0C0,    //0.30
   0x04C,    //0.40
   0x080,    //0.50
   0x099,    //0.60
   0x0B3     //0.70
};
// float g_multiplier = 0;
// const float g_modeTable[] = {
//    0.00,
//    0.10,
//    0.20,
//    0.30,
//    0.40,
//    0.50,
//    0.60,
//    0.70
// };


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

  //o2 sensor led OFF
  digitalWrite(PIN_LED, LOW);

  //Timer1
  TCCR1A = 0x00;        //Normal port operation, OC1A/OC1B disconnected.
  TCCR1B = _BV(ICNC1) | _BV(CS11) | _BV(CS10);  //InputCaputureNoiseCanceler:Enable, clock/64
  TIMSK1 = 0x00;        //InterruptMask: Interrupt disabled.

  //AnalogComparator
  ADCSRB = 0x00;                    //ADC:Free Running mode
  ACSR   = 0x00;
  DIDR1  = _BV(AIN1D) | _BV(AIN0D); //AIN0,1 DigitalInput disabled.

  //ADC
  ADMUX  = _BV(REFS0);  //Reference:AVCC, Connect:ADC0 pin
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);  //ADC:Enable,Conversion started.

  //DigitalInput disable
  DIDR1  = _BV(AIN1D) | _BV(AIN0D); //AIN0,1 DigitalInput disabled.
  //DIDR0  = _BV(ADC0D);

  g_flagEngineStart = false;

  //read ModePin
  g_mode = PINC & 0x07;
  g_multiplier = g_modeTable[g_mode];

  printf("g_mode=%d,\n", g_mode);
  printf("g_multiplier=%f,\n", g_multiplier);
  Serial.end();

  //delay 100ms
  delay(100);

  wdt_enable(WDTO_1S);

  noInterrupts();
}


void loop(){
  uint16_t injectorStartTime = 0;
  uint16_t injectorEndTime = 0;
  uint32_t injectorOnTime = 0;
  uint32_t injectorPlusTime = 0;
  uint8_t temp = 0;

  //wait for injector-on signal from ECU
  while(digitalRead(PIN_ECU_INJECTOR_SIG) != LOW){
    wdt_reset();
  };

  //Injector ON
  digitalWrite(PIN_INJECTOR_OUT, HIGH);
  injectorStartTime = TCNT1;  //save injector start time

  //O2 sensor
  temp = ACSR & _BV(ACO);
  if(temp == 0){
    digitalWrite(PIN_LED, LOW);
  }else{
    digitalWrite(PIN_LED, HIGH);
  }

  //wait for injector-off signal from ECU
  while(digitalRead(PIN_ECU_INJECTOR_SIG) != HIGH){};
  injectorEndTime = TCNT1;  //save injector end time;

  if(injectorStartTime < injectorEndTime){
    injectorOnTime = (injectorEndTime - injectorStartTime) * 4; //timer 1count = 4usec
  }else{
    injectorOnTime = ((0xFFFF - injectorStartTime) + injectorEndTime) * 4;
  }

  injectorPlusTime = (injectorOnTime * g_multiplier) >> 8;
  // injectorPlusTime = (uint16_t)((float)injectorOnTime * g_multiplier);

  //wait Increase time
  delayMicroseconds(injectorPlusTime);

  //Injector OFF
  digitalWrite(PIN_INJECTOR_OUT, LOW);

  //debug
  // printf("---------\n");
  // printf("start:%u, end:%u, OnTime:%u, PlusTime:%u\n",
  //   injectorStartTime,
  //   injectorEndTime,
  //   injectorOnTime,
  //   injectorPlusTime);
  // printf("---------\n");
  // delay(5000);
}

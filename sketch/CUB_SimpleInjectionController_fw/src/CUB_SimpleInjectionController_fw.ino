/**
 * Simple Injection Controller for SuperCUB(JBH-AA01)
 * http://www.suke-blog.com
 *
 * @version 0.2
 */

#include <Arduino.h>
#include <stdio.h>
#include <avr/wdt.h>

// #define DEBUG

// #define SERIAL_RATE 115200
// #define SERIAL_TIMEOUT 10000
#define SERIAL_RATE 250000
#define SERIAL_TIMEOUT 10000
// #define SERIAL_RATE 2000000
// #define SERIAL_TIMEOUT 1

//PIN Config
#define REG_ECU_INJECTOR_SIG PINB
#define PIN_ECU_INJECTOR_SIG_RAW PB0
#define PIN_ECU_INJECTOR_SIG 8
#define REG_INJECTOR_OUT PORTB
#define PIN_INJECTOR_OUT_RAW PB3
#define PIN_INJECTOR_OUT 11
#define PIN_ECU_SENSOR_O2 7
#define PIN_MODE0 A0
#define PIN_MODE1 A1
#define PIN_MODE2 A2
#define REG_LED PORTB
#define PIN_LED_RAW PB5
#define PIN_LED 13
#define AD_BATTERY    A7
#define AD_SENSOR_THL A6
#define AD_SENSOR_PB  A5
#define AD_SENSOR_TA  A4

#define WAIT_TIMEOUT_COUNT 150


#ifdef DEBUG
#define DBG_PRINT(x) printf(x)
#else
#define DBG_PRINT(x)
#endif

typedef struct _engineStatus{
  uint8_t magic[4];
  uint16_t rpm;
  uint16_t injectorOnTime;
  uint16_t injectorOnTimeOrg;
  uint16_t senseO2;
  uint16_t senseThrottle;
  uint16_t senseAirTemp;
  uint16_t senseAirFlow;
  uint16_t senseFuel;
  uint16_t senseOilTemp;
  uint16_t batteryVoltage;
}EngineStatus;

union{
  uint8_t data[14];
  EngineStatus field;
}g_engineStatus;

// EngineStatus g_engineStatus;
uint32_t g_icr1;


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

// for WDT loop
uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void get_mcusr(void)
{
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

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

  // noInterrupts();

  //GPIO setting
  pinMode(PIN_ECU_INJECTOR_SIG, INPUT_PULLUP);
  pinMode(PIN_INJECTOR_OUT, OUTPUT);
  pinMode(PIN_MODE0, INPUT_PULLUP);
  pinMode(PIN_MODE1, INPUT_PULLUP);
  pinMode(PIN_MODE2, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  //Injector off
  // digitalWrite(PIN_INJECTOR_OUT, LOW);
  REG_INJECTOR_OUT &= ~_BV(PIN_INJECTOR_OUT_RAW);

  //o2 sensor led OFF
  // digitalWrite(PIN_LED, LOW);
  REG_LED &= ~_BV(PIN_LED_RAW);

  //Timer0
  TIMSK0 = 0x00;  //Interrupt disable

  //Timer1
  TCCR1A = 0x00;                                //Normal port operation, OC1A/OC1B disconnected.
  TCCR1B = _BV(ICNC1) | _BV(CS11) | _BV(CS10);  //InputCaputureNoiseCanceler:Enable, clock/64
  // TIMSK1 = _BV(TOIE1);                          //InterruptMask: Overflow Interrupt:enable

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

  clearEngineStatus();

  //read ModePin
  g_mode = PINC & 0x07;
  g_multiplier = g_modeTable[g_mode];

  g_engineStatus.field.magic[0] = 0xF1;
  g_engineStatus.field.magic[1] = 0xF2;
  g_engineStatus.field.magic[2] = 0xF3;
  g_engineStatus.field.magic[3] = 0xF4;

  // printf("g_mode=%d,\n", g_mode);
  // printf("g_multiplier=%f,\n", g_multiplier);
  // Serial.end();

  // wdt_disable();
  // wdt_enable(WDTO_1S);
  // noInterrupts();

  // Serial.println("Start");
  DBG_PRINT("Start - ");
  DBG_PRINT(__DATE__);
  DBG_PRINT(" ");
  DBG_PRINT(__TIME__);

  //delay 100ms
  delay(100);
}


void clearEngineStatus(){
  memset(&g_engineStatus, 0x00, sizeof(g_engineStatus));
  g_icr1 = 0xFFFFFFFF;
}

void doAdc(){
  static uint8_t index = 0;
  uint16_t temp;

  temp = ADC;
}


void loop(){
  uint16_t injectorStartTime = 0;
  uint16_t injectorEndTime = 0;
  // uint32_t injectorOnTime = 0;
  uint32_t injectorPlusTime = 0;
  uint8_t engineStopCount = 0;
  uint16_t icr1_temp;
  uint16_t temp;

  // clear interrupt flag
  TIFR1 = 0x00;

  //wait for injector-on signal from ECU
  // while(digitalRead(PIN_ECU_INJECTOR_SIG) != LOW){
  // while(PORTB & _BV(PIN_ECU_INJECTOR_SIG_RAW) != LOW){
  // while(PINB & _BV(PIN_ECU_INJECTOR_SIG_RAW) != LOW){
  while( bit_is_set(REG_ECU_INJECTOR_SIG, PIN_ECU_INJECTOR_SIG_RAW) ){
    wdt_reset();

    //engine stop?
    if(TIFR1 & 0x01 == 0x01){
      TIFR1 = 0x00; //interrupt flag clear

      engineStopCount++;
      if(engineStopCount > WAIT_TIMEOUT_COUNT){
        engineStopCount = 0;

        clearEngineStatus();

        if(Serial.read() != -1){
          DBG_PRINT("Serial.read().\n");
          Serial.write((uint8_t *)&g_engineStatus, sizeof(g_engineStatus));
        }

      }
    }
  }


  //Injector ON
  // digitalWrite(PIN_INJECTOR_OUT, HIGH);
  REG_INJECTOR_OUT |= _BV(PIN_INJECTOR_OUT_RAW);
  injectorStartTime = TCNT1;  //save injector start time


  //O2 sensor
  temp = ACSR & _BV(ACO);
  if(temp == 0){
    // digitalWrite(PIN_LED, LOW);
    REG_LED &= ~_BV(PIN_LED_RAW);
  }else{
    // digitalWrite(PIN_LED, HIGH);
    REG_LED |= ~_BV(PIN_LED_RAW);
  }
  g_engineStatus.field.senseO2 = temp;

  //AD:AirFlow(100usec)
  g_engineStatus.field.senseAirFlow = analogRead(AD_SENSOR_PB);

  //AD:AirTemp(100usec)
  g_engineStatus.field.senseAirTemp = analogRead(AD_SENSOR_TA);


  //wait for injector-off signal from ECU
  // while(digitalRead(PIN_ECU_INJECTOR_SIG) != HIGH){};
  while( bit_is_clear(PINB, PINB0) ){};
  injectorEndTime = TCNT1;  //save injector end time;

  if(injectorStartTime < injectorEndTime){
    // injectorOnTime = (injectorEndTime - injectorStartTime) * 4; //timer 1count = 4usec
    g_engineStatus.field.injectorOnTimeOrg = (injectorEndTime - injectorStartTime) * 4; //timer 1count = 4usec
  }else{
    // injectorOnTime = ((0xFFFF - injectorStartTime) + injectorEndTime) * 4;
    g_engineStatus.field.injectorOnTimeOrg = ((0xFFFF - injectorStartTime) + injectorEndTime) * 4;
  }

  injectorPlusTime = (g_engineStatus.field.injectorOnTimeOrg * g_multiplier) >> 8;
  g_engineStatus.field.injectorOnTime = g_engineStatus.field.injectorOnTimeOrg + injectorPlusTime;

  //wait Increase time
  delayMicroseconds(injectorPlusTime);

  //Injector OFF
  // digitalWrite(PIN_INJECTOR_OUT, LOW);
  REG_INJECTOR_OUT &= ~_BV(PIN_INJECTOR_OUT_RAW);

  //RPM
  icr1_temp = ICR1;

  if(g_icr1 != 0xFFFFFFFF){
    if(icr1_temp > g_icr1){
      temp = icr1_temp - g_icr1;
    }else{
      temp = (0xFFFF - g_icr1) + icr1_temp + 1;
    }

    g_engineStatus.field.rpm = 1000000 / (temp * 4) * 2;
  }

  g_icr1 = icr1_temp;

  //AD:batteryVoltage(100usec)
  g_engineStatus.field.batteryVoltage = analogRead(AD_BATTERY);

  //AD:Throttle(100usec)
  g_engineStatus.field.senseThrottle = analogRead(AD_SENSOR_THL);


  if(Serial.read() != -1)
    Serial.write((uint8_t *)&g_engineStatus, sizeof(g_engineStatus));

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

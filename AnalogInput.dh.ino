/*
  Analog Input

  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED) connected to digital pin 13.
  The amount of time the LED will be on and off depends on the value obtained
  by analogRead().

  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13
    cathode (short leg) attached to ground

  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.

  created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogInput
*/

#include "wiring_private.h"

int sensorPin = A1; // select the input pin for the potentiometer
// int sensorPin2 = A0;
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor
// int sensorValue2 = 0;



// Wait for synchronization of registers between the clock domains
static __inline__ void syncADC() __attribute__((always_inline, unused));
static void syncADC() {
  while (ADC->STATUS.bit.SYNCBUSY == 1)
    ;
}

 // ATSAMR, for example, doesn't have a DAC
#ifdef DAC
// Wait for synchronization of registers between the clock domains
static __inline__ void syncDAC() __attribute__((always_inline, unused));
static void syncDAC() {
  while (DAC->STATUS.bit.SYNCBUSY == 1)
    ;
}
#endif

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_16(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_16(Tc* TCx) {
  while (TCx->COUNT16.STATUS.bit.SYNCBUSY);
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

uint32_t analogReadSetup(uint32_t pin)
{
  uint32_t valueRead = 0;

#if defined(PIN_A6)
  if (pin == 6) {
    pin = PIN_A6;
  } else
#endif
#if defined(PIN_A7)
  if (pin == 7) {
    pin = PIN_A7;
  } else 
#endif
  if (pin <= 5) {
    pin += A0;
  }

  pinPeripheral(pin, PIO_ANALOG);
 //ATSAMR, for example, doesn't have a DAC
#ifdef DAC

  #if defined(__SAMD51__)
    if (pin == A0 || pin == A1) { // Disable DAC, if analogWrite(A0,dval) used previously the DAC is enabled
    uint8_t channel = (pin == PIN_A0 ? 0 : 1);
    
    if(dacEnabled[channel]){
      dacEnabled[channel] = false;
      
      while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
      DAC->CTRLA.bit.ENABLE = 0;     // disable DAC
      
      while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
      DAC->DACCTRL[channel].bit.ENABLE = 0;
      
      while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
      DAC->CTRLA.bit.ENABLE = 1;     // enable DAC
    }
    
    while (DAC->SYNCBUSY.bit.ENABLE);
  #else
    if (pin == A0) { // Disable DAC, if analogWrite(A0,dval) used previously the DAC is enabled
      syncDAC();
    
    DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC
    //DAC->CTRLB.bit.EOEN = 0x00; // The DAC output is turned off.
    syncDAC();
  #endif
    }

#endif

  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input
  
  // Control A
  /*
   * Bit 1 ENABLE: Enable
   *   0: The ADC is disabled.
   *   1: The ADC is enabled.
   * Due to synchronization, there is a delay from writing CTRLA.ENABLE until the peripheral is enabled/disabled. The
   * value written to CTRL.ENABLE will read back immediately and the Synchronization Busy bit in the Status register
   * (STATUS.SYNCBUSY) will be set. STATUS.SYNCBUSY will be cleared when the operation is complete.
   *
   * Before enabling the ADC, the asynchronous clock source must be selected and enabled, and the ADC reference must be
   * configured. The first conversion after the reference is changed must not be used.
   */
  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC

  // Start conversion
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Clear the Data Ready flag
  ADC->INTFLAG.reg = ADC_INTFLAG_RESRDY;

  // Start conversion again, since The first conversion after the reference is changed must not be used.
  syncADC();
  ADC->SWTRIG.bit.START = 1;

  // Store the value
  while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  valueRead = ADC->RESULT.reg;

  syncADC();
  ADC->CTRLA.bit.ENABLE = 0x00;             // Disable ADC
  syncADC();


  return valueRead;
}







void averagingOn() {
  // Averaging (see datasheet table in AVGCTRL register description)
  Serial.print("AVGCTRL.reg before change: 0x");
  Serial.println(ADC->AVGCTRL.reg, HEX);
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_16 |    // 16 sample averaging)
                     ADC_AVGCTRL_ADJRES(0x4ul);   // Adjusting result (right shift) by 4

  Serial.print("AVGCTRL.reg after averaging turned on: 0x");
  Serial.println(ADC->AVGCTRL.reg, HEX);

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  Serial.print("CTRLB.reg before change: 0x");
  Serial.println(ADC->CTRLB.reg, HEX);
  ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_16BIT_Val;         // 16 bits resolution for averaging
  
  Serial.print("CTRLB.reg after change to 16 bit result: 0x");
  Serial.println(ADC->CTRLB.reg, HEX);

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains
}

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  // analogReadResolution(10);
  analogReference(AR_EXTERNAL);
  averagingOn();
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogReadSetup(sensorPin); // analogRead: will replace with own code
  // sensorValue2 = analogRead(sensorPin2);
  // turn the ledPin on
  /* digitalWrite(ledPin, HIGH);
  // stop the program for <sensorValue> milliseconds:
  delay(sensorValue);
  // turn the ledPin off:
  digitalWrite(ledPin, LOW);
  // stop the program for for <sensorValue> milliseconds: */ 
  delay(100); 
  Serial.print(sensorValue);
  Serial.print('\t');
  // Serial.print(sensorValue2);
  // Serial.print('\t');
  // Serial.print((1.0*sensorValue/sensorValue2)*360);
  Serial.println();
}

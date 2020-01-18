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

int sensorPin = A1; // select the input pin for the potentiometer
// int sensorPin2 = A0;
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor
// int sensorValue2 = 0;

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
  sensorValue = analogRead(sensorPin); // analogRead: will replace with own code
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

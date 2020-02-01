/*
  Analog Input (for shooter side)

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
#include <mcp_can.h>
#include <SPI.h>
#include "Adafruit_VL53L0X.h"

// CAN0 RESET, INT
#define CAN0_RST 31   // MCP25625 RESET connected to Arduino pin 31
#define CAN0_INT 30  // Set INT to Arduino pin 30

// Time of flight sensor out of range value
#define OUT_OF_RANGE -100

// CAN variables
unsigned long t_prev = 0;  // Variable to store last execution time
const unsigned int t_intvl = 10;  // Transmi interval in milliseconds
// CAN data to send - init w/ bogus data
byte data[] = {0xAA, 0x55, 0x01, 0x10, 0xFF, 0x12, 0x34, 0x56};


/*
The Manufacturer (8 bits)
The Device Type (5 bits)
An API ID (10 bits)
The Device ID (6 bits)

For Team created modules, FIRST says that 
Manufacturer - HAL_CAN_Man_kTeamUse = 8
Device Type - HAL_CAN_Dev_kMiscellaneous = 10
API ID is up to us
Device ID is unique to each module of a specific type (e.g., we can have more than 1 line follower)

CAN ID: (Device Type): 01010 (Mfr ID): 0000 1000 (API ID): 00 0000 0010 (Device ID):00 0001 
CAN ID: 01010 0000 1000 00 0000 0010 00 0001 
CAN ID: 0 1010 0000 1000 0000 0000 1000 0001 
which is: 0x0A080081

*/
#define CAN_ID 0x0a080081

// 1 if has time of flights, 0 if not
char tof_mode = 0;

MCP_CAN CAN0(8); // Set MCP25625 CS to Arduino pin 8

int ledPin = 13;      // select the pin for the LED

int sensorPin0 = A0; // select the input pin for the potentiometer
int sensorPin1 = A1;
int sensorPin2 = A2;
int sensorValue0 = 0;  // variable to store the value coming from the sensor
int sensorValueRaw0 = 0;
int sensorValue1 = 0;  // variable to store the value coming from the sensor
int sensorValueRaw1 = 0;
int sensorValue2 = 0;  // variable to store the value coming from the sensor
int sensorValueRaw2 = 0;

// TOF sensor distance
int tof_distance_0 = 0; // for testing
int tof_distance_1 = 1; // for testing

//Time of Flight Stuff
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// pack message into format used by 2 steering and 1 shooter to RoboRio
void packMsgShooter()
{
  int i;
  long ltemp = 0;
  int distance = 0;
  
  data[0] = (sensorValue0 >> 8) & 0x00ff;
  data[1] = sensorValue0 & 0x00ff;

  data[2] = (sensorValue1 >> 8) & 0x00ff;
  data[3] = sensorValue1 & 0x00ff;
  
  data[4] = (sensorValue2 >> 8) & 0x00ff;
  data[5] = sensorValue2 & 0x00ff;

  data[6] = 0x00;
  data[7] = 0x00;
}

// pack message into format used by 2 steering and 2 time of flight to RoboRio
void packMsgTOF()
{
  int i;
  long ltemp = 0;
  int distance = 0;
  
  data[0] = (sensorValue0 >> 8) & 0x00ff;
  data[1] = sensorValue0 & 0x00ff;
  
  data[2] = (sensorValue1 >> 8) & 0x00ff;
  data[3] = sensorValue1 & 0x00ff;
  
  data[4] = (tof_distance_0 >> 8) & 0x00ff;
  data[5] = tof_distance_0 & 0x00ff;

  // Then pack the TOF distance into the next 16 bits.
  // This is a unsigned value, in units of mm
  data[6] = (tof_distance_1 >> 8) & 0xff;
  data[7] = tof_distance_1 & 0xff;
}


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


uint32_t newAnalogRead(uint32_t pin)
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

    if (pin == A0) { // Disable DAC, if analogWrite(A0,dval) used previously the DAC is enabled
      syncDAC();
    
      DAC->CTRLA.bit.ENABLE = 0x00; // Disable DAC
      //DAC->CTRLB.bit.EOEN = 0x00; // The DAC output is turned off.
      syncDAC();

    }

#endif

  syncADC();
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber; // Selection for the positive ADC input
  
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

void enableAnalog() {
  // set clock slower due to source resistance
  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  Serial.print("CTRLB.reg before change: 0x");
  Serial.println(ADC->CTRLB.reg, HEX);
  ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV64_Val;         // set clock PRESCALER to 64 (default is 32)
  
  Serial.print("CTRLB.reg after change to PRESCALER / 64: 0x");
  Serial.println(ADC->CTRLB.reg, HEX);

  while( ADC->STATUS.bit.SYNCBUSY == 1 );          // Wait for synchronization of registers between the clock domains

  
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

}

void setup() {

  Serial.begin(115200);

  File file = fatfs.open (, FILE_READ);

  // wait for serial port connection
  while(!Serial);

  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  // analogReadResolution(10);
  analogReference(AR_EXTERNAL);
  enableAnalog();
  averagingOn();

  delay(5000);

  digitalWrite(ledPin, 1);
  
    // CAN chip RESET line
  pinMode(CAN0_RST, OUTPUT);
  // Configuring pin for /INT input
  pinMode(CAN0_INT, INPUT_PULLUP);
  
  // reset CAN chip
  digitalWrite(CAN0_RST, 0);
  delay(100);
  digitalWrite(CAN0_RST, 1);
  delay(500);  

  // Initialize MCP25625 running at 16MHz with a baudrate of 1Mb/s and
  // the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) {
    Serial.println("MCP25625 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP25625...");
  }

  Serial.println("Analog Input");
  
  CAN0.setMode(MCP_NORMAL);

  delay(500);

  digitalWrite(ledPin, 0);










  //Time of Flight Stuff 
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
  // power 
  Serial.println(F("VL53L0X API Simple Ranging example\n\n")); 
}

void loop() {
  
  // read the value from the sensor:
  sensorValueRaw0 = newAnalogRead(sensorPin0); // used to be analogRead(), made new function
  
  // sensorValue is angle in deg * 10 eg. max is 3599
  sensorValue0 = (int)((sensorValueRaw0 * 1.0) / 4096.0 * 3600.0);
  
    // read the value from the sensor:
  sensorValueRaw1 = newAnalogRead(sensorPin1); // used to be analogRead(), made new function
  
  // sensorValue is angle in deg * 10 eg. max is 3599
  sensorValue1 = (int)((sensorValueRaw1 * 1.0) / 4096.0 * 3600.0);

    // read the value from the sensor:
  sensorValueRaw2 = newAnalogRead(sensorPin2); // used to be analogRead(), made new function
  
  // sensorValue is angle in deg * 10 eg. max is 3599
  sensorValue2 = (int)((sensorValueRaw2 * 1.0) / 4096.0 * 3600.0);
  
  // turn the ledPin on
  /* digitalWrite(ledPin, HIGH);
  // stop the program for <sensorValue> milliseconds:
  delay(sensorValue);
  // turn the ledPin off:
  digitalWrite(ledPin, LOW);
  // stop the program for for <sensorValue> milliseconds: */ 
  // delay(100);
  Serial.print("Encoder Values:\t"); 
  Serial.print(sensorValueRaw0);
  Serial.print('\t');
  Serial.print(sensorValue0);
  Serial.print('\t');
  Serial.print(sensorValueRaw1);
  Serial.print('\t');
  Serial.print(sensorValue1);
  Serial.print('\t');
  Serial.print(sensorValueRaw2);
  Serial.print('\t');
  Serial.print(sensorValue2);
  Serial.print('\t');
  // Serial.print(sensorValue2);
  // Serial.print('\t');
  // Serial.print((1.0*sensorValue/sensorValue2)*360);
  Serial.println();


  // Time of Flight Stuff
  VL53L0X_RangingMeasurementData_t measure;
    
  Serial.print("Reading a measurement... ");
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    tof_distance_0 = measure.RangeMilliMeter;
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    tof_distance_0 = OUT_OF_RANGE;
    Serial.println(" out of range ");
  }

  // pack message for protocol from Feather CAN Line Follower to RoboRio
  if (tof_mode)
  {
    packMsgTOF();
  }
  else
  {
    packMsgShoot();
  }
    
  // send Extended msg
  // byte sndStat = CAN0.sendMsgBuf(CAN_ID | 0x80000000, 1, 8, data);
  byte sndStat = CAN0.sendMsgBuf(CAN_ID, 1, 8, data);

  /*
  for(i = 4; i < 6; i++) {
    Serial.println(data[i]);
  }
  */
  
  // byte sndStat = CAN_OK;
    
  /* debug printouts
  Serial.print("TEC: ");
  Serial.println(CAN0.errorCountTX());

  Serial.print("REC: ");
  Serial.println(CAN0.errorCountRX());
  */
  
  if(sndStat == CAN_OK) {
    Serial.println("Message Sent Successfully!");
  } else {
    Serial.println("Error Sending Message...");
  }
  
  // Serial.println();
    
  delay(100);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ MPPT Buck Converter [Rev. 1.2]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   This code is under MIT License
   Copyright (c) 2022 Sayantan Sinha
*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define CON PB1                                        // D9
#define MAX_COUNT 1000                                 // For 16 kHz switching frequency
#define D_MAX 750
#define D_MIN 0
#define D_NULL 0
#define D_INC 1
#define D_DCR 2

const byte pinVpv = 0;                                  // PV voltage @pin A0
const byte pinVo = 1;                                   // Output voltage @pin A1
const byte pinIpv = 2;                                  // PV current @pin A2
const byte pinIo = 3;                                   // Output current @pin A3

LiquidCrystal_I2C lcd(0x3f, 16, 2);

void setup()
{
  DDRB |= 1 << PB5;
  PORTB &= ~(1 << PB5);                              // Turn off the annoying LED @ pin 13

  TIMSK1 = 0;                                        // Disable Timer1 interrupts

  ADMUX = 0b01000000;                                // Select A_ref = +5 V
  ADCSRA = 0b10010111;                               // ADC enable, prescaler = f_CPU / 128 [Ref: Atmega328P datasheet, pp. 317-320]

  DDRB |= (1 << CON);                                // Set D9 as output pins to get sPWM outputs
  PORTB &= ~(1 << CON);                              // Set D9 LOW

  ICR1 = MAX_COUNT;                                  // Set the switching freq. f_sw = f_cpu / (2 * MAX_COUNT)
  TCCR1A = 0b00000010;                               // pin D9 (OC1A) & pin D10 (OC1B) disconnected to avoid any mistrigger
  TCCR1B = 0b00011001;                               // Fast PWM (Mode 14); Clock Select = System clock (No Prescaling) [Ref. Data Sheet, p. 172]
  OCR1A = D_MIN;                                     // Initially output current = max

  TCCR1A |= 0b10000000;                              // pin D9 (OC1A): Set on Compare Match

  lcd.begin();
  lcd.clear();
  Serial.begin(9600);
  delay(10);
}

void loop()
{
  static unsigned long tDispRefresh = 0;
  static unsigned long tMQTTRefresh = 0;
  static unsigned long tControl = 0;
  static unsigned long tInterval = 100;
  static float vpvPrev = 0;
  static float ppvPrev = 0;
  static int state = D_NULL;

  float vpv = (float)adcRead(pinVpv) / 37.1;        // Read PV voltage
  float vo = (float)adcRead(pinVo) / 50.7;          // Read output voltage
  float ipv = (float)adcRead(pinIpv) / 107.7;       // Read PV current
  float io = (float)adcRead(pinIo) / 157.9;         // Read output current

  float ppv = vpv * ipv;                            // Calculate PV power

  if (io < 0.1) {                                   // Output current is less than 0.1 A
    OCR1A = D_MIN;                                  // Set output current to max
    tInterval = 100;                                // Set interval to 100 ms
    state = D_NULL;                                 // Set state to D_NULL (neither increase or decrease o/p current)
  }
  else if (state == D_NULL) {                       // Output current > 0.1 A and state is D_NULL
    state = D_INC;                                  // Set the state to D_INC (reduce outout current)
  }
  if (io > 0.1) {
    if (millis() - tControl > tInterval) {
      tControl = millis();
      if (tInterval > 2000)
        tInterval = 2000;                          // Reset the interval time to 2 s if it is higher
      unsigned int d = OCR1A;

      if (state == D_INC) {                        // If state is set to D_INC (reduce outout current)
        if (ppv < ppvPrev && vpv > vpvPrev) {      // PV power decreased & PV voltage increased
          d = d > D_MIN ? d - 1 : d;               // Increase the output current
          state = D_DCR;                           // Flip the state to D_DCR (increase outout current)
          tInterval = 2000;
        }
        else                                       // PV power increased or PV voltage decreased/unchanged
          d = d < D_MAX ? d + 1 : d;               // Decrease output current
      }
      else if (state == D_DCR) {                   // If state is set to D_DCR (increase outout current)
        if (ppv > vpvPrev)                         // PV power increased
          d = d > D_MIN ? d - 1 : d;               // Increase current
        else {                                     // PV power decreased
          d = d < D_MAX ? d + 1 : d;               // Reduce output current
          state = D_INC;                           // Change the state to D_INC (reduce outout current)
          tInterval = 300000;                      // Set interval to 30 s to give a pause (for stability)
        }
      }

      ppvPrev = ppv;
      vpvPrev = vpv;

      OCR1A = d;

      //      lcd.clear();
      //      lcd.print(ppv);
      //      lcd.print("W  ");
      //      lcd.print(vpv);
      //      lcd.print("V");
      //      lcd.setCursor(0, 1);
      //      lcd.print(io);
      //      lcd.print("A  ");
      //      lcd.print(OCR1A);
    }
  }

  if ((millis() - tDispRefresh) >= 800) {
    tDispRefresh = millis();
    lcd.clear();
    lcd.print("P>");
    lcd.print(vpv);
    lcd.print("V ");
    lcd.print(ipv);
    lcd.print("A");
    lcd.setCursor(0, 1);
    lcd.print("B>");
    lcd.print(vo);
    lcd.print("V ");
    lcd.print(io);
    lcd.print("A");
  }
  if ((millis() - tMQTTRefresh) >= 12000) {
    tMQTTRefresh = millis();
    Serial.println(vpv);
    Serial.println(ipv);
    Serial.println(vo);
    Serial.print(io);
  }
}

unsigned int adcRead(const byte ch)
{
  if (ch > 15)
    return 0;
  ADMUX &= 0xF0;
  ADMUX |= ch;
  ADCSRA |= 0x40;           // Start conversion
  while (ADCSRA & 0x40);    // Wait until ADC start conversion bit goes low
  unsigned int a = ADCL;
  a |= (ADCH << 8);
  return (a);
}

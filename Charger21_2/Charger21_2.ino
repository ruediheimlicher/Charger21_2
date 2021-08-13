///
/// @mainpage	Charger21_2
///
/// @details	Description of the project
/// @n
/// @n
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @author		___ORGANIZATIONNAME___
/// @date		11.08.2021 11:13
/// @version	<#version#>
///
/// @copyright	(c) Ruedi Heimlicher, 2021
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
///


///
/// @file		Charger21_2.ino
/// @brief		Main sketch
///
/// @details	<#details#>
/// @n @a		Developed with [embedXcode+](https://embedXcode.weebly.com)
///
/// @author		Ruedi Heimlicher
/// @author		___ORGANIZATIONNAME___
/// @date		11.08.2021 11:13
/// @version	<#version#>
///
/// @copyright	(c) Ruedi Heimlicher, 2021
/// @copyright	GNU General Public Licence
///
/// @see		ReadMe.txt for references
/// @n
///


// Core library for code-sense - IDE-based
// !!! Help: http://bit.ly/2AdU7cu
#include "Arduino.h"
#include "gpio_MCP23S17.h"
#include <SPI.h>
// Include application, user and local libraries
#include "lcd.h"
//#include "settings.h"
#include "defines.h"
#include "adc.h"


// Set parameters

// Define structures and classes


// Define variables 
uint8_t loopLED;
static volatile uint8_t recvbuffer[64]={};
static volatile uint8_t sendbuffer[64]={};

volatile uint8_t           adcstatus=0x00;
volatile uint16_t          adctimercounter = 0;

volatile uint8_t           usbstatus=0x00;
static volatile uint8_t    motorstatus=0x00;
static volatile uint8_t    anschlagstatus=0x00;

#define USB_SEND  0 
volatile uint8_t          senderfolg = 0;

volatile uint8_t status=0;

volatile uint16_t           PWM_A=0;
volatile uint16_t           PWM_B=0;
static volatile uint8_t    pwmposition=0;
static volatile uint8_t    pwmdivider=0;

volatile uint16_t       batt_M = 0;
volatile uint16_t       batt_O = 0;
volatile uint16_t       curr_U = 0;
volatile uint16_t       curr_O = 0;

volatile char SPI_data='0';
volatile char SPI_dataArray[32];

// PWM
uint16_t counterStart = 3036;
volatile uint16_t timer2Counter = 0;
volatile uint8_t  sekundentimercounter = 0;
volatile uint16_t timer2sekunde = 0;

int8_t r;

uint16_t count=0;

uint16_t loopcount0=0;
uint8_t loopcount1=0;

volatile uint8_t  usbsendcounter = 0;
volatile uint16_t  usbrecvcounter = 0;


// define constants
//#define USB_DATENBREITE 64

#define TEST 0


// Prototypes
// !!! Help: http://bit.ly/2l0ZhTa


// Utilities


// Functions

void startTimer2(void)
{
   //timer2
   TCNT2   = 0; 
   //   TCCR2A |= (1 << WGM21);    // Configure timer 2 for CTC mode 
   TCCR2B |= (1 << CS20);     // Start timer at Fcpu/8 
   //   TIMSK2 |= (1 << OCIE2A);   // Enable CTC interrupt 
   TIMSK2 |= (1 << TOIE2);    // Enable OV interrupt 
   //OCR2A   = 5;             // Set CTC compare value with a prescaler of 64 
   TCCR2A = 0x00;
   
   sei();
}

void stopTimer2(void)
{
   TCCR2B = 0;
}

void slaveinit(void)
{
   //OSZIPORTDDR |= (1<<PULS);   // Output
   //OSZIPORT |= (1<<PULS);      // HI
   
  // LOOPLEDDDR |=(1<<LOOPLED);
  // LOOPLEDPORT |= (1<<LOOPLED);   // HI
   pinMode(LOOPLED, OUTPUT);
   
   //Pin 0 von   als Ausgang fuer OSZI
  /*
   OSZIPORTDDR |= (1<<OSZI_PULS_A);   //Pin 0 von  als Ausgang fuer LED TWI
   OSZIPORT |= (1<<OSZI_PULS_A);      // HI
   
    OSZIPORTDDR |= (1<<OSZI_PULS_B);      //Pin 1 von  als Ausgang fuer LED TWI
    OSZIPORT |= (1<<OSZI_PULS_B);      //Pin   von   als Ausgang fuer OSZI
  */
   pinMode(OSZI_PULS_A, OUTPUT);
   pinMode(OSZI_PULS_B, OUTPUT);
   
   
// ADC
//   ADCDDR &= (1<<ADC_M);
 //  ADCDDR &= (1<<ADC_O);
   
   pinMode(ADC_M, INPUT);
   pinMode(ADC_O, INPUT);

   //LCD
   pinMode(LCD_RSDS_PIN, OUTPUT);
   pinMode(LCD_ENABLE_PIN, OUTPUT);
   pinMode(LCD_CLOCK_PIN, OUTPUT);
   /*
   LCD_DDR |= (1<<LCD_RSDS_PIN);      //Pin 4 von PORT D als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_ENABLE_PIN);   //Pin 5 von PORT D als Ausgang fuer LCD
   LCD_DDR |= (1<<LCD_CLOCK_PIN);   //Pin 6 von PORT D als Ausgang fuer LCD
*/
   /*
   LADEDDR  |= (1<<LADESTROM_PWM_A); // Out für Ladestrom
   LADEDDR  |= (1<<LADESTROM_PWM_A); // Out für Ladestrom
   */
   pinMode(LADESTROM_PWM_A, OUTPUT);
   pinMode(LADESTROM_PWM_A, OUTPUT);
   
   
  // DDRB |= (1<<PB5);// OC1A
  // DDRB |= (1<<PB6);// OC1B
   
 //  DDRB |= (1<<PB4);// OC2A
   
   pinMode(25, OUTPUT);// OC1A
   pinMode(26, OUTPUT);// OC1A
   
   pinMode(24, OUTPUT);// OC2A
}

void timer0 (void) 
{ 
// Timer fuer Exp
   //TCCR0 |= (1<<CS01);                  // clock   /8
   //TCCR0 |= (1<<CS01)|(1<<CS02);         // clock   /64
   TCCR0B |= (1<<CS02)| (1<<CS02);         // clock   /256
   //TCCR0 |= (1<<CS00)|(1<<CS02);         // clock /1024
   TCCR0B |= (1 << CS10); // Set up timer 
   OCR0A = 0x2;
   
   //TIFR |= (1<<TOV0);                     //Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
   TIMSK0 |= (1<<TOIE0);                     //Overflow Interrupt aktivieren
   TCNT0 = TIMER0_STARTWERT;               //Rücksetzen des Timers

}

void timer1(void)
{
   // Clear OC1A and OC1B on Compare Match / Set OC1A and OC1B at Bottom; 
   // Wave Form Generator: Fast PWM 14, Top = ICR1
   
   TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (0<<WGM10);
   
   TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (1<<WGM12) | (1<<CS10) ;
   
   
   TCNT1H=0x00;
   TCNT1L=0x00;
   
   ICR1 = 1100;
   OCR1A = 10;
   OCR1B = 199;

   TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B)| (1<<TOIE1); // activate the compa, compb, ovf interupts
   

}
#pragma mark timer1 ISR
ISR(TIMER1_COMPA_vect)
{
   //LADEPORT &= ~(1<<LADESTROM_PWM_A);
   digitalWriteFast(LADESTROM_PWM_A, LOW);
} 

ISR(TIMER1_COMPB_vect)
{
  // LADEPORT &= ~(1<<LADESTROM_PWM_B);
   digitalWriteFast(LADESTROM_PWM_B, LOW);
}

ISR(TIMER1_OVF_vect){
  //TCNT1 = counterStart;
   //LADEPORT |= (1<<LADESTROM_PWM_A);
   digitalWriteFast(LADESTROM_PWM_A, HIGH);
   //LADEPORT |= (1<<LADESTROM_PWM_B);
   digitalWriteFast(LADESTROM_PWM_B, HIGH);
   adctimercounter++;
   if (adctimercounter >= ADC_TIMERDELAY)
   {
 //     adcstatus |= (1<<ADC_U_BIT);
 //     adcstatus |= (1<<ADC_I_BIT);
      adctimercounter = 0;
      OSZIATOGG;
   }
}

// Timer2 fuer Takt der Messung
void timer2(void)
{

//   TCCR2A |= (1<<WGM21);// Toggle OC2A
   TCCR2A |= (1<<COM2A1) | (1<<COM2A0) | (1<<WGM21) | (1<<WGM20);
 //  TCCR2B |= (1<<WGM22);
 //  TCCR2A |= (1<<WGM20);
 //  TCCR2A |= (1<<COM2A0);                  // CTC
   
   /*
    CS22   CS21   CS20   Description
    0    0     0     No clock source
    0    0     1     clk/1
    0    1     0     clk/8
    0    1     1     clk/32
    1    0     0     clk/64
    1    0     1     clk/128
    1    1     0     clk/256
    1    1     1     clk/1024
    */
   
   //TCCR2B |= (1<<CS22); //
   //TCCR2B |= (1<<CS21);//
   TCCR2B |= (1<<CS20) ;//| (1<<CS21);// |(1<<CS22)  ;
   TCCR2B |= (1 << WGM22); // Configure timer 2 for CTC mode
   TIMSK2 |= (1<<OCIE2A);        //interrupt on Compare Match A
   
   TIMSK2 |= (1<<TOIE2);                  //Overflow Interrupt aktivieren
   TCNT2 = 0;                             //Rücksetzen des Timers
   //OSZILO;
   OCR2A = TIMER2_COMPA; // 20ms
   //OCR2A = 10;
//   OCR2A = 0x02;
   //DDRB |= (1<<PORTB3);
   TIFR2 |= (1<<TOV2);                     //Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
}

ISR(TIMER2_COMPA_vect) // CTC Timer2
{
   //OSZIBTOGG;
   
   timer2Counter +=1;
   
 
   if (timer2Counter >= TIMER2_DELAY) // 10ms
   {
      timer2Counter = 0; 
      sekundentimercounter++;
      if (sekundentimercounter == 50)
      {
         adcstatus |= (1<<ADC_U_BIT);
         adcstatus |= (1<<ADC_I_BIT);
      }
      if (sekundentimercounter >= 100)
      {
         sekundentimercounter=0;
         timer2sekunde++;
         
         //OSZIATOGG;
      }
   } 

}
ISR (TIMER2_OVF_vect) 
{ 
 }



void sekundestop(void)
{
   TCNT2 = 0;
   timer2Counter=0;
   sekundentimercounter=0;
   timer2sekunde=0;
   
}


// Add setup code
void setup()
{
   Serial.begin(9600);
   slaveinit();
   /* initialize the LCD */
   
   lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

   lcd_puts("Guten Tag\0");
   _delay_ms(100);

  
   timer1();
   timer2();
   //initADC(0);

}
unsigned int packetCount = 0;
elapsedMillis msUntilNextSend;

elapsedMillis sinceRecv;
// Add loop code
void loop()
{
   loopcount0+=1;
   if (loopcount0==0xAF)
   {
      loopcount0=0;
      loopcount1+=1;
      digitalWrite(LOOPLED, !digitalRead(LOOPLED));
      //PORTD ^= (1<<PORTD6);
      
      lcd_gotoxy(0,0);
      
      lcd_puthex(recvbuffer[STROM_A_H_BYTE]);
      lcd_putc(' ');
      lcd_puthex(recvbuffer[STROM_A_L_BYTE]);           
      lcd_putc(' ');
      PWM_A = ((recvbuffer[STROM_A_H_BYTE]) << 8) | recvbuffer[STROM_A_L_BYTE] ;
      lcd_putint12(PWM_A); 
      lcd_putc(' ');
      lcd_puthex(recvbuffer[TASK]);           

      lcd_gotoxy(16,0);
      lcd_putint12(timer2sekunde);

      lcd_gotoxy(0,1);
      lcd_putc('O');
      lcd_putint12(batt_O);
      lcd_putc(' ');
      lcd_putc('M');
      lcd_putint12(batt_M);
      
  //    lcd_clr_line(3);
      lcd_gotoxy(0,3);
      lcd_putint12(usbrecvcounter);
      lcd_putc(' ');
      lcd_puthex(senderfolg);
      lcd_putc(' ');
      lcd_putint12(usbsendcounter);
    } // if loopcount

   if ((adcstatus & (1<<ADC_U_BIT)) || (adcstatus & (1<<ADC_I_BIT)))
   {
      sendbuffer[0] = TEENSY_DATA;
      adcstatus &= ~(1<<ADC_U_BIT);
      noInterrupts();
      //batt_M = readKanal(ADC_M);
      batt_M = analogRead(ADC_M);
      interrupts();
      Serial.print(F("ADC batt_M "));
      Serial.println(batt_M);
      sendbuffer[U_M_L_BYTE + DATA_START_BYTE] = batt_M & 0x00FF;
      sendbuffer[U_M_H_BYTE + DATA_START_BYTE] = (batt_M & 0xFF00)>>8;
      
      batt_O = analogRead(ADC_O);
      Serial.print(F("ADC batt_O "));
      Serial.println(batt_O);

      sendbuffer[U_O_L_BYTE + DATA_START_BYTE] = batt_O & 0x00FF;
      sendbuffer[U_O_H_BYTE + DATA_START_BYTE] = (batt_O & 0xFF00)>>8;
      
      adcstatus &= ~(1<<ADC_I_BIT);
      curr_U = analogRead(ADC_SHUNT_U);
      curr_O = analogRead(ADC_SHUNT_O);
      Serial.print(F("ADC usbsendcounter: "));
        Serial.println(usbsendcounter);
      senderfolg = RawHID.send(sendbuffer, 100);
      if (senderfolg > 0) 
      {
        Serial.print(F("ADC packet "));
        Serial.println(packetCount);
        packetCount = packetCount + 1;
        
      } else {
        Serial.println(F("Unable to transmit packet"));
      }
      usbsendcounter++;
   }

   
   
   if (sinceRecv > 10)
   {
      sinceRecv = 0;
      noInterrupts();
      r = RawHID.recv(recvbuffer, 0); 
      usbrecvcounter++;
      if (r > 0)
      {
         usbrecvcounter++;
         
         
         
         
      }
      interrupts();
   }
   
   /*
   if (n > 0) 
   {
     if (recvbuffer[0] == 0xAB) // Signature
     {
       //blink(2);
     }
   }
    */
   
   
   if (msUntilNextSend > 10000) {
      msUntilNextSend = 0;//msUntilNextSend - 2000;
      // first 2 bytes are a signature
      //recvbuffer[0] = 0xAB;
      //recvbuffer[1] = 0xCD;
      
      recvbuffer[2] = (usbsendcounter );
      /*
      // next 24 bytes are analog measurements
      for (int i=0; i<12; i++) {
        int val = analogRead(i);
        recvbuffer[i * 2 + 2] = highByte(val);
        recvbuffer[i * 2 + 3] = lowByte(val);
      }
      // fill the rest with zeros
      for (int i=26; i<62; i++) {
        recvbuffer[i] = 0;
      }
      // and put a count of packets sent at the end
      recvbuffer[62] = highByte(packetCount);
      recvbuffer[63] = lowByte(packetCount);
       */
      // actually send the packet
      /*
      senderfolg = RawHID.send(recvbuffer, 100);
      if (senderfolg > 0) 
      {
        Serial.print(F("Transmit packet "));
        Serial.println(packetCount);
        packetCount = packetCount + 1;
      } else {
        Serial.println(F("Unable to transmit packet"));
      }
      */
      //usbsendcounter++;
    }


}

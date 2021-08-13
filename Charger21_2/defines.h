//
//  Header.h
//  Charger21_Slave
//
//  Created by Ruedi Heimlicher on 05.08.2021.
//

#ifndef Header_h
#define Header_h

#define TIMER2_COMPA 0xA0 // 10ms
//#define TIMER2_DELAY 997   // 10ms mit OC2A 0xA0
#define TIMER2_DELAY 97
#define LADEPORT  PORTC
#define LADEDDR  DDRC
/*
#define LADESTROM_PWM_A PC0  
#define LADESTROM_PWM_B PC1  
*/
#define LADESTROM_PWM_A 10  
#define LADESTROM_PWM_B 11  

#define  TEENSY_DATA        0xFC // Daten des teensy lesen

#define DATACOUNT_LO_BYTE  5
#define DATACOUNT_HI_BYTE  6

#define DATA_START_BYTE 8 // erstes Databyte im buffer und sendbuffer
// ADC
#define ADCPORT PORTF
#define ADCDDR DDRF
/*
#define ADC_M  0
#define ADC_O  1
#define ADC_SHUNT_U 3
#define ADC_SHUNT_O 4
*/
#define ADC_M  38
#define ADC_O  39
#define ADC_SHUNT_U 40
#define ADC_SHUNT_O 41

#define ADC_TIMERDELAY  4800

#define ADC_U_BIT 0 // ISR: U messen
#define ADC_I_BIT 1 // ISR: I messen

#define U_MIN  3.0
#define U_OFF  2.5
#define U_MAX  4.2
// 
//USB Bytes 
#define TASK               7
//MARK: Charger Konstanten

// buffer
#define USB_DATENBREITE 64

#define  STROM_A_L_BYTE    8
#define  STROM_A_H_BYTE    9

#define  STROM_B_L_BYTE    10
#define  STROM_B_H_BYTE    11


// sendbuffer
#define U_M_L_BYTE 16
#define U_M_H_BYTE 17
#define U_O_L_BYTE 18
#define U_O_H_BYTE 19
#define I_SHUNT_L_BYTE 20
#define I_SHUNT_H_BYTE 21

//OSZI
#define OSZIPORT           PORTD
#define OSZIPORTDDR        DDRD
#define OSZIPORTPIN        PIND
/*
#define OSZI_PULS_A        0
#define OSZI_PULS_B        1
*/
#define OSZI_PULS_A        0
#define OSZI_PULS_B        1

#define OSZIALO OSZIPORT &= ~(1<<OSZI_PULS_A)
#define OSZIAHI OSZIPORT |= (1<<OSZI_PULS_A)
#define OSZIATOGG OSZIPORT ^= (1<<OSZI_PULS_A)

#define OSZIBLO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZIBHI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZIBTOGG OSZIPORT ^= (1<<OSZI_PULS_B)



#define TIMER0_STARTWERT   0x40

#define LOOPLEDDDR          DDRD    //DDRD
#define LOOPLEDPORT         PORTD   //PORTD
#define LOOPLED             6       //wie arduino 

#endif /* Header_h */

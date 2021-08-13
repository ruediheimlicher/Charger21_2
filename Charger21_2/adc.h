/*
 *  adc.h
 *  TWI_Master
 *
 *  Created by Sysadmin on 12.11.07.
 *  Copyright 2007 Ruedi Heimlicher. All rights reserved.
 *
 */

#include <stdio.h>
#include <util/delay.h>
#include <inttypes.h>
#include <stdlib.h>


struct adcwert16 {
  uint8_t wertH;
  uint8_t wertL;
  uint8_t wert8H;//obere 8 Bit
};

extern struct adcwert16 ADCWert16;
void initADC(uint8_t derKanal);
struct adcwert16 readKanal16Bit(uint8_t kanal);
void closeADC(void);
uint16_t readKanal(uint8_t derKanal);//Unsere Funktion zum ADC-Channel aus lesen

uint16_t readKanalOrig(uint8_t derKanal, uint8_t num); //Unsere Funktion zum ADC-Channel aus lesen
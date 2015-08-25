#ifndef __EEPROM_H
#define __EEPROM_H
#include "r_cg_userdefine.h"
#include "fdl.h"
void eeprom_write(int16_t ax,int16_t ay,int16_t gx,int16_t gy,int16_t gz,int16_t hx,int16_t hy,int16_t hz,
int16_t rateRollkp,int16_t ratePitchkp, int16_t rateYawkp, int16_t rateRollki, int16_t ratePitchki,int16_t rateYawki,int16_t rateRollkd,int16_t ratePitchkd,int16_t rateYawkd,int16_t angleRollkp,int16_t anglePitchkp,int16_t angleYawkp);

void eeprom_read(int16_t* ax,int16_t* ay,int16_t* gx,int16_t* gy,int16_t* gz,int16_t* hx,int16_t* hy,int16_t* hz,
float* rateRollkp,float* ratePitchkp, float* rateYawkp, float* rateRollki, float* ratePitchki,float* rateYawki,float* rateRollkd,float* ratePitchkd,float* rateYawkd,float* angleRollkp,float* anglePitchkp,float* angleYawkp);

void Data_FAL_Init(void);   
void Data_FAL_Erase(unsigned char blocknum);       
void Data_FAL_Write(unsigned char blocknum,unsigned long data_buff);         
unsigned long Data_FAL_Read(unsigned char blocknum);       
#endif


#include "eeprom.h"

extern __far const fal_descriptor_t  fal_descriptor_str;   
fal_status_t my_fal_status_t;
fal_request_t my_fal_request_t;

void Error_Handler(void)
{
	;
}


void Data_FAL_Init(void)    
{
	my_fal_status_t = FAL_Init((__far fal_descriptor_t*) &fal_descriptor_str);    
	if (my_fal_status_t != FAL_OK)  Error_Handler();
}  

void Data_FAL_Erase(unsigned char blocknum)
{
	my_fal_request_t.index_u16 = blocknum;   
	my_fal_request_t.command_enu = FAL_CMD_ERASE_BLOCK;	 //Erase block          
	FAL_Execute(&my_fal_request_t);     //zhixing erase block    
	while(my_fal_request_t.status_enu == FAL_BUSY) 
	FAL_Handler();    
	if (my_fal_request_t.status_enu != FAL_OK) 
	Error_Handler();      
} 

void Data_FAL_Write(unsigned char blocknum,unsigned long data_buff)
{
	my_fal_request_t.index_u16 = blocknum;     
	my_fal_request_t.data_u32 = data_buff;    
	my_fal_request_t.command_enu = FAL_CMD_WRITE_WORD;	 //Write 1 word                  
	FAL_Execute(&my_fal_request_t);     //zhixing write        
	while(my_fal_request_t.status_enu == FAL_BUSY) 
	FAL_Handler();    
	if (my_fal_request_t.status_enu != FAL_OK) 
	Error_Handler(); 
}    

unsigned long Data_FAL_Read(unsigned char blocknum)
{
	my_fal_request_t.index_u16 = blocknum;   
	my_fal_request_t.command_enu = FAL_CMD_READ_WORD;	 //read             
	FAL_Execute(&my_fal_request_t);     //zhixing read          
	if (my_fal_request_t.status_enu != FAL_OK) Error_Handler(); 
	return (my_fal_request_t.data_u32);     	
}


void eeprom_write(int16_t ax,int16_t ay,int16_t gx,int16_t gy,int16_t gz,int16_t hx,int16_t hy,int16_t hz,
int16_t rateRollkp,int16_t ratePitchkp, int16_t rateYawkp, int16_t rateRollki, int16_t ratePitchki,int16_t rateYawki,int16_t rateRollkd,int16_t ratePitchkd,int16_t rateYawkd,int16_t angleRollkp,int16_t anglePitchkp,int16_t angleYawkp)
{
	FAL_Open();             
	Data_FAL_Erase(0x0);               //erase block NO.
	Data_FAL_Write(0x00,gx);   //write data to flash.
	Data_FAL_Write(0x01,gy);   //write data to flash.
	Data_FAL_Write(0x02,gz);   //write data to flash.
	Data_FAL_Write(0x03,hx);   //write data to flash.
	Data_FAL_Write(0x04,hy);   //write data to flash.
	Data_FAL_Write(0x05,hz);   //write data to flash.
	Data_FAL_Write(0x06,ax);   //write data to flash.
	Data_FAL_Write(0x07,ay);   //write data to flash.
	Data_FAL_Write(0x08,rateRollkp);   //write data to flash.
	Data_FAL_Write(0x09,ratePitchkp);   //write data to flash.
	Data_FAL_Write(0x10,rateYawkp);   //write data to flash.
	Data_FAL_Write(0x11,rateRollki);   //write data to flash.
	Data_FAL_Write(0x12,ratePitchki);   //write data to flash.
	Data_FAL_Write(0x13,rateYawki);   //write data to flash.
	Data_FAL_Write(0x14,rateRollkd);   //write data to flash.
	Data_FAL_Write(0x15,ratePitchkd);   //write data to flash.
	Data_FAL_Write(0x16,rateYawkd);   //write data to flash.
	Data_FAL_Write(0x17,angleRollkp);   //write data to flash.
	Data_FAL_Write(0x18,anglePitchkp);   //write data to flash.
	Data_FAL_Write(0x19,angleYawkp);   //write data to flash.
	
	FAL_Close();
}
void eeprom_read(int16_t* ax,int16_t* ay,int16_t* gx,int16_t* gy,int16_t* gz,int16_t* hx,int16_t* hy,int16_t* hz,
float* rateRollkp,float* ratePitchkp, float* rateYawkp, float* rateRollki, float* ratePitchki,float* rateYawki,float* rateRollkd,float* ratePitchkd,float* rateYawkd,float* angleRollkp,float* anglePitchkp,float* angleYawkp)
{
	FAL_Open();             
	*gx=Data_FAL_Read(0x00);
	*gy=Data_FAL_Read(0x01);
	*gz=Data_FAL_Read(0x02);
	*hx=Data_FAL_Read(0x03);
	*hy=Data_FAL_Read(0x04);
	*hz=Data_FAL_Read(0x05);
	*ax=Data_FAL_Read(0x06);
	*ay=Data_FAL_Read(0x07);
	*rateRollkp=Data_FAL_Read(0x08);
	*ratePitchkp=Data_FAL_Read(0x09);
	*rateYawkp=Data_FAL_Read(0x10);
	*rateRollki=Data_FAL_Read(0x11);
	*ratePitchki=Data_FAL_Read(0x12);
	*rateYawki=Data_FAL_Read(0x13);
	*rateRollkd=Data_FAL_Read(0x14);
	*ratePitchkd=Data_FAL_Read(0x15);
	*rateYawkd=Data_FAL_Read(0x16);
	*angleRollkp=Data_FAL_Read(0x17);
	*anglePitchkp=Data_FAL_Read(0x18);
	*angleYawkp=Data_FAL_Read(0x19);
	FAL_Close();
}


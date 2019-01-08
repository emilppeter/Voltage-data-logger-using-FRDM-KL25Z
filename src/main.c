#include <MKL25Z4.H>
#include <stdio.h>
#include <string.h>
#include "gpio_defs.h"
#include "UART.h"
#define SCALE_FACTOR (3.2/2.2)
extern enum{CLI,Awaiting_samples,Saving_samples,Sending_data} next_state;
extern uint16_t g_samples[60],my_samples[60];
extern uint16_t g_NumSamplesRemaining;
extern uint16_t count;
extern uint16_t F_TPM_OVFLW;
extern uint32_t number;


void Awaiting_samples_state(void)
{	
	volatile float vref, vbat,res=0;
	volatile unsigned sum = 0;
	
	vref = 3.3; // measured from power supply rail
	Control_RGB_LEDs(0, 1, 0);
	PORTB->GPCLR = 0x10FFFF;
	PORTB->GPCHR = 0x10FFFF;
	switch(number)
	{
		case 0:ADC0->SC1[0] = 0x00; // start conversion on channel 0
						DMA0->DMA[0].DCR&=~DMA_DCR_EINT_MASK;
						while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK))
						;
						DMA0->DMA[0].DCR|=DMA_DCR_EINT_MASK;
						res = ADC0->R[0];
						break;
		case 3:ADC0->SC1[22] = 0x00; // start conversion on channel 0
						DMA0->DMA[0].DCR&=~DMA_DCR_EINT_MASK;
						while (!(ADC0->SC1[22] & ADC_SC1_COCO_MASK))
						;
						DMA0->DMA[0].DCR|=DMA_DCR_EINT_MASK;
						res = ADC0->R[22];
						break;
		case 6:ADC0->SC1[5] = 0x00; // start conversion on channel 0
						DMA0->DMA[0].DCR&=~DMA_DCR_EINT_MASK;
						while (!(ADC0->SC1[5] & ADC_SC1_COCO_MASK))
						;
						DMA0->DMA[0].DCR|=DMA_DCR_EINT_MASK;
						res = ADC0->R[5];
						break;
		case 11:ADC0->SC1[2] = 0x00; // start conversion on channel 0
						DMA0->DMA[0].DCR&=~DMA_DCR_EINT_MASK;
						while (!(ADC0->SC1[2] & ADC_SC1_COCO_MASK))
						;
						DMA0->DMA[0].DCR|=DMA_DCR_EINT_MASK;
						res = ADC0->R[2];
						break;
		case 14:ADC0->SC1[0] = 0x00; // start conversion on channel 0
						DMA0->DMA[0].DCR&=~DMA_DCR_EINT_MASK;
						while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK))
						;
						DMA0->DMA[0].DCR|=DMA_DCR_EINT_MASK;
						res = ADC0->R[0];
						break;
		case 23:ADC0->SC1[30] = 0x00; // start conversion on channel 0
						DMA0->DMA[0].DCR&=~DMA_DCR_EINT_MASK;
						while (!(ADC0->SC1[30] & ADC_SC1_COCO_MASK))
						;
						DMA0->DMA[0].DCR|=DMA_DCR_EINT_MASK;
						res = ADC0->R[30];
						break;			
	}	
	TPM1->STATUS &= ~TPM_STATUS_TOF_MASK;
}	
void Sending_data_state(void)
{
	uint32_t i=0;
	int res;
	float vbat=0;
	float vref = 3.3;
	Control_RGB_LEDs(1, 0, 1);
	PORTB->GPCLR = 0x1000FFFF;
	PORTB->GPCHR = 0x1000FFFF;
	DMA0->DMA[0].DSR_BCR = DMA_DSR_BCR_BCR(count*2);
	DMA0->DMA[0].SAR = DMA_SAR_SAR((uint32_t)&g_samples[0]);
	DMA0->DMA[0].DAR = DMA_DAR_DAR((uint32_t) &my_samples[0]);
	DMA0->DMA[0].DCR |= DMA_DCR_START_MASK;
	while (!(DMA0->DMA[0].DSR_BCR & DMA_DSR_BCR_DONE_MASK))
		;
	for(i=0;i<count;i++)
	{
		vbat = vref*(my_samples[i])/(4096);
		printf("\n\r%d.%f\n\r",i+1,vbat);
	}
	next_state=CLI;
}
void CLI_state(void)
{
	uint8_t buffer[80],c[10], * bp,channel_number[5],sampling_period[10],number_of_samples[10];
	uint32_t i,len=0,j,z;
	int period,temp;
	float temporary=0;
range:  i=0;len=0,period=0,temp=0;	
	memset(c, 0, sizeof(c));
	Control_RGB_LEDs(0, 0, 1);
	PORTB->GPCLR = 0x1FFFF;
	PORTB->GPCHR = 0x1FFFF;
	c[0]=UART0_Receive_Poll();
		while(c[i]!='\r')
		{	
			i++;
			c[i]=UART0_Receive_Poll();
			}		
		len=i;
		switch(c[0])
		{
			case 'R': printf("\n\rStart recording.\n\r");
								count=g_NumSamplesRemaining;
								Init_ADC();
								Init_DMA_To_Copy();
								Init_TPM0();
								next_state= Awaiting_samples;
								break;
			case 'S': printf("\n\rSending Data.\n\r");
								next_state= Sending_data;
								Init_DMA_To_Copy1();
								break;
			case 'C': memset(channel_number, 0, sizeof(channel_number));
								for(z=0,j=1;j<len;z++,j++)
								{
									channel_number[z]=c[j];
								}
								len=z;
								number=atoi(channel_number);
								temp=number;
								if ((temp==0) || (temp==3) || (temp==6) ||(temp==11) ||(temp==14 )|| (temp==23) || (temp==26) || (temp==27))
								{
									goto conti;;
								}
								else
								{
									printf("\n\rChannel number is invalid.Enter again.\n\r");
									goto range;
								}
								conti:printf("\n\rChannel %d selected.\n\r",temp);
								next_state= CLI;
								break;
			case 'P': memset(sampling_period, 0, sizeof(sampling_period));
								for(z=0,j=1;j<len;z++,j++)
								{
									sampling_period[z]=c[j];
								}
								len=z;
								period=atoi(sampling_period);
								if (period>1000000 || period<3)
								{
									printf("\n\rPeriod is out of range.Enter again.\n\r");
									goto range;
								}
								printf("\n\rSampling period set to %d.\n\r",period);
								temporary=((1000000.0)/(period));
								F_TPM_OVFLW=temporary;
								next_state= CLI;
								break;	
			case 'N': memset(number_of_samples, 0, sizeof(number_of_samples));
								for(z=0,j=1;j<len;z++,j++)
								{
									number_of_samples[z]=c[j];
								}
								len=z;
								g_NumSamplesRemaining=atoi(number_of_samples);
								temp=g_NumSamplesRemaining;
								if (g_NumSamplesRemaining>6000 || g_NumSamplesRemaining<=0)
								{
									printf("\n\rNumber of samples is out of range.Enter again.\n\r");
									goto range;									
								}	
								printf("\n\rSample count set to %d.\n\r",temp);
								next_state= CLI;
								break;
			default: printf("\n\rInvalid\n\r");
								break;
		}
}
void main(void)
{
	Init_UART0(115200);
	Init_RGB_LEDs();
	Init_TPM1(750);
	Init_Debug_signals();
	g_NumSamplesRemaining=0;
 	printf("ECE 592-066 Project 1");	
	Send_String_Poll("\n\rSubmitted by Emil Peter.\n\r");
	while(1)
	{
		switch(next_state)
		{
			case CLI:CLI_state();
							 break;
			case Awaiting_samples:Awaiting_samples_state();												
														break;
			case Sending_data:Sending_data_state();
												break;
			default:next_state=CLI;
							break;
		}	
	}	
}	

// *******************************ARM University Program Copyright © ARM Ltd 2013*************************************   

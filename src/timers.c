#include "timers.h"
#include <MKL25Z4.h>
#include "LEDs.h"
#include "user_defs.h"
#include <stdio.h>
uint16_t g_NumSamplesRemaining;
uint16_t g_samples[60];
uint16_t my_samples[60];
uint16_t count;
uint16_t F_TPM_OVFLW;
uint32_t number;
enum{CLI,Awaiting_samples,Saving_samples,Sending_data} next_state;
void Init_TPM0(void)
{
	// Turn on clock to TPM
	SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK;
	// Set clock source for tpm
	SIM->SOPT2 |= (SIM_SOPT2_TPMSRC(1) | SIM_SOPT2_PLLFLLSEL_MASK);
	// Load the counter and mod, given prescaler of 32
	TPM0->MOD = (F_TPM_CLOCK/(F_TPM_OVFLW*32))-1;
	// Set TPM to divide by 32 prescaler, enable counting (CMOD) and interrupts
	TPM0->SC = TPM_SC_CMOD(1) | TPM_SC_PS(5) | TPM_SC_TOIE_MASK;
}
void Init_TPM1(uint16_t period)
{
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;;
	
	// PTA12 connected to FTM1_CH0, Mux Alt 3
	// Set pin to FTM
	PORTA->PCR[12] &= ~PORT_PCR_MUX_MASK;          
	PORTA->PCR[12] |= PORT_PCR_MUX(3);          

	// Configure TPM
	SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK;
	SIM->SOPT2 |= (SIM_SOPT2_TPMSRC(1) | SIM_SOPT2_PLLFLLSEL_MASK);
	// Load the counter and mod
	TPM1->MOD = period-1;
	// Set TPM count direction to up with a divide by 2 prescaler
	TPM1->SC = TPM_SC_PS(7);
	// Continue operation in debug mode
	TPM1->CONF |= TPM_CONF_DBGMODE(3);
	// Set channel 1 to edge-aligned low-true PWM
	TPM1->CONTROLS[0].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSA_MASK;
	// Set initial duty cycle
	TPM1->CONTROLS[0].CnV = 375;
	// Start TPM
	TPM1->SC |= TPM_SC_CMOD(1);
}
#define ADC_POS (20)

void Init_ADC(void) {
	switch(number)
	{
		case 0:	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; 
						SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; 
						SIM->SOPT7 = SIM_SOPT7_ADC0TRGSEL(8)| SIM_SOPT7_ADC0ALTTRGEN(1);
						
						// Select analog for pin
						PORTE->PCR[20] &= ~PORT_PCR_MUX_MASK;	
						PORTE->PCR[20] |= PORT_PCR_MUX(0);	
						// Low power configuration, long sample time, 16 bit single-ended conversion, bus clock input
						ADC0->CFG1 = ADC_CFG1_ADLPC_MASK | ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(1) | ADC_CFG1_ADICLK(0);
						// Software trigger, compare function disabled, DMA disabled, voltage references VREFH and VREFL
						ADC0->SC2 = ADC_SC2_REFSEL(0)|ADC_SC2_ADTRG(1)|ADC_SC2_DMAEN(1);
						ADC0_OFS = ~ADC_OFS_OFS_MASK;
						break;
		case 3:	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; 
						SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; 
						SIM->SOPT7 = SIM_SOPT7_ADC0TRGSEL(8)| SIM_SOPT7_ADC0ALTTRGEN(1);
						
						// Select analog for pin
						PORTE->PCR[22] &= ~PORT_PCR_MUX_MASK;	
						PORTE->PCR[22] |= PORT_PCR_MUX(0);	
						// Low power configuration, long sample time, 16 bit single-ended conversion, bus clock input
						ADC0->CFG1 = ADC_CFG1_ADLPC_MASK | ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(1) | ADC_CFG1_ADICLK(0);
						// Software trigger, compare function disabled, DMA disabled, voltage references VREFH and VREFL
						ADC0->SC2 = ADC_SC2_REFSEL(0)|ADC_SC2_ADTRG(1)|ADC_SC2_DMAEN(1);
						ADC0_OFS = ~ADC_OFS_OFS_MASK;
						break;
		case 11:	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; 
							SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; 
							SIM->SOPT7 = SIM_SOPT7_ADC0TRGSEL(8)| SIM_SOPT7_ADC0ALTTRGEN(1);
							
							// Select analog for pin
							PORTC->PCR[2] &= ~PORT_PCR_MUX_MASK;	
							PORTC->PCR[2] |= PORT_PCR_MUX(0);	
							// Low power configuration, long sample time, 16 bit single-ended conversion, bus clock input
							ADC0->CFG1 = ADC_CFG1_ADLPC_MASK | ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(1) | ADC_CFG1_ADICLK(0);
							// Software trigger, compare function disabled, DMA disabled, voltage references VREFH and VREFL
							ADC0->SC2 = ADC_SC2_REFSEL(0)|ADC_SC2_ADTRG(1)|ADC_SC2_DMAEN(1);
							ADC0_OFS = ~ADC_OFS_OFS_MASK;
							break;
		case 14:	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; 
							SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK; 
							SIM->SOPT7 = SIM_SOPT7_ADC0TRGSEL(8)| SIM_SOPT7_ADC0ALTTRGEN(1);
							
							// Select analog for pin
							PORTE->PCR[0] &= ~PORT_PCR_MUX_MASK;	
							PORTE->PCR[0] |= PORT_PCR_MUX(0);	
							// Low power configuration, long sample time, 16 bit single-ended conversion, bus clock input
							ADC0->CFG1 = ADC_CFG1_ADLPC_MASK | ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(1) | ADC_CFG1_ADICLK(0);
							// Software trigger, compare function disabled, DMA disabled, voltage references VREFH and VREFL
							ADC0->SC2 = ADC_SC2_REFSEL(0)|ADC_SC2_ADTRG(1)|ADC_SC2_DMAEN(1);
							ADC0_OFS = ~ADC_OFS_OFS_MASK;
							break;
	case 23:	SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; 
						SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; 
						SIM->SOPT7 = SIM_SOPT7_ADC0TRGSEL(8)| SIM_SOPT7_ADC0ALTTRGEN(1);
						
						// Select analog for pin
						PORTE->PCR[30] &= ~PORT_PCR_MUX_MASK;	
						PORTE->PCR[30] |= PORT_PCR_MUX(0);	
						// Low power configuration, long sample time, 16 bit single-ended conversion, bus clock input
						ADC0->CFG1 = ADC_CFG1_ADLPC_MASK | ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(1) | ADC_CFG1_ADICLK(0);
						// Software trigger, compare function disabled, DMA disabled, voltage references VREFH and VREFL
						ADC0->SC2 = ADC_SC2_REFSEL(0)|ADC_SC2_ADTRG(1)|ADC_SC2_DMAEN(1);
						ADC0_OFS = ~ADC_OFS_OFS_MASK;
						break;
	case 6:		SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; 
						SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK; 
						SIM->SOPT7 = SIM_SOPT7_ADC0TRGSEL(8)| SIM_SOPT7_ADC0ALTTRGEN(1);
						
						// Select analog for pin
						PORTD->PCR[5] &= ~PORT_PCR_MUX_MASK;	
						PORTD->PCR[5] |= PORT_PCR_MUX(0);	
						// Low power configuration, long sample time, 16 bit single-ended conversion, bus clock input
						ADC0->CFG1 = ADC_CFG1_ADLPC_MASK | ADC_CFG1_ADLSMP_MASK | ADC_CFG1_MODE(1) | ADC_CFG1_ADICLK(0);
						// Software trigger, compare function disabled, DMA disabled, voltage references VREFH and VREFL
						ADC0->SC2 = ADC_SC2_REFSEL(0)|ADC_SC2_ADTRG(1)|ADC_SC2_DMAEN(1);
						ADC0_OFS = ~ADC_OFS_OFS_MASK;
						break;
	}
		
}	
void Init_DMA_To_Copy() 
{
	SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;
	SIM->SCGC6 |= SIM_SCGC6_DMAMUX_MASK;
	DMAMUX0->CHCFG[0]=0;
	DMA0->DMA[0].DCR = DMA_DCR_SSIZE(2) |
		DMA_DCR_DINC_MASK |DMA_DCR_DSIZE(2)|DMA_DCR_EINT_MASK|DMA_DCR_CS_MASK|DMA_DCR_ERQ_MASK|DMA_DCR_D_REQ_MASK;
	DMAMUX0->CHCFG[0] = DMAMUX_CHCFG_SOURCE(40);	
	DMA0->DMA[0].DSR_BCR = DMA_DSR_BCR_BCR(g_NumSamplesRemaining*2);
	DMA0->DMA[0].SAR = DMA_SAR_SAR((uint32_t)&ADC0_RA);
	DMA0->DMA[0].DAR = DMA_DAR_DAR((uint32_t) &g_samples[0]);
	DMAMUX0->CHCFG[0] |= DMAMUX_CHCFG_ENBL_MASK;
	NVIC_SetPriority(DMA0_IRQn, 2);
	NVIC_ClearPendingIRQ(DMA0_IRQn); 
	NVIC_EnableIRQ(DMA0_IRQn);	
	__enable_irq();
}
void Init_DMA_To_Copy1(void) 
{
	SIM->SCGC7 |= SIM_SCGC7_DMA_MASK;
	DMA0->DMA[0].DCR = DMA_DCR_SINC_MASK | DMA_DCR_SSIZE(0) |
		DMA_DCR_DINC_MASK |	DMA_DCR_DSIZE(0);
}
void DMA0_IRQHandler(void) 
	{
			Control_RGB_LEDs(1, 1, 0);
			PORTB->GPCLR = 0x100FFFF;
			PORTB->GPCHR = 0x100FFFF;
			DMA0->DMA[0].DSR_BCR|= DMA_DSR_BCR_DONE_MASK;
			DMAMUX0->CHCFG[0] &= ~DMAMUX_CHCFG_ENBL_MASK;
			g_NumSamplesRemaining=0;
			TPM0->SC = TPM_SC_CMOD(0);
			next_state=CLI;
			
}

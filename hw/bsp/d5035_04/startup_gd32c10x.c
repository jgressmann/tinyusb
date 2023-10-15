/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */


#include <stdint.h>
#include <gd32c10x.h>
#include "startup_gd32c10x.h"


/* Initialize segments */
extern uint32_t _sfixed;
extern uint32_t _efixed;
extern uint32_t _etext;
extern uint32_t _srelocate;
extern uint32_t _erelocate;
extern uint32_t _szero;
extern uint32_t _ezero;
extern uint32_t _sstack;
extern uint32_t _estack;

#define RCU_MODIFY(__delay)     do{                                     \
                                    volatile uint32_t i;                \
                                    if(0 != __delay){                   \
                                        RCU_CFG0 |= RCU_AHB_CKSYS_DIV2; \
                                        for(i=0; i<__delay; i++){       \
                                        }                               \
                                        RCU_CFG0 |= RCU_AHB_CKSYS_DIV4; \
                                        for(i=0; i<__delay; i++){       \
                                        }                               \
                                    }                                   \
                                }while(0)

int main(void);

void Reset_Handler(void)
{
	uint32_t *pSrc, *pDest;

	/* Initialize the relocate segment */
	pSrc  = &_etext;
	pDest = &_srelocate;

	if (pSrc != pDest) {
		for (; pDest < &_erelocate;) {
			*pDest++ = *pSrc++;
		}
	}

	/* Clear the zero segment */
	for (pDest = &_szero; pDest < &_ezero;) {
		*pDest++ = 0;
	}

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
#endif
    /* reset the RCU clock configuration to the default reset state */
    /* Set IRC8MEN bit */
    RCU_CTL |= RCU_CTL_IRC8MEN;

	RCU_MODIFY(0x50);

    RCU_CFG0 &= ~RCU_CFG0_SCS;

    /* Reset HXTALEN, CKMEN, PLLEN, PLL1EN and PLL2EN bits */
    RCU_CTL &= ~(RCU_CTL_PLLEN |RCU_CTL_PLL1EN | RCU_CTL_PLL2EN | RCU_CTL_CKMEN | RCU_CTL_HXTALEN);
    /* disable all interrupts */
    RCU_INT = 0x00ff0000U;

    /* Reset CFG0 and CFG1 registers */
    RCU_CFG0 = 0x00000000U;
    RCU_CFG1 = 0x00000000U;

    /* reset HXTALBPS bit */
    RCU_CTL &= ~(RCU_CTL_HXTALBPS);


	/* Set the vector table base address */
	pSrc      = (uint32_t *)&_sfixed;
	SCB->VTOR = ((uint32_t)pSrc & SCB_VTOR_TBLOFF_Msk);
	__DSB();

	/* Branch to main function */
	main();

	/* Infinite loop */
	for (;;);
}

static void Dummy_Handler(void)
{
	for (;;);
}



#define GD32C10X_ISR(handler) void handler(void) __attribute__((weak, alias("Dummy_Handler")))

GD32C10X_ISR(NMI_Handler);
GD32C10X_ISR(HardFault_Handler);
GD32C10X_ISR(MemManage_Handler);
GD32C10X_ISR(BusFault_Handler);
GD32C10X_ISR(UsageFault_Handler);
GD32C10X_ISR(SVC_Handler);
GD32C10X_ISR(DebugMon_Handler);
GD32C10X_ISR(PendSV_Handler);
GD32C10X_ISR(SysTick_Handler);
GD32C10X_ISR(WWDGT_IRQHandler);
GD32C10X_ISR(LVD_IRQHandler);
GD32C10X_ISR(TAMPER_IRQHandler);
GD32C10X_ISR(RTC_IRQHandler);
GD32C10X_ISR(FMC_IRQHandler);
GD32C10X_ISR(RCU_CTC_IRQHandler);
GD32C10X_ISR(EXTI0_IRQHandler);
GD32C10X_ISR(EXTI1_IRQHandler);
GD32C10X_ISR(EXTI2_IRQHandler);
GD32C10X_ISR(EXTI3_IRQHandler);
GD32C10X_ISR(EXTI4_IRQHandler);
GD32C10X_ISR(DMA0_Channel0_IRQHandler);
GD32C10X_ISR(DMA0_Channel1_IRQHandler);
GD32C10X_ISR(DMA0_Channel2_IRQHandler);
GD32C10X_ISR(DMA0_Channel3_IRQHandler);
GD32C10X_ISR(DMA0_Channel4_IRQHandler);
GD32C10X_ISR(DMA0_Channel5_IRQHandler);
GD32C10X_ISR(DMA0_Channel6_IRQHandler);
GD32C10X_ISR(ADC0_1_IRQHandler);
GD32C10X_ISR(CAN0_TX_IRQHandler);
GD32C10X_ISR(CAN0_RX0_IRQHandler);
GD32C10X_ISR(CAN0_RX1_IRQHandler);
GD32C10X_ISR(CAN0_EWMC_IRQHandler);
GD32C10X_ISR(EXTI5_9_IRQHandler);
GD32C10X_ISR(TIMER0_BRK_TIMER8_IRQHandler);
GD32C10X_ISR(TIMER0_UP_TIMER9_IRQHandler);
GD32C10X_ISR(TIMER0_TRG_CMT_TIMER10_IRQHandler);
GD32C10X_ISR(TIMER0_Channel_IRQHandler);
GD32C10X_ISR(TIMER1_IRQHandler);
GD32C10X_ISR(TIMER2_IRQHandler);
GD32C10X_ISR(TIMER3_IRQHandler);
GD32C10X_ISR(I2C0_EV_IRQHandler);
GD32C10X_ISR(I2C0_ER_IRQHandler);
GD32C10X_ISR(I2C1_EV_IRQHandler);
GD32C10X_ISR(I2C1_ER_IRQHandler);
GD32C10X_ISR(SPI0_IRQHandler);
GD32C10X_ISR(SPI1_IRQHandler);
GD32C10X_ISR(USART0_IRQHandler);
GD32C10X_ISR(USART1_IRQHandler);
GD32C10X_ISR(USART2_IRQHandler);
GD32C10X_ISR(EXTI10_15_IRQHandler);
GD32C10X_ISR(RTC_Alarm_IRQHandler);
GD32C10X_ISR(USBFS_WKUP_IRQHandler);
GD32C10X_ISR(TIMER7_BRK_TIMER11_IRQHandler);
GD32C10X_ISR(TIMER7_UP_TIMER12_IRQHandler);
GD32C10X_ISR(TIMER7_TRG_CMT_TIMER13_IRQHandler);
GD32C10X_ISR(TIMER7_Channel_IRQHandler);
GD32C10X_ISR(EXMC_IRQHandler);
GD32C10X_ISR(TIMER4_IRQHandler);
GD32C10X_ISR(SPI2_IRQHandler);
GD32C10X_ISR(UART3_IRQHandler);
GD32C10X_ISR(UART4_IRQHandler);
GD32C10X_ISR(TIMER5_IRQHandler);
GD32C10X_ISR(TIMER6_IRQHandler);
GD32C10X_ISR(DMA1_Channel0_IRQHandler);
GD32C10X_ISR(DMA1_Channel1_IRQHandler);
GD32C10X_ISR(DMA1_Channel2_IRQHandler);
GD32C10X_ISR(DMA1_Channel3_IRQHandler);
GD32C10X_ISR(DMA1_Channel4_IRQHandler);
GD32C10X_ISR(CAN1_TX_IRQHandler);
GD32C10X_ISR(CAN1_RX0_IRQHandler);
GD32C10X_ISR(CAN1_RX1_IRQHandler);
GD32C10X_ISR(CAN1_EWMC_IRQHandler);
GD32C10X_ISR(USBFS_IRQHandler);

#undef GD32C10X_ISR

typedef struct {
	void* pvStack;
	void* pfnReset_Handler;
	void* pfnNonMaskableInt_Handler;
	void* pfnHardFault_Handler;
	void* pfnMemManagement_Handler;
	void* pfnBusFault_Handler;
	void* pfnUsageFault_Handler;
	void* pvReservedM9;
	void* pvReservedM8;
	void* pvReservedM7;
	void* pvReservedM6;
	void* pfnSVCall_Handler;
	void* pfnDebugMonitor_Handler;
	void* pvReservedM3;
	void* pfnPendSV_Handler;
	void* pfnSysTick_Handler;
	void* pfnWWDGT_IRQHandler;
	void* pfnLVD_IRQHandler;
	void* pfnTAMPER_IRQHandler;
	void* pfnRTC_IRQHandler;
	void* pfnFMC_IRQHandler;
	void* pfnRCU_CTC_IRQHandler;
	void* pfnEXTI0_IRQHandler;
	void* pfnEXTI1_IRQHandler;
	void* pfnEXTI2_IRQHandler;
	void* pfnEXTI3_IRQHandler;
	void* pfnEXTI4_IRQHandler;
	void* pfnDMA0_Channel0_IRQHandler;
	void* pfnDMA0_Channel1_IRQHandler;
	void* pfnDMA0_Channel2_IRQHandler;
	void* pfnDMA0_Channel3_IRQHandler;
	void* pfnDMA0_Channel4_IRQHandler;
	void* pfnDMA0_Channel5_IRQHandler;
	void* pfnDMA0_Channel6_IRQHandler;
	void* pfnADC0_1_IRQHandler;
	void* pfnCAN0_TX_IRQHandler;
	void* pfnCAN0_RX0_IRQHandler;
	void* pfnCAN0_RX1_IRQHandler;
	void* pfnCAN0_EWMC_IRQHandler;
	void* pfnEXTI5_9_IRQHandler;
	void* pfnTIMER0_BRK_TIMER8_IRQHandler;
	void* pfnTIMER0_UP_TIMER9_IRQHandler;
	void* pfnTIMER0_TRG_CMT_TIMER10_IRQHandler;
	void* pfnTIMER0_Channel_IRQHandler;
	void* pfnTIMER1_IRQHandler;
	void* pfnTIMER2_IRQHandler;
	void* pfnTIMER3_IRQHandler;
	void* pfnI2C0_EV_IRQHandler;
	void* pfnI2C0_ER_IRQHandler;
	void* pfnI2C1_EV_IRQHandler;
	void* pfnI2C1_ER_IRQHandler;
	void* pfnSPI0_IRQHandler;
	void* pfnSPI1_IRQHandler;
	void* pfnUSART0_IRQHandler;
	void* pfnUSART1_IRQHandler;
	void* pfnUSART2_IRQHandler;
	void* pfnEXTI10_15_IRQHandler;
	void* pfnRTC_Alarm_IRQHandler;
	void* pfnUSBFS_WKUP_IRQHandler;
	void* pfnTIMER7_BRK_TIMER11_IRQHandler;
	void* pfnTIMER7_UP_TIMER12_IRQHandler;
	void* pfnTIMER7_TRG_CMT_TIMER13_IRQHandler;
	void* pfnTIMER7_Channel_IRQHandler;
	void* pvReserved63;
	void* pfnEXMC_IRQHandler;
	void* pvReserved65;
	void* pfnTIMER4_IRQHandler;
	void* pfnSPI2_IRQHandler;
	void* pfnUART3_IRQHandler;
	void* pfnUART4_IRQHandler;
	void* pfnTIMER5_IRQHandler;
	void* pfnTIMER6_IRQHandler;
	void* pfnDMA1_Channel0_IRQHandler;
	void* pfnDMA1_Channel1_IRQHandler;
	void* pfnDMA1_Channel2_IRQHandler;
	void* pfnDMA1_Channel3_IRQHandler;
	void* pfnDMA1_Channel4_IRQHandler;
	void* pvReserved77;
	void* pvReserved78;
	void* pfnCAN1_TX_IRQHandler;
	void* pfnCAN1_RX0_IRQHandler;
	void* pfnCAN1_RX1_IRQHandler;
	void* pfnCAN1_EWMC_IRQHandler;
	void* pfnUSBFS_IRQHandler;
} gd32c10x_vectors;


__attribute__((section(".vectors"), used)) static const gd32c10x_vectors vectors = {
	/* Configure Initial Stack Pointer, using linker-generated symbols */
	.pvStack                   = (void *)(&_estack),

	/* Cortex-M handlers */
	.pfnReset_Handler          = (void *)Reset_Handler,
	.pfnNonMaskableInt_Handler = (void *)NMI_Handler,
	.pfnHardFault_Handler      = (void *)HardFault_Handler,
	.pfnMemManagement_Handler  = (void *)MemManage_Handler,
	.pfnBusFault_Handler       = (void *)BusFault_Handler,
	.pfnUsageFault_Handler     = (void *)UsageFault_Handler,
	.pvReservedM9              = (void *)(0UL), /* Reserved */
	.pvReservedM8              = (void *)(0UL), /* Reserved */
	.pvReservedM7              = (void *)(0UL), /* Reserved */
	.pvReservedM6              = (void *)(0UL), /* Reserved */
	.pfnSVCall_Handler         = (void *)SVC_Handler,
	.pfnDebugMonitor_Handler   = (void *)DebugMon_Handler,
	.pvReservedM3              = (void *)(0UL), /* Reserved */
	.pfnPendSV_Handler         = (void *)PendSV_Handler,
	.pfnSysTick_Handler        = (void *)SysTick_Handler,

	/* Configurable interrupts */
	.pfnWWDGT_IRQHandler  = (void *)WWDGT_IRQHandler,        /* 16:Window Watchdog Timer */
	.pfnLVD_IRQHandler      = (void *)LVD_IRQHandler,      /* 17:LVD through EXTI Line detect */
	.pfnTAMPER_IRQHandler = (void *)TAMPER_IRQHandler, /* 18:Tamper through EXTI Line detect */
	.pfnRTC_IRQHandler = (void *)RTC_IRQHandler, /*  19:RTC through EXTI Line */
	.pfnFMC_IRQHandler = (void *)FMC_IRQHandler, /* 20:FMC */
	.pfnRCU_CTC_IRQHandler = (void *)RCU_CTC_IRQHandler, /* 21:RCU and CTC */
	.pfnEXTI0_IRQHandler = (void *)EXTI0_IRQHandler, /* 22:EXTI Line 0 */
	.pfnEXTI1_IRQHandler = (void *)EXTI1_IRQHandler, /* 23:EXTI Line 1 */
	.pfnEXTI2_IRQHandler     = (void *)EXTI2_IRQHandler, /*  24:EXTI Line 2 */
	.pfnEXTI3_IRQHandler    = (void *)EXTI3_IRQHandler,  /* 25:EXTI Line 3 */
	.pfnEXTI4_IRQHandler       = (void *)EXTI4_IRQHandler,     /* 26:EXTI Line 4 */
	.pfnDMA0_Channel0_IRQHandler       = (void *)DMA0_Channel0_IRQHandler,     /* 27:DMA0 Channel0 */
	.pfnDMA0_Channel1_IRQHandler     = (void *)DMA0_Channel1_IRQHandler,   /* 28:DMA0 Channel1 */
	.pfnDMA0_Channel2_IRQHandler     = (void *)DMA0_Channel2_IRQHandler,   /* 29:DMA0 Channel2 */
	.pfnDMA0_Channel3_IRQHandler     = (void *)DMA0_Channel3_IRQHandler,   /* 30:DMA0 Channel3 */
	.pfnDMA0_Channel4_IRQHandler     = (void *)DMA0_Channel4_IRQHandler,   /* 31:DMA0 Channel4 */
	.pfnDMA0_Channel5_IRQHandler     = (void *)DMA0_Channel5_IRQHandler,   /* 32:DMA0 Channel5 */
	.pfnDMA0_Channel6_IRQHandler     = (void *)DMA0_Channel6_IRQHandler,   /* 33:DMA0 Channel6 */
	.pfnADC0_1_IRQHandler     = (void *)ADC0_1_IRQHandler,   /* 34:ADC0 and ADC1 */
	.pfnCAN0_TX_IRQHandler     = (void *)CAN0_TX_IRQHandler,   /* 35:CAN0 TX */
	.pfnCAN0_RX0_IRQHandler     = (void *)CAN0_RX0_IRQHandler,   /* 36:CAN0 RX0 */
	.pfnCAN0_RX1_IRQHandler     = (void *)CAN0_RX1_IRQHandler,   /* 37:CAN0 RX1 */
	.pfnCAN0_EWMC_IRQHandler    = (void *)CAN0_EWMC_IRQHandler,  /* 38:CAN0 EWMC */
	.pfnEXTI5_9_IRQHandler    = (void *)EXTI5_9_IRQHandler,  /* 39:EXTI5 to EXTI9 */
	.pfnTIMER0_BRK_TIMER8_IRQHandler    = (void *)TIMER0_BRK_TIMER8_IRQHandler,  /* 40:TIMER0 Break and TIMER8 */
	.pfnTIMER0_UP_TIMER9_IRQHandler    = (void *)TIMER0_UP_TIMER9_IRQHandler,  /* 41:TIMER0 Update and TIMER9 */
	.pfnTIMER0_TRG_CMT_TIMER10_IRQHandler    = (void *)TIMER0_TRG_CMT_TIMER10_IRQHandler,  /* 42:TIMER0 Trigger and Commutation and TIMER10 */
	.pfnTIMER0_Channel_IRQHandler    = (void *)TIMER0_Channel_IRQHandler,  /* 43:TIMER0 Channel Capture Compare */
	.pfnTIMER1_IRQHandler     = (void *)TIMER1_IRQHandler,   /* 44:TIMER1*/
	.pfnTIMER2_IRQHandler     = (void *)TIMER2_IRQHandler,   /* 45:TIMER2 */
	.pfnTIMER3_IRQHandler     = (void *)TIMER3_IRQHandler,   /* 46:TIMER3 */
	.pfnI2C0_EV_IRQHandler     = (void *)I2C0_EV_IRQHandler,   /* 47:I2C0 Event */
	.pfnI2C0_ER_IRQHandler     = (void *)I2C0_ER_IRQHandler,   /* 48:I2C0 Error */
	.pfnI2C1_EV_IRQHandler     = (void *)I2C1_EV_IRQHandler,   /* 49:I2C1 Event */
	.pfnI2C1_ER_IRQHandler     = (void *)I2C1_ER_IRQHandler,   /* 50:I2C1 Error */
	.pfnSPI0_IRQHandler     = (void *)SPI0_IRQHandler,   /* 51:SPI0 */
	.pfnSPI1_IRQHandler     = (void *)SPI1_IRQHandler,   /* 52:SPI1 */
	.pfnUSART0_IRQHandler     = (void *)USART0_IRQHandler,   /* 53:USART0 */
	.pfnUSART1_IRQHandler     = (void *)USART1_IRQHandler,   /* 54:USART1 */
	.pfnUSART2_IRQHandler     = (void *)USART2_IRQHandler,   /* 55:USART2 */
	.pfnEXTI10_15_IRQHandler     = (void *)EXTI10_15_IRQHandler,   /* 56:EXTI10 to EXTI15 */
	.pfnRTC_Alarm_IRQHandler     = (void *)RTC_Alarm_IRQHandler,   /* 57:RTC Alarm */
	.pfnUSBFS_WKUP_IRQHandler     = (void *)USBFS_WKUP_IRQHandler,   /* 58:USBFS Wakeup */
	.pfnTIMER7_BRK_TIMER11_IRQHandler     = (void *)TIMER7_BRK_TIMER11_IRQHandler,   /* 59:TIMER7 Break and TIMER11 */
	.pfnTIMER7_UP_TIMER12_IRQHandler     = (void *)TIMER7_UP_TIMER12_IRQHandler,   /* 60:TIMER7 Update and TIMER12 */
	.pfnTIMER7_TRG_CMT_TIMER13_IRQHandler     = (void *)TIMER7_TRG_CMT_TIMER13_IRQHandler,   /* 61:TIMER7 Trigger and Commutation and TIMER13 */
	.pfnTIMER7_Channel_IRQHandler     = (void *)TIMER7_Channel_IRQHandler,   /* 62:TIMER7 Channel Capture Compare */
	.pvReserved63              = (void *)(0UL), /* 63:Reserved */
	.pfnEXMC_IRQHandler    = (void *)EXMC_IRQHandler,   /* 64:EXMC */
	.pvReserved65              = (void *)(0UL), /* 65:Reserved */
	.pfnTIMER4_IRQHandler    = (void *)TIMER4_IRQHandler,   /* 66:TIMER4 */
	.pfnSPI2_IRQHandler    = (void *)SPI2_IRQHandler,   /* 67:SPI2 */
	.pfnUART3_IRQHandler    = (void *)UART3_IRQHandler,   /* 268:UART3 */
	.pfnUART4_IRQHandler    = (void *)UART4_IRQHandler,   /* 69:UART4 */
	.pfnTIMER5_IRQHandler    = (void *)TIMER5_IRQHandler,   /* 70:TIMER5  */
	.pfnTIMER6_IRQHandler    = (void *)TIMER6_IRQHandler,   /* 71:TIMER6 */
	.pfnDMA1_Channel0_IRQHandler    = (void *)DMA1_Channel0_IRQHandler,   /* 72:DMA1 Channel0 */
	.pfnDMA1_Channel1_IRQHandler    = (void *)DMA1_Channel1_IRQHandler,   /* 73:DMA1 Channel1 */
	.pfnDMA1_Channel2_IRQHandler    = (void *)DMA1_Channel2_IRQHandler,   /* 74:DMA1 Channel2 */
	.pfnDMA1_Channel3_IRQHandler    = (void *)DMA1_Channel3_IRQHandler,   /* 75:DMA1 Channel3 */
	.pfnDMA1_Channel4_IRQHandler    = (void *)DMA1_Channel4_IRQHandler,   /* 76:DMA1 Channel4 */
	.pvReserved77              = (void *)(0UL), /* 77:Reserved */
	.pvReserved78              = (void *)(0UL), /* 78:Reserved */
	.pfnCAN1_TX_IRQHandler    = (void *)CAN1_TX_IRQHandler,   /* 79:CAN1 TX */
	.pfnCAN1_RX0_IRQHandler    = (void *)CAN1_RX0_IRQHandler,   /* 80:CAN1 RX0 */
	.pfnCAN1_RX1_IRQHandler    = (void *)CAN1_RX1_IRQHandler,   /* 81:CAN1 RX1 */
	.pfnCAN1_EWMC_IRQHandler    = (void *)CAN1_EWMC_IRQHandler,   /* 82:CAN1 EWMC */
	.pfnUSBFS_IRQHandler    = (void *)USBFS_IRQHandler   /* 83:USBFS */
};



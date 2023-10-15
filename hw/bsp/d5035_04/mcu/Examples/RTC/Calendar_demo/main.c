/*!
    \file    main.c
    \brief   RTC calendar 
    
    \version 2023-06-16, V1.2.0, firmware for GD32C10x
*/

/*
    Copyright (c) 2023, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32c10x.h"
#include "gd32c10x_eval.h"
#include "gd32c10x_lcd_eval.h"
#include "rtc.h"
#include "systick.h"
#include <stdio.h>

#define BACKUP_DATA   (0xABCD)

void rtc_process(void);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* COM1 config */
    gd_eval_com_init(EVAL_COM1);

    /* NVIC config */
    nvic_configuration();

    /* systick config */
    systick_config();

    /* initialize the LCD of GD EVAL board */
    gd_eval_lcd_init();

    /* clear the LCD screen */
    lcd_clear(WHITE);

    /* draw a rectangle */
    lcd_rectangle_draw(10, 10, 230, 310, BLUE);

    /* different RTC processes depending on the backup data to display calendar on LCD */
    rtc_process();

    /* clear reset flags */
    rcu_all_reset_flag_clear();

    while (1){
    }
}

/*!
    \brief      different processes depending on the backup data to display calendar on LCD
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rtc_process(void)
{
    uint32_t rtcsrc_flag = 0;

    printf("\r\n This is a RTC demo......" );

    /* get RTC clock entry selection */
    rtcsrc_flag = GET_BITS(RCU_BDCTL, 8, 9);

    if((bkp_data_read(BKP_DATA_0) != BACKUP_DATA) || (0U == rtcsrc_flag)){
        /* backup data register value is not correct or not yet programmed
        or RTC clock source is not configured (when the first time the program
        is executed or data in RCU_BDCTL is lost due to Vbat feeding) */
        printf("\r\n RTC has not been configured yet....");

        /* RTC configuration */
        rtc_configuration();

        printf("\r\n start the RTC configuration....");

        /* adjust time by values entered by the user on the hyper terminal */
        time_adjust();

        bkp_data_write(BKP_DATA_0, BACKUP_DATA);
    }else{
        /* check if the power on/down reset flag is set */
        if(rcu_flag_get(RCU_FLAG_PORRST) != RESET){
            printf("\r\n\n Power On/Down Reset occurred....");
        }else if(rcu_flag_get(RCU_FLAG_SWRST) != RESET){
            /* check if the pin reset flag is set */
            printf("\r\n\n External Reset occurred....");
        }

        /* allow access to BKP domain */
        rcu_periph_clock_enable(RCU_PMU);
        pmu_backup_write_enable();
    
        printf("\r\n No need to configure RTC....");
        /* wait for RTC registers synchronization */
        rtc_register_sync_wait();
        rtc_lwoff_wait();
        /* enable the RTC second and alarm interrupt*/
        rtc_interrupt_enable(RTC_INT_SECOND);
        /* wait until last write operation on RTC registers has finished */
        rtc_lwoff_wait();
    }
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(EVAL_COM1, (uint8_t)ch);
    while(RESET == usart_flag_get(EVAL_COM1, USART_FLAG_TBE));

    return ch;
}

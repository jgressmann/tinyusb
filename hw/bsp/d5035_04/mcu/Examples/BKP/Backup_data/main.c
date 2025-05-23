/*!
    \file    main.c
    \brief   backup data register
    
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

#define DATA                          (0x1226)
#define BKP_DATA_REG_MIN              (BKP_DATA_0)
#define BKP_DATA_REG_MAX              (BKP_DATA_41)

void led_config(void);
void rcu_config(void);
void backup_register_write_and_check(void);
void write_backup_register(uint16_t data);
uint32_t check_backup_register(uint16_t data);

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* configure leds */
    led_config();

    /* write data to backup register and check it after next power reset */
    backup_register_write_and_check();

    while(1){
    }
}

/*!
    \brief      configure leds
    \param[in]  none
    \param[out] none
    \retval     none
*/
void led_config(void)
{
    gd_eval_led_init(LED1);
    gd_eval_led_init(LED2);
}

/*!
    \brief      enable the peripheral clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_config(void)
{
    /* PMU clock enable */
    rcu_periph_clock_enable(RCU_PMU);
    /* BKP clock enable */
    rcu_periph_clock_enable(RCU_BKPI);
}

/*!
    \brief      write data to backup register and check it after next power reset
    \param[in]  none
    \param[out] none
    \retval     none
*/
void backup_register_write_and_check(void)
{
    /* check if backup data registers has been written */
    if(0x00 == check_backup_register(DATA)){
        /* backup data registers values are correct */
        /* turn on LED1 */
        gd_eval_led_on(LED1);
    }else{
        /* enable the peripheral clock */
        rcu_config();

        /* enable write access to the registers in backup domain */
        pmu_backup_write_enable();
        
        /* backup data registers values are not correct or they have not been written */
        /* write data to backup data registers */
        write_backup_register(DATA);
        /* turn on LED2 */
        gd_eval_led_on(LED2);
    }
}

/*!
    \brief      write data to backup DATAx registers
    \param[in]  data: the data to be written to backup data registers
      \arg        0x0000-0xFFFF
    \param[out] none
    \retval     none
*/
void write_backup_register(uint16_t data)
{
    bkp_data_register_enum temp = BKP_DATA_0;
    /* construct different data and write data to backup registers */
    for(temp = BKP_DATA_REG_MIN; temp <= BKP_DATA_REG_MAX; temp++){
        bkp_data_write(temp, (data + (temp * 0x50)));
    }
}

/*!
    \brief      check if the backup DATAx registers values are correct or not
    \param[in]  data: the data to be written to backup data registers
      \arg        0x0000-0xFFFF
    \param[out] none
    \retval     the number of data register
*/
uint32_t check_backup_register(uint16_t data)
{
    bkp_data_register_enum temp = BKP_DATA_0;
    /* check the data of backup registers */
    for(temp = BKP_DATA_REG_MIN; temp <= BKP_DATA_REG_MAX; temp++){
        if(bkp_data_read(temp) != (data + (temp * 0x50))){
            return temp;
        }
    }
    return 0;
}

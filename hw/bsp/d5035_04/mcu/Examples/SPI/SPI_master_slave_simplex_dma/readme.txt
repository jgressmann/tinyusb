/*!
    \file    readme.txt
    \brief   description of the master and slave simplex communication use dma demo

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

  This example is based on the GD32C103V-EVAL-V1.0 board, it shows SPI0 and SPI2 simplex communication use DMA mode.

  1.Master transmit slave receive mode:
    Connect SPI0 SCK  PIN(PA5) TO SPI2 SCK  PIN(PC10).
    Connect SPI0 MOSI PIN(PA7) TO SPI2 MOSI PIN(PC12).
    Connect SPI0 NSS PIN(PA2) TO SPI2 NSS PIN(PA4).

  Modify file: example/SPI/SPI_master_slave_simplex_dma/main.c, choose master transmit slave receive mode.
  #define MASTER_TRANSMIT_SLVAE_RECEIVE
  //#define SLAVE_TRANSMIT_MASTER_RECEIVE

  After the communicate is complete,if receive data is equal to send data, LED2 turn on, if not LED2 turn off.

  2.Slave transmit master receive mode:
    Connect SPI0 SCK  PIN(PA5) TO SPI2 SCK  PIN(PC10).
    Connect SPI0 MISO PIN(PA6) TO SPI2 MISO PIN(PC11).
    Connect SPI0 NSS PIN(PA2) TO SPI2 NSS PIN(PA4).

  Modify file: example/SPI/SPI_master_slave_simplex_dma/main.c, choose slave transmit master receive mode.
  //#define MASTER_TRANSMIT_SLVAE_RECEIVE
  #define SLAVE_TRANSMIT_MASTER_RECEIVE

  After the communicate is complete,if receive data is equal to send data, LED3 turn on, if not LED3 turn off.

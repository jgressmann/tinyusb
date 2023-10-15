/* SPDX-License-Identifier: MIT
 *
 * Copyright (c) 2022 Jean Gressmann <jean@0x42.de>
 *
 */

#include <bsp/board.h>
#include <supercan_board.h>
#include <gd32c10x.h>

#include <device/dcd.h>

#include "drv_usb_hw.h"
#include "drv_usbd_int.h"
#include "usbd_core.h"
#include "usbd_enum.h"

#include <tusb.h>
#include <device/usbd_pvt.h>


#if !defined(HWREV)
# error Define "HWREV"
#endif

#define RHPORT 0



//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM
//--------------------------------------------------------------------+
#define CONF_CPU_FREQUENCY 120000000
#define CONF_SYSTICK_FREQUENCY (CONF_CPU_FREQUENCY / 8)
#define CONF_APB2_FREQUENCY 120000000
#define USART_BAUDRATE 115200
#define BOARD_NAME "D5035-04"

uint32_t SystemCoreClock;

static uint8_t sc_core_init(usb_dev *udev, uint8_t config_index);
static uint8_t sc_core_deinit(usb_dev *udev, uint8_t config_index);
static uint8_t sc_core_req(usb_dev *udev, usb_req *req);
static uint8_t sc_core_in(usb_dev *udev, uint8_t ep_num);
static uint8_t sc_core_out(usb_dev *udev, uint8_t ep_num);


usb_core_driver driver;
// usb_desc desc;
usb_class_core cls = {
	.init     = sc_core_init,
    .deinit   = sc_core_deinit,
	.req_proc = sc_core_req,
    .data_in  = sc_core_in,
    .data_out = sc_core_out
};

/* The board has a 25Hz external crystal
 * Configure for 120 MHz core clock, 48 MHz USB clock.
 */
void board_init_xtal(void)
{
	/* CFG0:
	 * - AHB div by 1 => 120 MHz
	 * - APB1 div by 2 => 60 MHz (which is max allowed for bus)
	 * - APB2 div by 1 => 120 MHz (which is max allowed for bus)
	 * - USB prediv to 2,5 (120->48)
	 * - PLL multiplcation factor to 24 (5->120)
	 * - PLL from XTAL or IRC48M
	 */
	RCU_CFG0 =
		(RCU_CFG0 & ~(RCU_CFG0_AHBPSC | RCU_CFG0_APB1PSC | RCU_CFG0_APB2PSC | RCU_CFG0_USBFSPSC_2 | RCU_CFG0_USBFSPSC | RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4 | RCU_CFG0_PLLSEL | RCU_CFG0_PREDV0_LSB)) |
		(RCU_AHB_CKSYS_DIV1 | RCU_APB1_CKAHB_DIV2 | RCU_APB2_CKAHB_DIV1 | RCU_CKUSB_CKPLL_DIV2_5 | RCU_PLL_MUL24 | RCU_PLLSRC_HXTAL_IRC48M);


	/* CFG1:
	 * - PREDV0 to 5 to get 5 MHz out of xtal
	 * - select XTAL / IRC48M as PLL input
	 */
	RCU_CFG1 =
		(RCU_CFG1 & ~(RCU_CFG1_PREDV0 | RCU_CFG1_PREDV0SEL | RCU_CFG1_PLLPRESEL)) |
		(RCU_PREDV0_DIV5 | RCU_PREDV0SRC_HXTAL_IRC48M | RCU_PLLPRESRC_HXTAL);


	/* enable crystal, PLL */
	RCU_CTL |= RCU_CTL_HXTALEN | RCU_CTL_PLLEN;

	/* wait for crystal, PLL */
	while ((RCU_CTL_HXTALSTB | RCU_CTL_PLLSTB) != (RCU_CTL & (RCU_CTL_HXTALSTB | RCU_CTL_PLLSTB)));

	/* set wait state for flash to 3 (for 120 MHz), section 2.3.2. Read operations , GD32C10x User Manual */
	FMC_WS = (FMC_WS & ~(FMC_WS_WSCNT)) | (WS_WSCNT(3));

	/* switch to PLL */
	RCU_CFG0 =
		(RCU_CFG0 & ~(RCU_CFG0_SCS)) |
		(RCU_CKSYSSRC_PLL);


	/* wait for clock switch */
	while (RCU_SCSS_PLL != (RCU_CFG0 & RCU_CFG0_SCSS));

	SystemCoreClock = CONF_CPU_FREQUENCY;
}

void board_init_uart(void)
{
	// /* reset */
	// RCU_APB2RST |= RCU_USART0RST;
	// RCU_APB2RST &= ~RCU_USART0RST;

	/* configure clocks  */
	RCU_APB2EN |=
		RCU_APB2EN_PAEN |
		RCU_APB2EN_AFEN |
		RCU_APB2EN_USART0EN;

	/* configure pin PA9 (USART0_TX) */
	GPIO_CTL1(GPIOA) = (GPIO_CTL1(GPIOA) & ~GPIO_MODE_MASK(1)) | GPIO_MODE_SET(1, GPIO_MODE_AF_PP | GPIO_OSPEED_10MHZ);

	/* baud rate */
	USART_BAUD(USART0) = ((UINT32_C(CONF_APB2_FREQUENCY) + UINT32_C(USART_BAUDRATE) / 2)) / (UINT32_C(USART_BAUDRATE));

	USART_CTL0(USART0) =
		USART_TRANSMIT_ENABLE |
		USART_CTL0_UEN;

	while (!(USART_CTL0(USART0) & USART_CTL0_UEN));
}

void uart_send_buffer(uint8_t const *text, size_t len)
{
	for (size_t i = 0; i < len; ++i) {
		USART_DATA(USART0) = text[i];
		while(!(USART_STAT0(USART0) & USART_STAT0_TBE));
	}
}

void uart_send_str(const char* text)
{
	while (*text) {
		USART_DATA(USART0) = *text++;
		while(!(USART_STAT0(USART0) & USART_STAT0_TBE));
	}
}

void board_init(void)
{
	/* enable
	 * - PB pin group for debug LED
	 */
	RCU_APB2EN |=
		RCU_APB2EN_PBEN;

	/* debug LED on PB14, set push-pull output */
	GPIO_CTL1(GPIOB) = (GPIO_CTL1(GPIOB) & ~GPIO_MODE_MASK(6)) | GPIO_MODE_SET(6, GPIO_MODE_OUT_PP | GPIO_OSPEED_2MHZ);

	board_init_xtal();

	board_init_uart();

#if CFG_TUSB_DEBUG >= 2
	uart_send_str(BOARD_NAME " UART initialized\n");
#endif

#if CFG_TUSB_OS == OPT_OS_NONE
	SysTick_Config(CONF_SYSTICK_FREQUENCY / 1000);
#endif

#if CFG_TUSB_OS == OPT_OS_FREERTOS
	// If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
#define NVIC_PRIORITYGROUP_4 3
	NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	NVIC_SetPriority(USBFS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif

	usb_rcu_config();

	usb_timer_init();
}



//--------------------------------------------------------------------+
// Board porting API
//--------------------------------------------------------------------+

void board_led_write(bool state)
{
	GPIO_BOP(GPIOB) = BIT(14 + (!state) * 16);
}

uint32_t board_button_read(void)
{
	// this board has no button
	return 0;
}

int board_uart_read(uint8_t* buf, int len)
{
	(void) buf; (void) len;
	return 0;
}

int board_uart_write(void const * buf, int len)
{
	if (len < 0) {
		uart_send_str(buf);
	} else {
		uart_send_buffer(buf, len);
	}
	return len;
}

#if CFG_TUSB_OS  == OPT_OS_NONE
volatile uint32_t system_ticks = 0;

void SysTick_Handler(void)
{
	system_ticks++;
}

uint32_t board_millis(void)
{
	return system_ticks;
}
#endif

// Required by __libc_init_array in startup code if we are compiling using
// -nostdlib/-nostartfiles.
void _init(void)
{

}

void dcd_init(uint8_t rhport)
{
	LOG("dcd_init enter\n");
	(void)rhport;
	usb_core_driver* udev = &driver;

	//udev->dev.desc = &desc;

    /* class callbacks */
    udev->dev.class_core = &cls;

    /* create serial string */
    // serial_string_get(udev->dev.desc->strings[STR_IDX_SERIAL]);

    /* configure USB capabilities */
    (void)usb_basic_init (&udev->bp, &udev->regs, USB_CORE_ENUM_FS);

    usb_globalint_disable(&udev->regs);

	NVIC_SetPriority(USBFS_IRQn, SC_ISR_PRIORITY);

	LOG("prio tim2 %u\n", NVIC_GetPriority(TIMER2_IRQn));

	/* initializes the USB core*/
    (void)usb_core_init (udev->bp, &udev->regs);

	/* set device disconnect */
    usbd_disconnect (udev);

#ifndef USE_OTG_MODE
    usb_curmode_set(&udev->regs, DEVICE_MODE);
#endif /* USE_OTG_MODE */

    /* initializes device mode */
    (void)usb_devcore_init (udev);

	usb_globalint_enable(&udev->regs);

	/* set device connect */
    usbd_connect (udev);

    udev->dev.cur_status = (uint8_t)USBD_DEFAULT;

	LOG("prio usb %u\n", NVIC_GetPriority(USBFS_IRQn));

	LOG("dcd_init exit\n");
}

void dcd_int_enable(uint8_t rhport)
{
	LOG("dcd_int_enable\n");
	(void)rhport;
	NVIC_EnableIRQ(USBFS_IRQn);
}

void dcd_int_disable(uint8_t rhport)
{
	LOG("dcd_int_disable\n");
	(void)rhport;
	NVIC_DisableIRQ(USBFS_IRQn);
}


void dcd_set_address(uint8_t rhport, uint8_t dev_addr)
{
	LOG("dcd_set_address\n");
	(void)rhport;
	(void)dev_addr;
}


void dcd_connect(uint8_t rhport)
{
	LOG("dcd_connect\n");

	usb_core_driver* udev = &driver;
	(void)rhport;
	/* set device connect */
    usbd_connect (udev);
}

void dcd_disconnect(uint8_t rhport)
{
	LOG("dcd_disconnect\n");

	usb_core_driver* udev = &driver;
	(void)rhport;
	usbd_disconnect(udev);
}


// Configure endpoint's registers according to descriptor
bool dcd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const * desc_ep)
{
	(void)rhport;

	usbd_ep_setup(&driver, (const usb_desc_ep *)desc_ep);

	return true;
}

// Close all non-control endpoints, cancel all pending transfers if any.
// Invoked when switching from a non-zero Configuration by SET_CONFIGURE therefore
// required for multiple configuration support.
void dcd_edpt_close_all(uint8_t rhport)
{
	LOG("dcd_edpt_close_all\n");
	(void)rhport;
}

// Close an endpoint.
// Since it is weak, caller must TU_ASSERT this function's existence before calling it.
void dcd_edpt_close(uint8_t rhport, uint8_t ep_addr)
{
	(void)rhport;
	(void)ep_addr;

	LOG("dcd_edpt_close\n");
}


// Submit a transfer, When complete dcd_event_xfer_complete() is invoked to notify the stack
bool dcd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
	usb_core_driver* udev = &driver;
	(void)rhport;

	uint8_t pipe = ep_addr & ~0x80;

	if (pipe) {
		// LOG("dcd_edpt_xfer %02x\n", ep_addr);
		if (ep_addr & 0x80) {
			// always returns 0
			// LOG("usbd_ep_send %02x\n", ep_addr);
			(void)usbd_ep_send(udev, ep_addr, buffer, total_bytes);
		} else {
			// always returns 0
			// LOG("usbd_ep_recev %02x\n", ep_addr);
			(void)usbd_ep_recev(udev, ep_addr, buffer, total_bytes);
		}
	} else {
		// EP0 is handled by GD USB stack
	}

	return true;
}

// Stall endpoint, any queuing transfer should be removed from endpoint
void dcd_edpt_stall(uint8_t rhport, uint8_t ep_addr)
{
	(void)rhport;
	(void)ep_addr;

	// LOG("dcd_edpt_stall %02x\n", ep_addr);
}

// clear stall, data toggle is also reset to DATA0
// This API never calls with control endpoints, since it is auto cleared when receiving setup packet
void dcd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr)
{
	(void)rhport;
	(void)ep_addr;

	// LOG("usbd_ep_stall_clear %02x\n", ep_addr);

}

extern void usb_timer_irq(void);

void TIMER2_IRQHandler(void)
{
    usb_timer_irq();
}

//--------------------------------------------------------------------+
// Forward USB interrupt events to TinyUSB IRQ Handler
//--------------------------------------------------------------------+

__attribute__((section(".ramfunc"))) void USBFS_IRQHandler(void)
{
	usbd_isr (&driver);
}

static uint8_t sc_core_init(usb_dev *udev, uint8_t config_index)
{
	(void)udev;
	(void)config_index;

	return USBD_OK;
}
static uint8_t sc_core_deinit(usb_dev *udev, uint8_t config_index)
{
	(void)udev;
	(void)config_index;

	return USBD_OK;
}
static uint8_t sc_core_req(usb_dev *udev, usb_req *req)
{
	(void)udev;

	tud_dfu_runtime_reboot_to_dfu_cb(0);

	// uint8_t ep_num = 0;
	// dcd_event_setup_received(RHPORT, (uint8_t*)req, true);
	// dcd_event_xfer_complete(RHPORT, 0, udev->dev.transc_out[ep_num].xfer_count, XFER_RESULT_SUCCESS, true);

	return USBD_OK;
}
static uint8_t sc_core_in(usb_dev *udev, uint8_t ep_num)
{

	// LOG("sc_core_in %02x\n", 0x80 | ep_num);
	// LOG("> %02x l=%u\n", 0x80 | ep_num, udev->dev.transc_in[ep_num].xfer_count);
	dcd_event_xfer_complete(0, 0x80 | ep_num, udev->dev.transc_in[ep_num].xfer_count, XFER_RESULT_SUCCESS, true);
	return USBD_OK;
}
static uint8_t sc_core_out(usb_dev *udev, uint8_t ep_num)
{
	// LOG("sc_core_out %02x\n", ep_num);
	// LOG("< %02x l=%u\n", ep_num, udev->dev.transc_out[ep_num].xfer_count);
	dcd_event_xfer_complete(0, ep_num, udev->dev.transc_out[ep_num].xfer_count, XFER_RESULT_SUCCESS, true);
	return USBD_OK;
}


#define TIM_MSEC_DELAY                          0x01U
#define TIM_USEC_DELAY                          0x02U

__IO uint32_t delay_time = 0U;
__IO uint16_t timer_prescaler = 5U;
uint32_t usbfs_prescaler = 0U;

static void hw_time_set (uint8_t unit);
static void hw_delay    (uint32_t ntime, uint8_t unit);

/*!
    \brief      configure USB clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_rcu_config(void)
{
    uint32_t system_clock = rcu_clock_freq_get(CK_SYS);

    if (48000000U == system_clock) {
        timer_prescaler = 3U;
        usbfs_prescaler = RCU_CKUSB_CKPLL_DIV1;
    } else if (72000000U == system_clock) {
        timer_prescaler = 5U;
        usbfs_prescaler = RCU_CKUSB_CKPLL_DIV1_5;
    } else if (96000000U == system_clock) {
        timer_prescaler = 7U;
        usbfs_prescaler = RCU_CKUSB_CKPLL_DIV2;
    } else if (120000000U == system_clock) {
        timer_prescaler = 9U;
        usbfs_prescaler = RCU_CKUSB_CKPLL_DIV2_5;
    }  else {
        /*  reserved  */
    }

    rcu_usb_clock_config(usbfs_prescaler);

    rcu_periph_clock_enable(RCU_USBFS);
}


/*!
    \brief      initializes delay unit using Timer2
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_timer_init (void)
{
    // DO NOT SET interrupt prio, as the RTOS hasn't
	// set up proper handling yet
	// NVIC_SetPriority(TIMER2_IRQn, SC_ISR_PRIORITY);
	NVIC_EnableIRQ(TIMER2_IRQn);

    rcu_periph_clock_enable(RCU_TIMER2);
}

/*!
    \brief      delay in micro seconds
    \param[in]  usec: value of delay required in micro seconds
    \param[out] none
    \retval     none
*/
void usb_udelay (const uint32_t usec)
{
    hw_delay(usec, TIM_USEC_DELAY);
}

/*!
    \brief      delay in milliseconds
    \param[in]  msec: value of delay required in milliseconds
    \param[out] none
    \retval     none
*/
void usb_mdelay (const uint32_t msec)
{
    hw_delay(msec, TIM_MSEC_DELAY);
}

/*!
    \brief      timer base IRQ
    \param[in]  none
    \param[out] none
    \retval     none
*/
void usb_timer_irq (void)
{
	// LOG("usb_timer_irq\n");
    if (RESET != timer_interrupt_flag_get(TIMER2, TIMER_INT_UP)){
        timer_interrupt_flag_clear(TIMER2, TIMER_INT_UP);

        if (delay_time > 0x00U){
            delay_time--;
        } else {
            timer_disable(TIMER2);
        }
    }
}

/*!
    \brief      delay routine based on TIMER2
    \param[in]  nTime: delay Time
    \param[in]  unit: delay Time unit = milliseconds / microseconds
    \param[out] none
    \retval     none
*/
static void hw_delay(uint32_t ntime, uint8_t unit)
{
    delay_time = ntime;

    hw_time_set(unit);

    while (0U != delay_time) {
    }

    timer_disable(TIMER2);
}

/*!
    \brief      configures TIMER2 for delay routine based on TIMER2
    \param[in]  unit: msec /usec
    \param[out] none
    \retval     none
*/
static void hw_time_set(uint8_t unit)
{
    timer_parameter_struct  timer_basestructure;

    timer_disable(TIMER2);
    timer_interrupt_disable(TIMER2, TIMER_INT_UP);

    if (TIM_USEC_DELAY == unit) {
        timer_basestructure.period = 11U;
    } else if(TIM_MSEC_DELAY == unit) {
        timer_basestructure.period = 11999U;
    } else {
        /* no operation */
    }

    timer_basestructure.prescaler         = timer_prescaler;
    timer_basestructure.alignedmode       = TIMER_COUNTER_EDGE;
    timer_basestructure.counterdirection  = TIMER_COUNTER_UP;
    timer_basestructure.clockdivision     = TIMER_CKDIV_DIV1;
    timer_basestructure.repetitioncounter = 0U;

    timer_init(TIMER2, &timer_basestructure);

    timer_interrupt_flag_clear(TIMER2, TIMER_INT_UP);

    timer_auto_reload_shadow_enable(TIMER2);

    /* TIMER2 interrupt enable */
    timer_interrupt_enable(TIMER2, TIMER_INT_UP);

    /* TIMER2 enable counter */
    timer_enable(TIMER2);
}

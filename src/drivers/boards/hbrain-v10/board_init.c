/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_init.c
 *
 * VRBRAIN-specific early startup code.  This file implements the
 * nsh_archinitialize() function that is called early by nsh during startup.
 *
 * Code here is run before the rcS script is invoked; it should start required
 * subsystems and perform board-specific initialisation.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/spi.h>
#include <nuttx/i2c.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>

#include "stm32.h"
#include "board_config.h"
#include "stm32_uart.h"

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_led.h>
#include <drivers/drv_buzzer.h>

#include <systemlib/cpuload.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) lowsyslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message lowsyslog
#  else
#    define message printf
#  endif
#endif

__BEGIN_DECLS
extern void led_init();
extern void led_on(int led);
extern void led_off(int led);
extern void buzzer_init();
extern void buzzer_on(int buzzer);
extern void buzzer_off(int buzzer);
__END_DECLS

/****************************************************************************
 * Protected Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/************************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

__EXPORT void stm32_boardinitialize(void)
{
	/* enable sys logs */
	//syslog_enable(true);

	/* configure SPI interfaces */
	stm32_spiinitialize();

}

/****************************************************************************
 * Name: board_initialize
 *
 * Description:
 *   If CONFIG_BOARD_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_initialize().  board_initialize() will be
 *   called immediately after up_intiialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_INITIALIZE
__EXPORT void board_initialize(void)
{
  /* Perform NSH initialization here instead of from the NSH.  This
   * alternative NSH initialization is necessary when NSH is ran in user-space
   * but the initialization function must run in kernel space.
   */






}
#endif

static struct spi_dev_s *spi1;
static struct spi_dev_s *spi2;
static struct spi_dev_s *spi3;

#include <math.h>

#ifdef __cplusplus
__EXPORT int matherr(struct __exception *e)
{
	return 1;
}
#else
__EXPORT int matherr(struct exception *e)
{
	return 1;
}
#endif

__EXPORT int composite_archinitialize(void)
{
  return OK;
}

__EXPORT int cdcacm_archinitialize(void)
{
  return OK;
}

__EXPORT int usbmsc_archinitialize(void)
{
  return OK;
}

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

__EXPORT int nsh_archinitialize(void)
{
	int result;

	message("\n");

	/* configure always-on ADC pins */
	stm32_configgpio(GPIO_ADC1_IN10);
	stm32_configgpio(GPIO_ADC1_IN11);
	stm32_configgpio(GPIO_ADC1_IN12);
	stm32_configgpio(GPIO_ADC1_IN13);

	stm32_configgpio(GPIO_UART_SBUS_INVERTER);
#ifdef CONFIG_RC_INPUTS_TYPE(RC_INPUT_SBUS)
	stm32_gpiowrite(GPIO_UART_SBUS_INVERTER, 1);
#else
	stm32_gpiowrite(GPIO_UART_SBUS_INVERTER, 0);
#endif

	/* configure the high-resolution time/callout interface */
	hrt_init();

	/* configure CPU load estimation */
#ifdef CONFIG_SCHED_INSTRUMENTATION
	cpuload_initialize_once();
#endif


















	/* initial BUZZER state */
	drv_buzzer_start();
	buzzer_off(BUZZER_EXT);

	/* initial LED state */
	drv_led_start();
	led_off(LED_AMBER);
	led_off(LED_BLUE);
	led_off(LED_GREEN);
	led_off(LED_EXT1);
	led_off(LED_EXT2);


	/* Configure SPI-based devices */

	message("[boot] Initializing SPI port 1\n");
	spi1 = up_spiinitialize(1);

	if (!spi1) {
		message("[boot] FAILED to initialize SPI port 1\r\n");
		led_on(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI1 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi1, 10000000);
	SPI_SETBITS(spi1, 8);
	SPI_SETMODE(spi1, SPIDEV_MODE3);
	SPI_SELECT(spi1, GPIO_SPI_CS_EXP_MPU6000, false);
	SPI_SELECT(spi1, GPIO_SPI_CS_EXP_FREE, false);
	SPI_SELECT(spi1, GPIO_SPI_CS_EXP_HMC5983, false);
	SPI_SELECT(spi1, GPIO_SPI_CS_MS5611, false);
	up_udelay(20);

	message("[boot] Successfully initialized SPI port 1\r\n");

//	message("[boot] Initializing Wireless Module\n");
//	wireless_archinitialize();

	message("[boot] Initializing SPI port 2\n");
	spi2 = up_spiinitialize(2);

	if (!spi2) {
		message("[boot] FAILED to initialize SPI port 2\r\n");
		led_on(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI2 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi2, 10000000);
	SPI_SETBITS(spi2, 8);
	SPI_SETMODE(spi2, SPIDEV_MODE3);
	SPI_SELECT(spi2, GPIO_SPI_CS_MPU6000, false);

	message("[boot] Successfully initialized SPI port 2\n");

	/* Get the SPI port for the microSD slot */

	message("[boot] Initializing SPI port 3\n");
	spi3 = up_spiinitialize(3);

	if (!spi3) {
		message("[boot] FAILED to initialize SPI port 3\n");
		led_on(LED_AMBER);
		return -ENODEV;
	}

	/* Default SPI3 to 1MHz and de-assert the known chip selects. */
	SPI_SETFREQUENCY(spi3, 10000000);
	SPI_SETBITS(spi3, 8);
	SPI_SETMODE(spi3, SPIDEV_MODE3);
	SPI_SELECT(spi3, GPIO_SPI_CS_DATAFLASH, false);
	SPI_SELECT(spi3, GPIO_SPI_CS_SDCARD, false);

	message("[boot] Successfully initialized SPI port 3\n");

	/* Now bind the SPI interface to the MMCSD driver */
	result = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR, CONFIG_NSH_MMCSDSLOTNO, spi3);

	if (result != OK) {
		message("[boot] FAILED to bind SPI port 3 to the MMCSD driver\n");
		led_on(LED_AMBER);
		return -ENODEV;
	}

	message("[boot] Successfully bound SPI port 3 to the MMCSD driver\n");

	return OK;
}

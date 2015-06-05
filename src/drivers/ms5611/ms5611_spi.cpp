/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
  * @file ms5611_spi.cpp
  *
  * SPI interface for MS5611
  */

/* XXX trim includes */
#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <arch/board/board.h>
#include <systemlib/err.h>
#include <drivers/device/spi.h>

#include "ms5611.h"
#include "board_config.h"



#define BMP280_CHIP_ID_REG                   (0xD0)  /*Chip ID Register */
#define BMP280_RST_REG                       (0xE0) /*Softreset Register */
#define BMP280_STAT_REG                      (0xF3)  /*Status Register */
#define BMP280_CTRL_MEAS_REG                 (0xF4)  /*Ctrl Measure Register */
#define BMP280_CONFIG_REG                    (0xF5)  /*Configuration Register */
#define BMP280_PRESSURE_MSB_REG              (0xF7)  /*Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG              (0xF8)  /*Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG             (0xF9)  /*Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG           (0xFA)  /*Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG           (0xFB)  /*Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG          (0xFC)  /*Temperature XLSB Reg */


/* SPI protocol address bits */
#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)
#define ADDR_INCREMENT			(1<<6)

#if defined(SPIDEV_MS5611) || defined(SPIDEV_EXP_MS5611) || defined(SPIDEV_IMU_MS5611)

device::Device *MS5611_spi_interface(ms5611::prom_u &prom_buf, enum BusSensor bustype);

class MS5611_SPI : public device::SPI
{
public:
	MS5611_SPI(int bus, spi_dev_e device, ms5611::prom_u &prom_buf);
	virtual ~MS5611_SPI();

	virtual int	init();
	virtual int	read(unsigned offset, void *data, unsigned count);
	virtual int	ioctl(unsigned operation, unsigned &arg);

private:
	ms5611::prom_u	&_prom;

	/**
	 * Send a reset command to the MS5611.
	 *
	 * This is required after any bus reset.
	 */
	int		_reset();

	/**
	 * Send a measure command to the MS5611.
	 *
	 * @param addr		Which address to use for the measure operation.
	 */
	int		_measure(unsigned addr);

	/**
	 * Read the MS5611 PROM
	 *
	 * @return		OK if the PROM reads successfully.
	 */
	int		_read_prom();

	/**
	 * Read a 16-bit register value.
	 *
	 * @param reg		The register to read.
	 */
	uint16_t	_reg16(unsigned reg);

	/**
	 * Wrapper around transfer() that prevents interrupt-context transfers
	 * from pre-empting us. The sensor may (does) share a bus with sensors
	 * that are polled from interrupt context (or we may be pre-empted)
	 * so we need to guarantee that transfers complete without interruption.
	 */
	int		_transfer(uint8_t *send, uint8_t *recv, unsigned len);
	int toto =0;
};

device::Device *
MS5611_spi_interface(ms5611::prom_u &prom_buf, enum BusSensor bustype)
{
	switch (bustype) {
	case TYPE_BUS_SENSOR_INTERNAL:
#ifdef SPI_BUS_MS5611
		return new MS5611_SPI(SPI_BUS_MS5611, (spi_dev_e)SPIDEV_MS5611, prom_buf);
#else
		errx(0, "Internal SPI not available");
#endif
		break;
	case TYPE_BUS_SENSOR_IMU:
#ifdef SPI_BUS_IMU_MS5611
		return new MS5611_SPI(SPI_BUS_IMU_MS5611, (spi_dev_e)SPIDEV_IMU_MS5611, prom_buf);
#else
		errx(0, "External IMU SPI not available");
#endif
		break;
	case TYPE_BUS_SENSOR_EXTERNAL:
#ifdef SPI_BUS_EXP_MS5611
		return new MS5611_SPI(SPI_BUS_EXP_MS5611, (spi_dev_e)SPIDEV_EXP_MS5611, prom_buf);
#else
		errx(0, "External EXP SPI not available");
#endif
		break;
	}
}

MS5611_SPI::MS5611_SPI(int bus, spi_dev_e device, ms5611::prom_u &prom_buf) :
	SPI("MS5611_SPI", nullptr, bus, device, SPIDEV_MODE3, 6*1000*1000),
	_prom(prom_buf)
{
	// enable debug() calls
	_debug_enabled = false;
}

MS5611_SPI::~MS5611_SPI()
{
}

int
MS5611_SPI::init()
{
	int ret;

	ret = SPI::init();
	if (ret != OK) {
		debug("SPI init failed");
		goto out;
	}

	/* send reset command */
	ret = _reset();
	if (ret != OK) {
		debug("reset failed");
		goto out;
	}

	/* read PROM */
	ret = _read_prom();
	if (ret != OK) {
		debug("prom readout failed");
		goto out;
	}

out:
	return ret;
}

int
MS5611_SPI::read(unsigned offset, void *data, unsigned count)
{
	union _cvt {
		uint8_t	b[4];
		uint32_t w;
	} *cvt = (_cvt *)data;

	int ret;
	uint8_t	REG1[4] = {BMP280_TEMPERATURE_MSB_REG | DIR_READ, 0x00, 0x00, 0x00};
	uint8_t	REG2[4] = {BMP280_PRESSURE_MSB_REG | DIR_READ, 0x00, 0x00, 0x00};

	if(offset == 0)
	{

		ret = _transfer(&REG1[0], &REG1[0], sizeof(REG1));
		if (ret == OK) {
						/* fetch the raw value */
						cvt->b[0] = REG1[3];
						cvt->b[1] = REG1[2];
						cvt->b[2] = REG1[1];
						cvt->b[3] = 0;

						ret = count;
					}
	}
	else
	{

		ret = _transfer(&REG2[0], &REG2[0], sizeof(REG2));
		if (ret == OK) {
						/* fetch the raw value */
						cvt->b[0] = REG2[3];
						cvt->b[1] = REG2[2];
						cvt->b[2] = REG2[1];
						cvt->b[3] = 0;

						ret = count;
					}
	}


	//eteind l'alimentation
	uint8_t cmd[2] = {BMP280_CTRL_MEAS_REG | DIR_WRITE, 0xFC};
	ret = _transfer(&cmd[0], nullptr, 2);

	return ret;
}

int
MS5611_SPI::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {
	case IOCTL_RESET:
		ret = _reset();
		break;

	case IOCTL_MEASURE:
		ret = _measure(arg);
		break;

	default:
		ret = EINVAL;
	}

	if (ret != OK) {
		errno = ret;
		return -1;
	}
	return 0;
}

int
MS5611_SPI::_reset()
{
	int ret ;

/*	uint8_t cmd1 = ADDR_RESET_CMD | DIR_WRITE;
	ret = _transfer(&cmd1, nullptr, 1);*/
	//reset
	uint8_t cmd[2] = {BMP280_RST_REG | DIR_WRITE, 0xB6};
	ret = _transfer(&cmd[0], nullptr, 2);
	//config
//	uint8_t cmd[2] = {BMP280_CTRL_MEAS_REG | DIR_WRITE, 0xFF};
	//ret = _transfer(&cmd[0], nullptr, 2);
	uint8_t cmd2[2] = {BMP280_CONFIG_REG | DIR_WRITE, 0x08};
	ret = _transfer(&cmd2[0], nullptr, 2);

	return  OK;
}

int
MS5611_SPI::_measure(unsigned addr)
{

int ret;

// alimante le composant
uint8_t cmd[2] = {BMP280_CTRL_MEAS_REG | DIR_WRITE, 0xFE};
	ret = _transfer(&cmd[0], nullptr, 2);
	uint8_t cmd2[2] = {BMP280_CONFIG_REG | DIR_WRITE, 0x08};
	ret = _transfer(&cmd2[0], nullptr, 2);

	return ret;

}


int
MS5611_SPI::_read_prom()
{
	/*
	 * Wait for PROM contents to be in the device (2.8 ms) in the case we are
	 * called immediately after reset.
	 */
	usleep(3000);

	/* read and convert PROM words */
        bool all_zero = true;
	for (int i = 0; i < 8; i++) {
		uint8_t cmd = (ADDR_PROM_SETUP + (i * 2));
		_prom.c[i] = _reg16(cmd);
                if (_prom.c[i] != 0)
			all_zero = false;
                //debug("prom[%u]=0x%x", (unsigned)i, (unsigned)_prom.c[i]);
	}

	/* calculate CRC and return success/failure accordingly */
	int ret = ms5611::crc4(&_prom.c[0]) ? OK : -EIO;
        if (ret != OK) {
		debug("crc failed");
        }
        if (all_zero) {
		debug("prom all zero");
		ret = -EIO;
        }
        return ret;
}

uint16_t
MS5611_SPI::_reg16(unsigned reg)
{
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

	_transfer(cmd, cmd, sizeof(cmd));

	return (uint16_t)(cmd[1] << 8) | cmd[2];
}

int
MS5611_SPI::_transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	return transfer(send, recv, len);
}

#endif /* SPIDEV_MS5611 or SPIDEV_EXP_MS5611 or SPIDEV_IMU_MS5611 */

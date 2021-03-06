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
  * @file bmp280_spi.cpp
  *
  * SPI interface for BMP280
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

#include "bmp280.h"
#include "board_config.h"



/* SPI protocol address bits */
#define DIR_READ			(1<<7)
#define DIR_WRITE			(0x7F)
#define ADDR_INCREMENT		(1<<6)

#if defined(SPIDEV_BMP280) || defined(SPIDEV_EXP_BMP280) || defined(SPIDEV_IMU_BMP280)

device::Device *BMP280_spi_interface(bmp280::prom_u &prom_buf, enum BusSensor bustype);

class BMP280_SPI : public device::SPI
{
public:
	BMP280_SPI(int bus, spi_dev_e device, bmp280::prom_u &prom_buf);
	virtual ~BMP280_SPI();

	virtual int	init();
	virtual int	read(unsigned offset, void *data, unsigned count);
	virtual int	ioctl(unsigned operation, unsigned &arg);

private:
	bmp280::prom_u	&_prom;

	/**
	 * Send a reset command to the BMP280.
	 *
	 * This is required after any bus reset.
	 */
	int		_reset();

	/**
	 * Send a measure command to the BMP280.
	 *
	 * @param addr		Which address to use for the measure operation.
	 */
	int		_checkConfig(unsigned addr);

	/**
	 * Read the BMP280 PROM
	 *
	 * @return		OK if the PROM reads successfully.
	 */
	int		_read_prom();

	/**
	 * Read the BMP280 PROM twice
	 *
	 * @return		OK if the PROM reads successfully.
	 */
	int		_second_read_prom();


	/**
	 * Read a 16-bit register value.
	 *
	 * @param reg		The register to read.
	 */
	uint16_t	_reg16(unsigned reg);
	int16_t		_reg16s(unsigned reg);
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
BMP280_spi_interface(bmp280::prom_u &prom_buf, enum BusSensor bustype)
{
	switch (bustype) {
	case TYPE_BUS_SENSOR_INTERNAL:
#ifdef SPI_BUS_BMP280
		return new BMP280_SPI(SPI_BUS_BMP280, (spi_dev_e)SPIDEV_BMP280, prom_buf);
#else
		errx(0, "Internal SPI not available");
#endif
		break;
	case TYPE_BUS_SENSOR_IMU:
#ifdef SPI_BUS_IMU_BMP280
		return new BMP280_SPI(SPI_BUS_IMU_BMP280, (spi_dev_e)SPIDEV_IMU_BMP280, prom_buf);
#else
		errx(0, "External IMU SPI not available");
#endif
		break;
	case TYPE_BUS_SENSOR_EXTERNAL:
#ifdef SPI_BUS_EXP_BMP280
		return new BMP280_SPI(SPI_BUS_EXP_BMP280, (spi_dev_e)SPIDEV_EXP_BMP280, prom_buf);
#else
		errx(0, "External EXP SPI not available");
#endif
		break;
	}
}

BMP280_SPI::BMP280_SPI(int bus, spi_dev_e device, bmp280::prom_u &prom_buf) :
	SPI("BMP280_SPI", nullptr, bus, device, SPIDEV_MODE3, 6*1000*1000),
	_prom(prom_buf)
{
	// enable debug() calls
	_debug_enabled = false;
}

BMP280_SPI::~BMP280_SPI()
{
}

int
BMP280_SPI::init()
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

	/* check PROM */
	ret = _second_read_prom();
	if (ret != OK) {
		debug("prom readout failed");
		goto out;
	}

out:
	return ret;
}

int
BMP280_SPI::read(unsigned offset, void *data, unsigned count)
{
	union _cvt {
		uint8_t	b[4];
		uint32_t w;
	} *cvt = (_cvt *)data;

	int ret;
	uint8_t	TEMPERATURE[4] = {BMP280_TEMPERATURE_MSB_REG | DIR_READ, 0x00, 0x00, 0x00};
	uint8_t	PRESSURE[4] = {BMP280_PRESSURE_MSB_REG | DIR_READ, 0x00, 0x00, 0x00};

	if(offset == READ_TEMPERATURE)
	{

		ret = _transfer(&TEMPERATURE[0], &TEMPERATURE[0], sizeof(TEMPERATURE));
		if (ret == OK) {
						/* fetch the raw value */
						cvt->b[0] = TEMPERATURE[3];
						cvt->b[1] = TEMPERATURE[2];
						cvt->b[2] = TEMPERATURE[1];
						cvt->b[3] = 0;
						cvt->w = (cvt->w) / 16;
						ret = count;
					}
	}
	else if (offset == READ_PRESSURE)
	{

		ret = _transfer(&PRESSURE[0], &PRESSURE[0], sizeof(PRESSURE));
		if (ret == OK) {
						/* fetch the raw value */
						cvt->b[0] = PRESSURE[3];
						cvt->b[1] = PRESSURE[2];
						cvt->b[2] = PRESSURE[1];
						cvt->b[3] = 0;
						cvt->w = (cvt->w) / 16;
						ret = count;
					}
	}
	return ret;
}

int
BMP280_SPI::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {
	case IOCTL_RESET:
		ret = _reset();
		break;

	case IOCTL_CHECKCONFIG:
		ret = _checkConfig(arg);
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
BMP280_SPI::_reset()
{
	/* warning with config it2.24 max rate = 26 hz so we call a 100hz for nothing */
	// try dynamic device to have 83hz

	int ret ;
	/* CTRL_MEAS_REG(0xF4): 		osrs_t 		| 		 osrs_p 	   | mode 			*/
	/*				   bit:			7 6 5  		| 		 4 3 2         | 1 0  			*/
	uint8_t ctrl_meas = 0 | OVERSAMPLING_2 << 5 | OVERSAMPLING_16 << 2 | NORMAL_MODE;//config it2.24
	//try dynamic device uint8_t ctrl_meas = 0 | OVERSAMPLING_1 << 5 | OVERSAMPLING_4 << 2 | NORMAL_MODE;

	uint8_t CTRL_MEAS[2] = {BMP280_CTRL_MEAS_REG & DIR_WRITE, ctrl_meas};
	ret = _transfer(&CTRL_MEAS[0], nullptr, 2);

	/* CTRL_MEAS_REG(0xF5):  		t_sb  		| 		filter   | X | SPI_3W_en */
	/*				   bit:			7 6 5  		| 		4 3 2    | 1 |   0   	 */
	uint8_t config_reg = 0 | T_STANDBY_05 << 5  |  FILTER_4 << 2 | SPI_4_WIRE_EN;//config it2.24
	//try dynamic device uint8_t config_reg = 0 | T_STANDBY_05 << 5 |  FILTER_16 << 2 | SPI_4_WIRE_EN;

	uint8_t CONFIG_REG[2] = {BMP280_CONFIG_REG & DIR_WRITE, config_reg};
	ret = _transfer(&CONFIG_REG[0], nullptr, 2);
	return  ret;
}

int
BMP280_SPI::_checkConfig(unsigned addr)
{
	int ret;
	uint8_t ctrl_meas_value = 0;
	uint8_t	CTRL_MEAS[2] = {BMP280_CTRL_MEAS_REG | DIR_READ, 0x00};
	ret = _transfer(&CTRL_MEAS[0], &CTRL_MEAS[0], sizeof(CTRL_MEAS));
	if (ret == OK)
	{
		ctrl_meas_value = CTRL_MEAS[1];
	}

	uint8_t config_reg_value = 0;
	uint8_t	CONFIG_REG[2] = {BMP280_CONFIG_REG | DIR_READ, 0x00};
	ret = _transfer(&CONFIG_REG[0], &CONFIG_REG[0], sizeof(CONFIG_REG));
	if (ret == OK)
	{
		config_reg_value = CONFIG_REG[1];
	}
	// for plug and play
	if (((config_reg_value == 0x00) || (ctrl_meas_value == 0x00)) ||
	    ((config_reg_value == 0xFF) && (ctrl_meas_value == 0xFF)))
	{
		ret = -1;
	}


	return ret;

}

int
BMP280_SPI::_read_prom()
{
	/*
	 * Wait for PROM contents to be in the device (2.8 ms) in the case we are
	 * called immediately after reset.
	 */
	usleep(3000);
	int ret;
	/* read and convert PROM words */
    bool one_zero = false;
	for (int i = 0; i < 12; i++) {
		uint8_t cmd = (BMP280_ADDR_PROM_SETUP + (i * 2));

		if (i == 0 ) _prom.c[i] = _reg16(cmd);
		else if (i == 3 ) _prom.c[i] = _reg16(cmd);
		else
		{
			_prom.c[i] = _reg16s(cmd);
		}
		if (_prom.c[i] == 0xFFFF)
		one_zero = true;
	}
    if (one_zero)
    {
		debug("prom one zero");
		ret = -EIO;
    }
    else
    {
    	ret = OK;
    }
    return ret;
}

int
BMP280_SPI::_second_read_prom()
{
	/*
	 * Wait for PROM contents to be in the device (2.8 ms) in the case we are
	 * called immediately after reset.
	 */
	usleep(3000);
//	uint8_t buffer[24];
	uint16_t reponse;
	int ret;
	/* read and convert PROM words */
    bool one_disparity = false;
	for (int i = 0; i < 12; i++) {
		uint8_t cmd = (BMP280_ADDR_PROM_SETUP + (i * 2));

		if (i == 0)
		{
			reponse = _reg16(cmd);
			if (_prom.c[0] != reponse) one_disparity = true;
		}
		else if (i == 3)
		{
			reponse = _reg16(cmd);
			if (_prom.c[3] != reponse) one_disparity = true;
		}
		else
		{
			reponse = _reg16s(cmd);
			if (_prom.c[i] != reponse) one_disparity = true;
		}

	}
    if (one_disparity)
    {
		debug("prom one disparity");
		ret = -EIO;
    }
    else
    {
    	ret = OK;
    }
    return ret;
}

uint16_t
BMP280_SPI::_reg16(unsigned reg)
{
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

	_transfer(cmd, cmd, sizeof(cmd));

	return (uint16_t)(cmd[2] << 8) | cmd[1];
}
int16_t
BMP280_SPI::_reg16s(unsigned reg)
{
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

	_transfer(cmd, cmd, sizeof(cmd));

	return (int16_t)(cmd[2] << 8) | cmd[1];
}


int
BMP280_SPI::_transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	return transfer(send, recv, len);
}

#endif /* SPIDEV_BMP280 or SPIDEV_EXP_BMP280 or SPIDEV_IMU_BMP280 */

/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
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
 * @file bmp280.h
 *
 * Shared defines for the bmp280 driver.
 */

#include <board_config.h>

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
#define BMP280_ADDR_PROM_SETUP				 (0x88)  /*adresse prom param*/

#define OVERSAMPLING_1  1
#define OVERSAMPLING_2  2
#define OVERSAMPLING_4  3
#define OVERSAMPLING_8  4
#define OVERSAMPLING_16 5

#define FILTER_1  1
#define FILTER_2  2
#define FILTER_4  3
#define FILTER_8  4
#define FILTER_16 5

#define SPI_4_WIRE_EN 0
#define SPI_3_WIRE_EN 1

#define SLEEP_MODE  0
#define FORCED_MODE 1
#define NORMAL_MODE 3

#define T_STANDBY_05  0
#define T_STANDBY_625 1
#define T_STANDBY_125  2
#define T_STANDBY_250  3
#define T_STANDBY_500  4
#define T_STANDBY_1000 5
#define T_STANDBY_2000 6
#define T_STANDBY_4000 7

#define READ_TEMPERATURE 1
#define READ_PRESSURE 2

/* interface ioctls */
#define IOCTL_RESET				2
#define IOCTL_MEASURE			3

namespace bmp280
{

/**
 * Calibration PROM as reported by the device.
 */
#pragma pack(push,1)
struct prom_s {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
};

/**
 * Grody hack for crc4()
 */
union prom_u {
	uint16_t c[12];
	prom_s s;
};
#pragma pack(pop)


} /* namespace */

/* interface factories */
extern device::Device *BMP280_spi_interface(bmp280::prom_u &prom_buf, enum BusSensor bustype) weak_function;


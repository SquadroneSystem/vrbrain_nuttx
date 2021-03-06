/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file bmp280.cpp
 * Driver for the BMP280 barometric pressure sensor connected via I2C or SPI.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include <board_config.h>

#include <drivers/device/device.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include "bmp280.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/*
 * BMP280 internal constants and data structures.
 */

/* internal conversion time: 9.17 ms, so should not be read at rates higher than 100 Hz */
#define BMP280_CONVERSION_INTERVAL	10000	/* microseconds */
#define BMP280_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */

#define BMP280_BARO_DEVICE_PATH_INT		"/dev/bmp280_int"
#define BMP280_BARO_DEVICE_PATH_EXP		"/dev/bmp280_exp"
#define BMP280_BARO_DEVICE_PATH_IMU		"/dev/bmp280_imu"

class BMP280 : public device::CDev
{
public:
	BMP280(device::Device *interface, const char *path, bmp280::prom_u &prom_buf);
	~BMP280();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	Device			*_interface;

	bmp280::prom_s		_prom;

	struct work_s		_work;
	unsigned		_measure_ticks;

	RingBuffer		*_reports;

	bool			_collect_phase;
	unsigned		_measure_phase;

	/* intermediate temperature values per BMP280 datasheet */
	int32_t			_TEMP;
	int64_t			_OFF;
	int64_t			_SENS;
	float			_P;
	float			_T;
	float			_Alt;
	uint32_t                _D1;
	uint32_t                _D2;

	/* altitude conversion calibration */
	unsigned		_msl_pressure;	/* in kPa */

	orb_advert_t		_baro_topic;

	int			_class_instance;

	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;

	/**
	 * Initialize the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start_cycle();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop_cycle();

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	/**
	 * Issue a measurement command for the current state.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	virtual int		checkconfig();

	double bmp280_compensate_T_double(int32_t adc_T);
	double bmp280_compensate_P_double(int32_t adc_P);

	/**
	 * Collect the result of the most recent measurement.
	 */
	virtual int		collect();
};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int bmp280_main(int argc, char *argv[]);

BMP280::BMP280(device::Device *interface, const char *path, bmp280::prom_u &prom_buf) :
	CDev("BMP280", path),
	_interface(interface),
	_prom(prom_buf.s),
	_measure_ticks(0),
	_reports(nullptr),
	_collect_phase(false),
	_measure_phase(0),
	_TEMP(0),
	_OFF(0),
	_SENS(0),
	_msl_pressure(101325),
	_baro_topic(-1),
	_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "bmp280_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "bmp280_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "bmp280_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "bmp280_buffer_overflows"))
{
	// enable debug() calls
	_debug_enabled = true;

	// work_cancel in stop_cycle called from the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

BMP280::~BMP280()
{
	/* make sure we are truly inactive */
	stop_cycle();

	if (_class_instance != -1)
		unregister_class_devname(BARO_DEVICE_PATH, _class_instance);

	/* free any existing reports */
	if (_reports != nullptr)
		delete _reports;

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);

	delete _interface;
}

int
BMP280::init()
{
	int ret;

	ret = CDev::init();
	if (ret != OK) {
		debug("CDev init failed");
		goto out;
	}

	/* allocate basic report buffers */
	_reports = new RingBuffer(2, sizeof(baro_report));

	if (_reports == nullptr) {
		debug("can't get memory for reports");
		ret = -ENOMEM;
		goto out;
	}

	/* register alternate interfaces if we have to */
	_class_instance = register_class_devname(BARO_DEVICE_PATH);

	struct baro_report brp;
	/* do a first measurement cycle to populate reports with valid data */
	_measure_phase = 0;
	_reports->flush();

	/* this do..while is goto without goto */
	do {
		/* do temperature first */
		if (OK != checkconfig()) {
			ret = -EIO;
			break;
		}

		usleep(BMP280_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		_reports->get(&brp);

		ret = OK;

		if (_class_instance == CLASS_DEVICE_PRIMARY) {

			_baro_topic = orb_advertise(ORB_ID(sensor_baro), &brp);

			if (_baro_topic < 0)
				debug("failed to create sensor_baro publication");
		}

	} while (0);

out:
	return ret;
}

ssize_t
BMP280::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct baro_report);
	struct baro_report *brp = reinterpret_cast<struct baro_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1)
		return -ENOSPC;

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(brp)) {
				ret += sizeof(*brp);
				brp++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_measure_phase = 0;
		_reports->flush();

		/* do temperature first */
		if (OK != checkconfig()) {
			ret = -EIO;
			break;
		}

		usleep(BMP280_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have a report, copy it out */
		if (_reports->get(brp))
			ret = sizeof(*brp);

	} while (0);

	return ret;
}

int
BMP280::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop_cycle();
				_measure_ticks = 0;
				return OK;

				/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

				/* zero would be bad */
			case 0:
				return -EINVAL;

				/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(BMP280_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start_cycle();

					return OK;
				}

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(BMP280_CONVERSION_INTERVAL))
						return -EINVAL;

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start_cycle();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0)
			return SENSOR_POLLRATE_MANUAL;

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
		/* lower bound is mandatory, upper bound is a sanity check */
		if ((arg < 1) || (arg > 100))
			return -EINVAL;

		irqstate_t flags = irqsave();
		if (!_reports->resize(arg)) {
			irqrestore(flags);
			return -ENOMEM;
		}
		irqrestore(flags);		
		return OK;
	}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/*
		 * Since we are initialized, we do not need to do anything, since the
		 * PROM is correctly read and the part does not need to be configured.
		 */
		return OK;

	case BAROIOCSMSLPRESSURE:

		/* range-check for sanity */
		if ((arg < 80000) || (arg > 120000))
			return -EINVAL;

		_msl_pressure = arg;
		return OK;

	case BAROIOCGMSLPRESSURE:
		return _msl_pressure;

	default:
		break;
	}

	/* give it to the bus-specific superclass */
	// return bus_ioctl(filp, cmd, arg);
	return CDev::ioctl(filp, cmd, arg);
}

void
BMP280::start_cycle()
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&BMP280::cycle_trampoline, this, 1);
}

void
BMP280::stop_cycle()
{
	work_cancel(HPWORK, &_work);
}

void
BMP280::cycle_trampoline(void *arg)
{
	BMP280 *dev = reinterpret_cast<BMP280 *>(arg);

	dev->cycle();
}

void
BMP280::cycle()
{
	int ret;

	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		ret = collect();
		if (ret != OK) {
			if (ret == -6) {
				/*
				 * The bmp280 seems to regularly fail to respond to
				 * its address; this happens often enough that we'd rather not
				 * spam the console with a message for this.
				 */
			} else {
				//log("collection error %d", ret);
			}
			/* reset the collection state machine and try again */
			start_cycle();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 * Don't inject one after temperature measurements, so we can keep
		 * doing pressure measurements at something close to the desired rate.
		 */
		if ((_measure_phase != 0) &&
		    (_measure_ticks > USEC2TICK(BMP280_CONVERSION_INTERVAL))) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&BMP280::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(BMP280_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	ret = checkconfig();
	if (ret != OK) {
		//log("measure error %d", ret);
		/* reset the collection state machine and try again */
		start_cycle();
		return;
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&BMP280::cycle_trampoline,
		   this,
		   USEC2TICK(BMP280_CONVERSION_INTERVAL));
}

int
BMP280::checkconfig()
{
	int ret;

	perf_begin(_measure_perf);

	/*
	 * In phase zero, request temperature; in other phases, request pressure.
	 */
	//unsigned addr = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;
	unsigned addr = BMP280_CTRL_MEAS_REG;
	/*
	 * Send the command to begin measuring.
	 */
	ret = _interface->ioctl(IOCTL_CHECKCONFIG, addr);
	if (OK != ret)
		perf_count(_comms_errors);

	perf_end(_measure_perf);

	return ret;
}

int32_t t_fine;




double BMP280::bmp280_compensate_T_double(int32_t adc_T)
{
double var1, var2, T;
var1 = (((double)adc_T)/16384.0 - ((double)_prom.dig_T1)/1024.0) * ((double)_prom.dig_T2);
var2 = ((((double)adc_T)/131072.0 - ((double)_prom.dig_T1)/8192.0) *
(((double)adc_T)/131072.0 - ((double) _prom.dig_T1)/8192.0)) * ((double)_prom.dig_T3);
t_fine = (int32_t)(var1 + var2);
T = (var1 + var2) / 5120.0;
return T;
}


double BMP280::bmp280_compensate_P_double(int32_t adc_P)
{
	double var1, var2, p;
	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)_prom.dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)_prom.dig_P5) * 2.0;
	var2 = (var2/4.0)+(((double)_prom.dig_P4) * 65536.0);
	var1 = (((double)_prom.dig_P3) * var1 * var1 / 524288.0 + ((double)_prom.dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*((double)_prom.dig_P1);
	if (var1 == 0.0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)_prom.dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)_prom.dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)_prom.dig_P7)) / 16.0;
	return p;
}

int
BMP280::collect()
{
	int ret;
	uint32_t raw;
	int32_t temperature;
	int32_t pression;

	perf_begin(_sample_perf);

	struct baro_report report;
	/* this should be fairly close to the end of the conversion, so the best approximation of the time */
	report.timestamp = hrt_absolute_time();
        report.error_count = perf_event_count(_comms_errors);

	/* read the most recent measurement - read offset/size are hardcoded in the interface */
	ret = _interface->read(READ_TEMPERATURE, (void *)&raw, 0);
	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}
	temperature = raw ;
	report.temperature = (float)bmp280_compensate_T_double(temperature);// raw temp *100

	ret = _interface->read(READ_PRESSURE, (void *)&raw, 0);
	if (ret < 0) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	pression = raw ;
	report.pressure = (float)bmp280_compensate_P_double(pression)/100.0f ; //raw press *100

	/* handle a measurement */

	/* pressure calculation, result in Pa */
	int32_t P = pression;

	_P = (float)bmp280_compensate_P_double(pression)/10;
	_T = (float)bmp280_compensate_T_double(temperature)/100;

	/* altitude calculations based on http://www.kansasflyer.org/index.asp?nav=Avi&sec=Alti&tab=Theory&pg=1 */

	/*
	 * PERFORMANCE HINT:
	 *
	 * The single precision calculation is 50 microseconds faster than the double
	 * precision variant. It is however not obvious if double precision is required.
	 * Pending more inspection and tests, we'll leave the double precision variant active.
	 *
	 * Measurements:
	 * 	double precision: bmp280_read: 992 events, 258641us elapsed, min 202us max 305us
	 *	single precision: bmp280_read: 963 events, 208066us elapsed, min 202us max 241us
	 */

	/* tropospheric properties (0-11km) for standard atmosphere */
	const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const double a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const double g  = 9.80665;	/* gravity constant in m/s/s */
	const double R  = 287.05;	/* ideal gas constant in J/kg/K */

	/* current pressure at MSL in kPa */
	double p1 = _msl_pressure / 1000.0;

	/* measured pressure in kPa */
	double p = P / 1000.0;

	/*
	 * Solve:
	 *
	 *     /        -(aR / g)     \
	 *    | (p / p1)          . T1 | - T1
	 *     \                      /
	 * h = -------------------------------  + h1
	 *                   a
	 */
	report.altitude = (((pow((p / p1), (-(a * R) / g))) * T1) - T1) / a;
	_Alt = report.altitude;

	/* publish it */
	if (_baro_topic > 0 && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_baro), _baro_topic, &report);
	}

	if (_reports->force(&report)) {
		perf_count(_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);


	/* update the measurement state machine */
	INCREMENT(_measure_phase, BMP280_MEASUREMENT_RATIO + 1);

	perf_end(_sample_perf);

	return OK;
}

void
BMP280::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
	printf("TEMP:           %d\n", _TEMP);
	printf("SENS:           %lld\n", _SENS);
	printf("OFF:            %lld\n", _OFF);
	printf("P:              %.3f\n", _P);
	printf("T:              %.3f\n", _T);
	printf("alt:            %.3f\n", _Alt);
	printf("MSL pressure:   %10.4f\n", (double)(_msl_pressure / 100.f));
	printf("dig_T1             %u\n", _prom.dig_T1);
	printf("dig_T2             %i\n", _prom.dig_T2);
	printf("dig_T3             %i\n", _prom.dig_T3);
	printf("dig_P1             %u\n", _prom.dig_P1);
	printf("dig_P2             %i\n", _prom.dig_P2);
	printf("dig_P3             %i\n", _prom.dig_P3);
	printf("dig_P4             %i\n", _prom.dig_P4);
	printf("dig_P5             %i\n", _prom.dig_P5);
	printf("dig_P6             %i\n", _prom.dig_P6);
	printf("dig_P7             %i\n", _prom.dig_P7);
	printf("dig_P8             %i\n", _prom.dig_P8);
	printf("dig_P9             %i\n", _prom.dig_P9);

}

/**
 * Local functions in support of the shell command.
 */
namespace bmp280
{

BMP280	*g_dev_int;
BMP280	*g_dev_exp;
BMP280	*g_dev_imu;

void	start(enum BusSensor bustype);
void	reset(enum BusSensor bustype);
void	info(enum BusSensor bustype);
void	calibrate(enum BusSensor bustype, unsigned altitude);


/**
 * Start the driver.
 */
void
start(enum BusSensor bustype)
{
	int fd;
	prom_u prom_buf;
	BMP280 *g_dev = nullptr;
	const char *path;

	switch (bustype) {
	case TYPE_BUS_SENSOR_INTERNAL:
#ifdef SPI_BUS_BMP280
		g_dev = g_dev_int;
		path = BMP280_BARO_DEVICE_PATH_INT;
#endif
		break;
	case TYPE_BUS_SENSOR_IMU:
#ifdef SPI_BUS_IMU_BMP280
		g_dev = g_dev_imu;
		path = BMP280_BARO_DEVICE_PATH_IMU;
#endif
		break;
	case TYPE_BUS_SENSOR_EXTERNAL:
#ifdef SPI_BUS_EXP_BMP280
		g_dev = g_dev_exp;
		path = BMP280_BARO_DEVICE_PATH_EXP;
#endif
		break;
	}

	if (g_dev != nullptr)
		/* if already started, the still command succeeded */
		errx(0, "already started");

	device::Device *interface = nullptr;

	/* create the driver, try SPI first, fall back to I2C if unsuccessful */
	if (BMP280_spi_interface != nullptr)
		interface = BMP280_spi_interface(prom_buf, bustype);

	if (interface == nullptr)
		errx(1, "failed to allocate an interface");

	if (interface->init() != OK) {
		delete interface;
		errx(1, "interface init failed");
	}

	g_dev = new BMP280(interface, path, prom_buf);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(path, O_RDONLY);

	if (fd < 0) {
		warnx("can't open baro device");
		goto fail;
	}
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		warnx("failed setting default poll rate");
		goto fail;
	}

	exit(0);

fail:

	if (g_dev != nullptr) {
		delete interface;
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Reset the driver.
 */
void
reset(enum BusSensor bustype)
{
	const char *path;

	switch (bustype) {
	case TYPE_BUS_SENSOR_INTERNAL:
#ifdef SPI_BUS_BMP280
		path = BMP280_BARO_DEVICE_PATH_INT;
#endif
		break;
	case TYPE_BUS_SENSOR_IMU:
#ifdef SPI_BUS_IMU_BMP280
		path = BMP280_BARO_DEVICE_PATH_IMU;
#endif
		break;
	case TYPE_BUS_SENSOR_EXTERNAL:
#ifdef SPI_BUS_EXP_BMP280
		path = BMP280_BARO_DEVICE_PATH_EXP;
#endif
		break;
	}

	int fd = open(path, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "driver poll restart failed");

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info(enum BusSensor bustype)
{
	BMP280 *g_dev = nullptr;

    switch (bustype) {
    case TYPE_BUS_SENSOR_INTERNAL:
		g_dev = g_dev_int;
    	break;
    case TYPE_BUS_SENSOR_IMU:
		g_dev = g_dev_imu;
    	break;
    case TYPE_BUS_SENSOR_EXTERNAL:
		g_dev = g_dev_exp;
    	break;
    }

	if (g_dev == nullptr)
		errx(1, "driver not running");

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

/**
 * Calculate actual MSL pressure given current altitude
 */
void
calibrate(enum BusSensor bustype, unsigned altitude)
{
	struct baro_report report;
	float	pressure;
	float	p1;
	const char *path;

	switch (bustype) {
	case TYPE_BUS_SENSOR_INTERNAL:
#ifdef SPI_BUS_BMP280
		path = BMP280_BARO_DEVICE_PATH_INT;
#endif
		break;
	case TYPE_BUS_SENSOR_IMU:
#ifdef SPI_BUS_IMU_BMP280
		path = BMP280_BARO_DEVICE_PATH_IMU;
#endif
		break;
	case TYPE_BUS_SENSOR_EXTERNAL:
#ifdef SPI_BUS_EXP_BMP280
		path = BMP280_BARO_DEVICE_PATH_EXP;
#endif
		break;
	}

	int fd = open(path, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'bmp280 start' if the driver is not running)", path);

	/* start the sensor polling at max */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MAX))
		errx(1, "failed to set poll rate");

	/* average a few measurements */
	pressure = 0.0f;

	for (unsigned i = 0; i < 20; i++) {
		struct pollfd fds;
		int ret;
		ssize_t sz;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 1000);

		if (ret != 1)
			errx(1, "timed out waiting for sensor data");

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report))
			err(1, "sensor read failed");

		pressure += report.pressure;
	}

	pressure /= 20;		/* average */
	pressure /= 10;		/* scale from millibar to kPa */

	/* tropospheric properties (0-11km) for standard atmosphere */
	const float T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
	const float a  = -6.5 / 1000;	/* temperature gradient in degrees per metre */
	const float g  = 9.80665f;	/* gravity constant in m/s/s */
	const float R  = 287.05f;	/* ideal gas constant in J/kg/K */

	warnx("averaged pressure %10.4fkPa at %um", (double)pressure, altitude);

	p1 = pressure * (powf(((T1 + (a * (float)altitude)) / T1), (g / (a * R))));

	warnx("calculated MSL pressure %10.4fkPa", (double)p1);

	/* save as integer Pa */
	p1 *= 1000.0f;

	if (ioctl(fd, BAROIOCSMSLPRESSURE, (unsigned long)p1) != OK)
		err(1, "BAROIOCSMSLPRESSURE");

	exit(0);
}

} // namespace

void
bmp280_usage()
{
	warnx("missing command: try 'start', 'info', 'reset', 'calibrate'");
	warnx("options:");
	warnx("    -X only external bus");
	warnx("    -U only external IMU");
	warnx("    -I only internal bus");
}

int
bmp280_main(int argc, char *argv[])
{
	int ch;
	enum BusSensor bustype = TYPE_BUS_SENSOR_NONE;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "XIU")) != EOF) {
		switch (ch) {
		case 'I':
			bustype = TYPE_BUS_SENSOR_INTERNAL;
			break;
		case 'X':
			bustype = TYPE_BUS_SENSOR_EXTERNAL;
			break;
		case 'U':
			bustype = TYPE_BUS_SENSOR_IMU;
			break;
		default:
			bmp280_usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start"))
		bmp280::start(bustype);


	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset"))
		bmp280::reset(bustype);

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info"))
		bmp280::info(bustype);

	/*
	 * Perform MSL pressure calibration given an altitude in metres
	 */
	if (!strcmp(verb, "calibrate")) {
		if (argc < 2)
			errx(1, "missing altitude");

		long altitude = strtol(argv[optind+1], nullptr, 10);

		bmp280::calibrate(bustype, altitude);
	}

	errx(1, "unrecognised command, try 'start', 'reset' or 'info'");
}

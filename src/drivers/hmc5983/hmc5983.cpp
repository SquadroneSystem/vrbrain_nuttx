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
 * @file hmc5983.cpp
 *
 * Driver for the HMC5983 magnetometer connected via SPI.
 */

#include <nuttx/config.h>

#include <drivers/device/spi.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <board_config.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>

#include <float.h>
#include <getopt.h>
#include <lib/conversion/rotation.h>

//Define the type of orientation of the board
//#define HMC5983_CHIP_FORWARD

#define DIR_READ			0x80
#define DIR_READ_MULT       0xC0
#define DIR_WRITE			0x00

/*
 * HMC5983 internal constants and data structures.
 */

#define HMC5983L_DEVICE_PATH_INT	"/dev/hmc5983_int"
#define HMC5983L_DEVICE_PATH_EXP	"/dev/hmc5983_exp"
#define HMC5983L_DEVICE_PATH_IMU	"/dev/hmc5983_imu"

/* Max measurement rate is 160Hz, however with 160 it will be set to 166 Hz, therefore workaround using 150 */
#define HMC5983_CONVERSION_INTERVAL	(1000000 / 150)	/* microseconds */

#define ADDR_CONF_A			0x00
#define ADDR_CONF_B			0x01
#define ADDR_MODE			0x02
#define ADDR_DATA_OUT_X_MSB		0x03
#define ADDR_DATA_OUT_X_LSB		0x04
#define ADDR_DATA_OUT_Z_MSB		0x05
#define ADDR_DATA_OUT_Z_LSB		0x06
#define ADDR_DATA_OUT_Y_MSB		0x07
#define ADDR_DATA_OUT_Y_LSB		0x08
#define ADDR_STATUS			0x09
#define ADDR_ID_A			0x0a
#define ADDR_ID_B			0x0b
#define ADDR_ID_C			0x0c

/* modes not changeable outside of driver */
#define HMC5983L_MODE_NORMAL		(0 << 0)  /* default */
#define HMC5983L_MODE_POSITIVE_BIAS	(1 << 0)  /* positive bias */
#define HMC5983L_MODE_NEGATIVE_BIAS	(1 << 1)  /* negative bias */

#define HMC5983L_AVERAGING_1		(0 << 5) /* conf a register */
#define HMC5983L_AVERAGING_2		(1 << 5)
#define HMC5983L_AVERAGING_4		(2 << 5)
#define HMC5983L_AVERAGING_8		(3 << 5)

#define MODE_REG_CONTINOUS_MODE		(0 << 0)
#define MODE_REG_SINGLE_MODE		(1 << 0) /* default */

#define STATUS_REG_DATA_OUT_LOCK	(1 << 1) /* page 16: set if data is only partially read, read device to reset */
#define STATUS_REG_DATA_READY		(1 << 0) /* page 16: set if all axes have valid measurements */

#define ID_A_WHO_AM_I			'H'
#define ID_B_WHO_AM_I			'4'
#define ID_C_WHO_AM_I			'3'

/*
  the HMC5983 can only handle high SPI bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  SPI speed
 */
#define HMC5983_LOW_BUS_SPEED				1000*1000
#define HMC5983_HIGH_BUS_SPEED				8*1000*1000

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class HMC5983 : public device::SPI
{
public:
	HMC5983(int bus, const char *path, spi_dev_e device, enum Rotation rotation, enum BusSensor bustype);
	virtual ~HMC5983();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

protected:
	virtual int		probe();

private:
	work_s			_work;
	unsigned		_measure_ticks;

	RingBuffer		*_reports;
	mag_scale		_scale;
	float 			_range_scale;
	float 			_range_ga;
	uint8_t			_range_bits;
	uint8_t			_conf_reg;
	bool			_collect_phase;
	int			_class_instance;

	orb_advert_t		_mag_topic;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;
	perf_counter_t		_buffer_overflows;
	perf_counter_t		_range_errors;
	perf_counter_t		_conf_errors;

	/* status reporting */
	bool			_sensor_ok;		/**< sensor was found and reports ok */
	bool			_calibrated;		/**< the calibration is valid */

	int			_bus;			/**< the bus the device is connected to */
	enum Rotation		_rotation;
	enum BusSensor 		_bustype;

	struct mag_report	_last_report;           /**< used for info() */

	/**
	 * Test whether the device supported by the driver is present at a
	 * specific address.
	 *
	 * @param address	The I2C bus address to probe.
	 * @return		True if the device is present.
	 */
	int			probe_address(uint8_t address);

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void			start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void			stop();

	/**
	 * Reset the device
	 */
	int			reset();

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test strap, 0 to disable
	 */
	int			calibrate(struct file *filp, unsigned enable);

	/**
	 * Perform the on-sensor scale calibration routine.
	 *
	 * @note The sensor will continue to provide measurements, these
	 *	 will however reflect the uncalibrated sensor state until
	 *	 the calibration routine has been completed.
	 *
	 * @param enable set to 1 to enable self-test positive strap, -1 to enable
	 *        negative strap, 0 to set to normal mode
	 */
	int			set_excitement(unsigned enable);

	/**
	 * Set the sensor range.
	 *
	 * Sets the internal range to handle at least the argument in Gauss.
	 */
	int 			set_range(unsigned range);

	/**
	 * check the sensor range.
	 *
	 * checks that the range of the sensor is correctly set, to
	 * cope with communication errors causing the range to change
	 */
	void 			check_range(void);

	/**
	 * check the sensor configuration.
	 *
	 * checks that the config of the sensor is correctly set, to
	 * cope with communication errors causing the configuration to
	 * change
	 */
	void 			check_conf(void);

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
	 * Write a register.
	 *
	 * @param reg		The register to write.
	 * @param val		The value to write.
	 * @return		OK on write success.
	 */
	int			write_reg(uint8_t reg, uint8_t val);

	/**
	 * Read a register.
	 *
	 * @param reg		The register to read.
	 * @param val		The value read.
	 * @return		OK on read success.
	 */
	int			read_reg(uint8_t reg, uint8_t &val);

	/**
	 * Issue a measurement command.
	 *
	 * @return		OK if the measurement command was successful.
	 */
	int			measure();

	/**
	 * Collect the result of the most recent measurement.
	 */
	int			collect();

	/**
	 * Convert a big-endian signed 16-bit value to a float.
	 *
	 * @param in		A signed 16-bit big-endian value.
	 * @return		The floating-point representation of the value.
	 */
	float			meas_to_float(uint8_t in[2]);

	/**
	 * Check the current calibration and update device status
	 *
	 * @return 0 if calibration is ok, 1 else
	 */
	 int 			check_calibration();

	 /**
	 * Check the current scale calibration
	 *
	 * @return 0 if scale calibration is ok, 1 else
	 */
	 int 			check_scale();

	 /**
	 * Check the current offset calibration
	 *
	 * @return 0 if offset calibration is ok, 1 else
	 */
	 int 			check_offset();

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int hmc5983_main(int argc, char *argv[]);


HMC5983::HMC5983(int bus, const char *path, spi_dev_e device, enum Rotation rotation, enum BusSensor bustype) :
	SPI("HMC5983", path, bus, device, SPIDEV_MODE3, HMC5983_LOW_BUS_SPEED),
	_measure_ticks(0),
	_reports(nullptr),
	_range_scale(0), /* default range scale from counts to gauss */
	_range_ga(1.3f),
	_range_bits(0),
	_conf_reg(0),
	_class_instance(-1),
	_mag_topic(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "hmc5983_read")),
	_comms_errors(perf_alloc(PC_COUNT, "hmc5983_comms_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "hmc5983_buffer_overflows")),
	_range_errors(perf_alloc(PC_COUNT, "hmc5983_range_errors")),
	_conf_errors(perf_alloc(PC_COUNT, "hmc5983_conf_errors")),
	_sensor_ok(false),
	_calibrated(false),
	_bus(bus),
	_rotation(rotation),
	_bustype(bustype)
{
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_HMC5983;

	// enable debug() calls
	_debug_enabled = false;

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

HMC5983::~HMC5983()
{
	/* make sure we are truly inactive */
	stop();

	if (_reports != nullptr)
		delete _reports;

	if (_class_instance != -1)
		unregister_class_devname(MAG_DEVICE_PATH, _class_instance);

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
	perf_free(_range_errors);
	perf_free(_conf_errors);
}

int
HMC5983::init()
{
	int ret = ERROR;

	/* do SPI init (and probe) first */
	if (SPI::init() != OK)
		goto out;

	/* allocate basic report buffers */
	_reports = new RingBuffer(2, sizeof(mag_report));
	if (_reports == nullptr)
		goto out;

	/* reset the device configuration */
	reset();

	_class_instance = register_class_devname(MAG_DEVICE_PATH);

	ret = OK;
	/* sensor is ok, but not calibrated */
	_sensor_ok = true;
out:
	return ret;
}

int HMC5983::set_range(unsigned range)
{
	if (range < 1) {
		_range_bits = 0x00;
		_range_scale = 1.0f / 1370.0f;
		_range_ga = 0.88f;

	} else if (range <= 1) {
		_range_bits = 0x01;
		_range_scale = 1.0f / 1090.0f;
		_range_ga = 1.3f;

	} else if (range <= 2) {
		_range_bits = 0x02;
		_range_scale = 1.0f / 820.0f;
		_range_ga = 1.9f;

	} else if (range <= 3) {
		_range_bits = 0x03;
		_range_scale = 1.0f / 660.0f;
		_range_ga = 2.5f;

	} else if (range <= 4) {
		_range_bits = 0x04;
		_range_scale = 1.0f / 440.0f;
		_range_ga = 4.0f;

	} else if (range <= 4.7f) {
		_range_bits = 0x05;
		_range_scale = 1.0f / 390.0f;
		_range_ga = 4.7f;

	} else if (range <= 5.6f) {
		_range_bits = 0x06;
		_range_scale = 1.0f / 330.0f;
		_range_ga = 5.6f;

	} else {
		_range_bits = 0x07;
		_range_scale = 1.0f / 230.0f;
		_range_ga = 8.1f;
	}

	int ret;

	/*
	 * Send the command to set the range
	 */
	ret = write_reg(ADDR_CONF_B, (_range_bits << 5));

	if (OK != ret)
		perf_count(_comms_errors);

	uint8_t range_bits_in;
	ret = read_reg(ADDR_CONF_B, range_bits_in);

	if (OK != ret)
		perf_count(_comms_errors);

	return !(range_bits_in == (_range_bits << 5));
}

/**
   check that the range register has the right value. This is done
   periodically to cope with I2C bus noise causing the range of the
   compass changing.
 */
void HMC5983::check_range(void)
{
	int ret;

	uint8_t range_bits_in;
	ret = read_reg(ADDR_CONF_B, range_bits_in);
	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}
	if (range_bits_in != (_range_bits<<5)) {
		perf_count(_range_errors);
		ret = write_reg(ADDR_CONF_B, (_range_bits << 5));
		if (OK != ret)
			perf_count(_comms_errors);
	}
}

/**
   check that the configuration register has the right value. This is
   done periodically to cope with I2C bus noise causing the
   configuration of the compass to change.
 */
void HMC5983::check_conf(void)
{
	int ret;

	uint8_t conf_reg_in;
	ret = read_reg(ADDR_CONF_A, conf_reg_in);
	if (OK != ret) {
		perf_count(_comms_errors);
		return;
	}
	if (conf_reg_in != _conf_reg) {
		perf_count(_conf_errors);
		ret = write_reg(ADDR_CONF_A, _conf_reg);
		if (OK != ret)
			perf_count(_comms_errors);
	}
}

int
HMC5983::probe()
{
	uint8_t data[3] = {0, 0, 0};



	if (read_reg(ADDR_ID_A, data[0]) ||
	    read_reg(ADDR_ID_B, data[1]) ||
	    read_reg(ADDR_ID_C, data[2]))
		debug("read_reg fail");



	if ((data[0] != ID_A_WHO_AM_I) ||
	    (data[1] != ID_B_WHO_AM_I) ||
	    (data[2] != ID_C_WHO_AM_I)) {
		debug("ID byte mismatch (%02x,%02x,%02x)", data[0], data[1], data[2]);
		return -EIO;
	}

	return OK;
}

ssize_t
HMC5983::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct mag_report);
	struct mag_report *mag_buf = reinterpret_cast<struct mag_report *>(buffer);
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
			if (_reports->get(mag_buf)) {
				ret += sizeof(struct mag_report);
				mag_buf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(HMC5983_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		if (_reports->get(mag_buf)) {
			ret = sizeof(struct mag_report);
		}
	} while (0);

	return ret;
}

int
HMC5983::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
		switch (arg) {

			/* switching to manual polling */
		case SENSOR_POLLRATE_MANUAL:
			stop();
			_measure_ticks = 0;
			return OK;

			/* external signalling (DRDY) not supported */
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
				_measure_ticks = USEC2TICK(HMC5983_CONVERSION_INTERVAL);

				/* if we need to start the poll state machine, do it */
				if (want_start)
					start();

				return OK;
			}

			/* adjust to a legal polling interval in Hz */
		default: {
				/* do we need to start internal polling? */
				bool want_start = (_measure_ticks == 0);

				/* convert hz to tick interval via microseconds */
				unsigned ticks = USEC2TICK(1000000 / arg);

				/* check against maximum rate */
				if (ticks < USEC2TICK(HMC5983_CONVERSION_INTERVAL))
					return -EINVAL;

				/* update interval for next measurement */
				_measure_ticks = ticks;

				/* if we need to start the poll state machine, do it */
				if (want_start)
					start();

				return OK;
			}
		}
	}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0)
			return SENSOR_POLLRATE_MANUAL;

		return 1000000/TICK2USEC(_measure_ticks);

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
		return reset();

	case MAGIOCSSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return ioctl(filp, SENSORIOCSPOLLRATE, arg);

	case MAGIOCGSAMPLERATE:
		/* same as pollrate because device is in single measurement mode*/
		return 1000000/TICK2USEC(_measure_ticks);

	case MAGIOCSRANGE:
		return set_range(arg);

	case MAGIOCGRANGE:
		return _range_ga;

	case MAGIOCSLOWPASS:
	case MAGIOCGLOWPASS:
		/* not supported, no internal filtering */
		return -EINVAL;

	case MAGIOCSSCALE:
		/* set new scale factors */
		memcpy(&_scale, (mag_scale *)arg, sizeof(_scale));
		/* check calibration, but not actually return an error */
		(void)check_calibration();
		return 0;

	case MAGIOCGSCALE:
		/* copy out scale factors */
		memcpy((mag_scale *)arg, &_scale, sizeof(_scale));
		return 0;

	case MAGIOCCALIBRATE:
		return calibrate(filp, arg);

	case MAGIOCEXSTRAP:
		return set_excitement(arg);

	case MAGIOCSELFTEST:
		return check_calibration();

	case MAGIOCGEXTERNAL:
		return (_bustype != TYPE_BUS_SENSOR_INTERNAL);

	default:
		/* give it to the superclass */
		return SPI::ioctl(filp, cmd, arg);
	}
}

void
HMC5983::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&HMC5983::cycle_trampoline, this, 1);
}

void
HMC5983::stop()
{
	work_cancel(HPWORK, &_work);
}

int
HMC5983::reset()
{
	/* set range */
	return set_range(_range_ga);
}

void
HMC5983::cycle_trampoline(void *arg)
{
	HMC5983 *dev = (HMC5983 *)arg;

	dev->cycle();
}

void
HMC5983::cycle()
{
	/* collection phase? */
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			log("collection error");
			/* restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/*
		 * Is there a collect->measure gap?
		 */
		if (_measure_ticks > USEC2TICK(HMC5983_CONVERSION_INTERVAL)) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&HMC5983::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(HMC5983_CONVERSION_INTERVAL));

			return;
		}
	}

	/* measurement phase */
	if (OK != measure())
		log("measure error");

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&HMC5983::cycle_trampoline,
		   this,
		   USEC2TICK(HMC5983_CONVERSION_INTERVAL));
}

int
HMC5983::measure()
{
	int ret;

	/*
	 * Send the command to begin a measurement.
	 */
	ret = write_reg(ADDR_MODE, MODE_REG_SINGLE_MODE);

	if (OK != ret)
		perf_count(_comms_errors);

	return ret;
}

int
HMC5983::collect()
{
#pragma pack(push, 1)
	struct { /* status register and data as read back from the device */
		uint8_t		cmd;
		uint8_t		x[2];
		uint8_t		z[2];
		uint8_t		y[2];
	}	hmc_report;
#pragma pack(pop)
	struct {
		int16_t		x, y, z;
	} report;
	int	ret = -EIO;
	uint8_t check_counter;

	perf_begin(_sample_perf);
	struct mag_report new_report;

	/* this should be fairly close to the end of the measurement, so the best approximation of the time */
	new_report.timestamp = hrt_absolute_time();
        new_report.error_count = perf_event_count(_comms_errors);

	/*
	 * @note  We could read the status register here, which could tell us that
	 *        we were too early and that the output registers are still being
	 *        written.  In the common case that would just slow us down, and
	 *        we're better off just never being early.
	 */

	/* get measurements from the device */
    hmc_report.cmd = DIR_READ_MULT | ADDR_DATA_OUT_X_MSB;

    // sensor transfer at high clock speed
    set_frequency(HMC5983_HIGH_BUS_SPEED);

    ret = transfer((uint8_t *)&hmc_report, (uint8_t *)&hmc_report, sizeof(hmc_report));

	if (ret != OK) {
		perf_count(_comms_errors);
		debug("data/status read error");
		goto out;
	}

	/* swap the data we just received */
	report.x = (((int16_t)hmc_report.x[0]) << 8) + hmc_report.x[1];
	report.y = (((int16_t)hmc_report.y[0]) << 8) + hmc_report.y[1];
	report.z = (((int16_t)hmc_report.z[0]) << 8) + hmc_report.z[1];

	/*
	 * If any of the values are -4096, there was an internal math error in the sensor.
	 * Generalise this to a simple range check that will also catch some bit errors.
	 */
	if ((abs(report.x) > 2048) ||
	    (abs(report.y) > 2048) ||
	    (abs(report.z) > 2048)) {
		perf_count(_comms_errors);
		goto out;
	}

	/*
	 * RAW outputs
	 *
	 * to align the sensor axes with the board, x and y need to be flipped
	 * and y needs to be negated
	 */
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1) || defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
	new_report.x_raw = report.y;
	new_report.y_raw = -report.x;
	/* z remains z */
	new_report.z_raw = report.z;

	/* scale values for output */

#ifdef PX4_I2C_BUS_ONBOARD
	if (_bus == PX4_I2C_BUS_ONBOARD) {
		// convert onboard so it matches offboard for the
		// scaling below
		report.y = -report.y;
		report.x = -report.x;
        }
#endif

#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V40) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V45) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V50) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V51) || defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)

	if (_bustype == TYPE_BUS_SENSOR_INTERNAL) {

		new_report.x_raw = report.y;
		new_report.y_raw = report.x;
		new_report.z_raw = -report.z;

	} else if (_bustype == TYPE_BUS_SENSOR_EXTERNAL) {

		new_report.x_raw = report.y;
		new_report.y_raw = report.x;
		new_report.z_raw = -report.z;

	} else if (_bustype == TYPE_BUS_SENSOR_IMU) {

		new_report.x_raw = -report.y;
		new_report.y_raw = report.x;
		new_report.z_raw = report.z;

	}

#elif defined(CONFIG_ARCH_BOARD_VRHERO_V10)

#ifdef	HMC5983_CHIP_FORWARD
	new_report.x_raw = -report.z;
	new_report.y_raw = -report.x;
	new_report.z_raw = report.y;
#else
	new_report.x_raw = report.z;
	new_report.y_raw = report.x;
	new_report.z_raw = report.y;
#endif

#endif

	/* the standard external mag by 3DR has x pointing to the
	 * right, y pointing backwards, and z down, therefore switch x
	 * and y and invert y */
#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1) || defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
	new_report.x = ((-report.y * _range_scale) - _scale.x_offset) * _scale.x_scale;
	/* flip axes and negate value for y */
	new_report.y = ((report.x * _range_scale) - _scale.y_offset) * _scale.y_scale;
	/* z remains z */
	new_report.z = ((report.z * _range_scale) - _scale.z_offset) * _scale.z_scale;

#elif defined(CONFIG_ARCH_BOARD_VRBRAIN_V40) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V45) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V50) || defined(CONFIG_ARCH_BOARD_VRBRAIN_V51) || defined(CONFIG_ARCH_BOARD_VRUBRAIN_V51)

	if (_bustype == TYPE_BUS_SENSOR_INTERNAL) {

		new_report.x = ((report.y * _range_scale) - _scale.x_offset) * _scale.x_scale;
		new_report.y = ((report.x * _range_scale) - _scale.y_offset) * _scale.y_scale;
		new_report.z = ((-report.z * _range_scale) - _scale.z_offset) * _scale.z_scale;

	} else if (_bustype == TYPE_BUS_SENSOR_EXTERNAL) {

		new_report.x = ((report.y * _range_scale) - _scale.x_offset) * _scale.x_scale;
		new_report.y = ((report.x * _range_scale) - _scale.y_offset) * _scale.y_scale;
		new_report.z = ((-report.z * _range_scale) - _scale.z_offset) * _scale.z_scale;

	} else if (_bustype == TYPE_BUS_SENSOR_IMU) {

		new_report.x = ((report.y * _range_scale) - _scale.x_offset) * _scale.x_scale;
		new_report.y = ((report.x * _range_scale) - _scale.y_offset) * _scale.y_scale;
		new_report.z = ((-report.z * _range_scale) - _scale.z_offset) * _scale.z_scale;

	}

#elif defined(CONFIG_ARCH_BOARD_VRHERO_V10)

#ifdef	HMC5983_CHIP_FORWARD
	new_report.x = - ((report.z * _range_scale) - _scale.x_offset) * _scale.x_scale;
	new_report.y = - ((report.x * _range_scale) - _scale.y_offset) * _scale.y_scale;
	new_report.z = ((report.y * _range_scale) - _scale.z_offset) * _scale.z_scale;
#else
	new_report.x = ((report.z * _range_scale) - _scale.x_offset) * _scale.x_scale;
	new_report.y = ((report.x * _range_scale) - _scale.y_offset) * _scale.y_scale;
	new_report.z = ((report.y * _range_scale) - _scale.z_offset) * _scale.z_scale;
#endif

#endif

	// apply user specified rotation
	rotate_3f(_rotation, new_report.x, new_report.y, new_report.z);

	if (_class_instance == CLASS_DEVICE_PRIMARY && !(_pub_blocked)) {

		if (_mag_topic != -1) {
			/* publish it */
			orb_publish(ORB_ID(sensor_mag), _mag_topic, &new_report);
		} else {
			_mag_topic = orb_advertise(ORB_ID(sensor_mag), &new_report);

			if (_mag_topic < 0)
				debug("failed to create sensor_mag publication");
		}
	}

	_last_report = new_report;

	/* post a report to the ring */
	if (_reports->force(&new_report)) {
		perf_count(_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	/*
	  periodically check the range register and configuration
	  registers. With a bad I2C cable it is possible for the
	  registers to become corrupt, leading to bad readings. It
	  doesn't happen often, but given the poor cables some
	  vehicles have it is worth checking for.
	 */
	check_counter = perf_event_count(_sample_perf) % 256;
	if (check_counter == 0) {
		check_range();
	}
	if (check_counter == 128) {
		check_conf();
	}

	ret = OK;

out:
	perf_end(_sample_perf);
	return ret;
}

int HMC5983::calibrate(struct file *filp, unsigned enable)
{
	struct mag_report report;
	ssize_t sz;
	int ret = 1;
	uint8_t good_count = 0;

	// XXX do something smarter here
	int fd = (int)enable;

	struct mag_scale mscale_previous = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	struct mag_scale mscale_null = {
		0.0f,
		1.0f,
		0.0f,
		1.0f,
		0.0f,
		1.0f,
	};

	float sum_excited[3] = {0.0f, 0.0f, 0.0f};

	/* expected axis scaling. The datasheet says that 766 will
	 * be places in the X and Y axes and 713 in the Z
	 * axis. Experiments show that in fact 766 is placed in X,
	 * and 713 in Y and Z. This is relative to a base of 660
	 * LSM/Ga, giving 1.16 and 1.08 */
	float expected_cal[3] = { 1.16f, 1.08f, 1.08f };

	warnx("starting mag scale calibration");

	/* start the sensor polling at 50 Hz */
	if (OK != ioctl(filp, SENSORIOCSPOLLRATE, 50)) {
		warn("failed to set 2Hz poll rate");
		ret = 1;
		goto out;
	}

	/* Set to 2.5 Gauss. We ask for 3 to get the right part of
         * the chained if statement above. */
	if (OK != ioctl(filp, MAGIOCSRANGE, 3)) {
		warnx("failed to set 2.5 Ga range");
		ret = 1;
		goto out;
	}

	if (OK != ioctl(filp, MAGIOCEXSTRAP, 1)) {
		warnx("failed to enable sensor calibration mode");
		ret = 1;
		goto out;
	}

	if (OK != ioctl(filp, MAGIOCGSCALE, (long unsigned int)&mscale_previous)) {
		warn("WARNING: failed to get scale / offsets for mag");
		ret = 1;
		goto out;
	}

	if (OK != ioctl(filp, MAGIOCSSCALE, (long unsigned int)&mscale_null)) {
		warn("WARNING: failed to set null scale / offsets for mag");
		ret = 1;
		goto out;
	}

	// discard 10 samples to let the sensor settle
	for (uint8_t i = 0; i < 10; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("timed out waiting for sensor data");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("periodic read failed");
			ret = -EIO;
			goto out;
		}
	}

	/* read the sensor up to 50x, stopping when we have 10 good values */
	for (uint8_t i = 0; i < 50 && good_count < 10; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = ::poll(&fds, 1, 2000);

		if (ret != 1) {
			warn("timed out waiting for sensor data");
			goto out;
		}

		/* now go get it */
		sz = ::read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			warn("periodic read failed");
			ret = -EIO;
			goto out;
		}
		float cal[3] = {fabsf(expected_cal[0] / report.x), 
				fabsf(expected_cal[1] / report.y), 
				fabsf(expected_cal[2] / report.z)};

		if (cal[0] > 0.7f && cal[0] < 1.35f &&
		    cal[1] > 0.7f && cal[1] < 1.35f &&
		    cal[2] > 0.7f && cal[2] < 1.35f) {
			good_count++;
			sum_excited[0] += cal[0];
			sum_excited[1] += cal[1];
			sum_excited[2] += cal[2];
		}

		//warnx("periodic read %u", i);
		//warnx("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
		//warnx("cal: %.6f  %.6f  %.6f", (double)cal[0], (double)cal[1], (double)cal[2]);
	}

	if (good_count < 5) {
		warn("failed calibration");
		ret = -EIO;
		goto out;
	}

#if 0
	warnx("measurement avg: %.6f  %.6f  %.6f", 
	      (double)sum_excited[0]/good_count, 
	      (double)sum_excited[1]/good_count, 
	      (double)sum_excited[2]/good_count);
#endif

	float scaling[3];

	scaling[0] = sum_excited[0] / good_count;
	scaling[1] = sum_excited[1] / good_count;
	scaling[2] = sum_excited[2] / good_count;

	warnx("axes scaling: %.6f  %.6f  %.6f", (double)scaling[0], (double)scaling[1], (double)scaling[2]);

	/* set scaling in device */
	mscale_previous.x_scale = scaling[0];
	mscale_previous.y_scale = scaling[1];
	mscale_previous.z_scale = scaling[2];

	ret = OK;

out:

	if (OK != ioctl(filp, MAGIOCSSCALE, (long unsigned int)&mscale_previous)) {
		warn("WARNING: failed to set new scale / offsets for mag");
	}

	/* set back to normal mode */
	/* Set to 1.1 Gauss */
	if (OK != ::ioctl(fd, MAGIOCSRANGE, 1)) {
		warnx("failed to set 1.1 Ga range");
	}

	if (OK != ::ioctl(fd, MAGIOCEXSTRAP, 0)) {
		warnx("failed to disable sensor calibration mode");
	}

	if (ret == OK) {
		if (!check_scale()) {
			warnx("mag scale calibration successfully finished.");
		} else {
			warnx("mag scale calibration finished with invalid results.");
			ret = ERROR;
		}

	} else {
		warnx("mag scale calibration failed.");
	}

	return ret;
}

int HMC5983::check_scale()
{
	bool scale_valid;

	if ((-FLT_EPSILON + 1.0f < _scale.x_scale && _scale.x_scale < FLT_EPSILON + 1.0f) &&
		(-FLT_EPSILON + 1.0f < _scale.y_scale && _scale.y_scale < FLT_EPSILON + 1.0f) &&
		(-FLT_EPSILON + 1.0f < _scale.z_scale && _scale.z_scale < FLT_EPSILON + 1.0f)) {
		/* scale is one */
		scale_valid = false;
	} else {
		scale_valid = true;
	}

	/* return 0 if calibrated, 1 else */
	return !scale_valid;
}

int HMC5983::check_offset()
{
	bool offset_valid;

	if ((-2.0f * FLT_EPSILON < _scale.x_offset && _scale.x_offset < 2.0f * FLT_EPSILON) &&
		(-2.0f * FLT_EPSILON < _scale.y_offset && _scale.y_offset < 2.0f * FLT_EPSILON) &&
		(-2.0f * FLT_EPSILON < _scale.z_offset && _scale.z_offset < 2.0f * FLT_EPSILON)) {
		/* offset is zero */
		offset_valid = false;
	} else {
		offset_valid = true;
	}

	/* return 0 if calibrated, 1 else */
	return !offset_valid;
}

int HMC5983::check_calibration()
{
	bool offset_valid = (check_offset() == OK);
	bool scale_valid  = (check_scale() == OK);

	if (_calibrated != (offset_valid && scale_valid)) {
		warnx("mag cal status changed %s%s", (scale_valid) ? "" : "scale invalid ",
					  (offset_valid) ? "" : "offset invalid");
		_calibrated = (offset_valid && scale_valid);


		// XXX Change advertisement

		/* notify about state change */
		struct subsystem_info_s info = {
			true,
			true,
			_calibrated,
			SUBSYSTEM_TYPE_MAG};
		static orb_advert_t pub = -1;

		if (!(_pub_blocked)) {
			if (pub > 0) {
				orb_publish(ORB_ID(subsystem_info), pub, &info);
			} else {
				pub = orb_advertise(ORB_ID(subsystem_info), &info);
			}
		}
	}

	/* return 0 if calibrated, 1 else */
	return (!_calibrated);
}

int HMC5983::set_excitement(unsigned enable)
{
	int ret;
	/* arm the excitement strap */
	ret = read_reg(ADDR_CONF_A, _conf_reg);

	if (OK != ret)
		perf_count(_comms_errors);

	if (((int)enable) < 0) {
		_conf_reg |= 0x01;

	} else if (enable > 0) {
		_conf_reg |= 0x02;

	} else {
		_conf_reg &= ~0x03;
	}

        // ::printf("set_excitement enable=%d regA=0x%x\n", (int)enable, (unsigned)_conf_reg);

	ret = write_reg(ADDR_CONF_A, _conf_reg);

	if (OK != ret)
		perf_count(_comms_errors);

	uint8_t conf_reg_ret;
	read_reg(ADDR_CONF_A, conf_reg_ret);

	//print_info();

	return !(_conf_reg == conf_reg_ret);
}

int
HMC5983::write_reg(uint8_t reg, uint8_t val)
{
	uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = val;

	// general register transfer at low clock speed
	set_frequency(HMC5983_LOW_BUS_SPEED);

	return transfer(cmd, nullptr, sizeof(cmd));
}

int
HMC5983::read_reg(uint8_t reg, uint8_t &val)
{
	uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0};

	// general register transfer at low clock speed
	set_frequency(HMC5983_LOW_BUS_SPEED);

	int ret = transfer(cmd, cmd, sizeof(cmd));
	val = cmd[1];

	return ret;
}

float
HMC5983::meas_to_float(uint8_t in[2])
{
	union {
		uint8_t	b[2];
		int16_t	w;
	} u;

	u.b[0] = in[1];
	u.b[1] = in[0];

	return (float) u.w;
}

void
HMC5983::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	printf("output  (%.2f %.2f %.2f)\n", (double)_last_report.x, (double)_last_report.y, (double)_last_report.z);
	printf("offsets (%.2f %.2f %.2f)\n", (double)_scale.x_offset, (double)_scale.y_offset, (double)_scale.z_offset);
	printf("scaling (%.2f %.2f %.2f) 1/range_scale %.2f range_ga %.2f\n", 
	       (double)_scale.x_scale, (double)_scale.y_scale, (double)_scale.z_scale,
	       (double)1.0/_range_scale, (double)_range_ga);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace hmc5983
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

HMC5983	*g_dev_int;
HMC5983	*g_dev_exp;
HMC5983	*g_dev_imu;

void	hmc5983_usage();
void	start(enum Rotation rotation, enum BusSensor bustype);
void	test(enum BusSensor bustype);
void	reset(enum BusSensor bustype);
void	info(enum BusSensor bustype);
int	calibrate(enum BusSensor bustype);

/**
 * Start the driver.
 */
void
start(enum Rotation rotation, enum BusSensor bustype)
{
	int fd;

	switch (bustype) {
	case TYPE_BUS_SENSOR_INTERNAL:
#ifdef SPI_BUS_HMC5983
		if (g_dev_int != nullptr)
			errx(0, "already started internal");
		g_dev_int = new HMC5983(SPI_BUS_HMC5983, HMC5983L_DEVICE_PATH_INT, (spi_dev_e)SPIDEV_HMC5983, rotation, bustype);
		if (g_dev_int != nullptr && OK != g_dev_int->init()) {
			delete g_dev_int;
			g_dev_int = nullptr;
		}
#endif
		break;
	case TYPE_BUS_SENSOR_IMU:
#ifdef SPI_BUS_IMU_HMC5983
		if (g_dev_imu != nullptr)
			errx(0, "already started external IMU");
		g_dev_imu = new HMC5983(SPI_BUS_IMU_HMC5983, HMC5983L_DEVICE_PATH_IMU, (spi_dev_e)SPIDEV_IMU_HMC5983, rotation, bustype);
		if (g_dev_imu != nullptr && OK != g_dev_imu->init()) {
			delete g_dev_imu;
			g_dev_imu = nullptr;
		}
#endif
		break;
	case TYPE_BUS_SENSOR_EXTERNAL:
#ifdef SPI_BUS_EXP_HMC5983
		if (g_dev_exp != nullptr)
			errx(0, "already started external EXP");
		g_dev_exp = new HMC5983(SPI_BUS_EXP_HMC5983, HMC5983L_DEVICE_PATH_EXP, (spi_dev_e)SPIDEV_EXP_HMC5983, rotation, bustype);
		if (g_dev_exp != nullptr && OK != g_dev_exp->init()) {
			delete g_dev_exp;
			g_dev_exp = nullptr;
		}
#endif
		break;
	}

	if (g_dev_int == nullptr && g_dev_exp == nullptr && g_dev_imu == nullptr)
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	if (g_dev_int != nullptr) {
		fd = open(HMC5983L_DEVICE_PATH_INT, O_RDONLY);
		if (fd < 0)
			goto fail;

		if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
			goto fail;
		close(fd);
	}

	if (g_dev_exp != nullptr) {
		fd = open(HMC5983L_DEVICE_PATH_EXP, O_RDONLY);
		if (fd < 0)
			goto fail;

		if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
			goto fail;
		close(fd);
	}

	if (g_dev_imu != nullptr) {
		fd = open(HMC5983L_DEVICE_PATH_IMU, O_RDONLY);
		if (fd < 0)
			goto fail;

		if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
			goto fail;
		close(fd);
	}

	exit(0);

fail:
	if (g_dev_int != nullptr) {
		delete g_dev_int;
		g_dev_int = nullptr;
	}
	if (g_dev_exp != nullptr) {
		delete g_dev_exp;
		g_dev_exp = nullptr;
	}
	if (g_dev_imu != nullptr) {
		delete g_dev_imu;
		g_dev_imu = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(enum BusSensor bustype)
{
	struct mag_report report;
	ssize_t sz;
	int ret;
	const char *path;

    switch (bustype) {
    case TYPE_BUS_SENSOR_INTERNAL:
    	path = HMC5983L_DEVICE_PATH_INT;
    	break;
    case TYPE_BUS_SENSOR_IMU:
    	path = HMC5983L_DEVICE_PATH_IMU;
    	break;
    case TYPE_BUS_SENSOR_EXTERNAL:
    	path = HMC5983L_DEVICE_PATH_EXP;
    	break;
    }

	int fd = open(path, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'hmc5983 start' if the driver is not running", path);

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report))
		err(1, "immediate read failed");

	warnx("single read");
	warnx("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
	warnx("time:        %lld", report.timestamp);

	/* check if mag is onboard or external */
	if ((ret = ioctl(fd, MAGIOCGEXTERNAL, 0)) < 0)
		errx(1, "failed to get if mag is onboard or external");
	warnx("device active: %s", ret ? "external" : "onboard");

	/* set the queue depth to 5 */
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10))
		errx(1, "failed to set queue depth");

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2))
		errx(1, "failed to set 2Hz poll rate");

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1)
			errx(1, "timed out waiting for sensor data");

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report))
			err(1, "periodic read failed");

		warnx("periodic read %u", i);
		warnx("measurement: %.6f  %.6f  %.6f", (double)report.x, (double)report.y, (double)report.z);
		warnx("time:        %lld", report.timestamp);
	}

	errx(0, "PASS");
}


/**
 * Automatic scale calibration.
 *
 * Basic idea:
 *
 *   output = (ext field +- 1.1 Ga self-test) * scale factor
 *
 * and consequently:
 *
 *   1.1 Ga = (excited - normal) * scale factor
 *   scale factor = (excited - normal) / 1.1 Ga
 *
 *   sxy = (excited - normal) / 766	| for conf reg. B set to 0x60 / Gain = 3
 *   sz  = (excited - normal) / 713	| for conf reg. B set to 0x60 / Gain = 3
 *
 * By subtracting the non-excited measurement the pure 1.1 Ga reading
 * can be extracted and the sensitivity of all axes can be matched.
 *
 * SELF TEST OPERATION
 * To check the HMC5983L for proper operation, a self test feature in incorporated
 * in which the sensor offset straps are excited to create a nominal field strength
 * (bias field) to be measured. To implement self test, the least significant bits
 * (MS1 and MS0) of configuration register A are changed from 00 to 01 (positive bias)
 * or 10 (negetive bias), e.g. 0x11 or 0x12.
 * Then, by placing the mode register into single-measurement mode (0x01),
 * two data acquisition cycles will be made on each magnetic vector.
 * The first acquisition will be a set pulse followed shortly by measurement
 * data of the external field. The second acquisition will have the offset strap
 * excited (about 10 mA) in the positive bias mode for X, Y, and Z axes to create
 * about a ±1.1 gauss self test field plus the external field. The first acquisition
 * values will be subtracted from the second acquisition, and the net measurement
 * will be placed into the data output registers.
 * Since self test adds ~1.1 Gauss additional field to the existing field strength,
 * using a reduced gain setting prevents sensor from being saturated and data registers
 * overflowed. For example, if the configuration register B is set to 0x60 (Gain=3),
 * values around +766 LSB (1.16 Ga * 660 LSB/Ga) will be placed in the X and Y data
 * output registers and around +713 (1.08 Ga * 660 LSB/Ga) will be placed in Z data
 * output register. To leave the self test mode, change MS1 and MS0 bit of the
 * configuration register A back to 00 (Normal Measurement Mode), e.g. 0x10.
 * Using the self test method described above, the user can scale sensor
 */
int calibrate(enum BusSensor bustype)
{
	int ret;
	const char *path;

    switch (bustype) {
    case TYPE_BUS_SENSOR_INTERNAL:
    	path = HMC5983L_DEVICE_PATH_INT;
    	break;
    case TYPE_BUS_SENSOR_IMU:
    	path = HMC5983L_DEVICE_PATH_IMU;
    	break;
    case TYPE_BUS_SENSOR_EXTERNAL:
    	path = HMC5983L_DEVICE_PATH_EXP;
    	break;
    }

	int fd = open(path, O_RDONLY);

	if (fd < 0)
		err(1, "%s open failed (try 'hmc5983 start' if the driver is not running", path);

	if (OK != (ret = ioctl(fd, MAGIOCCALIBRATE, fd))) {
		warnx("failed to enable sensor calibration mode");
	}

	close(fd);

	if (ret == OK) {
		errx(0, "PASS");

	} else {
		errx(1, "FAIL");
	}
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
    	path = HMC5983L_DEVICE_PATH_INT;
    	break;
    case TYPE_BUS_SENSOR_IMU:
    	path = HMC5983L_DEVICE_PATH_IMU;
    	break;
    case TYPE_BUS_SENSOR_EXTERNAL:
    	path = HMC5983L_DEVICE_PATH_EXP;
    	break;
    }

	int fd = open(path, O_RDONLY);

	if (fd < 0)
		err(1, "failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0)
		err(1, "driver poll restart failed");

        close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info(enum BusSensor bustype)
{
	HMC5983 *g_dev = nullptr;

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

} // namespace

void
hmc5983_usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'reset', 'status', 'calibrate'");
	warnx("options:");
	warnx("    -R rotation");
	warnx("    -X only external bus");
	warnx("    -U only external IMU");
	warnx("    -I only internal bus");
	warnx("    -C calibrate on start");
}

int
hmc5983_main(int argc, char *argv[])
{
	int ch;
	enum Rotation rotation = ROTATION_NONE;
	enum BusSensor bustype = TYPE_BUS_SENSOR_NONE;
    bool calibrate = false;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "XIUR:C")) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (enum Rotation)atoi(optarg);
			break;
		case 'I':
			bustype = TYPE_BUS_SENSOR_INTERNAL;
			break;
		case 'X':
			bustype = TYPE_BUS_SENSOR_EXTERNAL;
			break;
		case 'U':
			bustype = TYPE_BUS_SENSOR_IMU;
			break;
		case 'C':
			calibrate = true;
			break;
		default:
			hmc5983_usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];	

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		hmc5983::start(rotation, bustype);
		if (calibrate) {
			if (hmc5983::calibrate(bustype) == 0) {
				errx(0, "calibration successful");
				
			} else {
				errx(1, "calibration failed");
			}
		}
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test"))
		hmc5983::test(bustype);

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset"))
		hmc5983::reset(bustype);

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info") || !strcmp(verb, "status"))
		hmc5983::info(bustype);

	/*
	 * Autocalibrate the scaling
	 */
	if (!strcmp(verb, "calibrate")) {
		if (hmc5983::calibrate(bustype) == 0) {
			errx(0, "calibration successful");

		} else {
			errx(1, "calibration failed");
		}
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' 'calibrate' or 'info'");
}
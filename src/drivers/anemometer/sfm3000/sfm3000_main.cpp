/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "sfm3000.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

void
SFM3000::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

I2C bus driver for SENSIRION SFM3000 Low Pressure DropDigital Flow Meter (i2c).

The sensor/driver must be enabled using the parameter SENS_EN_SFM3000.

Setup/usage information: https://docs.px4.io/master/en/sensor/tfmini.html
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("sfm3000", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("anemometer");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 1, 25, "Sensor rotation - forward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

I2CSPIDriverBase *SFM3000::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				     int runtime_instance)
{
	SFM3000 *instance = new SFM3000(iterator.configuredBusOption(), iterator.bus(), cli.orientation, cli.bus_frequency);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	instance->start(); /// Ahhha! Multi-instance,OK?
	return instance;
}

extern "C" __EXPORT int sfm3000_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = SFM3000;
        BusCLIArguments cli{true, false};// I2C ? SPI
        cli.orientation = windspeed_s::ROTATION_FORWARD_FACING;
        cli.default_i2c_frequency = 400000;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.orientation = atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_ANEMO_DEVTYPE_SFM3000);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}

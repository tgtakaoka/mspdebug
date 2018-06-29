/* MSPDebug - debugging tool for MSP430 MCUs
 * Copyright (C) 2009, 2010 Daniel Beer
 * Copyright (C) 2018 Tadashi G. Takaoka
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "ctrlc.h"
#include "simio.h"
#include "stab.h"

/* Module under test */
#include "simio_clock.c"


/*
 * Helper functions for testing timer simio.
 */

static char **setup_args(const char *text)
{
	static char args_buf[80];
	static char *args;

	strncpy(args_buf, text, sizeof(args_buf));
	args = args_buf;
	return &args;
}

static int* setup_clocks(int mclk, int smclk, int aclk)
{
	static int clocks[SIMIO_NUM_CLOCKS];

	clocks[SIMIO_MCLK] = mclk;
	clocks[SIMIO_SMCLK] = smclk;
	clocks[SIMIO_ACLK] = aclk;
	return clocks;
}

static struct simio_device *create_clock(const char *arg)
{
	return simio_clock.create(setup_args(arg));
}

static int config_clock(struct simio_device *dev, const char *param,
			const char *arg)
{
	return simio_clock.config(dev, param, setup_args(arg));
}

static uint8_t read_clock(struct simio_device *dev, address_t address)
{
	struct clock *clk = (struct clock *)dev;
	uint8_t data;
	assert(simio_clock.read_b(dev, address, &data) == 0);
	return data;
}

static void write_clock(struct simio_device *dev, address_t address, uint8_t data)
{
	struct clock *clk = (struct clock *)dev;
	assert(simio_clock.write_b(dev, address, data) == 0);
}

static void step_mclk(struct simio_device *dev, int mclk)
{
	uint16_t status_register = 0;
	simio_clock.step(dev, status_register, setup_clocks(mclk, 0, 0));
}


/*
 * Working variables for tests.
 */

static struct simio_device *dev;


/*
 * Set up and tear down for each test.
 */

static void set_up()
{
	setup_args("");
	setup_clocks(0, 0, 0);
	dev = NULL;
}

static void tear_down()
{
	if (dev != NULL) {
		simio_clock.destroy(dev);
		dev = NULL;
	}
}

#define assert_not(e) assert(!(e))

/*
 * Set up for globals.
 */

static void set_up_globals()
{
	ctrlc_init();
	stab_init();
}


/*
 * Tests for clock simio.
 */

static void test_create_no_option()
{
	dev = create_clock("");

	assert(dev == NULL);
}

static void test_create_with_basic()
{
	dev = create_clock("basic");

	assert(dev != NULL);
	assert(dev->type != NULL);
	assert(strcmp(dev->type->name, "clock") == 0);

	// Check default values.
	struct clock *clk = (struct clock *)dev;
	assert(clk->clock_type == CLOCK_TYPE_BASIC);
}

static void test_create_with_basic_plus()
{
	dev = create_clock("basic+");

	assert(dev != NULL);
	assert(dev->type != NULL);
	assert(strcmp(dev->type->name, "clock") == 0);

	// Check default values.
	struct clock *clk = (struct clock *)dev;
	assert(clk->clock_type == CLOCK_TYPE_BASIC_PLUS);
}


/*
 * Test runner.
 */

static void run_test(void (*test)(), const char *test_name)
{
	set_up();

	test();
	printf("  PASS %s\n", test_name);

	tear_down();
}

#define RUN_TEST(test) run_test(test, #test)

int main(int argc, char **argv)
{
	set_up_globals();

	RUN_TEST(test_create_no_option);
	RUN_TEST(test_create_with_basic);
	RUN_TEST(test_create_with_basic_plus);
}

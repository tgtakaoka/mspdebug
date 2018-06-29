/* MSPDebug - debugging tool for MSP430 MCUs
 * Copyright (C) 2018, Tadashi G. Takaoka
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

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "simio_device.h"
#include "simio_clock.h"
#include "output.h"

/* Register address */
#define DCOCTL		0x0056
#define BCSCTL1		0x0057
#define BCSCTL2		0x0058
#define BCSCTL3		0x0053
#define CALDCO_16MHZ	0x10F8
#define CALBC1_16MHZ	0x10F9
#define CALDCO_12MHZ	0x10FA
#define CALBC1_12MHZ	0x10FB
#define CALDCO_8MHZ	0x10FC
#define CALBC1_8MHZ	0x10FD
#define CALDCO_1MHZ	0x10FE
#define CALBC1_1MHZ	0x10FF

/* DCOCTL */
#define DCO2		0x80	/* DCO Select Bit 2 */
#define DCO1		0x40	/* DCO Select Bit 1 */
#define DCO0		0x20	/* DCO Select Bit 0 */
#define MOD4		0x10	/* Modulation Bit 4 */
#define MOD3		0x08	/* Modulation Bit 3 */
#define MOD2		0x04	/* Modulation Bit 2 */
#define MOD1		0x02	/* Modulation Bit 1 */
#define MOD0		0x01	/* Modulation Bit 0 */

/* BCSCTL1 */
#define XT2OFF		0x80	/* Enable XT2CLK */
#define XTS		0x40	/* LFXTCLK 0: Low Freq. / 1: High Freq. */
#define DIVA1		0x20	/* ACLK Divider 1 */
#define DIVA0		0x10	/* ACLK Divider 0 */
#define XT5V		0x08	/* XT5V should be always be reset */
#define RSEL3		0x08	/* Range Select Bit 3 */
#define RSEL2		0x04	/* Range Select Bit 2 */
#define RSEL1		0x02	/* Range Select Bit 1 */
#define RSEL0		0x01	/* Range Select Bit 0 */
#define DIVA_0		0x00	/* ACLK Divider 0: /1 */
#define DIVA_1		0x10	/* ACLK Divider 1: /2 */
#define DIVA_2		0x20	/* ACLK Divider 2: /4 */
#define DIVA_3		0x30	/* ACLK Divider 3: /8 */

/* BCSCTL2 */
#define SELM1		0x80	/* MCLK Source Select 1 */
#define SELM0		0x40	/* MCLK Source Select 0 */
#define DIVM1		0x20	/* MCLK Divider 1 */
#define DIVM0		0x10	/* MCLK Divider 0 */
#define SELS		0x08	/* SMCLK Source Select 0:DCOCLK / 1:XT2CLK/LFXTCLK */
#define DIVS1		0x04	/* SMCLK Divider 1 */
#define DIVS0		0x02	/* SMCLK Divider 0 */
#define DCOR		0x01	/* Enable External Register : 1 */
#define DIVS_0		0x00	/* SMCLK Divider 0: /1 */
#define DIVS_1		0x02	/* SMCLK Divider 1: /2 */
#define DIVS_2		0x04	/* SMCLK Divider 2: /4 */
#define DIVS_3		0x06	/* SMCLK Divider 3: /8 */
#define DIVM_0		0x00	/* MCLK Divider 0: /1 */
#define DIVM_1		0x10	/* MCLK Divider 1: /2 */
#define DIVM_2		0x20	/* MCLK Divider 2: /4 */
#define DIVM_3		0x30	/* MCLK Divider 3: /8 */
#define SELM_0		0x00	/* MCLK Source Select 0: DCOCLK */
#define SELM_1		0x40	/* MCLK Source Select 1: DCOCLK */
#define SELM_2		0x80	/* MCLK Source Select 2: XT2CLK/LFXTCLK */
#define SELM_3		0xc0	/* MCLK Source Select 3: LFXTCLK */

/* BCSCTL3 only for CLOCK_TYPE_BASIC_PLUS */
#define XT2S1		0x80	/* Mode 1 for XT2 */
#define XT2S0		0x40	/* Mode 0 for XT2 */
#define LFXT1S1		0x20	/* Mode 1 for LFXT1 (XTS = 0) */
#define LFXT1S0		0x10	/* Mode 0 for LFXT1 (XTS = 0) */
#define XCAP1		0x08	/* XIN/XOUT Cap 1 */
#define XCAP0		0x04	/* XIN/XOUT Cap 0 */
#define XT2OF		0x02	/* XT2 oscillator fault flag */
#define LFXT1OF		0x01	/* LFXT1 oscillator fault flag */
#define LFXT1S_0	0x00	/* Mode 0 for LFXT1 : Normal operation */
#define LFXT1S_1	0x10	/* Mode 1 for LFXT1 : Reserved */
#define LFXT1S_2	0x20	/* Mode 2 for LFXT1 : VLO */
#define LFXT1S_3	0x30	/* Mode 3 for LFXT1 : Digital input signal */
#define XT2S_0		0x00	/* Mode 0 for XT2 : 0.4 - 1 MHz */
#define XT2S_1		0x40	/* Mode 1 for XT2 : 1 - 4 MHz */
#define XT2S_2		0x80	/* Mode 2 for XT2 : 2 - 16 MHz */
#define XT2S_3		0xc0	/* Mode 3 for XT2 : Digital input signal */

typedef enum {
	CLOCK_TYPE_BASIC,
	CLOCK_TYPE_BASIC_PLUS,
} clock_type_t;

struct clock {
	struct simio_device	base;

	/* Config parameters */
	clock_type_t clock_type;
	int32_t lfxt1_hz;
	int32_t xt2_hz;
	int32_t vlo_hz;
	int32_t dco4_3_hz;
	int32_t dco7_3_hz;
	double srsel;
	double sdco;

	int32_t dco_hz;
	int32_t mclk_hz;
	int32_t smclk_hz;
	int32_t aclk_hz;

	int32_t aclk_counter;
	int32_t smclk_counter;

	/* Registers for Basic/Basic+ */
	uint8_t dcoctl;
	uint8_t bcsctl1;
	uint8_t bcsctl2;
	uint8_t bcsctl3;
};

#define TAG	"simio: clock: "

static struct simio_device *clock_create(char **arg_text)
{
	char *type_text = get_arg(arg_text);
	clock_type_t clock_type;

	if (!type_text) {
		printc_err(TAG "clock type required\n");
		return NULL;
	}

	if (!strcasecmp(type_text, "basic"))
		clock_type = CLOCK_TYPE_BASIC;
	else if (!strcasecmp(type_text, "basic+"))
		clock_type = CLOCK_TYPE_BASIC_PLUS;
	else {
		printc_err(TAG "unknown clock type: %s\n", type_text);
		return NULL;
	}

	struct clock *clk = malloc(sizeof(*clk));
	if (!clk) {
		pr_error(TAG "can't allocate memory");
		return NULL;
	}

	memset(clk, 0, sizeof(*clk));
	clk->base.type = &simio_clock;
	clk->clock_type = clock_type;
	if (clock_type == CLOCK_TYPE_BASIC) {
		clk->dco4_3_hz = 750000;
		clk->srsel = 1.65d;
		clk->sdco = 1.12d;
	}
	if (clock_type == CLOCK_TYPE_BASIC_PLUS) {
		clk->vlo_hz = 12000;
		clk->dco7_3_hz = 1140000;
		clk->srsel = 1.35d;
		clk->sdco = 1.08d;
	}

	return (struct simio_device *)clk;
}

static void clock_destroy(struct simio_device *dev)
{
	struct clock *clk = (struct clock *)dev;

	free(clk);
}

static int32_t calc_dcoclk(struct clock *clk, uint8_t dcoctl, uint8_t bcsctl1)
{
	double dcoclk;
	int d_rsel;
	int dco = (dcoctl & 0xe0) >> 5;
	int mod = (dcoctl & 0x1f);
	double next_dcoclk;

	if (clk->clock_type == CLOCK_TYPE_BASIC) {
		dcoclk = clk->dco4_3_hz;
		d_rsel = (bcsctl1 & 0x7) - 4;
	} else {
		dcoclk = clk->dco7_3_hz;
		d_rsel = (bcsctl1 & 0xf) - 7;
	}
	dcoclk *= pow(clk->srsel, d_rsel);
	dcoclk *= pow(clk->sdco, dco - 3);
	next_dcoclk = dcoclk * clk->sdco;
	return (int32_t)((32 * dcoclk * next_dcoclk) / (mod * dcoclk + (32 - mod) * next_dcoclk));
}

static void update_clock_frequencies(struct clock *clk)
{
	int32_t dcoclk, lfxtclk, mclk, smclk;

	dcoclk = calc_dcoclk(clk, clk->dcoctl, clk->bcsctl1);

	/* Select LFXTCLK */
	if (clk->clock_type == CLOCK_TYPE_BASIC_PLUS
	    && (clk->bcsctl3 & (LFXT1S1 | LFXT1S0)) == LFXT1S_2) {
		lfxtclk = clk->vlo_hz;
	} else {
		lfxtclk = clk->lfxt1_hz;
	}

	/* Select MCLK */
	switch (clk->bcsctl2 & (SELM1 | SELM0)) {
	default:
		mclk = dcoclk;
		break;
	case SELM_2:
		mclk = (clk->xt2_hz > 0) ? clk->xt2_hz : lfxtclk;
		break;
	case SELM_3:
		mclk = lfxtclk;
	}

	/* Select SMCLK */
	if (clk->bcsctl2 & SELS) {
		smclk = (clk->xt2_hz > 0) ? clk->xt2_hz : lfxtclk;
	} else {
		smclk = dcoclk;
	}

	clk->dco_hz = dcoclk;
	clk->mclk_hz = mclk >> ((clk->bcsctl2 & (DIVM1 | DIVM0)) >> 4);
	clk->smclk_hz = smclk >> ((clk->bcsctl2 & (DIVS1 | DIVS0)) >> 1);
	clk->aclk_hz = lfxtclk >> ((clk->bcsctl1 & (DIVA1 | DIVA0)) >> 4);
}

static void clock_reset(struct simio_device *dev)
{
	struct clock *clk = (struct clock *)dev;

	clk->aclk_counter = 0;
	clk->smclk_counter = 0;

	clk->dcoctl = 0x60;
	clk->bcsctl2 = 0x00;
	if (clk->clock_type == CLOCK_TYPE_BASIC)
		clk->bcsctl1 = 0x84;
	if (clk->clock_type == CLOCK_TYPE_BASIC_PLUS) {
		clk->bcsctl1 = 0x87;
		clk->bcsctl3 = 0x03;
	}

	update_clock_frequencies(clk);
}

static uint32_t multiply_power_of_10(uint32_t val, int pow) {
	while (pow > 0) {
		val *= 10;
		pow--;
	}
	while (pow < 0) {
		val /= 10;
		pow++;
	}
	return val;
}

static int config_frequency(struct clock *clk, int32_t *hz, char **arg_text)
{
	const char *freq_text = get_arg(arg_text);
	const char *p;
	int32_t freq = 0;
	int frac = -1;

	if (!freq_text) {
		printc_err(TAG "config: expected frequency\n");
		return -1;
	}

	for (p = freq_text; *p; p++) {
		if (isdigit(*p)) {
			freq *= 10;
			freq += *p - '0';
			if (frac >= 0) frac++;
		} else if (*p == '.' && frac < 0) {
			frac = 0;
		} else {
			break;
		}
	}
	if (frac < 0) frac = 0;
	if (*p == 0 || !strcasecmp(p, "Hz")) {
		frac = 0 - frac;
	} else if (!strcasecmp(p, "kHz")) {
		frac = 3 - frac;
	} else if (!strcasecmp(p, "MHz")) {
		frac = 6 - frac;
	} else {
		printc_err(TAG "config: illegal frequency: %s\n", freq_text);
		return -1;
	}
	freq = multiply_power_of_10(freq, frac);

	*hz = freq;
	update_clock_frequencies(clk);
	return 0;
}

static int config_ratio(struct clock *clk, double *data, char **arg_text)
{
	const char *double_text = get_arg(arg_text);
	char *next;
	double value;

	if (!double_text) {
		printc_err(TAG "config: expected floating point value\n");
		return -1;
	}

	value = strtod(double_text, &next);
	if (*next != 0 || errno == ERANGE) {
		printc_err(TAG "config: illegal value\n");
		return -1;
	}
	if (value <= 1.0d) {
		printc_err(TAG "config: must be greater than 1: %g\n", value);
		return -1;
	}
	if (value >= 1.8d) {
		printc_err(TAG "config: must be less than 1.8: %g\n", value);
		return -1;
	}

	*data = value;
	update_clock_frequencies(clk);
	return 0;
}

static int clock_config(struct simio_device *dev,
			const char *param, char **arg_text)
{
	struct clock *clk = (struct clock *)dev;

	if (!strcasecmp(param, "lfxt1"))
		return config_frequency(clk, &clk->lfxt1_hz, arg_text);
	if (!strcasecmp(param, "xt2"))
		return config_frequency(clk, &clk->xt2_hz, arg_text);
	if (clk->clock_type == CLOCK_TYPE_BASIC) {
		if (!strcasecmp(param, "dco4_3"))
			return config_frequency(clk, &clk->dco4_3_hz, arg_text);
	}
	if (clk->clock_type == CLOCK_TYPE_BASIC_PLUS) {
		if (!strcasecmp(param, "vlo"))
			return config_frequency(clk, &clk->vlo_hz, arg_text);
		if (!strcasecmp(param, "dco7_3"))
			return config_frequency(clk, &clk->dco7_3_hz, arg_text);
	}
	if (!strcasecmp(param, "srsel"))
		return config_ratio(clk, &clk->srsel, arg_text);
	if (!strcasecmp(param, "sdco"))
		return config_ratio(clk, &clk->sdco, arg_text);

	printc_err(TAG "config: unknown parameter: %s\n", param);
	return -1;
}

static void reverse(char *start, char * end)
{
	for (end--; start < end; start++, end--) {
		const char c = *start;
		*start = *end;
		*end = c;
	}
}

static char *suppress_suffix_zeroes(char *buf, char *end)
{
	while (end > buf && *--end == '0')
		;
	if (*end == '.') end--;
	return ++end;
}

static const char *to_frequency(uint32_t hz, char *const buf, size_t len)
{
	char *p = buf;
	int digit = 0;

	len -= 4; // room for unit and end of string..
	do {
		if (digit && digit % 3 == 0 && hz < 1000)
			*p++ = '.';
		*p++ = hz % 10 + '0';
		hz /= 10;
		digit++;
	} while (hz > 0 && p < buf + len);
	*p = 0;
	reverse(buf, p);
	if (digit > 3)
		p = suppress_suffix_zeroes(buf, p);
	if (digit > 6)
		*p++ = 'M';
	else if (digit > 3)
		*p++ = 'k';
	strcpy(p, "Hz");

	return buf;
}

static int clock_info(struct simio_device *dev)
{
	struct clock *clk = (struct clock *)dev;
	const char *clock_type;
	char buf[20];

	if (clk->clock_type == CLOCK_TYPE_BASIC)
		clock_type = "Basic";
	if (clk->clock_type == CLOCK_TYPE_BASIC_PLUS)
		clock_type = "Basic+";

	printc("Clock type: %s\n", clock_type);
	printc("LFXT1:	    %s\n", clk->lfxt1_hz == 0
	       ? "(no connection)"
	       : to_frequency(clk->lfxt1_hz, buf, sizeof(buf)));
	printc("XT2:	    %s\n", clk->xt2_hz == 0
	       ? "(no connection)"
	       : to_frequency(clk->xt2_hz, buf, sizeof(buf)));
	if (clk->clock_type == CLOCK_TYPE_BASIC) {
		printc("DCO4_3:	    %s\n",
		       to_frequency(clk->dco4_3_hz, buf, sizeof(buf)));
	}
	if (clk->clock_type == CLOCK_TYPE_BASIC_PLUS) {
		printc("VLO:	    %s\n", clk->vlo_hz == 0
		       ? "(unconfigured)"
		       : to_frequency(clk->vlo_hz, buf, sizeof(buf)));
		printc("DCO7_3:	    %s\n",
		       to_frequency(clk->dco7_3_hz, buf, sizeof(buf)));
	}
	printc("Step RSEL:  %g\n", clk->srsel);
	printc("Step DCO:   %g\n", clk->sdco);
	printc("\n");

	printc("DCOCTL:	     %02x\n", clk->dcoctl);
	printc("BCSCTL1:     %02x\n", clk->bcsctl1);
	printc("BCSCTL2:     %02x\n", clk->bcsctl2);
	if (clk->clock_type == CLOCK_TYPE_BASIC_PLUS)
		printc("BCSCTL3:     %02x\n", clk->bcsctl3);
	printc("\n");

	printc("DCOCLK %s\n", to_frequency(clk->dco_hz, buf, sizeof(buf)));
	printc("MCLK   %s\n", to_frequency(clk->mclk_hz, buf, sizeof(buf)));
	printc("SMCLK  %s\n", to_frequency(clk->smclk_hz, buf, sizeof(buf)));
	printc("ACLK   %s\n", to_frequency(clk->aclk_hz, buf, sizeof(buf)));
	printc("ACLK counter:  %d\n", clk->aclk_counter);
	printc("SMCLK counter: %d\n", clk->smclk_counter);

	return 0;
}

static void check_crystal(uint32_t hz, uint32_t low, uint32_t high,
			  const char *name)
{
	if (hz < low || hz > high) {
		char buf_hz[20], buf_low[20], buf_high[20];
		to_frequency(hz, buf_hz, sizeof(buf_hz));
		to_frequency(low, buf_low, sizeof(buf_low));
		to_frequency(high, buf_high, sizeof(buf_high));
		printc_dbg(TAG "%sS: %s must be %s ~ %s, but %s\n",
			   name, name, buf_low, buf_high, buf_hz);
	}
}

static void bcsctl3_write(struct clock *clk, uint8_t data)
{
	char buf[20];

	to_frequency(clk->xt2_hz, buf, sizeof(buf));
	switch (data & (XT2S1 | XT2S0)) {
	case XT2S_0:
		check_crystal(clk->xt2_hz, 400000, 1000000, "XT2");
		break;
	case XT2S_1:
		check_crystal(clk->xt2_hz, 1000000, 4000000, "XT2");
		break;
	case XT2S_2:
		check_crystal(clk->xt2_hz, 2000000, 16000000, "XT2");
		break;
	case XT2S_3:
		printc_dbg(TAG "XT2S: Digital input not supported\n");
		break;
	}

	if (clk->bcsctl2 & XTS) {
		switch (data & (LFXT1S1 | LFXT1S0)) {
		case LFXT1S_0:
			check_crystal(clk->lfxt1_hz, 400000, 1000000, "LFXT1");
			break;
		case LFXT1S_1:
			check_crystal(clk->lfxt1_hz, 1000000, 3000000, "LFXT1");
			break;
		case LFXT1S_2:
			check_crystal(clk->lfxt1_hz, 3000000, 16000000, "LFXT1");
			break;
		case LFXT1S_3:
			printc_dbg(TAG "LFXT1S: Digital input not supported\n");
			break;
		}
	} else {
		switch (data & (LFXT1S1 | LFXT1S0)) {
		case LFXT1S_1:
			printc_dbg(TAG "LFXT1S: Reserved mode\n");
			printc_dbg(TAG "LFXT1S: Fallback to mode 0\n");
			break;
		case LFXT1S_2:
			if (clk->vlo_hz > 0)
				break;
			printc_dbg(TAG "LFXT1S: VLO mode without VLO set\n");
			break;
		case LFXT1S_3:
			printc_dbg(TAG "LFXT1S: Digital input mode\n");
			printc_dbg(TAG "LFXT1S: Fallback to mode 0\n");
		}
	}
}

static int clock_write_b(struct simio_device *dev,
			 address_t addr, uint8_t data)
{
	struct clock *clk = (struct clock *)dev;

	switch (addr) {
	case BCSCTL3:
		if (clk->clock_type == CLOCK_TYPE_BASIC_PLUS) {
			bcsctl3_write(clk, data);
			update_clock_frequencies(clk);
			return 0;
		}
		break;
	case DCOCTL:
		clk->dcoctl = data;
		update_clock_frequencies(clk);
		return 0;
	case BCSCTL1:
		clk->bcsctl1 = data;
		update_clock_frequencies(clk);
		return 0;
	case BCSCTL2:
		clk->bcsctl2 = data;
		update_clock_frequencies(clk);
		return 0;
	}

	return 1;
}

static uint8_t calc_calibrate_dco(struct clock *clk, address_t addr,
				  const int32_t target_dco)
{
	const uint8_t rsel_max = (clk->clock_type == CLOCK_TYPE_BASIC) ? 0x4: 0x8;
	uint16_t cal = (rsel_max << 8);
	uint16_t bit = cal;
	int32_t min_delta = INT32_MAX;
	uint16_t best_cal = cal;

	while (bit != 0) {
		int32_t delta = calc_dcoclk(clk, cal & 0xff, cal >> 8) - target_dco;
		if (abs(delta) < min_delta) {
			min_delta = abs(delta);
			best_cal = cal;
		}
		if (delta > 0)
			cal &= ~bit;
		cal |= (bit >>= 1);
	}

	return (addr % 2 == 0) ? best_cal : best_cal >> 8;
}

static int clock_read_b(struct simio_device *dev,
			address_t addr, uint8_t *data)
{
	struct clock *clk = (struct clock *)dev;

	switch (addr) {
	case DCOCTL:
		*data = clk->dcoctl;
		return 0;
	case BCSCTL1:
		*data = clk->bcsctl1;
		return 0;
	case BCSCTL2:
		*data = clk->bcsctl2;
		return 0;
	}

	if (clk->clock_type == CLOCK_TYPE_BASIC_PLUS) {
		switch (addr) {
		case BCSCTL3:
			*data = clk->bcsctl3;
			return 0;
		case CALDCO_16MHZ:
		case CALBC1_16MHZ:
			*data = calc_calibrate_dco(clk, addr, 16000000);
			return 0;
		case CALDCO_12MHZ:
		case CALBC1_12MHZ:
			*data = calc_calibrate_dco(clk, addr, 12000000);
			return 0;
		case CALDCO_8MHZ:
		case CALBC1_8MHZ:
			*data = calc_calibrate_dco(clk, addr, 8000000);
			return 0;
		case CALDCO_1MHZ:
		case CALBC1_1MHZ:
			*data = calc_calibrate_dco(clk, addr, 1000000);
			return 0;
		}
	}

	return 1;
}

static void clock_step(struct simio_device *dev,
		       uint16_t status, const int *clocks)
{
	struct clock *clk = (struct clock *)dev;
	uint32_t duration;

	duration = clocks[SIMIO_MCLK] * clk->mclk_hz;

	clk->aclk_counter += duration;
	clk->smclk_counter += duration;

	((int *)clocks)[SIMIO_ACLK] = clk->aclk_counter / clk->aclk_hz;
	((int *)clocks)[SIMIO_SMCLK] = clk->smclk_counter / clk->smclk_hz;

	clk->aclk_counter %= clk->aclk_hz;
	clk->smclk_counter %= clk->smclk_hz;
}

const struct simio_class simio_clock = {
	.name = "clock",
	.help =
	"This peripheral implements the clock system module.\n"
	"\n"
	"Constructor arguments: <basic|basic+>\n"
	"    Specify the type of clock system.\n"
	"\n"
	"Config arguments are:\n"
	"    lfxt1 <frequency>\n"
	"	 Specify LFXT1 crystal frequency\n"
	"    xt2: <frequency>\n"
	"	 Specify XT2 crystal frequency\n"
	"    srsel: <double>\n"
	"	 Frequency step between range RSEL and RSEL+1\n"
	"    sdco: <double>\n"
	"	 Frequency step between tap DCO and DCO+1\n"
	"Config arguments for basic clock are:\n"
	"    dco4_3: <frequency>\n"
	"	 DCO frequency after reset (RSEL:4, DCO:3)\n"
	"Config arguments for basic+ clock are:\n"
	"    vlo: <frequency>\n"
	"	 Specify VLO frequency\n"
	"    dco7_3: <frequency>\n"
	"	 DCO frequency after reset (RSEL:7, DCO:3)\n",

	.create			= clock_create,
	.destroy		= clock_destroy,
	.reset			= clock_reset,
	.config			= clock_config,
	.info			= clock_info,
	.write_b		= clock_write_b,
	.read_b			= clock_read_b,
//	.check_interrupt	= clock_check_interrupt,
//	.ack_interrupt		= clock_ack_interrupt,
	.step			= clock_step
};

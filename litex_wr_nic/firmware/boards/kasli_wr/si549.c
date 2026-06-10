/*
 * This file is part of LiteX-WR-NIC.
 *
 * Copyright (c) 2026 TechnoSystem
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Kasli v2.0 Si549 DCXO initial-frequency setup, run from WRPC firmware.
 *
 * Transport: the gateware exposes a wbgen-compatible GPIO (COR/SOR/DDR/PSR)
 * on the WR-core aux Wishbone at BASE_AUXWB, whose bits drive the bit-bang
 * lines of the two Si549DAC instances (see litex_wr_nic/gateware/wb_gpio.py).
 * We therefore reuse the stock wb_gpio + bb_i2c drivers unchanged.
 *
 * GPIO bit map (must match the Kasli gateware target):
 *   0: Main   SCL     1: Main   SDA     4: Main   fw_enable (bus owner select)
 *   2: Helper SCL     3: Helper SDA     5: Helper fw_enable
 * from C this is a plain "1 = line high / release, 0 = line low" GPIO, 
 * exactly what bb_i2c expects.
 *
 * Setting fw_enable hands the corresponding I2C bus from the gateware ADPLL
 * programmer to firmware for the duration of the sequence; clearing it hands
 * the bus back so runtime SoftPLL DAC strobes resume disciplining the chip.
 * The host CSR bit-bang path (test_si549_setup.py) still takes priority over
 * this firmware path inside Si549DAC, so it remains usable as a fallback.
 */

#include <stdint.h>

#include "board.h"
#include "dev/gpio.h"
#include "dev/bb_i2c.h"
#include "dev/syscon.h"
#include "pp-printf.h"
#include "si549.h"

#define SI549_I2C_ADDR    0x67

#define SI549_REG_FCAL      7    /* bit3 = MS_ICAL2 (start FCAL)              */
#define SI549_REG_OE        17   /* bit0 = ODC_OE                            */
#define SI549_REG_HSDIV_LO  23
#define SI549_REG_HSDIV_HI  24   /* [6:4] = LSDIV[2:0], [2:0] = HSDIV[10:8]  */
#define SI549_REG_FBDIV_0   26   /* FBDIV[7:0], then 27..31 sequentially     */
#define SI549_REG_FCAL_OVR  69
#define SI549_REG_PAGE      255

/*
 * Per-target divider profiles
 *
 * Main (125 MHz, LSDIV=0):
 *     FBDIV_real = 125e6 * 88 * 1 / 152.6e6 = 72.083887
 *     Fvco       = 152.6 MHz * 72.083887    = 11000.0 MHz   (in range)
 *     reg        = round(72.083887 * 2^32)  = 0x048_15791F25   (0x048 = 72)
 *
 * Helper (62.498092 MHz, LSDIV=1):
 *     The DDMTD helper must run a hair off the main so the DDMTD beat note is a
 *     low, measurable frequency -> standard (N-1)/N offset:
 *         Fhelper  = 62.5 MHz * 32767/32768 = 62.498092 MHz
 *     FBDIV_real = 62.498092e6 * 88 * 2 / 152.6e6 = 72.081645
 *     reg        = round(72.081645 * 2^32)        = 0x048_14E8F442
 * Both share HSDIV=88 and Fvco~11 GHz; only LSDIV/FBDIV differ.
 *
 * NOTE -- this firmware sets only the NOMINAL FREQUENCY. The SoftPLL +/- pull
 * range is a separate quantity set by the gateware `adpll_scale` (the DAC->ADPLL
 * gain), NOT here: once init hands the bus back, the gateware ADPLLProgrammer
 * disciplines the chip via ADPLL reg 231, with
 *     delta_f_ppm = (dac - 0x8000) * adpll_scale/256 * 0.0001164
 * defaulting to +/-100 ppm. See adpll_scale_for_ppm() in
 * litex_wr_nic/gateware/si549/core.py for that derivation.
 */
#define MAIN_HSDIV     0x058              /* 88 */
#define MAIN_LSDIV     0                  /* /1 */
#define MAIN_FBDIV     0x04815791F25ULL   /* 72.083887 -> 125.000000 MHz */

#define HELPER_HSDIV   0x058              /* 88 */
#define HELPER_LSDIV   1                  /* /2 */
#define HELPER_FBDIV   0x04814E8F442ULL   /* 72.081645 -> 62.498092 MHz (62.5*32767/32768) */

/* Si549 max settling time on a large frequency change */
#define SI549_FCAL_MS  30
#define SI549_SETTLE_MS 40

/* GPIO device on the aux Wishbone + per-line pins (see bit map above). */
static struct gpio_device   si549_gpio;
static const struct gpio_pin pin_main_scl   = { &si549_gpio, 0 };
static const struct gpio_pin pin_main_sda   = { &si549_gpio, 1 };
static const struct gpio_pin pin_helper_scl = { &si549_gpio, 2 };
static const struct gpio_pin pin_helper_sda = { &si549_gpio, 3 };
static const struct gpio_pin pin_main_en    = { &si549_gpio, 4 };
static const struct gpio_pin pin_helper_en  = { &si549_gpio, 5 };

static struct i2c_bus bus_main;
static struct i2c_bus bus_helper;

/*
 * si549_reg_write - single-register write: START | addr<<1 | reg | val | STOP.
 * Returns 0 if every byte was ACKed, -1 otherwise. Unlike the host script we
 * can ACK-check every byte: firmware I2C has none of the ~17 ms UARTBone read
 * latency that trips the Si549 watchdog, so there is no reason not to.
 */
static int si549_reg_write(const struct i2c_bus *bus, uint8_t reg, uint8_t val)
{
	int ack = 0;

	bb_i2c_start(bus);
	ack |= bb_i2c_put_byte(bus, SI549_I2C_ADDR << 1);
	ack |= bb_i2c_put_byte(bus, reg);
	ack |= bb_i2c_put_byte(bus, val);
	bb_i2c_stop(bus);

	return ack == 0 ? 0 : -1;
}

static int si549_setup(const char *name, const struct i2c_bus *bus,
		       const struct gpio_pin *en,
		       uint16_t hsdiv, uint8_t lsdiv, uint64_t fbdiv)
{
	int err = 0;
	int i;

	/*
	 * Pre-set SCL/SDA high on the (currently programmer-owned) lines, then
	 * take ownership: with the output register already high, flipping
	 * fw_enable causes no glitch on either line.
	 */
	bb_i2c_init(bus);            /* SCL = 1, SDA = 1 (release) */
	gen_gpio_out(en, 1);         /* firmware now owns this bus */
	bb_i2c_stop(bus);            /* clean idle (STOP) */

	if (!bb_i2c_devprobe(bus, SI549_I2C_ADDR)) {
		pp_printf("[si549] %s: no ACK at 0x%02x\n", name, SI549_I2C_ADDR);
		gen_gpio_out(en, 0);
		return -1;
	}

	/* 5.5 prelude (Table 5.6): must precede any divider write. */
	err |= si549_reg_write(bus, SI549_REG_PAGE,     0x00);
	err |= si549_reg_write(bus, SI549_REG_FCAL_OVR, 0x00);
	err |= si549_reg_write(bus, SI549_REG_OE,       0x00);

	/* Dividers. */
	err |= si549_reg_write(bus, SI549_REG_HSDIV_LO, hsdiv & 0xff);
	err |= si549_reg_write(bus, SI549_REG_HSDIV_HI,
			       ((lsdiv & 0x7) << 4) | ((hsdiv >> 8) & 0x7));
	for (i = 0; i < 6; i++)
		err |= si549_reg_write(bus, SI549_REG_FBDIV_0 + i,
				       (uint8_t)((fbdiv >> (8 * i)) & 0xff));

	/* Start FCAL, wait for VCO calibration, re-enable output, settle. */
	err |= si549_reg_write(bus, SI549_REG_FCAL, 0x08);
	timer_delay_ms(SI549_FCAL_MS);
	err |= si549_reg_write(bus, SI549_REG_OE,   0x01);
	timer_delay_ms(SI549_SETTLE_MS);

	/* Hand the bus back to the gateware ADPLL programmer. */
	gen_gpio_out(en, 0);

	if (err)
		pp_printf("[si549] %s: NACK(s) during setup\n", name);
	else
		pp_printf("[si549] %s: configured (HSDIV=%u LSDIV=%u)\n",
			  name, (unsigned)hsdiv, (unsigned)lsdiv);

	return err;
}

int kasli_si549_init(void)
{
	int r_main, r_helper;

	wb_gpio_create(&si549_gpio, BASE_AUXWB);
	bb_i2c_create(&bus_main,   &pin_main_scl,   &pin_main_sda);
	bb_i2c_create(&bus_helper, &pin_helper_scl, &pin_helper_sda);

	r_main   = si549_setup("Main",   &bus_main,   &pin_main_en,
			       MAIN_HSDIV,   MAIN_LSDIV,   MAIN_FBDIV);
	r_helper = si549_setup("Helper", &bus_helper, &pin_helper_en,
			       HELPER_HSDIV, HELPER_LSDIV, HELPER_FBDIV);

	return (r_main || r_helper) ? -1 : 0;
}

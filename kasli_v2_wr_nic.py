#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2025 Warsaw University of Technology
# SPDX-License-Identifier: BSD-2-Clause

import os
import argparse

from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex_boards.platforms import sinara_kasli

from litex.build.generic_platform import *
from litex.build.io               import DifferentialInput

from litex.soc.interconnect.csr     import *
from litex.soc.interconnect         import wishbone

from litex.soc.integration.soc_core import *
from litex.soc.integration.builder  import *

from litex.soc.cores.clock import S7PLL

from litex_wr_nic.gateware.uart    import UARTShared
from litex_wr_nic.gateware.soc     import LiteXWRNICSoC
from litex_wr_nic.gateware.qpll    import SharedQPLL
from litex_wr_nic.gateware.measurement import MultiClkMeasurement
from litex_wr_nic.gateware.nic.phy import LiteEthPHYWRGMII
from litex_wr_nic.gateware.si549.core import Si549DAC

# Platform extensions ------------------------------------------------------------------------------

_kasli_v2_wr_extensions = [
    # Board I2C bus — used by WRPC as the SFP I2C bus.
    # The firmware must configure the PCA9548 mux at 0x70/0x71 to route to the
    # desired SFP channel before any SFP EEPROM access (see board BSP).
    ("sfp_i2c", 0,
        Subsignal("scl", Pins("J16")),
        Subsignal("sda", Pins("M17")),
        IOStandard("LVCMOS25")),

    # SPI Flash — for WRPC SDBFS and firmware storage.
    ("flash", 0,
        Subsignal("cs_n", Pins("T19")),
        Subsignal("mosi", Pins("P22")),
        Subsignal("miso", Pins("R22")),
        Subsignal("wp",   Pins("P21")),
        Subsignal("hold", Pins("R21")),
        IOStandard("LVCMOS25")),
]

# CRG ----------------------------------------------------------------------------------------------

class _CRG(LiteXModule):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()

        # Clock domains.
        self.cd_sys           = ClockDomain()
        self.cd_refclk_pcie   = ClockDomain()  # Dummy QPLL0 refclk (PCIe absent)
        self.cd_refclk_eth    = ClockDomain()  # WR GTP refclk (CDR clean, tunable)
        self.cd_clk_125m_gtp  = ClockDomain()  # WR GTP refclk (same signal), required by the LiteXWRNICSoC
        self.cd_clk_62m5_dmtd = ClockDomain()  # DDMTD helper clock (Helper Si549)
        self.cd_clk10m_in     = ClockDomain()  # 10MHz ext ref (unused, tied to 0), needed to satisfy LiteXWRNICSoC

        # # #

        # Sys PLL: free-running from clk125_gtp (F10/E10, fixed 125MHz oscillator).
        # IBUFDS_GTE2 ODIV2 gives 62.5MHz to the PLL.
        clk125_gtp   = platform.request("clk125_gtp")
        platform.add_period_constraint(clk125_gtp.p, 1e9/125e6)

        # Only one of the IBUFDS_GTE2’s O or ODIV2 outputs can be routed to the FPGA logic
        clk125_div2  = Signal()
        self.specials += Instance("IBUFDS_GTE2",
            i_CEB   = 0,
            i_I     = clk125_gtp.p,
            i_IB    = clk125_gtp.n,
            o_ODIV2 = clk125_div2,
        )
        self.pll = pll = S7PLL(speedgrade=-3)
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk125_div2, 62.5e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq, margin=0)

        # WR GTP reference clock: CDR-cleaned Main Si549 output (F6/E6, 125MHz).
        # This clock is disciplined to the WR master by the SoftPLL + Main Si549.
        cdr_clk_clean = platform.request("cdr_clk_clean")
        platform.add_period_constraint(cdr_clk_clean.p, 1e9/125e6)

        cdr_clk_se    = Signal()
        self.specials += Instance("IBUFDS_GTE2",
            i_CEB = 0,
            i_I   = cdr_clk_clean.p,
            i_IB  = cdr_clk_clean.n,
            o_O   = cdr_clk_se,
        )
        self.comb += [
            self.cd_clk_125m_gtp.clk.eq(cdr_clk_se),
            self.cd_refclk_eth.clk.eq(cdr_clk_se),
            # We're not allowed to drive the PCIe dummy clock from the cdr_clk_se,
            # as this would effectively mean that we want to drive both GTREFCLK0
            # and GTREFCLK1 from the same MGTREFPins - and with current QPLL settings
            # it causes clock fanout error:
            # ERROR: [DRC RTSTAT-2] Partially routed nets: 1 net(s) are partially routed. The problem bus(es) and/or net(s) are clk_125m_gtp_clk.
        ]

        # DDMTD helper clock: direct output of Helper Si549 (W19/W20, ~62.5MHz).
        helper_clk_pads = platform.request("ddmtd_helper_clk")
        self.specials += Instance("IBUFGDS",
            p_DIFF_TERM   = "TRUE",
            p_IBUF_LOW_PWR = "FALSE",
            i_I  = helper_clk_pads.p,
            i_IB = helper_clk_pads.n,
            o_O  = self.cd_clk_62m5_dmtd.clk,
        )
        platform.add_period_constraint(helper_clk_pads.p, 1e9/62.5e6)

# BaseSoC ------------------------------------------------------------------------------------------

class BaseSoC(LiteXWRNICSoC):
    def __init__(self, sys_clk_freq=125e6,
        # White Rabbit Parameters.
        with_white_rabbit         = True,
        white_rabbit_sfp_connector = 0,
        white_rabbit_cpu_firmware  = "litex_wr_nic/firmware/kasli_v2_wrc.bram",
    ):
        # Platform ---------------------------------------------------------------------------------

        platform      = sinara_kasli.Platform(hw_rev="v2.0")
        platform.add_extension(_kasli_v2_wr_extensions)
        platform.name = "kasli_v2_wr_nic"

        # Clocking ---------------------------------------------------------------------------------

        self.crg = _CRG(platform, sys_clk_freq)

        # Shared QPLL.
        # with_pcie=True is required so that the Ethernet/WR channel maps to QPLL1,
        # matching the WRPC platform (g_gtp_enable_pll1='1').
        self.qpll = SharedQPLL(platform,
            with_pcie           = True,   # Forces ETH -> QPLL1; PCIe QPLL0 unused
            with_eth            = with_white_rabbit,
            eth_refclk_freq     = 125e6,
            eth_refclk_from_pll = False,  # Use IBUFDS_GTE2 directly (cd_refclk_eth)
        )
        # downgrades Vivado DRC check REQP-49 from error to warning,
        # allowing a PLL-generated clock to drive QPLL refclk input, but we use 
        # cd_refclk_eth directly, so it should be safe to skip this.
        # self.qpll.enable_pll_refclk()

        # SoCMini ----------------------------------------------------------------------------------

        SoCMini.__init__(self, platform,
            clk_freq      = sys_clk_freq,
            ident         = "LiteX-WR-NIC on Kasli v2.0.",
            ident_version = True,
        )

        # UART -------------------------------------------------------------------------------------
        # UARTShared multiplexes the physical serial port between WRPC and LiteX.
        # Auto-mode selects based on last RX activity.
        # LiteX crossover UART is accessible via JTAGBone (litex_term crossover).

        self.uart = UARTShared(pads=platform.request("serial"), sys_clk_freq=sys_clk_freq)

        # JTAGBone ---------------------------------------------------------------------------------

        self.add_jtagbone()
        platform.add_period_constraint(self.jtagbone_phy.cd_jtag.clk, 1e9/20e6)
        platform.add_false_path_constraints(self.jtagbone_phy.cd_jtag.clk, self.crg.cd_sys.clk)

        # White Rabbit -----------------------------------------------------------------------------

        if with_white_rabbit:
            # White Rabbit Core.
            # ------------------
            self.add_wr_core(
                # CPU.
                cpu_firmware     = white_rabbit_cpu_firmware,

                # Board name (4 chars).
                # The WRPC firmware must have a BSP registered for "KS2W".
                board_name       = "KS2W",

                # SFP.
                # sfp_i2c uses the board I2C bus (J16/M17); WRPC firmware is
                # responsible for configuring the PCA9548 mux at 0x70/0x71
                # to route to SFP0 before any SFP EEPROM access.
                sfp_pads         = platform.request("sfp",     white_rabbit_sfp_connector),
                sfp_i2c_pads     = platform.request("sfp_i2c", 0),
                sfp_tx_polarity  = 0,
                sfp_rx_polarity  = 0,

                # QPLL.
                qpll             = self.qpll,
                with_ext_clk     = False,  # No 10MHz external reference on Kasli WR NIC

                # Serial.
                serial_pads      = self.uart.shared_pads,

                # Flash.
                flash_pads       = platform.request("flash", 0),

                # No 1-Wire temperature sensor on Kasli v2.0 WR NIC.
            )
            self.add_sources()

            # Si549 DAC Bridges.
            # ------------------
            # RefClk DAC: translates WRPC SoftPLL DPLL output to ADPLL writes on
            # the Main Si549. The Main Si549 output goes through Si5324 CDR and
            # then to the GTP reference clock (F6/E6, Y18/Y19).
            self.refclk_dac = Si549DAC(
                pads          = platform.request("ddmtd_main_dcxo_i2c"),
                load          = self.dac_refclk_load,
                value         = self.dac_refclk_data,
                sys_clk_freq  = sys_clk_freq,
            )

            # DMTD DAC: translates WRPC SoftPLL HPLL output to ADPLL writes on
            # the Helper Si549. The Helper Si549 output goes directly to fabric
            # as the DDMTD helper clock (W19/W20).
            self.dmtd_dac = Si549DAC(
                pads          = platform.request("ddmtd_helper_dcxo_i2c"),
                load          = self.dac_dmtd_load,
                value         = self.dac_dmtd_data,
                sys_clk_freq  = sys_clk_freq,
            )

            # Timing Constraints.
            # -------------------
            platform.add_platform_command("create_clock -name wr_txoutclk -period 16.000 [get_pins -hierarchical *gtpe2_i/TXOUTCLK]")
            platform.add_platform_command("create_clock -name wr_rxoutclk -period 16.000 [get_pins -hierarchical *gtpe2_i/RXOUTCLK]")

            # White Rabbit Ethernet PHY (over White Rabbit Fabric).
            # ------------------------------------------------------
            self.ethphy0 = LiteEthPHYWRGMII(
                wrf_stream2wb = self.wrf_stream2wb,
                wrf_wb2stream = self.wrf_wb2stream,
            )

            # Leds.
            # -----
            self.comb += [
                platform.request("user_led", 0).eq(~self.led_link),
                platform.request("user_led", 1).eq(~self.led_act),
                platform.request("user_led", 2).eq(~self.led_pps),
            ]

        # Timing Constraints -----------------------------------------------------------------------

        asynchronous_clk_domains = [
            self.crg.cd_sys.clk,
            self.crg.cd_clk_62m5_dmtd.clk,
            self.crg.cd_clk_125m_gtp.clk,
            "wr_txoutclk",
            "wr_rxoutclk",
        ]
        platform.add_false_path_constraints(*asynchronous_clk_domains)

        # Clk Measurement (Debug) ------------------------------------------------------------------

        self.clk_measurement = MultiClkMeasurement(clks={
            "clk0" : ClockSignal("sys"),
            "clk1" : ClockSignal("clk_62m5_dmtd"),
            "clk2" : ClockSignal("clk_125m_gtp"),
        })

# Build --------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX-WR-NIC on Kasli v2.0.")

    # Build/Load/Flash Arguments.
    parser.add_argument("--build", action="store_true", help="Build bitstream.")
    parser.add_argument("--load",  action="store_true", help="Load bitstream.")
    parser.add_argument("--flash", action="store_true", help="Flash bitstream.")

    # WR SFP connector (0-3).
    parser.add_argument("--sfp-connector", default=0, type=int, choices=[0, 1, 2, 3],
        help="SFP connector index for White Rabbit (default: 0).")

    # Probes.
    parser.add_argument("--with-wishbone-fabric-interface-probe", action="store_true")
    parser.add_argument("--with-wishbone-slave-probe",            action="store_true")
    parser.add_argument("--with-dac-vcxo-probe",                  action="store_true")

    args = parser.parse_args()

    # Build Firmware.
    if args.build:
        print("Building firmware...")
        r = os.system("cd litex_wr_nic/firmware && ./build.py --target kasli_v2")
        if r != 0:
            raise RuntimeError("Firmware build failed.")

    # Build SoC/Gateware.
    soc = BaseSoC(
        white_rabbit_sfp_connector = args.sfp_connector,
    )
    if args.with_wishbone_fabric_interface_probe:
        soc.add_wishbone_fabric_interface_probe()
    if args.with_wishbone_slave_probe:
        soc.add_wishbone_slave_probe()
    if args.with_dac_vcxo_probe:
        soc.add_dac_vcxo_probe()

    builder = Builder(soc, csr_csv="test/csr.csv")
    builder.build(run=args.build)

    # Load FPGA.
    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(builder.get_bitstream_filename(mode="sram"))

    # Flash FPGA.
    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0x0000_0000, builder.get_bitstream_filename(mode="flash"))
        prog.flash(0x002e_0000, "litex_wr_nic/firmware/sdb-wrpc.bin")

if __name__ == "__main__":
    main()

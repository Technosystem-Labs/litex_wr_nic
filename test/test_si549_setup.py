#!/usr/bin/env python3
#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2026 TechnoSystem
# SPDX-License-Identifier: BSD-2-Clause
#
# Si549 initial-frequency setup over UARTBone.
#
# Drives the bit-bang CSRs of the two Si549DAC instances on Kasli v2 to issue
# the frequency-update sequence.
# Volatile only -- nothing is committed to NVM. Must be re-run after every
# Si549 power cycle. Run BEFORE WRPC firmware starts touching the ADPLL
# (i.e. before/just after CPU reset is released), otherwise the gateware
# ADPLL programmer and this script will fight for the bus.
#
# CSR namespace:
#   refclk_dac_*  -- Main Si549   (ddmtd_main_dcxo_i2c)
#   dmtd_dac_*    -- Helper Si549 (ddmtd_helper_dcxo_i2c)
#
# Both Si549s are addressed at I2C 0x67 on their own dedicated bus.
#
# Default target frequencies:
#   Main:   125 MHz nominal      (drives GTP MGTREFCLK0 via ADCLK948)
#   Helper: 62.498 MHz nominal   (DDMTD helper clock; 62.5 MHz * 32767/32768)
#
# Fxtal = 152.6 MHz, Fvco = 11000 MHz, HSDIV = 88 for both. Main uses
# LSDIV=/1, Helper uses LSDIV=/2 with a slightly lower FBDIV (the standard
# DDMTD (N-1)/N twist; see ARTIQ wrpll/si549.rs for the same pattern).
#
# Usage:
#   litex_server --uart --uart-port /dev/ttyUSB1 --uart-baudrate 115200 &
#   ./test_si549_setup.py                # main 125 MHz, helper 62.498 MHz
#   ./test_si549_setup.py --only main    # only Main Si549
#   ./test_si549_setup.py --only helper  # only Helper Si549
#   ./test_si549_setup.py --main-fbdiv 0x... --main-lsdiv 1   # custom main
#

import argparse
import sys
import time

from litex import RemoteClient

# urv CPU debug-port (matches test/test_cpu.py). Holding the CPU in reset
# is NOT about I2C access (the CPU has no path to the DDMTD I2C buses);
# it's about preventing WRPC firmware from running while the 5.5
# sequence squelches the Si549 output for ~30 ms (large-frequency-change
# settling, datasheet Table 2.2). If WRPC is alive during that dropout,
# its softpll / clock-monitor / PHY state machines latch undefined state
# and never recover -- symptom: `freqmon checkvco` hangs in
# measure_vcxo_freq waiting for a measurement that never becomes valid.
CPU_DBG_OFFSET = 0x20b00
CPU_RST_REG    = 0x0


def cpu_reset(bus, hold):
    addr = bus.mems.wr_wb_slave.base + CPU_DBG_OFFSET + CPU_RST_REG
    bus.write(addr, 1 if hold else 0)

# Si549 register addresses (datasheet 5.7) ---------------------------------------------------------
REG_PAGE        = 255
REG_FCAL_OVR    = 69
REG_OE          = 17     # bit0 = ODC_OE
REG_FCAL        = 7      # bit3 = MS_ICAL2 (start FCAL)
REG_HSDIV_LO    = 23
REG_HSDIV_HI    = 24     # [6:4]=LSDIV[2:0], [2:0]=HSDIV[10:8]
REG_FBDIV_0     = 26     # FBDIV[7:0]   (then 27..31 sequentially)

SI549_ADDR    = 0x67  # default; overridable via --address

# Per-target divider profiles. Fxtal = 152.6 MHz, Fvco = 11000 MHz, HSDIV = 88.
#
# Main:   125 MHz nominal -> drives GTP MGTREFCLK0 via ADCLK948 fanout.
# Helper: 62.5 MHz * 32767/32768 = 62.498092 MHz nominal -> direct fabric
#         clock for the DDMTD helper. The (N-1)/N offset matches ARTIQ's
#         WRPLL helper DCXO configuration; the WRPC SoftPLL HPLL only
#         needs a tiny ADPLL pull to lock from there.
#
# FBDIV is 11.32 fixed-point; same Fvco for both, only LSDIV/FBDIV differ.
MAIN_HSDIV   = 0x058         # 88
MAIN_LSDIV   = 0             # /1
MAIN_FBDIV   = 0x04815791F25 # 125 MHz @ Fxtal=152.6 MHz

HELPER_HSDIV = 0x058         # 88
HELPER_LSDIV = 1             # /2
HELPER_FBDIV = 0x04814E8F442 # 125 MHz * 32767/32768 (then /2 via LSDIV -> 62.498 MHz)


# ---- Bit-bang primitives ---------------------------------------------------------------------


class BBus:
    """Wraps the bit-bang CSRs of one Si549DAC instance."""
    def __init__(self, bus, prefix, address=SI549_ADDR):
        r = bus.regs
        self.address = address
        self.bitbang = getattr(r, prefix + "_bitbang_enable")
        self.sda_oe  = getattr(r, prefix + "_sda_oe")
        self.sda_out = getattr(r, prefix + "_sda_out")
        self.sda_in  = getattr(r, prefix + "_sda_in")
        self.scl_oe  = getattr(r, prefix + "_scl_oe")
        self.scl_out = getattr(r, prefix + "_scl_out")

    # ARTIQ i2c::init equivalent ------------------------------------------
    def acquire(self):
        # CRITICAL: cycle bitbang_enable through 0 first. If a previous run
        # crashed mid-transaction, bitbang_enable is still 1 and the CSR
        # storage holds whatever line state the previous run left (e.g. SCL
        # low, SDA driven). Writing the new CSR values one at a time would
        # then walk the lines through bogus intermediate states (which the
        # chip can latch as spurious START/STOP edges) BEFORE the bus
        # recovery sequence even starts. By forcing bitbang=0 first, the
        # mux hands the lines back to the programmer (which sits in IDLE
        # with both lines released -> pulled high), so the subsequent CSR
        # writes are completely invisible to the chip until we flip
        # bitbang=1 atomically with everything already parked correctly.
        self.bitbang.write(0)
        # Park CSRs while the mux is ignoring them.
        self.scl_out.write(1)      # SCL parked high
        self.scl_oe.write(1)       # actively drive SCL (forever)
        self.sda_out.write(0)      # SDA parked low (open-drain)
        self.sda_oe.write(0)       # SDA released for now
        # Take over the bus. Lines transition: HiZ-high (pullup) -> driven
        # high (push-pull SCL) + HiZ (SDA). No edge on either line.
        self.bitbang.write(1)
        # ARTIQ-style init check: SDA should be high when idle. If something
        # is holding it low (chip mid-transaction from a prior crash), pulse
        # SCL up to 9 times until SDA goes free.
        if not self.sda_i():
            for _ in range(9):
                self.scl_out.write(0)
                self.scl_out.write(1)
                if self.sda_i():
                    break
            if not self.sda_i():
                raise IOError("SDA stuck low after 9 SCL recovery pulses")
        # Issue a clean STOP to force the chip back to true idle.
        self.scl_out.write(0)
        self.sda_oe.write(1)       # SDA driven low (while SCL low)
        self.scl_out.write(1)      # SCL high (SDA still low)
        self.sda_oe.write(0)       # SDA released -> STOP edge

    def release(self):
        # Hand the bus back to the gateware programmer cleanly: SDA released,
        # SCL released (via push-pull high then bitbang=0 -> mux hands to
        # programmer-in-IDLE -> HiZ -> pullup high). Avoids any falling edge
        # on either line that the chip could mis-interpret.
        self.sda_oe.write(0)
        self.scl_out.write(1)
        self.bitbang.write(0)

    # Low-level helpers, mirroring ARTIQ's scl_o / sda_oe / sda_i naming.
    # scl_o(True)/(False)  -- drive SCL high / low (push-pull, scl_oe is always 1)
    # sda_oe_set(True)     -- drive SDA low (open-drain, sda_out is always 0)
    # sda_oe_set(False)    -- release SDA (HiZ; pullup -> high)
    def scl_o(self, high):
        self.scl_out.write(1 if high else 0)

    def sda_oe_set(self, drive_low):
        self.sda_oe.write(1 if drive_low else 0)

    def sda_i(self):
        return self.sda_in.read() & 1

    # ARTIQ start() ------------------------------------------------------
    # Pre: SCL high, SDA released (idle). Post: SCL high, SDA driven low.
    def start(self):
        self.scl_o(True)            # ensure SCL high
        self.sda_oe_set(True)       # SDA pulled low while SCL high  =>  START

    # ARTIQ stop() -------------------------------------------------------
    # Pre: SCL low (typical end-of-transaction). Post: SCL high, SDA released.
    def stop(self):
        self.scl_o(False)           # SCL low (no-op if already)
        self.sda_oe_set(True)       # SDA low (so we can release it later)
        self.scl_o(True)            # SCL high while SDA low
        self.sda_oe_set(False)      # SDA released  =>  STOP

    # ARTIQ write() ------------------------------------------------------
    # Pre: SCL high (start() leaves SCL high; previous byte exited SCL high).
    # Post: depends on check_ack.
    #
    # IMPORTANT: when check_ack=True we read sda_in. UARTBone CSR writes are
    # non-blocking (~0.02 ms each, queued in litex_server) but a CSR read
    # forces a queue flush + round-trip and takes ~17 ms. During that 17 ms
    # SCL is frozen HIGH, and the Si549 evidently has an internal "no SCL
    # transition since START" watchdog -- holding SCL high for 17 ms after
    # the ACK clock makes the chip silently reset its I2C state machine, and
    # the NEXT byte then NACKs. So: read ACK only when strictly needed
    # (address byte, end-of-transaction). Intermediate ACK clocks pass with
    # check_ack=False -- SCL still toggles through the 9th clock so the chip
    # sees a valid byte boundary, but we don't pause to inspect SDA.
    def write_byte(self, value, check_ack=True):
        for bit in range(7, -1, -1):
            self.scl_o(False)
            self.sda_oe_set(((value >> bit) & 1) == 0)
            self.scl_o(True)
        # ACK slot.
        self.scl_o(False)
        self.sda_oe_set(False)
        self.scl_o(True)
        if check_ack:
            return self.sda_i() == 0
        return None

    # ARTIQ read() -------------------------------------------------------
    # Pre: SCL high (from preceding write_byte's ACK clock).
    # Post: SCL high, SDA released (NACK) or driven low (ACK from master).
    #
    # Same UARTBone-vs-watchdog trap as write_byte: each sda_i read freezes
    # SCL high for ~17 ms. Reading after every bit would make a single byte
    # take ~150 ms and trip the chip's timeout long before STOP. We instead
    # let SCL toggle through all 8 bits with NO reads, then SHIFT SCL LOW
    # IMMEDIATELY before issuing the bulk read. The CSR fabric mirrors the
    # transitions in order, so the chip-side trace is: 8 fast clocks + a low.
    # We then read sda_in once -- it returns the *current* line state, which
    # is the bit the chip drove during the LAST clock. To recover all 8 bits
    # we have to read after each clock anyway... so for a 1-byte read we
    # accept the slow path. This is only used for the identity probe, which
    # only reads a single byte once.
    def read_byte(self, ack):
        self.scl_o(False)
        self.sda_oe_set(False)
        data = 0
        for bit in range(7, -1, -1):
            self.scl_o(False)
            self.scl_o(True)
            if self.sda_i():
                data |= 1 << bit
        self.scl_o(False)
        if ack:
            self.sda_oe_set(True)
        self.scl_o(True)
        return data

    # High-level transactions ---------------------------------------------
    def probe(self, addr):
        """Address-only probe: START, write addr<<1, STOP. Returns True if ACKed."""
        self.start()
        ack = self.write_byte(addr << 1)
        self.stop()
        return ack

    def reg_write(self, reg, val):
        # NO ACK checks at all. Every sda_in.read() forces a UARTBone queue
        # flush and keeps SCL frozen HIGH for ~17 ms, which is long enough
        # to trip the Si549's I2C watchdog and make the chip silently drop
        # the rest of the transaction. Symptom: early writes (PAGE/FCAL_OVR/
        # OE=0) land, later writes (dividers/FCAL/OE=1) get swallowed, the
        # output stays disabled because OE=0 was applied but OE=1 wasn't.
        # Caller must verify chip presence with probe() before/after the
        # configuration sequence.
        self.start()
        self.write_byte(self.address << 1, check_ack=False)
        self.write_byte(reg & 0xff,         check_ack=False)
        self.write_byte(val & 0xff,         check_ack=False)
        self.stop()

    def reg_read(self, reg):
        # Two-stage random read: write reg pointer (no ACK checks except
        # address), STOP, START again, write addr+R (ACK-checked), then
        # read the data byte. Si549 needs STOP+new START rather than
        # repeated START between the pointer-write and read-address.
        self.start()
        if not self.write_byte(self.address << 1, check_ack=True):
            self.stop()
            raise IOError(f"NACK on device address (read reg 0x{reg:02x})")
        self.write_byte(reg & 0xff, check_ack=False)
        self.stop()

        self.start()
        if not self.write_byte((self.address << 1) | 1, check_ack=True):
            self.stop()
            raise IOError(f"NACK on read address (reg 0x{reg:02x})")
        val = self.read_byte(ack=False)
        self.stop()
        return val


# ---- 5.5 setup sequence ---------------------------------------------------------------------

def setup(bbus, name, hsdiv, lsdiv, fbdiv):
    print(f"[{name}] acquiring bus, target HSDIV={hsdiv} LSDIV={lsdiv} FBDIV=0x{fbdiv:011X}")
    bbus.acquire()

    # 0. Sanity probe: chip alive at expected address?
    if not bbus.probe(bbus.address):
        bbus.release()
        raise IOError(f"[{name}] address 0x{bbus.address:02x} did not ACK after acquire (bus not ready)")
    print(f"[{name}] chip ACKs at 0x{bbus.address:02x}")

    # NOTE: we do NOT do a write/readback identity check (ARTIQ-style reg-23
    # ping). UARTBone CSR reads each force a queue flush + UART round-trip
    # (~17 ms per read) and freeze SCL HIGH for the duration. read_byte
    # needs 8 such reads -> ~136 ms with SCL essentially static, which
    # trips the Si549 I2C watchdog and makes it stop driving SDA (returns
    # 0xFF). The same watchdog also kills any reg_write that does mid-byte
    # ACK checks; we already work around it by ACK-checking only the
    # device-address byte. End-of-run we re-probe to confirm the chip is
    # still alive on the bus.

    # 1. 5.5 prelude (Table 5.6 of the datasheet). Must precede any
    # divider write; FCAL_OVR=1 at power-up blocks them otherwise.
    bbus.reg_write(REG_PAGE,     0x00)
    bbus.reg_write(REG_FCAL_OVR, 0x00)
    bbus.reg_write(REG_OE,       0x00)

    # 2. Update dividers (Table 5.6).
    bbus.reg_write(REG_HSDIV_LO, hsdiv & 0xff)
    bbus.reg_write(REG_HSDIV_HI, ((lsdiv & 0x7) << 4) | ((hsdiv >> 8) & 0x7))
    for i in range(6):
        bbus.reg_write(REG_FBDIV_0 + i, (fbdiv >> (8 * i)) & 0xff)

    # 3. Start FCAL, wait for VCO calibration, re-enable output.
    bbus.reg_write(REG_FCAL, 0x08)
    time.sleep(0.030)
    bbus.reg_write(REG_OE,   0x01)

    # 4. Si549 max settling time on a large frequency change.
    time.sleep(0.040)

    # 5. Re-probe to confirm the chip survived the sequence (any internal
    # NACK/timeout would have left the slave in a state where it ignores
    # its own address until the next STOP idle, which we just issued).
    if not bbus.probe(bbus.address):
        bbus.release()
        raise IOError(f"[{name}] chip stopped ACKing after setup -- writes likely failed")

    bbus.release()
    print(f"[{name}] setup complete (chip still ACKing post-config)")


# ---- main -------------------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description="Volatile Si549 setup over UARTBone.")
    p.add_argument("--only", choices=["main", "helper", "both"], default="both",
                   help="Which Si549 to program (default: both).")
    p.add_argument("--address", type=lambda x: int(x, 0), default=SI549_ADDR,
                   help=f"7-bit Si549 I2C address (default: 0x{SI549_ADDR:02X}).")
    p.add_argument("--probe", action="store_true",
                   help="Scan I2C bus 0x08-0x77 and print which addresses ACK; do not configure.")
    p.add_argument("--no-cpu-reset", action="store_true",
                   help="Don't hold urv CPU in reset across the sequence. "
                        "Default is to reset (avoids `freqmon checkvco` hang "
                        "and other WRPC-state breakage from the 5.5 OE=0 dropout).")
    p.add_argument("--adpll-scale", default=256,
                   help="Scale factor for both ADPLLs to adjust ppms according to: "
                        "+/- full_range_ppm = [(1<<16) - 1]/2 * 0.0001164 * scale/256. " 
                        "If we had scale=256 we achieve ~ +/- 3.8 ppm. (default: 256)")
    # Per-target divider overrides. If none given, the per-target defaults
    # (MAIN_* / HELPER_*) above are used.
    p.add_argument("--main-hsdiv",   type=lambda x: int(x, 0), default=MAIN_HSDIV)
    p.add_argument("--main-lsdiv",   type=lambda x: int(x, 0), default=MAIN_LSDIV)
    p.add_argument("--main-fbdiv",   type=lambda x: int(x, 0), default=MAIN_FBDIV)
    p.add_argument("--helper-hsdiv", type=lambda x: int(x, 0), default=HELPER_HSDIV)
    p.add_argument("--helper-lsdiv", type=lambda x: int(x, 0), default=HELPER_LSDIV)
    p.add_argument("--helper-fbdiv", type=lambda x: int(x, 0), default=HELPER_FBDIV)
    return p.parse_args()


def scan(bbus, name):
    bbus.acquire()
    found = []
    for addr in range(0x08, 0x78):
        try:
            if bbus.probe(addr):
                found.append(addr)
        except Exception:
            pass
    bbus.release()
    if found:
        print(f"[{name}] ACK from: " + ", ".join(f"0x{a:02x}" for a in found))
    else:
        print(f"[{name}] no devices ACKed (check wiring / pullups / WRPC reset state)")


def main():
    args = parse_args()
    bus = RemoteClient()
    bus.open()
    cpu_held = False

    scale = int(args.adpll_scale)
    if scale >= (1<<16) - 1:
        print(f"ADPLL scale must be representable on 16 bits, so must be smaller than {1<<16}")
        sys.exit(1)

    print("Before setting scale:")    
    print("refclk:", bus.regs.refclk_dac_adpll_scale.read())
    print("helper:", bus.regs.dmtd_dac_adpll_scale.read())    

    bus.regs.refclk_dac_adpll_scale.write(scale)
    bus.regs.dmtd_dac_adpll_scale.write(scale)

    print("After setting scale:")
    print("refclk:", bus.regs.refclk_dac_adpll_scale.read())
    print("helper:", bus.regs.dmtd_dac_adpll_scale.read())

    try:
        # Per-target tuple: (name, csr-prefix, hsdiv, lsdiv, fbdiv)
        targets = []
        if args.only in ("main", "both"):
            targets.append(("main", "refclk_dac",
                            args.main_hsdiv, args.main_lsdiv, args.main_fbdiv))
        if args.only in ("helper", "both"):
            targets.append(("helper", "dmtd_dac",
                            args.helper_hsdiv, args.helper_lsdiv, args.helper_fbdiv))

        if args.probe:
            for name, prefix, *_ in targets:
                scan(BBus(bus, prefix, args.address), name)
            return

        # Hold CPU in reset for the duration of the 5.5 sequence so WRPC
        # doesn't observe the OE=0 -> 30 ms FCAL -> OE=1 dropout while
        # running. CPU is released at the end -> WRPC boots fresh.
        if not args.no_cpu_reset:
            print("holding urv CPU in reset...")
            cpu_reset(bus, hold=True)
            cpu_held = True
            time.sleep(0.05)  # let any in-flight WRPC I2C transactions drain

        for name, prefix, hsdiv, lsdiv, fbdiv in targets:
            setup(BBus(bus, prefix, args.address), name, hsdiv, lsdiv, fbdiv)

        if cpu_held:
            print("releasing urv CPU reset (WRPC will boot fresh)")
            cpu_reset(bus, hold=False)
            cpu_held = False
        
    except IOError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        # Even on exception, release the CPU so the user isn't stuck with
        # a halted WRPC. They'll need to re-run the script if config failed.
        if cpu_held:
            cpu_reset(bus, hold=False)
        bus.close()


if __name__ == "__main__":
    main()

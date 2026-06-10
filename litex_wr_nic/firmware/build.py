#!/usr/bin/env python3

#
# This file is part of LiteX-WR-NIC.
#
# Copyright (c) 2024 Warsaw University of Technology
# Copyright (c) 2024 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import argparse
import os
import shutil
import tarfile
import subprocess

from litex.build import tools

# Toolchain and firmware variables -----------------------------------------------------------------

TOOLCHAIN_URL     = "https://gitlab.com/ohwr/project/wrpc-sw/-/wikis/uploads/9f9224d2249848ed3e854636de9c08dc/riscv-11.2-small.tgz"
TOOLCHAIN_ARCHIVE = "riscv-11.2-small.tgz"
TOOLCHAIN_DIR     = "riscv-11.2-small"

REPO_URL          = "https://gitlab.com/ohwr/project/wrpc-sw.git"
CLONE_DIR         = "wrpc-sw"

COMMIT_HASH       = "baf7749610b2880bf243b38a9a1608af8e0e688d"

# Per-target configuration (config file, firmware output name).
TARGET_CONFIG = {
    "spec_a7"  : ("spec_a7_defconfig",   "spec_a7_wrc.bram"),
    "acorn"    : ("spec_a7_defconfig",   "spec_a7_wrc.bram"),
    "kasli_v2" : ("kasli_v2_defconfig",  "kasli_v2_wrc.bram"),
}

# Legacy defaults (used when CONFIG_SRC / FIRMWARE_DEST are referenced directly).
CONFIG_SRC        = "spec_a7_defconfig"

FIRMWARE_SRC      = os.path.join(CLONE_DIR, "wrc.bram")
FIRMWARE_DEST     = "spec_a7_wrc.bram"

SDBFS_DEST        = "sdb-wrpc.bin"
SDBFS_SRC         = "sdbfs"

# Build Helpers/Functions --------------------------------------------------------------------------

def run_command(command, cwd=None):
    """Run a shell command."""
    try:
        subprocess.run(command, cwd=cwd, check=True, shell=True)
    except subprocess.CalledProcessError as e:
        print(f"Error: Command '{e.cmd}' failed with exit code {e.returncode}.")
        exit(1)

def init_riscv_toolchain():
    """Download and extract the RISC-V toolchain, and add it to PATH."""
    if not os.path.exists(TOOLCHAIN_DIR):
        print("Downloading RISC-V toolchain...")
        run_command(f"wget {TOOLCHAIN_URL}")
        print("Extracting toolchain...")
        with tarfile.open(TOOLCHAIN_ARCHIVE, "r:gz") as tar:
            tar.extractall()
    toolchain_bin_path = os.path.join(os.getcwd(), TOOLCHAIN_DIR, "bin")
    os.environ["PATH"] = toolchain_bin_path + os.pathsep + os.environ["PATH"]

def check_riscv_toolchain():
    """Check if riscv32-elf-gcc is available."""
    try:
        subprocess.run(["riscv32-elf-gcc", "--version"], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    except (subprocess.CalledProcessError, FileNotFoundError):
        print("Error: riscv32-elf-gcc not found in PATH.")
        exit(1)

def clone_repository():
    """Clone the repository if it does not exist."""
    if not os.path.exists(CLONE_DIR):
        run_command(f"git clone {REPO_URL} --recursive")

def checkout_commit(target="spec_a7"):
    """Checkout the specific commit in the repository."""
    # Ensure no kp/ki modifications.
    run_command(f"git checkout softpll/spll_main.c", cwd=CLONE_DIR)
    run_command(f"git checkout {COMMIT_HASH}", cwd=CLONE_DIR)

    # For Acorn / Kasli v2.0: adapt SoftPLL kp/ki for Si549 / MMCM phase-shift
    # characteristics.  The Si549 ADPLL has much lower gain than the VCXO+DAC
    # combo on SPEC-A7, so the loop filter gains must be reduced accordingly.
    # Exact values require lab tuning; Acorn values are a reasonable starting
    # point for Kasli v2.0 as well.
    if target in ("acorn", "kasli_v2"):
        tools.replace_in_file(f"{CLONE_DIR}/softpll/spll_main.c", "s->pi.kp = -1100;", "s->pi.kp = -150;")
        tools.replace_in_file(f"{CLONE_DIR}/softpll/spll_main.c", "s->pi.ki = -30;", "s->pi.ki = -2;")

    # For Kasli v2.0: overlay the generic board.c with the Kasli-specific
    # version that initialises the PCA9548 I2C muxes for SFP0 access, plus the
    # si549.c/.h that run the Si549 setup from firmware over the aux WB.
    if target == "kasli_v2":
        src_dir = os.path.join(os.path.dirname(__file__), "boards", "kasli_wr")
        dst_dir = os.path.join(CLONE_DIR, "boards", "generic")
        for f in ("board.c", "si549.c", "si549.h"):
            shutil.copy(os.path.join(src_dir, f), os.path.join(dst_dir, f))

        # The generic-PHY target object list lives in boards/boards.mk; add the
        # new si549 object so it gets compiled and linked (board.c overlays an
        # existing object, but si549.o is new). Reset boards.mk to pristine
        # first: `git checkout COMMIT_HASH` does not discard prior working-tree
        # edits, so without this the append would stack up across rebuilds.
        run_command("git checkout boards/boards.mk", cwd=CLONE_DIR)
        tools.replace_in_file(f"{CLONE_DIR}/boards/boards.mk",
            "obj-$(CONFIG_TARGET_GENERIC_PHY_16BIT) += boards/generic/board.o boards/generic/generic-storage.o",
            "obj-$(CONFIG_TARGET_GENERIC_PHY_16BIT) += boards/generic/board.o boards/generic/generic-storage.o boards/generic/si549.o")

def copy_config_file(config_src):
    """Copy the configuration file to the repository."""
    config_name = os.path.basename(config_src)
    config_dest = os.path.join(CLONE_DIR, f"configs/{config_name}")
    if not os.path.exists(config_src):
        print(f"Error: Configuration file {config_src} does not exist.")
        exit(1)
    shutil.copy(config_src, config_dest)

def build_firmware(config_src):
    """Build the firmware."""
    config_name = os.path.basename(config_src)
    run_command(f"make {config_name}", cwd=CLONE_DIR)
    run_command("make", cwd=CLONE_DIR)

def copy_firmware(firmware_dest):
    """Copy the resulting firmware to the destination."""
    if not os.path.exists(FIRMWARE_SRC):
        print(f"Error: Firmware file {FIRMWARE_SRC} does not exist.")
        exit(1)
    shutil.copy(FIRMWARE_SRC, firmware_dest)

def build_sdbfs():
    """Build the SDB filesystem."""
    sdbfs_tool_path = os.path.join(CLONE_DIR, "tools", "gensdbfs")
    sdbfs_src_path = os.path.abspath(SDBFS_SRC)
    sdbfs_dest_path = os.path.abspath(SDBFS_DEST)

    if not os.path.exists(sdbfs_tool_path):
        print(f"Error: SDBFS tool {sdbfs_tool_path} does not exist.")
        exit(1)

    tools_path = os.path.join(CLONE_DIR, "tools")
    os.environ["PATH"] = tools_path + os.pathsep + os.environ["PATH"]

    # Run the command to generate SDB filesystem
    run_command(f"./gensdbfs -b 65536 {sdbfs_src_path} {sdbfs_dest_path}", cwd=tools_path)


# Main ---------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="LiteX-WR-NIC firmware builder.")
    parser.add_argument("--target", default="spec_a7", help="Target Board.", choices=list(TARGET_CONFIG.keys()))
    args = parser.parse_args()

    config_src, firmware_dest = TARGET_CONFIG[args.target]

    init_riscv_toolchain()
    check_riscv_toolchain()
    clone_repository()
    checkout_commit(args.target)
    copy_config_file(config_src)
    build_firmware(config_src)
    copy_firmware(firmware_dest)
    build_sdbfs()
    print("Build process completed successfully.")

if __name__ == "__main__":
    main()

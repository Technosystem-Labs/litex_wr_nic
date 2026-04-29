"""
Prerequisites:
    litex_server --uart --uart-port /dev/ttyUSB0 --csr-csv csr.csv
"""

import sys
from litex.tools.litex_client import RemoteClient


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "on"

    c = RemoteClient(csr_csv="csr.csv")
    c.open()

    print("Storage_val:", c.regs.main_storage_val.read())
    c.regs.main_storage_val.write(3)
    print("Storage_val:", c.regs.main_storage_val.read())
    print("Const:", c.regs.main_const_id.read())

    c.close()

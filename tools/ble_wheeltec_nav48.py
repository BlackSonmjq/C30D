"""Double-click entry point for the WHEELTEC module running nav48 firmware."""

import sys

from ble_vofa_bridge import main


if __name__ == "__main__":
    args = sys.argv[1:]
    if not any(arg.split("=", 1)[0] in ("--address", "--name") for arg in args):
        args = ["--address", "D4:14:A7:D0:A4:71", *args]
    raise SystemExit(main(args))

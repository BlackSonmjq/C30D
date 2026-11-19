# -*- mode: python ; coding: utf-8 -*-
from argparse import ArgumentParser
from pathlib import Path
from PyInstaller.utils.hooks import collect_submodules, collect_dynamic_libs, copy_metadata

parser = ArgumentParser()
parser.add_argument("--wheeltec-nav48", action="store_true")
wheeltec_nav48 = parser.parse_args().wheeltec_nav48
entry = "ble_wheeltec_nav48.py" if wheeltec_nav48 else "ble_vofa_bridge.py"
name = "BLE-WHEELTEC-nav48" if wheeltec_nav48 else "WHEELTEC-BLE-VOFA-Align"
root = Path(SPECPATH).resolve().parent
hidden = collect_submodules("bleak.backends.winrt") + collect_submodules("winrt")
data = [(str(root/"tools/testdata/nav48-frame.bin"),"testdata")]
data += copy_metadata("bleak")
binaries = collect_dynamic_libs("winrt")
a = Analysis([str(root/"tools"/entry)], pathex=[str(root/"tools")],
             binaries=binaries, datas=data, hiddenimports=hidden,
             hookspath=[], runtime_hooks=[], excludes=["tkinter","PyQt5","PySide6"])
pyz = PYZ(a.pure)
exe = EXE(pyz,a.scripts,a.binaries,a.datas,[],name=name,
          debug=False,bootloader_ignore_signals=False,strip=False,upx=False,
          console=True,disable_windowed_traceback=False)

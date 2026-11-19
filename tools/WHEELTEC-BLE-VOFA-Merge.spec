from pathlib import Path
from PyInstaller.utils.hooks import collect_submodules, collect_dynamic_libs, copy_metadata

root = Path(SPECPATH)
hidden = collect_submodules("bleak.backends.winrt") + collect_submodules("winrt")
a = Analysis([str(root / "ble_vofa_bridge.py")], pathex=[str(root)],
             binaries=collect_dynamic_libs("winrt"), datas=copy_metadata("bleak"),
             hiddenimports=hidden, hookspath=[], runtime_hooks=[],
             excludes=["tkinter", "PyQt5", "PySide6"])
pyz = PYZ(a.pure)
exe = EXE(pyz, a.scripts, a.binaries, a.datas, [], name="WHEELTEC-BLE-VOFA-Merge",
          debug=False, bootloader_ignore_signals=False, strip=False, upx=False,
          console=True, disable_windowed_traceback=False)

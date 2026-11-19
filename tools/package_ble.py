"""Build and smoke-test the standalone Windows x64 bridge. Never scans BLE."""
from pathlib import Path
import hashlib
import importlib.metadata
import json
import os
import platform
import shutil
import struct
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.stdout.reconfigure(encoding="utf-8",errors="replace")
if sys.platform!="win32" or struct.calcsize("P")!=8:
    raise SystemExit("Package on Windows x64")
dist=ROOT/"build/vofa-align"
work=ROOT/"build/pyinstaller-work"
cache=ROOT/"build/pyinstaller-cache"
for path in (dist,work,cache):
    assert path.resolve().is_relative_to(ROOT)
    path.mkdir(parents=True,exist_ok=True)
env=os.environ.copy()
env["PYINSTALLER_CONFIG_DIR"]=str(cache)
env["PYTHONIOENCODING"]="utf-8"
cmd=[sys.executable,"-m","PyInstaller","--noconfirm","--clean","--distpath",str(dist),
     "--workpath",str(work),str(ROOT/"tools/WHEELTEC-BLE-VOFA-Align.spec")]
result=subprocess.run(cmd,cwd=ROOT,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
(dist/"packaging.log").write_bytes(result.stdout)
if result.returncode:
    print(result.stdout.decode("utf-8",errors="replace"))
    raise SystemExit(result.returncode)
exe=dist/"WHEELTEC-BLE-VOFA-Align.exe"
# Also test a copy without the source tree on PATH or a Python installation on PATH.
portable=ROOT/"build"/("portable-check-"+str(time.time_ns()))
portable.mkdir()
copy=portable/exe.name
shutil.copy2(exe,copy)
clean=os.environ.copy()
clean["PATH"]=str(Path(os.environ["SystemRoot"])/"System32")+";"+os.environ["SystemRoot"]
clean.pop("PYTHONPATH",None)
clean.pop("PYTHONHOME",None)
launch=portable/"different-working-directory"
launch.mkdir()
check=subprocess.run([str(copy),"--self-test"],cwd=launch,env=clean,
                     stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=60)
output=check.stdout.decode("utf-8",errors="replace")
(dist/"self-test.log").write_text(output,encoding="utf-8")
print(output)
if check.returncode:
    raise SystemExit(f"Packaged self-test failed: {check.returncode}")
assert len(list((portable/"logs").glob("selftest-*.csv")))==1
assert not (launch/"logs").exists()
raw=exe.read_bytes()
pe=int.from_bytes(raw[0x3C:0x40],"little")
assert raw[pe:pe+4]==b"PE\0\0" and int.from_bytes(raw[pe+4:pe+6],"little")==0x8664
versions={d.metadata["Name"]:d.version for d in importlib.metadata.distributions()}
manifest={"exe":exe.name,"bytes":len(raw),"sha256":hashlib.sha256(raw).hexdigest(),
          "platform":platform.platform(),"python":sys.version,"packages":versions,
          "portable_copy_self_test":True,"ble_connected":False,"firmware_flashed":False,
          "sources_sha256":{name:hashlib.sha256((ROOT/"tools"/name).read_bytes()).hexdigest()
                            for name in ("ble_vofa_bridge.py","vofa_protocol.py","vofa_legacy.py",
                                         "WHEELTEC-BLE-VOFA-Align.spec","testdata/nav48-frame.bin")}}
(dist/"package-manifest.json").write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding="utf-8")
(dist/"requirements-lock.txt").write_text("\n".join(f"{k}=={v}" for k,v in sorted(versions.items()))+"\n",encoding="utf-8")
for name in ("BLE-VOFA使用说明.md","VOFA通道表.md"):
    if (ROOT/name).exists():
        shutil.copy2(ROOT/name,dist/name)
(dist/"自检.cmd").write_text('@echo off\n"%~dp0WHEELTEC-BLE-VOFA-Align.exe" --self-test\n'
                            'if errorlevel 1 (echo SELF-TEST FAILED) else (echo SELF-TEST PASSED)\npause\n',
                            encoding="ascii")
print("Packaged:",exe)

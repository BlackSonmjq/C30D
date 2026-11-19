"""Build only; never connects to a probe or flashes a board.

python tools/build.py [--toolchain G:/keil5/ARM/ARMCC/bin]
Uses the sources/defines/include paths in the delivered uVision project.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def build():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--toolchain", default=os.environ.get("ARMCC_BIN", "G:/keil5/ARM/ARMCC/bin"))
    args = parser.parse_args()
    tool = Path(args.toolchain)
    for name in ("armcc.exe", "armasm.exe", "armlink.exe", "fromelf.exe"):
        if not (tool / name).is_file():
            raise SystemExit(f"Missing {tool / name}; set ARMCC_BIN or --toolchain")
    project = ROOT / "USER/WHEELTEC.uvprojx"
    target = ET.parse(project).getroot().find("./Targets/Target")
    controls = target.find("./TargetOption/TargetArmAds/Cads/VariousControls")
    includes = [(project.parent / p.replace("\\", "/")).resolve()
                for p in controls.findtext("IncludePath").split(";")]
    defines = [d for d in controls.findtext("Define").split(",") if d != "__CC_ARM"]
    output = ROOT / "OBJ"
    output.mkdir(exist_ok=True)
    log_path = output / "build.log"
    commands = []
    objects = []
    sources = []
    warnings = 0
    with log_path.open("w", encoding="utf-8") as log:
        def run(cmd):
            nonlocal warnings
            commands.append([str(x) for x in cmd])
            result = subprocess.run(cmd, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            message = result.stdout.decode("gbk", errors="replace")
            warnings += len(re.findall(r"\bWarning:", message))
            log.write(repr([str(x) for x in cmd]) + "\n" + message + "\n")
            log.flush()
            if result.returncode:
                print(message)
                raise SystemExit(f"Build failed ({result.returncode}); see {log_path}")
        for entry in target.findall("./Groups/Group/Files/File"):
            source = (project.parent / entry.findtext("FilePath").replace("\\", "/")).resolve()
            if source.suffix.lower() not in (".c", ".s"):
                continue
            obj = output / f"fw_{len(objects):02d}_{source.stem}.o"
            sources.append(source)
            objects.append(obj)
            if source.suffix.lower() == ".s":
                command = [tool / "armasm.exe", "--cpu", "Cortex-M4.fp", "--apcs=interwork",
                           "--pd", "__MICROLIB SETA 1", "-o", obj, source]
            else:
                command = [tool / "armcc.exe", "--c99", "--cpu", "Cortex-M4.fp", "--apcs=interwork",
                           "--library_type=microlib", "--split_sections", "-O2", "-D__MICROLIB"]
                command += ["-D" + d for d in defines]
                for include in includes:
                    command += ["-I", include]
                command += ["-c", source, "-o", obj]
            run(command)
            print(f"Compiled {source.relative_to(ROOT).as_posix()}")
        scatter = output / "rebuild.sct"
        scatter.write_text("""LR_IROM1 0x08000000 0x00100000 {
  ER_IROM1 0x08000000 0x00100000 {
    *.o (RESET, +First)
    *(InRoot$$Sections)
    .ANY (+RO)
  }
  RW_IRAM1 0x20000000 0x00020000 { .ANY (+RW +ZI) }
}
""", encoding="ascii")
        axf = output / "C30D_rebuild.axf"
        run([tool / "armlink.exe", "--cpu", "Cortex-M4.fp", "--library_type=microlib",
             "--strict", "--scatter", scatter, "--map", "--info", "sizes",
             "--list", output / "C30D_rebuild.map", "--output", axf, *objects])
        run([tool / "fromelf.exe", "--i32combined", "--output", output / "C30D_rebuild.hex", axf])
        run([tool / "fromelf.exe", "--bin", "--output", output / "C30D_rebuild.bin", axf])
    image = (output / "C30D_rebuild.bin").read_bytes()
    sp = int.from_bytes(image[0:4], "little")
    reset = int.from_bytes(image[4:8], "little")
    assert 0x20000000 <= sp <= 0x20020000 and sp % 8 == 0
    assert reset & 1 and 0x08000000 <= (reset & ~1) < 0x08000000 + len(image)
    # Verify Intel HEX checksums and ensure it describes the same binary image.
    decoded, base, eof = {}, 0, False
    for line in (output / "C30D_rebuild.hex").read_text(encoding="ascii").splitlines():
        record = bytes.fromhex(line.removeprefix(":"))
        assert line.startswith(":") and not eof and len(record) == record[0] + 5 and sum(record) % 256 == 0
        address, kind, data = int.from_bytes(record[1:3], "big"), record[3], record[4:-1]
        if kind == 0:
            for offset, byte in enumerate(data):
                absolute = base + address + offset
                assert 0x08000000 <= absolute < 0x08100000 and absolute not in decoded
                decoded[absolute] = byte
        elif kind == 4:
            assert len(data) == 2
            base = int.from_bytes(data, "big") << 16
        elif kind == 1:
            assert not data
            eof = True
        elif kind in (3, 5):
            assert len(data) == 4
        else:
            raise AssertionError(f"Unexpected Intel HEX record {kind}")
    assert eof and decoded
    for absolute, byte in decoded.items():
        assert image[absolute - 0x08000000] == byte
    assert max(decoded) - 0x08000000 < len(image)
    inputs = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    # Header/config changes are part of the source identity as well.
    for p in ROOT.rglob("*.h"):
        if p.relative_to(ROOT).parts[0] in ("OBJ","build",".git"):
            continue
        inputs[p.relative_to(ROOT).as_posix()] = hashlib.sha256(p.read_bytes()).hexdigest()
    inputs["USER/WHEELTEC.uvprojx"] = hashlib.sha256(project.read_bytes()).hexdigest()
    config = (ROOT / "USER/nav_config.h").read_text(encoding="utf-8")
    calibration_flag = re.search(r"^#define\s+NAV_CALIBRATED\s+([01])\b", config, re.M)
    assert calibration_flag, "NAV_CALIBRATED must be explicitly 0 or 1"
    result = {"compiled_units": len(objects), "binary_bytes": len(image),
              "initial_sp": hex(sp), "reset_vector": hex(reset), "flashed": False,
              "errors": 0, "warnings": warnings, "calibration_ready": calibration_flag.group(1) == "1",
              "field_tested": False, "sources_sha256": inputs,
              "artifacts_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                                    for p in (axf, output / "C30D_rebuild.hex", output / "C30D_rebuild.bin")}}
    (output / "build-manifest.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    (output / "build-commands.json").write_text(json.dumps(commands, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Built {len(objects)} units; {len(image)} binary bytes; 0 errors, {warnings} warnings; no flashing performed.")


if __name__ == "__main__":
    build()

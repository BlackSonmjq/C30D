"""Use fixed scene data in host simulations, while testing the real validator."""
from pathlib import Path
import subprocess


def build_validator(root,compiler,output,label="calibration-validator",window=3):
    root,output=Path(root),Path(output)
    output.mkdir(parents=True,exist_ok=True)
    obj=output/(label+".o")
    subprocess.run([str(compiler),"-std=c99","-O2",f"-DNAV_FILTER_FRAMES={window}",
                    "-Dnav_calibration=host_unused_calibration","-Dnav_plan=host_unused_plan",
                    "-I",str(root/"USER"),"-c",str(root/"USER/nav_calibration.c"),"-o",str(obj)],check=True)
    return obj

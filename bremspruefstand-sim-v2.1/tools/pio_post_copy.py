from pathlib import Path
from shutil import copy2

Import("env")

project_dir = Path(env.subst("$PROJECT_DIR"))
build_dir = Path(env.subst("$BUILD_DIR"))


def copy_firmware_artifacts(*_args, **_kwargs):
    for name in ("firmware.bin", "firmware.elf"):
        src = build_dir / name
        dst = project_dir / name
        if src.exists():
            copy2(src, dst)
            print(f"[post] copied {src.name} -> {dst}")


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", copy_firmware_artifacts)

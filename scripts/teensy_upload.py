# scripts/teensy_upload.py

Import("env")

import os
import platform
import stat
import subprocess
import sys
import time
from pathlib import Path


PROJECT_DIR = Path(env.subst("$PROJECT_DIR"))
TOOLS_DIR = PROJECT_DIR / "tools"

MCU = "TEENSY41"

# Keep this small. The retry is the real fallback behavior.
SETTLE_SECONDS_AFTER_REBOOT = 0.25


def is_windows():
    return sys.platform.startswith("win")


def exe_name(name):
    return f"{name}.exe" if is_windows() else name


def ensure_executable(path):
    if is_windows():
        return

    try:
        mode = path.stat().st_mode
        path.chmod(mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    except OSError:
        pass


def host_tools_subdir():
    system = sys.platform
    machine = platform.machine().lower()

    if system.startswith("linux"):
        if machine in ("x86_64", "amd64"):
            return Path("linux") / "x86_64"

        if machine in ("aarch64", "arm64"):
            return Path("linux") / "aarch64"

        return None

    if system == "darwin":
        return Path("macos")

    if is_windows():
        return Path("win")

    return None


def bundled_teensy_loader_cli():
    subdir = host_tools_subdir()

    if subdir is None:
        print(
            "No bundled Teensy loader mapping for "
            f"{sys.platform}/{platform.machine()}; using fallback loader."
        )
        return None

    tool_dir = TOOLS_DIR / subdir

    candidates = [
        tool_dir / exe_name("teensy_loader_cli"),
    ]

    # Useful if the repo accidentally stores the Windows filename explicitly
    # or a POSIX binary with no extension inside tools/win.
    if is_windows():
        candidates.append(tool_dir / "teensy_loader_cli")
    else:
        candidates.append(tool_dir / "teensy_loader_cli.exe")

    for candidate in candidates:
        if candidate.is_file():
            ensure_executable(candidate)
            return str(candidate)

    print(f"No bundled Teensy loader found in {tool_dir}; using fallback loader.")
    return None


def platformio_tool_path(name):
    """
    Prefer PlatformIO's packaged tool-teensy binary.
    Fall back to PATH if the package path cannot be resolved.
    """
    binary_name = exe_name(name)

    try:
        package_dir = env.PioPlatform().get_package_dir("tool-teensy")
    except Exception:
        package_dir = None

    if package_dir:
        candidate = Path(package_dir) / binary_name
        if candidate.is_file():
            return str(candidate)

    return binary_name


def run_soft_reboot_once():
    reboot_tool = platformio_tool_path("teensy_reboot")

    try:
        subprocess.run([reboot_tool, "-s"], check=False)
    except FileNotFoundError:
        print(f"{reboot_tool} not found; skipping soft reboot.")

    if SETTLE_SECONDS_AFTER_REBOOT > 0:
        time.sleep(SETTLE_SECONDS_AFTER_REBOOT)


def run_loader(loader, firmware):
    cmd = [
        loader,
        "--mcu=TEENSY41",
        "-w",
        "-v",
        firmware,
    ]

    print("Running:", " ".join(f'"{x}"' if " " in x else x for x in cmd))

    try:
        return subprocess.run(cmd).returncode
    except FileNotFoundError:
        print(f"Unable to run Teensy loader: {loader}")
        return 127


def on_upload(source, target, env):
    firmware = str(source[0])

    run_soft_reboot_once()

    bundled_loader = bundled_teensy_loader_cli()

    if bundled_loader:
        print(f"Using bundled teensy_loader_cli: {bundled_loader}")
        return run_loader(bundled_loader, firmware)

    fallback_loader = platformio_tool_path("teensy_loader_cli")

    print(f"Using fallback teensy_loader_cli: {fallback_loader}")

    first_result = run_loader(fallback_loader, firmware)

    if first_result == 0:
        return 0

    print(
        "Fallback teensy_loader_cli failed with exit code "
        f"{first_result}; trying once more."
    )

    return run_loader(fallback_loader, firmware)


env.Replace(UPLOADCMD=on_upload)
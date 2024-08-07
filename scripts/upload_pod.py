import os
import sys
import csv
import json
import hashlib
import subprocess
from os.path import join

Import("env")

if sys.platform == "darwin":
    default_upload_speed = "460800"
else:
    default_upload_speed = "921600"

def get_partition_address(partition_name):
    partition_file = join(env.subst("$PROJECT_DIR"), "quackhead.csv")
    with open(partition_file, 'r') as file:
        reader = csv.DictReader(file, fieldnames=["name", "type", "subtype", "offset", "size", "flags"])
        for row in reader:
            if row["name"] == partition_name:
                return row["offset"]
    raise ValueError(f"Partition {partition_name} not found in {partition_file}")

def auto_detect_port():
    try:
        result = subprocess.run(["pio", "device", "list", "--json-output"], capture_output=True, text=True)
        ports = json.loads(result.stdout)
        if not ports:
            raise RuntimeError("No serial ports found. Please specify the upload port using the PORT argument.")
        for port in ports:
            if port['description'] != "n/a":
                return port['port']
        return ports[0]['port']
    except Exception as e:
        raise RuntimeError(f"Failed to auto-detect serial port: {e}")

def calculate_checksum(file_path):
    hash_md5 = hashlib.md5()
    with open(file_path, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            hash_md5.update(chunk)
    return hash_md5.hexdigest()

def pod_uploader(source, target, env, always=True):
    build_dir = env.subst("$BUILD_DIR")
    dummy_file_path = join(build_dir, "upload_pod_done")
    firmware_path = join(env.subst("$PROJECT_DIR"), "warbler.pod")
    current_checksum = calculate_checksum(firmware_path)
    if not always and os.path.exists(dummy_file_path):
        with open(dummy_file_path, 'r') as f:
            stored_checksum = f.read().strip()
        if stored_checksum == current_checksum:
            print("upload_pod has already been executed with the same firmware. Skipping...")
            return

    pod_base_address = get_partition_address("pod")

    upload_port = env.subst("$UPLOAD_PORT") or auto_detect_port()  # Use the provided port or auto-detect
    upload_speed = env.GetProjectOption("upload_speed", default=default_upload_speed)

    upload_command = [
        env.subst("$PYTHONEXE"),
        env.subst("$UPLOADER"),
        "--chip", "esp32",
        "--port", upload_port,
        "--baud", "$UPLOAD_SPEED",
        "write_flash", pod_base_address, firmware_path
    ]

    result = env.Execute(" ".join(upload_command))
    if result == 0:  # Check if the command was successful
        # Create the dummy file to signal that upload_pod has been executed
        with open(dummy_file_path, 'w') as f:
            f.write(current_checksum)
    else:
        print(f"Upload failed: {e}")
        raise Exception("upload_pod failed")

def before_pod(source, target, env):
    pod_uploader(source, target, env, False)

def upload_pod(source, target, env):
    pod_uploader(source, target, env, True)

if os.uname().sysname == "Darwin":
    env.Replace(UPLOAD_SPEED=default_upload_speed)

# Add custom target for upload_pod
env.AddCustomTarget(
    name="upload_pod",
    dependencies=None,
    actions=[upload_pod],
    title="Upload POD",
    description="Upload warbler.pod to the POD partition"
)

env.AddPreAction("upload", before_pod)

# from os.path import join
# from SCons.Script import (DefaultEnvironment, Builder)

# env = DefaultEnvironment()

# # Define a custom uploader
# def upload_pod(source, target, env):
#     firmware_path = join(env.subst("$PROJECT_DIR"), "warbler.pod")
#     upload_port = env.subst("$UPLOAD_PORT")
#     upload_speed = env.subst("$UPLOAD_SPEED")
#     upload_command = [
#         env.subst("$PYTHONEXE"),
#         env.subst("$UPLOADER"),
#         "--chip", "esp32",
#         "--port", upload_port,
#         "--baud", upload_speed,
#         "write_flash", "0x610000", firmware_path
#     ]
#     env.Execute(" ".join(upload_command))

# # Add custom target
# env.AddCustomTarget(
#     name="upload_pod",
#     dependencies=None,
#     actions=[upload_pod],
#     title="Upload POD",
#     description="Upload warbler.pod to the POD partition"
# )
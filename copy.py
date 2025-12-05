Import("env")
import os
import shutil
from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

project_dir = env['PROJECT_DIR']

# The folder under .pio/build is named after the board, NOT the env name
build_dir = os.path.join(project_dir, ".pio", "build", "teensy41")

hex_src = os.path.join(build_dir, "firmware.hex")

# Destination folder
firmware_dir = os.path.join(project_dir, "firmware")
hex_dst = os.path.join(firmware_dir, "firmware.hex")

def copy_hex(source, target, env):
    print("Copying firmware.hex to firmware/ ...")
    os.makedirs(firmware_dir, exist_ok=True)
    shutil.copy(hex_src, hex_dst)
    print(f"Copied: {hex_src} → {hex_dst}")

# Attach script to post-build action
env.AddPostAction(hex_src, copy_hex)

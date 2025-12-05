Import("env")
import os
import shutil
import subprocess
from datetime import datetime

project_dir = env['PROJECT_DIR']
build_dir = os.path.join(project_dir, ".pio", "build", "teensy41")
hex_src = os.path.join(build_dir, "firmware.hex")

firmware_dir = os.path.join(project_dir, "firmware")
hex_dst = os.path.join(firmware_dir, "firmware.hex")


def copy_hex(source, target, env):
    print("Copying firmware.hex to firmware/ ...")
    os.makedirs(firmware_dir, exist_ok=True)
    shutil.copy(hex_src, hex_dst)
    print(f"Copied to: {hex_dst}")

    git_commit_and_push()


def git_commit_and_push():
    # Generate timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    commit_message = f"Auto-update firmware ({timestamp})"

    try:
        subprocess.run(["git", "add", "firmware/firmware.hex"], check=True)
        subprocess.run(["git", "commit", "-m", commit_message], check=True)
        subprocess.run(["git", "push"], check=True)
        print("Firmware pushed to repo successfully!")
    except subprocess.CalledProcessError:
        print("Git commit/push skipped (no changes or Git not configured).")


env.AddPostAction(hex_src, copy_hex)
# Note: This script copies the built firmware.hex to the firmware/ directory
# and commits/pushes it to the Git repository after each build.
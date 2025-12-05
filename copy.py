Import("env")
import os
import shutil
import subprocess
from datetime import datetime

project_dir = env['PROJECT_DIR']
build_dir = os.path.join(project_dir, ".pio", "build", "teensy41")
hex_src = os.path.join(build_dir, "firmware.hex")

firmware_dir = os.path.join(project_dir, "firmware")


def copy_hex(source, target, env):
    print("Copying firmware.hex to firmware/ ...")

    # Create target directory
    os.makedirs(firmware_dir, exist_ok=True)

    # Create timestamped filename
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    hex_dst = os.path.join(firmware_dir, f"firmware_{timestamp}.hex")

    # Copy to timestamped file
    shutil.copy(hex_src, hex_dst)
    print(f"Copied to: {hex_dst}")

    # Commit and push firmware automatically
    git_commit_and_push(hex_dst, timestamp)


def git_commit_and_push(new_file, timestamp):
    commit_message = f"Auto firmware update ({timestamp})"

    try:
        # Add new timestamped file
        subprocess.run(["git", "add", new_file], check=True)

        # Commit with timestamp
        subprocess.run(["git", "commit", "-m", commit_message], check=True)

        # Push to remote
        subprocess.run(["git", "push"], check=True)

        print("Firmware pushed to repo successfully!")

    except subprocess.CalledProcessError as e:
        print("Git commit/push skipped (probably no changes or Git not configured).")
        print("Error:", e)


env.AddPostAction(hex_src, copy_hex)

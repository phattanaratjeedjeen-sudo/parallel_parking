import os
import json
import csv
import bisect
import re

BASE_DIR = os.path.join(os.path.dirname(__file__), "Pose_and_steer")
FOLDERS = ["30", "35", "40"]


def parse_file(path):
    with open(path, "r") as f:
        text = f.read()
    # Strip comment lines starting with #
    lines = [l for l in text.splitlines() if not l.strip().startswith("#")]
    return json.loads("\n".join(lines))


def find_pairs(folder_path):
    files = os.listdir(folder_path)
    pose_files = {f for f in files if f.endswith("_pose.csv.txt")}
    steer_files = {f for f in files if f.endswith("_steer.csv.txt")}

    pairs = []
    for pose_file in sorted(pose_files):
        base = pose_file.replace("_pose.csv.txt", "")
        steer_file = base + "_steer.csv.txt"
        if steer_file in steer_files:
            pairs.append((pose_file, steer_file, base))
    return pairs


def merge_and_save(folder_path, pose_file, steer_file, base_name):
    pose_data = parse_file(os.path.join(folder_path, pose_file))
    steer_data = parse_file(os.path.join(folder_path, steer_file))

    # Build steer lookup: elapsed_s -> wheel_angle
    steer_times = [row[0] for row in steer_data]
    steer_angles = [row[1] for row in steer_data]

    def nearest_angle(t):
        idx = bisect.bisect_left(steer_times, t)
        if idx == 0:
            return steer_angles[0]
        if idx >= len(steer_times):
            return steer_angles[-1]
        before = steer_times[idx - 1]
        after = steer_times[idx]
        if abs(t - before) <= abs(t - after):
            return steer_angles[idx - 1]
        return steer_angles[idx]

    out_path = os.path.join(folder_path, base_name + ".csv")
    with open(out_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "elapsed_s", "velocity_ms", "accel_ms2", "jerk_ms3", "direction", "wheel_angle_deg"])
        for row in pose_data:
            x, y, elapsed_s, velocity_ms, accel_ms2, jerk_ms3, direction = row
            wheel_angle = nearest_angle(elapsed_s)
            writer.writerow([x, y, elapsed_s, velocity_ms, accel_ms2, jerk_ms3, direction, wheel_angle])

    print(f"  Saved: {out_path}")


for folder in FOLDERS:
    folder_path = os.path.join(BASE_DIR, folder)
    pairs = find_pairs(folder_path)
    print(f"\nFolder {folder}: {len(pairs)} pair(s) found")
    for pose_file, steer_file, base_name in pairs:
        print(f"  Merging {pose_file} + {steer_file}")
        merge_and_save(folder_path, pose_file, steer_file, base_name)

print("\nDone.")

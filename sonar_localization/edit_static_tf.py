#!/usr/bin/env python3
"""
edit_static_tf.py

Read a ROS2 MCAP bag, patch (or insert) a static TF between two frames,
and write the result to a new bag.

  • If the transform <parent> → <child> already exists anywhere in /tf_static,
    every occurrence is updated with the new values.
  • If it does not exist, it is appended to the first /tf_static message found.
  • If the bag has no /tf_static messages at all, the script exits with a warning.

Usage
─────
  python3 edit_static_tf.py <bag> <parent_frame> <child_frame> \\
      [--xyz X Y Z] [--rpy R P Y] [-o OUTPUT] [--inplace]

Arguments
─────────
  bag              Path to the input ROS2 bag directory
  parent_frame     TF parent frame ID
  child_frame      TF child frame ID

Options
───────
  --xyz X Y Z      Translation in metres          (default: 0 0 0)
  --rpy R P Y      Rotation in degrees, RPY order (default: 0 0 0)
  -o, --output     Output bag directory            (default: <bag>_patched)
  --inplace        Overwrite the input bag (writes to temp then replaces)
  -h, --help       Show this help and exit

Examples
────────
  # Add/update sonar → base_link with a 10 cm offset and 22.5° pitch
  python3 edit_static_tf.py ./my_bag base_link sonar_link \\
      --xyz -0.109 -0.037 0.168 --rpy 0 22.5 0

  # Overwrite the original bag
  python3 edit_static_tf.py ./my_bag base_link sonar_link \\
      --xyz 0.1 0 0 --inplace
"""

import argparse
import math
import os
import shutil
import sys
import tempfile

import rclpy
from rclpy.serialization import deserialize_message, serialize_message
import rosbag2_py
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


# ─────────────────────────────────────────────────────────────────────────────
# Math helpers
# ─────────────────────────────────────────────────────────────────────────────

def rpy_to_quaternion(roll_rad, pitch_rad, yaw_rad):
    """Convert roll-pitch-yaw (radians) to quaternion (x, y, z, w)."""
    cr, sr = math.cos(roll_rad  / 2), math.sin(roll_rad  / 2)
    cp, sp = math.cos(pitch_rad / 2), math.sin(pitch_rad / 2)
    cy, sy = math.cos(yaw_rad   / 2), math.sin(yaw_rad   / 2)
    w =  cr * cp * cy + sr * sp * sy
    x =  sr * cp * cy - cr * sp * sy
    y =  cr * sp * cy + sr * cp * sy
    z =  cr * cp * sy - sr * sp * cy
    return x, y, z, w


def quaternion_to_rpy(qx, qy, qz, qw):
    """Convert quaternion to roll-pitch-yaw in degrees (for display)."""
    roll  = math.degrees(math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2)))
    pitch = math.degrees(math.asin( max(-1, min(1, 2*(qw*qy - qz*qx)))))
    yaw   = math.degrees(math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2)))
    return roll, pitch, yaw


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

GREEN  = "\033[32m"
YELLOW = "\033[33m"
RED    = "\033[31m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

def ok(msg):   print(f"{GREEN}[OK]{RESET}     {msg}")
def info(msg): print(f"{BOLD}[INFO]{RESET}   {msg}")
def warn(msg): print(f"{YELLOW}[WARN]{RESET}   {msg}")
def err(msg):  print(f"{RED}[ERROR]{RESET}  {msg}")


def open_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def open_writer(bag_path: str) -> rosbag2_py.SequentialWriter:
    if os.path.exists(bag_path):
        shutil.rmtree(bag_path)
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return writer


def make_transform(parent: str, child: str,
                   x: float, y: float, z: float,
                   qx: float, qy: float, qz: float, qw: float) -> TransformStamped:
    tf = TransformStamped()
    tf.header.frame_id       = parent
    tf.child_frame_id        = child
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = z
    tf.transform.rotation.x    = qx
    tf.transform.rotation.y    = qy
    tf.transform.rotation.z    = qz
    tf.transform.rotation.w    = qw
    return tf


# ─────────────────────────────────────────────────────────────────────────────
# Core
# ─────────────────────────────────────────────────────────────────────────────

def patch_bag(bag_in: str, bag_out: str,
              parent: str, child: str,
              x: float, y: float, z: float,
              qx: float, qy: float, qz: float, qw: float):

    TF_STATIC = "/tf_static"

    reader = open_reader(bag_in)
    topic_types = reader.get_all_topics_and_types()

    has_tf_static = any(t.name == TF_STATIC for t in topic_types)

    writer = open_writer(bag_out)
    for tm in topic_types:
        writer.create_topic(tm)

    # If the bag has no /tf_static topic at all, register it and inject one message
    # at timestamp 0 so it arrives before any other data.
    if not has_tf_static:
        next_id = max((t.id for t in topic_types), default=-1) + 1
        tf_meta = rosbag2_py.TopicMetadata(
            id=next_id,
            name=TF_STATIC,
            type="tf2_msgs/msg/TFMessage",
            serialization_format="cdr",
        )
        writer.create_topic(tf_meta)
        new_msg = TFMessage()
        new_msg.transforms.append(
            make_transform(parent, child, x, y, z, qx, qy, qz, qw))
        serialized = bytes(serialize_message(new_msg))
        writer.write(TF_STATIC, serialized, 0)
        warn(f"Bag had no {TF_STATIC} topic — created new topic and injected transform at t=0")

        # Copy remaining messages unchanged and return early
        while reader.has_next():
            t, d, ts = reader.read_next()
            writer.write(t, bytes(d), ts)
        del reader, writer
        new_rpy = quaternion_to_rpy(qx, qy, qz, qw)
        ok(f"Inserted new transform  {parent} → {child}")
        ok(f"  xyz=[{x:.4f}, {y:.4f}, {z:.4f}] m  "
           f"rpy=[{new_rpy[0]:.2f}, {new_rpy[1]:.2f}, {new_rpy[2]:.2f}]°")
        return

    patched_count = 0   # number of existing occurrences updated
    inserted      = False
    first_ts      = None  # earliest timestamp seen — used as fallback inject point

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if first_ts is None:
            first_ts = timestamp

        if topic == TF_STATIC:
            msg = deserialize_message(data, TFMessage)

            # Update all existing occurrences of this transform pair
            for tf in msg.transforms:
                if tf.header.frame_id == parent and tf.child_frame_id == child:
                    old_t = tf.transform.translation
                    old_q = tf.transform.rotation
                    old_rpy = quaternion_to_rpy(old_q.x, old_q.y, old_q.z, old_q.w)
                    info(f"  Found existing transform at t={timestamp / 1e9:.3f}s:")
                    info(f"    xyz=[{old_t.x:.4f}, {old_t.y:.4f}, {old_t.z:.4f}] m  "
                         f"rpy=[{old_rpy[0]:.2f}, {old_rpy[1]:.2f}, {old_rpy[2]:.2f}]°")

                    tf.transform.translation.x = x
                    tf.transform.translation.y = y
                    tf.transform.translation.z = z
                    tf.transform.rotation.x    = qx
                    tf.transform.rotation.y    = qy
                    tf.transform.rotation.z    = qz
                    tf.transform.rotation.w    = qw
                    patched_count += 1

            # If not found yet, append to the first /tf_static message
            if patched_count == 0 and not inserted:
                msg.transforms.append(
                    make_transform(parent, child, x, y, z, qx, qy, qz, qw))
                inserted = True
                info(f"  No existing transform found — inserting into /tf_static "
                     f"at t={timestamp / 1e9:.3f}s")

            data = bytes(serialize_message(msg))

        writer.write(topic, bytes(data), timestamp)

    # /tf_static topic was in metadata but had zero messages — inject one now
    if patched_count == 0 and not inserted:
        inject_ts = first_ts if first_ts is not None else 0
        new_msg = TFMessage()
        new_msg.transforms.append(
            make_transform(parent, child, x, y, z, qx, qy, qz, qw))
        writer.write(TF_STATIC, bytes(serialize_message(new_msg)), inject_ts)
        warn(f"/tf_static topic had no messages — injected new transform at t={inject_ts / 1e9:.3f}s")
        inserted = True

    del reader
    del writer

    new_rpy = quaternion_to_rpy(qx, qy, qz, qw)
    if patched_count > 0:
        ok(f"Updated {patched_count} occurrence(s) of  {parent} → {child}")
    else:
        ok(f"Inserted new transform  {parent} → {child}")
    ok(f"  xyz=[{x:.4f}, {y:.4f}, {z:.4f}] m  "
       f"rpy=[{new_rpy[0]:.2f}, {new_rpy[1]:.2f}, {new_rpy[2]:.2f}]°")


# ─────────────────────────────────────────────────────────────────────────────
# Inspect helper
# ─────────────────────────────────────────────────────────────────────────────

def inspect_bag(bag_path: str):
    """Print all /tf_static transforms found in the bag."""
    TF_STATIC = "/tf_static"
    reader = open_reader(bag_path)
    topic_types = reader.get_all_topics_and_types()

    print()
    print(f"{BOLD}Topics in bag:{RESET}")
    for t in topic_types:
        marker = f"  {GREEN}←{RESET}" if t.name == TF_STATIC else ""
        print(f"  {t.name}  ({t.type}){marker}")
    print()

    has_tf_static = any(t.name == TF_STATIC for t in topic_types)
    if not has_tf_static:
        warn(f"No {TF_STATIC} topic found in bag metadata.")
        del reader
        return

    print(f"{BOLD}/tf_static transforms:{RESET}")
    found_any = False
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != TF_STATIC:
            continue
        msg = deserialize_message(data, TFMessage)
        for tf in msg.transforms:
            t = tf.transform.translation
            q = tf.transform.rotation
            rpy = quaternion_to_rpy(q.x, q.y, q.z, q.w)
            print(f"  {tf.header.frame_id}  →  {tf.child_frame_id}")
            print(f"    xyz=[{t.x:.4f}, {t.y:.4f}, {t.z:.4f}] m  "
                  f"rpy=[{rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}]°")
            found_any = True

    if not found_any:
        warn(f"{TF_STATIC} topic exists in metadata but has zero messages.")
    print()
    del reader


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def parse_args():
    parser = argparse.ArgumentParser(
        description="Patch a static TF in a ROS2 MCAP bag.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("bag",          help="Input bag directory")
    parser.add_argument("parent_frame", help="TF parent frame ID", default="usv/base_link")
    parser.add_argument("child_frame",  help="TF child frame ID", default="usv/sonar_link")
    parser.add_argument("--xyz", nargs=3, type=float, default=[0.38, 0.08, -0.525],
                        metavar=("X", "Y", "Z"), help="Translation in metres")
    parser.add_argument("--rpy", nargs=3, type=float, default=[0.0, 30.0, 180.0],
                        metavar=("R", "P", "Y"), help="Rotation in degrees (roll pitch yaw)")
    parser.add_argument("-o", "--output", default=None,
                        help="Output bag directory (default: <bag>_patched)")
    parser.add_argument("--inplace", action="store_true",
                        help="Overwrite the input bag")
    parser.add_argument("--inspect", action="store_true",
                        help="Print all existing /tf_static transforms and exit (no writing)")
    return parser.parse_args()


def main():
    args = parse_args()

    if not os.path.isdir(args.bag):
        err(f"Bag path not found or not a directory: {args.bag}")
        sys.exit(1)

    if args.inspect:
        inspect_bag(args.bag)
        return

    x, y, z = args.xyz
    roll, pitch, yaw = [math.radians(d) for d in args.rpy]
    qx, qy, qz, qw = rpy_to_quaternion(roll, pitch, yaw)

    print()
    print(f"{BOLD}╔══════════════════════════════════════╗{RESET}")
    print(f"{BOLD}║   ROS2 Static TF Bag Patcher         ║{RESET}")
    print(f"{BOLD}╚══════════════════════════════════════╝{RESET}")
    print()
    info(f"Input bag    : {args.bag}")
    info(f"Transform    : {args.parent_frame}  →  {args.child_frame}")
    info(f"Translation  : x={x}  y={y}  z={z}  m")
    info(f"Rotation     : roll={args.rpy[0]}°  pitch={args.rpy[1]}°  yaw={args.rpy[2]}°")
    print()

    # Resolve output path
    if args.inplace:
        tmp_out = tempfile.mktemp(prefix=".tf_patch_tmp_")
        final_out = args.bag
    elif args.output:
        tmp_out = args.output
        final_out = None
    else:
        tmp_out = args.bag.rstrip("/") + "_patched"
        final_out = None

    rclpy.init()
    try:
        patch_bag(args.bag, tmp_out,
                  args.parent_frame, args.child_frame,
                  x, y, z, qx, qy, qz, qw)
    finally:
        rclpy.shutdown()

    # Replace original if --inplace
    if args.inplace:
        backup = args.bag.rstrip("/") + "_backup"
        shutil.move(args.bag, backup)
        shutil.move(tmp_out, final_out)
        ok(f"Original bag backed up to: {backup}")
        ok(f"Patched bag written to   : {final_out}")
    else:
        ok(f"Patched bag written to   : {tmp_out}")

    print()


if __name__ == "__main__":
    main()

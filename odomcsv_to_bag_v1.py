#!/usr/bin/env python3
"""Convert odometry.csv plus optional path.csv into one ROS 1 Noetic bag.

Odometry CSV columns:
  seq, timestamp_sec, timestamp_nsec, frame_id, child_frame_id,
  position_x/y/z, orientation_x/y/z/w,
  linear_velocity_x/y/z, angular_velocity_x/y/z,
  pose_covariance_0..35, twist_covariance_0..35

Path CSV columns (as in the supplied path.csv):
  path_header_seq, path_stamp_sec, path_stamp_nsec, path_frame_id,
  pose_index, pose_header_seq, pose_stamp_sec, pose_stamp_nsec,
  pose_frame_id, position_x/y/z, orientation_x/y/z/w

With --path-csv, this writes both topics:
  /vins_estimator/odometry : nav_msgs/Odometry
  /vins_estimator/path     : nav_msgs/Path

Each path CSV row adds one PoseStamped to a cumulative Path snapshot. Therefore
row N is written as a Path message containing poses 0..N, like a typical
published nav_msgs/Path whose poses array grows over time.

Example:
  source /opt/ros/noetic/setup.bash
  python3 odomcsv_to_bag_v3.py /session/odometry.csv \
      --path-csv /session/path.csv -o /session/combined.bag

If the two CSVs are from the same session but their timestamps use different
origins, add --align-path-to-odom-start. This shifts the path header and all
PoseStamped timestamps by one constant offset so the first path event aligns
with the first odometry timestamp. By default, original timestamps are kept.
"""

import argparse
import csv
import math
import os

import rosbag
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path

DEFAULT_ODOM_TOPIC = "/vins_estimator/odometry"
DEFAULT_PATH_TOPIC = "/vins_estimator/path"


def number(row, key, default=0.0):
    value = row.get(key, "")
    if value is None or value.strip() == "":
        return default
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("Non-finite value in column {}: {}".format(key, value))
    return result


def integer(row, key, default=None):
    value = row.get(key, "")
    if value is None or value.strip() == "":
        if default is not None:
            return default
        raise ValueError("Empty integer column: {}".format(key))
    return int(value)


def covariance(row, prefix):
    values = []
    for i in range(36):
        key = "{}_{}".format(prefix, i)
        if key not in row:
            raise ValueError("Missing required CSV column: {}".format(key))
        values.append(number(row, key))
    return values


def timestamp_ns(sec, nsec, label="timestamp"):
    sec = int(sec)
    nsec = int(nsec)
    if sec < 0 or nsec < 0 or nsec >= 1_000_000_000:
        raise ValueError("Invalid {}: sec={}, nsec={}".format(label, sec, nsec))
    return sec * 1_000_000_000 + nsec


def time_from_ns(value):
    if value < 0:
        raise ValueError("ROS timestamp would be negative: {} ns".format(value))
    sec, nsec = divmod(int(value), 1_000_000_000)
    return rospy.Time(secs=sec, nsecs=nsec)


def read_csv(path):
    with open(path, "r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fields = set(reader.fieldnames or [])
    if not rows:
        raise ValueError("CSV has no data rows: {}".format(path))
    return rows, fields


def load_odometry(csv_path):
    rows, columns = read_csv(csv_path)
    required = [
        "timestamp_sec", "timestamp_nsec", "frame_id", "child_frame_id",
        "position_x", "position_y", "position_z",
        "orientation_x", "orientation_y", "orientation_z", "orientation_w",
        "linear_velocity_x", "linear_velocity_y", "linear_velocity_z",
        "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
    ]
    missing = [key for key in required if key not in columns]
    for prefix in ("pose_covariance", "twist_covariance"):
        missing.extend("{}_{}".format(prefix, i) for i in range(36)
                       if "{}_{}".format(prefix, i) not in columns)
    if missing:
        raise ValueError("Odometry CSV is missing columns: " + ", ".join(missing))

    for row in rows:
        row["_timestamp_ns"] = timestamp_ns(
            row["timestamp_sec"], row["timestamp_nsec"], "odometry timestamp"
        )
    rows.sort(key=lambda row: row["_timestamp_ns"])
    return rows


def load_path(csv_path):
    rows, columns = read_csv(csv_path)
    required = [
        "path_header_seq", "path_stamp_sec", "path_stamp_nsec", "path_frame_id",
        "pose_index", "pose_header_seq", "pose_stamp_sec", "pose_stamp_nsec",
        "pose_frame_id", "position_x", "position_y", "position_z",
        "orientation_x", "orientation_y", "orientation_z", "orientation_w",
    ]
    missing = [key for key in required if key not in columns]
    if missing:
        raise ValueError("Path CSV is missing columns: " + ", ".join(missing))

    for row in rows:
        row["_path_timestamp_ns"] = timestamp_ns(
            row["path_stamp_sec"], row["path_stamp_nsec"], "Path header timestamp"
        )
        row["_pose_timestamp_ns"] = timestamp_ns(
            row["pose_stamp_sec"], row["pose_stamp_nsec"], "PoseStamped timestamp"
        )
        row["_pose_index"] = integer(row, "pose_index")
    rows.sort(key=lambda row: row["_pose_index"])
    return rows


def make_odometry(row):
    msg = Odometry()
    seq_text = (row.get("seq") or "").strip()
    if seq_text:
        msg.header.seq = int(seq_text)
    msg.header.stamp = time_from_ns(row["_timestamp_ns"])
    msg.header.frame_id = (row.get("frame_id") or "world").strip() or "world"
    msg.child_frame_id = (row.get("child_frame_id") or "world").strip() or "world"

    msg.pose.pose.position.x = number(row, "position_x")
    msg.pose.pose.position.y = number(row, "position_y")
    msg.pose.pose.position.z = number(row, "position_z")
    msg.pose.pose.orientation.x = number(row, "orientation_x")
    msg.pose.pose.orientation.y = number(row, "orientation_y")
    msg.pose.pose.orientation.z = number(row, "orientation_z")
    msg.pose.pose.orientation.w = number(row, "orientation_w")

    msg.twist.twist.linear.x = number(row, "linear_velocity_x")
    msg.twist.twist.linear.y = number(row, "linear_velocity_y")
    msg.twist.twist.linear.z = number(row, "linear_velocity_z")
    msg.twist.twist.angular.x = number(row, "angular_velocity_x")
    msg.twist.twist.angular.y = number(row, "angular_velocity_y")
    msg.twist.twist.angular.z = number(row, "angular_velocity_z")
    msg.pose.covariance = covariance(row, "pose_covariance")
    msg.twist.covariance = covariance(row, "twist_covariance")
    return msg


def make_pose_stamped(row, offset_ns=0):
    pose = PoseStamped()
    seq_text = (row.get("pose_header_seq") or "").strip()
    if seq_text:
        pose.header.seq = int(seq_text)
    pose.header.stamp = time_from_ns(row["_pose_timestamp_ns"] + offset_ns)
    pose.header.frame_id = (row.get("pose_frame_id") or "world").strip() or "world"
    pose.pose.position.x = number(row, "position_x")
    pose.pose.position.y = number(row, "position_y")
    pose.pose.position.z = number(row, "position_z")
    pose.pose.orientation.x = number(row, "orientation_x")
    pose.pose.orientation.y = number(row, "orientation_y")
    pose.pose.orientation.z = number(row, "orientation_z")
    pose.pose.orientation.w = number(row, "orientation_w")
    return pose


def main():
    parser = argparse.ArgumentParser(
        description="Convert odometry.csv and optional path.csv into one ROS 1 bag."
    )
    parser.add_argument("csv_file", help="Input odometry CSV file")
    parser.add_argument("-o", "--output", default="odometry.bag",
                        help="Output .bag path (default: ./odometry.bag)")
    parser.add_argument("--topic", default=DEFAULT_ODOM_TOPIC,
                        help="Odometry topic (default: {})".format(DEFAULT_ODOM_TOPIC))
    parser.add_argument("--path-csv", help="Optional path.csv to add as nav_msgs/Path")
    parser.add_argument("--path-topic", default=DEFAULT_PATH_TOPIC,
                        help="Path topic (default: {})".format(DEFAULT_PATH_TOPIC))
    parser.add_argument("--align-path-to-odom-start", action="store_true",
                        help="Shift path timestamps so its first header stamp matches the first odometry stamp")
    parser.add_argument("--overwrite", action="store_true",
                        help="Replace the output bag if it already exists")
    args = parser.parse_args()

    odom_csv = os.path.abspath(os.path.expanduser(args.csv_file))
    path_csv = (os.path.abspath(os.path.expanduser(args.path_csv))
                if args.path_csv else None)
    output_path = os.path.abspath(os.path.expanduser(args.output))

    if not os.path.isfile(odom_csv):
        parser.error("Odometry CSV does not exist: {}".format(odom_csv))
    if path_csv and not os.path.isfile(path_csv):
        parser.error("Path CSV does not exist: {}".format(path_csv))
    if output_path in (odom_csv, path_csv):
        parser.error("Output bag path cannot be the same as an input CSV path.")
    if os.path.exists(output_path) and not args.overwrite:
        parser.error("Output already exists: {} (use --overwrite to replace it)".format(output_path))
    if args.align_path_to_odom_start and not path_csv:
        parser.error("--align-path-to-odom-start requires --path-csv")

    try:
        odom_rows = load_odometry(odom_csv)
        path_rows = load_path(path_csv) if path_csv else []

        path_offset_ns = 0
        if path_rows and args.align_path_to_odom_start:
            path_offset_ns = odom_rows[0]["_timestamp_ns"] - path_rows[0]["_path_timestamp_ns"]

        # Build a unified event stream and sort it by bag timestamp, so ROS bag
        # playback follows time order even though the two CSVs are read separately.
        events = []
        serial = 0
        for row in odom_rows:
            events.append((row["_timestamp_ns"], serial, args.topic, make_odometry(row)))
            serial += 1

        if path_rows:
            accumulated_poses = []
            for row in path_rows:
                accumulated_poses.append(make_pose_stamped(row, path_offset_ns))
                path_msg = Path()
                path_seq_text = (row.get("path_header_seq") or "").strip()
                if path_seq_text:
                    path_msg.header.seq = int(path_seq_text)
                path_msg.header.stamp = time_from_ns(row["_path_timestamp_ns"] + path_offset_ns)
                path_msg.header.frame_id = (row.get("path_frame_id") or "world").strip() or "world"
                # Snapshot the list so each bag message retains the path size at that update.
                path_msg.poses = list(accumulated_poses)
                event_time_ns = row["_path_timestamp_ns"] + path_offset_ns
                events.append((event_time_ns, serial, args.path_topic, path_msg))
                serial += 1

        events.sort(key=lambda event: (event[0], event[1]))
        os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)

        # Open in write mode only after validating and constructing all messages.
        with rosbag.Bag(output_path, "w") as bag:
            for event_time_ns, _, topic, msg in events:
                bag.write(topic, msg, t=time_from_ns(event_time_ns))

    except Exception as exc:
        # Do not remove a pre-existing user bag on failure. Existing output is
        # only allowed when --overwrite was explicitly requested.
        if os.path.exists(output_path):
            try:
                os.remove(output_path)
            except OSError:
                pass
        parser.exit(2, "ERROR: {}\n".format(exc))

    print("Odometry messages: {}".format(len(odom_rows)))
    print("Odometry topic: {} [nav_msgs/Odometry]".format(args.topic))
    if path_rows:
        print("Path updates: {}".format(len(path_rows)))
        print("Path topic: {} [nav_msgs/Path]".format(args.path_topic))
        print("Final Path message contains {} poses.".format(len(path_rows)))
        if args.align_path_to_odom_start:
            print("Path timestamp offset applied: {:+.9f} seconds".format(path_offset_ns / 1e9))
        else:
            print("Path timestamps kept unchanged.")
    print("Output bag: {}".format(output_path))
    print("Inspect with: rosbag info '{}'".format(output_path))


if __name__ == "__main__":
    main()

"""
source /opt/ros/noetic/setup.bash
source /home/idle-omen2/catkin_ws/install_isolated/setup.bash

SENSOR_SESSION="/home/idle-omen2/sensor_logs_local/session_20261012_150118"
VIO_SESSION="/home/idle-omen2/vio_logs_local/session_20261010_150122"

PYTHONNOUSERSITE=1 python3 \
/home/idle-omen2/Python/combined_csv_bag.py \
--dataset "$SENSOR_SESSION" \
--vio-dataset "$VIO_SESSION" \
--output "$SENSOR_SESSION/combined_all_topics.bag"

"""

#!/usr/bin/env python3
"""Create ONE ROS 1 Noetic bag from sensor logs plus VIO odometry/path CSVs.

Sensor dataset folder (from the sensor converter):
  gps_data.csv, imu_data.csv, camera_frames.csv, frames/

VIO dataset folder:
  odometry.csv, path.csv

Topics written:
  /idle_cam/image_raw/compressed       sensor_msgs/CompressedImage
  /idle_cam/camera_info                sensor_msgs/CameraInfo
  /ext_mav/gps_input/gps_input         mavros_msgs/GPSINPUT
  /mav/global_position/global          sensor_msgs/NavSatFix
  /imu                                 sensor_msgs/Imu
  /imu/mag                             sensor_msgs/MagneticField
  /vins_estimator/odometry             nav_msgs/Odometry
  /vins_estimator/path                 nav_msgs/Path
  /feature_tracker/feature_img         sensor_msgs/Image (optional, from feature_img.csv)

Timestamp note:
  Sensor events use the first IMU timestamp as the reference and start at ROS
  time 1 second, preserving sensor inter-stream timing as in the original code.
  By default, odometry stamps are assumed to share that boot-time clock, which
  preserves VIO startup delay relative to the sensors. Path header stamps are
  aligned to the first odometry message because the supplied path header timestamps
  use a different origin. If odometry stamps do not share the IMU clock, pass
  --vio-time-mode align-start. Exact synchronization cannot be recovered between
  unrelated clocks or unrelated recordings without a known offset.
"""

import argparse
import base64
import csv
import heapq
import math
import os
import sys
import zlib

import rosbag
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GPSINPUT
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import (
    CameraInfo,
    CompressedImage,
    Image,
    Imu,
    MagneticField,
    NavSatFix,
    NavSatStatus,
)

# ---------------------------------------------------------------------------
# Topics
# ---------------------------------------------------------------------------
CAMERA_TOPIC = "/idle_cam/image_raw/compressed"
CAMERA_INFO_TOPIC = "/idle_cam/camera_info"
GPS_TOPIC = "/ext_mav/gps_input/gps_input"
NAVSAT_TOPIC = "/mav/global_position/global"
IMU_TOPIC = "/imu"
MAG_TOPIC = "/imu/mag"
ODOM_TOPIC = "/vins_estimator/odometry"
PATH_TOPIC = "/vins_estimator/path"
FEATURE_IMG_TOPIC = "/feature_tracker/feature_img"

# feature_img.csv uses one Base64 field per image. Python's csv module has a
# small default field limit, so raise it before any feature-image row is read.
csv.field_size_limit(sys.maxsize)

BAG_START_NS = 1_000_000_000  # Keep the bag timeline starting at ROS time 1.0 s.


# ---------------------------------------------------------------------------
# CSV / timestamp helpers
# ---------------------------------------------------------------------------
def safe_int(value, default=0):
    if value is None:
        return default
    value = str(value).strip()
    if value == "":
        return default
    try:
        # Avoid converting large nanosecond timestamps through float, which
        # would lose precision. Fall back to float only for values like "12.0".
        return int(value)
    except (ValueError, TypeError, OverflowError):
        try:
            return int(float(value))
        except (ValueError, TypeError, OverflowError):
            return default


def safe_float(value, default=0.0):
    if value is None:
        return default
    value = str(value).strip()
    if value == "":
        return default
    try:
        result = float(value)
        return result if math.isfinite(result) else default
    except (ValueError, TypeError, OverflowError):
        return default


def required_float(row, key):
    value = row.get(key, "")
    if value is None or str(value).strip() == "":
        raise ValueError("Empty required numeric column: {}".format(key))
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("Non-finite value in column {}: {}".format(key, value))
    return result


def read_csv_rows(csv_path):
    if not os.path.isfile(csv_path):
        raise FileNotFoundError("CSV file not found: {}".format(csv_path))
    with open(csv_path, "r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        fields = set(reader.fieldnames or [])
        rows = list(reader)
    if not rows:
        raise ValueError("CSV has no data rows: {}".format(csv_path))
    return rows, fields


def first_timestamp_ns(csv_path, key="timestamp_ns"):
    with open(csv_path, "r", newline="", encoding="utf-8-sig") as csv_file:
        for row in csv.DictReader(csv_file):
            value = row.get(key)
            if value is None or str(value).strip() == "":
                continue
            try:
                result = int(value)
            except (ValueError, TypeError, OverflowError):
                try:
                    result = int(float(value))
                except (ValueError, TypeError, OverflowError):
                    continue
            if result > 0:
                return result
    return None


def ns_to_ros_time(relative_ns):
    """Map nonnegative relative nanoseconds to ROS time, with a 1 s base."""
    relative_ns = max(0, int(relative_ns)) + BAG_START_NS
    secs, nsecs = divmod(relative_ns, 1_000_000_000)
    return rospy.Time(int(secs), int(nsecs))


def time_from_absolute_ns(timestamp_ns):
    timestamp_ns = int(timestamp_ns)
    if timestamp_ns < 0:
        raise ValueError("Negative ROS timestamp: {} ns".format(timestamp_ns))
    secs, nsecs = divmod(timestamp_ns, 1_000_000_000)
    return rospy.Time(secs=int(secs), nsecs=int(nsecs))


def split_timestamp(sec_value, nsec_value, label):
    sec = int(sec_value)
    nsec = int(nsec_value)
    if sec < 0 or nsec < 0 or nsec >= 1_000_000_000:
        raise ValueError("Invalid {}: sec={}, nsec={}".format(label, sec, nsec))
    return sec * 1_000_000_000 + nsec


def normalize_quaternion(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def merge_generators(generators):
    """Merge time-sorted event generators without storing all image data in RAM.

    Each yielded event is (relative_ns, priority, topic, message). All generators
    must yield nondecreasing relative_ns values.
    """
    heap = []
    for generator_index, generator in enumerate(generators):
        try:
            event = next(generator)
            heapq.heappush(heap, (event[0], event[1], generator_index, event))
        except StopIteration:
            pass

    while heap:
        _, _, generator_index, event = heapq.heappop(heap)
        yield event
        try:
            next_event = next(generators[generator_index])
            heapq.heappush(
                heap,
                (next_event[0], next_event[1], generator_index, next_event),
            )
        except StopIteration:
            pass


# ---------------------------------------------------------------------------
# Sensor event generators (preserve the behavior of the supplied converter)
# ---------------------------------------------------------------------------
def generate_imu_events(csv_file, reference_ns):
    with open(csv_file, "r", newline="", encoding="utf-8-sig") as csv_file_obj:
        reader = csv.DictReader(csv_file_obj)
        for seq, row in enumerate(reader):
            timestamp_ns = safe_int(row.get("timestamp_ns"))
            if timestamp_ns <= 0:
                continue
            relative_ns = timestamp_ns - reference_ns
            if relative_ns < 0:
                continue
            stamp = ns_to_ros_time(relative_ns)
            frame_id = row.get("frame_id") or "imu_link"

            imu_msg = Imu()
            imu_msg.header.seq = seq
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = frame_id

            qx, qy, qz, qw = normalize_quaternion(
                safe_float(row.get("qx")),
                safe_float(row.get("qy")),
                safe_float(row.get("qz")),
                safe_float(row.get("qw"), 1.0),
            )
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            for i in range(9):
                imu_msg.orientation_covariance[i] = safe_float(row.get("q_cov{}".format(i)))

            imu_msg.angular_velocity.x = safe_float(row.get("wx"))
            imu_msg.angular_velocity.y = safe_float(row.get("wy"))
            imu_msg.angular_velocity.z = safe_float(row.get("wz"))
            for i in range(9):
                imu_msg.angular_velocity_covariance[i] = safe_float(row.get("w_cov{}".format(i)))

            imu_msg.linear_acceleration.x = safe_float(row.get("ax"))
            imu_msg.linear_acceleration.y = safe_float(row.get("ay"))
            imu_msg.linear_acceleration.z = safe_float(row.get("az"))
            for i in range(9):
                imu_msg.linear_acceleration_covariance[i] = safe_float(row.get("a_cov{}".format(i)))

            yield relative_ns, 1, IMU_TOPIC, imu_msg

            mag_msg = MagneticField()
            mag_msg.header.seq = seq
            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = frame_id
            # Input magnetometer values are in microtesla; ROS uses tesla.
            mag_msg.magnetic_field.x = safe_float(row.get("mag_x")) * 1e-6
            mag_msg.magnetic_field.y = safe_float(row.get("mag_y")) * 1e-6
            mag_msg.magnetic_field.z = safe_float(row.get("mag_z")) * 1e-6
            mag_msg.magnetic_field_covariance[0] = -1.0
            yield relative_ns, 2, MAG_TOPIC, mag_msg


def generate_gps_events(csv_file, gps_reference_ns, gps_offset_ns):
    with open(csv_file, "r", newline="", encoding="utf-8-sig") as csv_file_obj:
        reader = csv.DictReader(csv_file_obj)
        for seq, row in enumerate(reader):
            timestamp_ns = safe_int(row.get("timestamp_ns"))
            if timestamp_ns <= 0:
                continue
            relative_ns = timestamp_ns - gps_reference_ns + gps_offset_ns
            if relative_ns < 0:
                continue
            stamp = ns_to_ros_time(relative_ns)
            frame_id = row.get("frame_id") or "gps_link"

            gps_msg = GPSINPUT()
            gps_msg.header.seq = seq
            gps_msg.header.stamp = stamp
            gps_msg.header.frame_id = frame_id
            gps_msg.fix_type = safe_int(row.get("fix_type"))
            gps_msg.gps_id = safe_int(row.get("gps_id"))
            gps_msg.ignore_flags = safe_int(row.get("ignore_flags"))
            gps_msg.time_week_ms = safe_int(row.get("time_week_ms"))
            gps_msg.time_week = safe_int(row.get("time_week"))
            gps_msg.lat = safe_int(row.get("lat"))
            gps_msg.lon = safe_int(row.get("lon"))
            gps_msg.alt = safe_float(row.get("alt"))
            gps_msg.hdop = safe_float(row.get("hdop"))
            gps_msg.vdop = safe_float(row.get("vdop"))
            gps_msg.vn = safe_float(row.get("vn"))
            gps_msg.ve = safe_float(row.get("ve"))
            gps_msg.vd = safe_float(row.get("vd"))
            gps_msg.speed_accuracy = safe_float(row.get("speed_acc"))
            gps_msg.horiz_accuracy = safe_float(row.get("horiz_acc"))
            gps_msg.vert_accuracy = safe_float(row.get("vert_acc"))
            gps_msg.satellites_visible = safe_int(row.get("satellites"))
            if hasattr(gps_msg, "yaw"):
                gps_msg.yaw = safe_int(row.get("yaw"))

            navsat_msg = NavSatFix()
            navsat_msg.header.seq = seq
            navsat_msg.header.stamp = stamp
            navsat_msg.header.frame_id = frame_id
            fix_type = safe_int(row.get("fix_type"))
            navsat_msg.status.status = (
                NavSatStatus.STATUS_FIX if fix_type >= 2 else NavSatStatus.STATUS_NO_FIX
            )
            navsat_msg.status.service = NavSatStatus.SERVICE_GPS
            navsat_msg.latitude = safe_float(row.get("lat")) / 1e7
            navsat_msg.longitude = safe_float(row.get("lon")) / 1e7
            navsat_msg.altitude = safe_float(row.get("alt"))
            horiz_acc = safe_float(row.get("horiz_acc"))
            vert_acc = safe_float(row.get("vert_acc"))
            east_variance = horiz_acc * horiz_acc / 2.0
            north_variance = horiz_acc * horiz_acc / 2.0
            up_variance = vert_acc * vert_acc
            navsat_msg.position_covariance = [
                east_variance, 0.0, 0.0,
                0.0, north_variance, 0.0,
                0.0, 0.0, up_variance,
            ]
            navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            yield relative_ns, 0, GPS_TOPIC, gps_msg
            yield relative_ns, 0, NAVSAT_TOPIC, navsat_msg


def generate_camera_events(csv_file, frames_dir, reference_ns):
    # The source converter assumes camera_frames.csv is chronological.
    with open(csv_file, "r", newline="", encoding="utf-8-sig") as csv_file_obj:
        reader = csv.DictReader(csv_file_obj)
        for seq, row in enumerate(reader):
            timestamp_ns = safe_int(row.get("timestamp_ns"))
            if timestamp_ns <= 0:
                continue
            filename = (row.get("filename") or "").strip()
            if not filename:
                continue
            image_path = os.path.join(frames_dir, filename)
            if not os.path.isfile(image_path):
                continue
            try:
                with open(image_path, "rb") as image_file:
                    jpeg_data = image_file.read()
            except OSError:
                continue
            if not (
                len(jpeg_data) >= 4
                and jpeg_data.startswith(b"\xff\xd8")
                and jpeg_data.endswith(b"\xff\xd9")
            ):
                continue

            relative_ns = timestamp_ns - reference_ns
            if relative_ns < 0:
                continue
            stamp = ns_to_ros_time(relative_ns)

            img_msg = CompressedImage()
            img_msg.header.seq = seq
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = "camera"
            img_msg.format = "bgr8; jpeg [rgb8]"
            img_msg.data = jpeg_data

            cam_info = CameraInfo()
            cam_info.header.seq = seq
            cam_info.header.stamp = stamp
            cam_info.header.frame_id = "camera"
            cam_info.width = 1280
            cam_info.height = 720
            cam_info.distortion_model = "plumb_bob"
            cam_info.D = [
                0.08537173793341997,
                -0.15724369761506035,
                -0.0027100251818195555,
                -0.00067514168023245,
                0.0,
            ]
            cam_info.K = [
                930.2033563961248, 0.0, 637.3689177579088,
                0.0, 928.6204814264393, 351.2245451749831,
                0.0, 0.0, 1.0,
            ]
            cam_info.R = [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
            ]
            cam_info.P = [
                930.2033563961248, 0.0, 637.3689177579088, 0.0,
                0.0, 928.6204814264393, 351.2245451749831, 0.0,
                0.0, 0.0, 1.0, 0.0,
            ]
            cam_info.binning_x = 0
            cam_info.binning_y = 0

            yield relative_ns, 3, CAMERA_TOPIC, img_msg
            yield relative_ns, 4, CAMERA_INFO_TOPIC, cam_info


# ---------------------------------------------------------------------------
# VINS-Mono feature-tracking visualization image events
# ---------------------------------------------------------------------------
def generate_feature_image_events(csv_path, imu_reference_ns):
    """Rebuild sensor_msgs/Image from zlib-compressed, Base64 BGR8 bytes.

    The logger stores pixels losslessly: data_base64 is Base64(zlib(raw BGR8)).
    It is deliberately not JPEG-compressed, so the resulting Image.data is the
    same raw byte layout represented by encoding=bgr8 and step=width*3.
    The input clock is the camera/IMU boot-time clock used by the other sensor
    CSVs; events are placed on the common bag timeline using the IMU reference.
    """
    required = {
        "seq", "timestamp_sec", "timestamp_nsec", "frame_id", "height",
        "width", "encoding", "is_bigendian", "step", "data_format",
        "data_base64",
    }
    previous_relative_ns = -1

    with open(csv_path, "r", newline="", encoding="utf-8-sig") as csv_file_obj:
        reader = csv.DictReader(csv_file_obj)
        columns = set(reader.fieldnames or [])
        missing = sorted(required - columns)
        if missing:
            raise ValueError(
                "Feature image CSV missing columns: " + ", ".join(missing)
            )

        for line_number, row in enumerate(reader, start=2):
            timestamp_ns = split_timestamp(
                row["timestamp_sec"], row["timestamp_nsec"],
                "feature image timestamp at line {}".format(line_number),
            )
            relative_ns = timestamp_ns - imu_reference_ns
            if relative_ns < 0:
                continue
            if relative_ns < previous_relative_ns:
                raise ValueError(
                    "feature_img.csv timestamps are not chronological at line {}".
                    format(line_number)
                )
            previous_relative_ns = relative_ns

            width = safe_int(row.get("width"), -1)
            height = safe_int(row.get("height"), -1)
            encoding = (row.get("encoding") or "").strip()
            is_bigendian = safe_int(row.get("is_bigendian"), -1)
            step = safe_int(row.get("step"), -1)
            data_format = (row.get("data_format") or "").strip()

            if width <= 0 or height <= 0:
                raise ValueError(
                    "Invalid feature image dimensions at line {}: {}x{}".
                    format(line_number, width, height)
                )
            if encoding != "bgr8":
                raise ValueError(
                    "Expected bgr8 feature image at line {}, got {!r}".
                    format(line_number, encoding)
                )
            if is_bigendian not in (0, 1):
                raise ValueError(
                    "Invalid is_bigendian at feature image line {}".format(line_number)
                )
            if step != width * 3:
                raise ValueError(
                    "Invalid feature image step at line {}: got {}, expected {}".
                    format(line_number, step, width * 3)
                )
            if data_format != "zlib_base64":
                raise ValueError(
                    "Unsupported feature image data_format at line {}: {!r}".
                    format(line_number, data_format)
                )

            encoded = (row.get("data_base64") or "").strip()
            if not encoded:
                raise ValueError(
                    "Empty feature image payload at line {}".format(line_number)
                )
            try:
                compressed = base64.b64decode(encoded, validate=True)
                raw_data = zlib.decompress(compressed)
            except (ValueError, zlib.error) as exc:
                raise ValueError(
                    "Cannot decode feature image payload at line {}: {}".
                    format(line_number, exc)
                )

            expected_size = step * height
            if len(raw_data) != expected_size:
                raise ValueError(
                    "Feature image payload size mismatch at line {}: got {}, expected {}".
                    format(line_number, len(raw_data), expected_size)
                )

            msg = Image()
            msg.header.seq = safe_int(row.get("seq"), line_number - 2)
            msg.header.stamp = ns_to_ros_time(relative_ns)
            msg.header.frame_id = (row.get("frame_id") or "camera").strip() or "camera"
            msg.height = height
            msg.width = width
            msg.encoding = encoding
            msg.is_bigendian = is_bigendian
            msg.step = step
            msg.data = raw_data

            # Priority 4 places the tracked visualization after the captured
            # camera image and before the VIO odometry event at the same time.
            yield relative_ns, 4, FEATURE_IMG_TOPIC, msg


# ---------------------------------------------------------------------------
# VIO odometry and path events
# ---------------------------------------------------------------------------
def load_odometry_rows(csv_path):
    rows, columns = read_csv_rows(csv_path)
    required = [
        "timestamp_sec", "timestamp_nsec", "frame_id", "child_frame_id",
        "position_x", "position_y", "position_z",
        "orientation_x", "orientation_y", "orientation_z", "orientation_w",
        "linear_velocity_x", "linear_velocity_y", "linear_velocity_z",
        "angular_velocity_x", "angular_velocity_y", "angular_velocity_z",
    ]
    missing = [key for key in required if key not in columns]
    for prefix in ("pose_covariance", "twist_covariance"):
        missing.extend(
            "{}_{}".format(prefix, index)
            for index in range(36)
            if "{}_{}".format(prefix, index) not in columns
        )
    if missing:
        raise ValueError("Odometry CSV missing columns: " + ", ".join(missing))

    for row in rows:
        row["_timestamp_ns"] = split_timestamp(
            row["timestamp_sec"], row["timestamp_nsec"], "odometry timestamp"
        )
    rows.sort(key=lambda row: row["_timestamp_ns"])
    return rows


def load_path_rows(csv_path):
    rows, columns = read_csv_rows(csv_path)
    required = [
        "path_header_seq", "path_stamp_sec", "path_stamp_nsec", "path_frame_id",
        "pose_index", "pose_header_seq", "pose_stamp_sec", "pose_stamp_nsec",
        "pose_frame_id", "position_x", "position_y", "position_z",
        "orientation_x", "orientation_y", "orientation_z", "orientation_w",
    ]
    missing = [key for key in required if key not in columns]
    if missing:
        raise ValueError("Path CSV missing columns: " + ", ".join(missing))

    for row in rows:
        row["_path_timestamp_ns"] = split_timestamp(
            row["path_stamp_sec"], row["path_stamp_nsec"], "Path header timestamp"
        )
        row["_pose_timestamp_ns"] = split_timestamp(
            row["pose_stamp_sec"], row["pose_stamp_nsec"], "PoseStamped timestamp"
        )
        row["_pose_index"] = int(row["pose_index"])
    rows.sort(key=lambda row: row["_pose_index"])
    # A merge heap requires each source's events to be chronological.
    stamps = [row["_path_timestamp_ns"] for row in rows]
    if any(stamps[i] > stamps[i + 1] for i in range(len(stamps) - 1)):
        raise ValueError("path.csv path_stamp timestamps are not chronological by pose_index.")
    return rows


def covariance(row, prefix):
    return [required_float(row, "{}_{}".format(prefix, i)) for i in range(36)]


def make_odometry_message(row, timestamp_ns):
    msg = Odometry()
    seq_text = (row.get("seq") or "").strip()
    if seq_text:
        msg.header.seq = int(seq_text)
    msg.header.stamp = time_from_absolute_ns(timestamp_ns)
    msg.header.frame_id = (row.get("frame_id") or "world").strip() or "world"
    msg.child_frame_id = (row.get("child_frame_id") or "world").strip() or "world"

    msg.pose.pose.position.x = required_float(row, "position_x")
    msg.pose.pose.position.y = required_float(row, "position_y")
    msg.pose.pose.position.z = required_float(row, "position_z")
    msg.pose.pose.orientation.x = required_float(row, "orientation_x")
    msg.pose.pose.orientation.y = required_float(row, "orientation_y")
    msg.pose.pose.orientation.z = required_float(row, "orientation_z")
    msg.pose.pose.orientation.w = required_float(row, "orientation_w")

    msg.twist.twist.linear.x = required_float(row, "linear_velocity_x")
    msg.twist.twist.linear.y = required_float(row, "linear_velocity_y")
    msg.twist.twist.linear.z = required_float(row, "linear_velocity_z")
    msg.twist.twist.angular.x = required_float(row, "angular_velocity_x")
    msg.twist.twist.angular.y = required_float(row, "angular_velocity_y")
    msg.twist.twist.angular.z = required_float(row, "angular_velocity_z")
    msg.pose.covariance = covariance(row, "pose_covariance")
    msg.twist.covariance = covariance(row, "twist_covariance")
    return msg


def make_pose_stamped(row, stamp_ns):
    pose = PoseStamped()
    seq_text = (row.get("pose_header_seq") or "").strip()
    if seq_text:
        pose.header.seq = int(seq_text)
    pose.header.stamp = time_from_absolute_ns(stamp_ns)
    pose.header.frame_id = (row.get("pose_frame_id") or "world").strip() or "world"
    pose.pose.position.x = required_float(row, "position_x")
    pose.pose.position.y = required_float(row, "position_y")
    pose.pose.position.z = required_float(row, "position_z")
    pose.pose.orientation.x = required_float(row, "orientation_x")
    pose.pose.orientation.y = required_float(row, "orientation_y")
    pose.pose.orientation.z = required_float(row, "orientation_z")
    pose.pose.orientation.w = required_float(row, "orientation_w")
    return pose


def generate_odometry_events(rows, imu_reference_ns, align_start=False):
    if not rows:
        return
    first_odom_ns = rows[0]["_timestamp_ns"]
    for row in rows:
        if align_start:
            # Fallback if odometry timestamps do not share the sensor clock origin.
            relative_ns = row["_timestamp_ns"] - first_odom_ns
        else:
            # Default: treat VIO header stamps as the same boot-time clock used by
            # camera/IMU timestamp_ns, preserving VIO's offset from sensor startup.
            relative_ns = row["_timestamp_ns"] - imu_reference_ns
        if relative_ns < 0:
            raise ValueError(
                "An odometry timestamp precedes the IMU time reference. "
                "Check whether the clocks match, or rerun with --vio-time-mode align-start."
            )
        output_stamp_ns = BAG_START_NS + relative_ns
        msg = make_odometry_message(row, output_stamp_ns)
        yield relative_ns, 5, ODOM_TOPIC, msg


def generate_path_events(rows, odom_start_relative_ns):
    if not rows:
        return
    # Path header stamps can come from a different clock origin. Align the first
    # Path update to the first odometry message, while preserving path timing gaps.
    first_path_ns = rows[0]["_path_timestamp_ns"]
    accumulated_poses = []

    for row in rows:
        relative_header_ns = (
            odom_start_relative_ns + row["_path_timestamp_ns"] - first_path_ns
        )
        relative_pose_ns = (
            odom_start_relative_ns + row["_pose_timestamp_ns"] - first_path_ns
        )
        if relative_header_ns < 0 or relative_pose_ns < 0:
            raise ValueError("Path timestamp becomes negative after alignment.")

        accumulated_poses.append(
            make_pose_stamped(row, BAG_START_NS + relative_pose_ns)
        )
        path_msg = Path()
        seq_text = (row.get("path_header_seq") or "").strip()
        if seq_text:
            path_msg.header.seq = int(seq_text)
        path_msg.header.stamp = time_from_absolute_ns(
            BAG_START_NS + relative_header_ns
        )
        path_msg.header.frame_id = (row.get("path_frame_id") or "world").strip() or "world"
        # Each bag event is a snapshot whose poses[] grows with each CSV row.
        path_msg.poses = list(accumulated_poses)
        yield relative_header_ns, 6, PATH_TOPIC, path_msg


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description=(
            "Combine sensor CSVs and VIO odometry/path CSVs into one ROS 1 Noetic bag."
        )
    )
    parser.add_argument(
        "--dataset", required=True,
        help="Sensor session directory containing gps_data.csv, imu_data.csv, camera_frames.csv and frames/",
    )
    parser.add_argument(
        "--vio-dataset", default=None,
        help="Directory containing odometry.csv and path.csv (default: same as --dataset)",
    )
    parser.add_argument("--odometry-csv", default=None, help="Override odometry CSV path")
    parser.add_argument("--path-csv", default=None, help="Override path CSV path")
    parser.add_argument(
        "--feature-img-csv", default=None,
        help="Optional feature_img.csv (default: <vio-dataset>/feature_img.csv if present)",
    )
    parser.add_argument("--output", default=None, help="Output bag path (default: <dataset>/combined_all_topics.bag)")
    parser.add_argument("--gps-offset", type=float, default=0.0, help="GPS time offset in seconds, applied as in the original sensor converter")
    parser.add_argument(
        "--vio-time-mode", choices=("shared-clock", "align-start"),
        default="shared-clock",
        help=(
            "shared-clock preserves odometry timing relative to IMU timestamp_ns (default); "
            "align-start moves the first odometry message to 1 s if VIO uses a different clock"
        ),
    )
    parser.add_argument("--overwrite", action="store_true", help="Replace the output bag if it already exists")
    args = parser.parse_args()

    sensor_dir = os.path.abspath(os.path.expanduser(args.dataset))
    vio_dir = os.path.abspath(os.path.expanduser(args.vio_dataset or args.dataset))

    gps_csv = os.path.join(sensor_dir, "gps_data.csv")
    imu_csv = os.path.join(sensor_dir, "imu_data.csv")
    camera_csv = os.path.join(sensor_dir, "camera_frames.csv")
    frames_dir = os.path.join(sensor_dir, "frames")
    odom_csv = os.path.abspath(os.path.expanduser(args.odometry_csv)) if args.odometry_csv else os.path.join(vio_dir, "odometry.csv")
    path_csv = os.path.abspath(os.path.expanduser(args.path_csv)) if args.path_csv else os.path.join(vio_dir, "path.csv")
    feature_img_csv = (
        os.path.abspath(os.path.expanduser(args.feature_img_csv))
        if args.feature_img_csv else os.path.join(vio_dir, "feature_img.csv")
    )
    output_path = os.path.abspath(os.path.expanduser(args.output)) if args.output else os.path.join(sensor_dir, "combined_all_topics.bag")

    for required_file in (gps_csv, imu_csv, camera_csv, odom_csv, path_csv):
        if not os.path.isfile(required_file):
            parser.error("Required input file not found: {}".format(required_file))
    if not os.path.isdir(frames_dir):
        parser.error("Camera frames directory not found: {}".format(frames_dir))
    if args.feature_img_csv and not os.path.isfile(feature_img_csv):
        parser.error("Feature image CSV not found: {}".format(feature_img_csv))
    if output_path in (gps_csv, imu_csv, camera_csv, odom_csv, path_csv, feature_img_csv):
        parser.error("Output bag path cannot be the same as an input CSV path.")
    if os.path.exists(output_path) and not args.overwrite:
        parser.error("Output already exists: {} (use --overwrite to replace it)".format(output_path))

    imu_first_ns = first_timestamp_ns(imu_csv)
    camera_first_ns = first_timestamp_ns(camera_csv)
    gps_first_ns = first_timestamp_ns(gps_csv)
    if imu_first_ns is None:
        parser.error("No valid timestamp_ns found in imu_data.csv")
    if camera_first_ns is None:
        parser.error("No valid timestamp_ns found in camera_frames.csv")

    gps_offset_ns = int(args.gps_offset * 1e9)

    try:
        odom_rows = load_odometry_rows(odom_csv)
        path_rows = load_path_rows(path_csv)

        first_odom_ns = odom_rows[0]["_timestamp_ns"]
        if args.vio_time_mode == "shared-clock":
            odom_start_relative_ns = first_odom_ns - imu_first_ns
            if odom_start_relative_ns < 0:
                raise ValueError(
                    "The first odometry timestamp is earlier than the first IMU timestamp. "
                    "If VIO timestamps use a different clock origin, rerun with "
                    "--vio-time-mode align-start. Otherwise check that these are from "
                    "the same recording and clock domain."
                )
            align_vio_start = False
        else:
            odom_start_relative_ns = 0
            align_vio_start = True

        sensor_generators = [
            iter(()) if gps_first_ns is None else generate_gps_events(gps_csv, gps_first_ns, gps_offset_ns),
            generate_imu_events(imu_csv, imu_first_ns),
            generate_camera_events(camera_csv, frames_dir, imu_first_ns),
            generate_odometry_events(odom_rows, imu_first_ns, align_start=align_vio_start),
            generate_path_events(path_rows, odom_start_relative_ns),
        ]

        if os.path.isfile(feature_img_csv):
            print("Feature image CSV: {}".format(feature_img_csv))
            sensor_generators.append(
                generate_feature_image_events(feature_img_csv, imu_first_ns)
            )
        else:
            print("Feature image CSV not present; skipping {}".format(FEATURE_IMG_TOPIC))

        os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
        count_map = {
            GPS_TOPIC: 0,
            NAVSAT_TOPIC: 0,
            IMU_TOPIC: 0,
            MAG_TOPIC: 0,
            CAMERA_TOPIC: 0,
            CAMERA_INFO_TOPIC: 0,
            ODOM_TOPIC: 0,
            PATH_TOPIC: 0,
            FEATURE_IMG_TOPIC: 0,
        }

        print("\nSensor folder: {}".format(sensor_dir))
        print("VIO folder:    {}".format(vio_dir))
        print("VIO time mode: {}".format(args.vio_time_mode))
        print("Feature image: {}".format(feature_img_csv if os.path.isfile(feature_img_csv) else "disabled"))
        print("Output bag:    {}\n".format(output_path))
        created_output = False
        try:
            with rosbag.Bag(output_path, mode="w") as bag:
                created_output = True
                total = 0
                for relative_ns, priority, topic, msg in merge_generators(sensor_generators):
                    # All events from generators are relative to the common 1 s bag start.
                    stamp = ns_to_ros_time(relative_ns)
                    msg.header.stamp = stamp
                    bag.write(topic, msg, t=stamp)
                    count_map[topic] += 1
                    total += 1
                    if total % 5000 == 0:
                        print("Messages written: {}".format(total))
        except Exception:
            if created_output and os.path.exists(output_path):
                try:
                    os.remove(output_path)
                except OSError:
                    pass
            raise

    except Exception as exc:
        parser.exit(2, "ERROR: {}\n".format(exc))

    print("\n" + "=" * 72)
    print("CONVERSION COMPLETE")
    print("=" * 72)
    for topic, count in count_map.items():
        print("{} : {}".format(topic, count))
    print("\nTOTAL messages: {}".format(sum(count_map.values())))
    print("Final Path message contains {} poses.".format(len(path_rows)))
    print("Output: {}".format(output_path))
    print("Verify with: rosbag info '{}'".format(output_path))
    print("=" * 72)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nConversion interrupted.")




        """
source /opt/ros/noetic/setup.bash
source /home/idle-omen2/catkin_ws/install_isolated/setup.bash

SENSOR_SESSION="/home/idle-omen2/sensor_logs_local/session_20261012_150118"
VIO_SESSION="/home/idle-omen2/vio_logs_local/session_20261012_150122"

PYTHONNOUSERSITE=1 python3 \
  /home/idle-omen2/Python/combined_csv_bag.py \
  --dataset "$SENSOR_SESSION" \
  --vio-dataset "$VIO_SESSION" \
  --output "$SENSOR_SESSION/combined_all_topics.bag"
 
 
 """       

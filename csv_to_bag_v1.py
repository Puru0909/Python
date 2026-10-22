#!/usr/bin/env python3

import os
import csv
import heapq
import argparse
import math

import rospy
import rosbag

from sensor_msgs.msg import (
    Imu,
    MagneticField,
    NavSatFix,
    NavSatStatus,
    CompressedImage,
    CameraInfo
)

from mavros_msgs.msg import GPSINPUT

# ============================================================
# TOPICS
# ============================================================
CAMERA_TOPIC = "/idle_cam/image_raw/compressed"
CAMERA_INFO_TOPIC = "/idle_cam/camera_info"
GPS_TOPIC = "/ext_mav/gps_input/gps_input"
NAVSAT_TOPIC = "/mav/global_position/global"
IMU_TOPIC = "/imu"
MAG_TOPIC = "/imu/mag"

# ============================================================
# CSV HELPERS
# ============================================================
def safe_int(value, default=0):
    if value is None: return default
    value = str(value).strip()
    if value == "": return default
    try: return int(float(value))
    except (ValueError, TypeError): return default

def safe_float(value, default=0.0):
    if value is None: return default
    value = str(value).strip()
    if value == "": return default
    try: return float(value)
    except (ValueError, TypeError): return default

def get_first_timestamp(csv_file):
    with open(csv_file, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            value = row.get("timestamp_ns")
            if value is None: continue
            value = str(value).strip()
            if value == "": continue
            try: return int(float(value))
            except (ValueError, TypeError): continue
    return None

def ns_to_ros_time(relative_ns):
    if relative_ns < 0: relative_ns = 0
    relative_ns += 1_000_000_000 # Start bag at 1.0 sec
    sec = relative_ns // 1_000_000_000
    nsec = relative_ns % 1_000_000_000
    return rospy.Time(int(sec), int(nsec))

def normalize_quaternion(qx, qy, qz, qw):
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-12: return (0.0, 0.0, 0.0, 1.0)
    return (qx/norm, qy/norm, qz/norm, qw/norm)

# ============================================================
# IMU + MAG EVENTS
# ============================================================
def generate_imu_events(csv_file, reference_ns):
    with open(csv_file, "r", newline="") as f:
        reader = csv.DictReader(f)
        for seq, row in enumerate(reader):
            timestamp_ns = safe_int(row.get("timestamp_ns"))
            if timestamp_ns <= 0: continue
            
            relative_ns = timestamp_ns - reference_ns
            if relative_ns < 0: continue
            
            stamp = ns_to_ros_time(relative_ns)
            frame_id = row.get("frame_id") or "imu_link"

            # IMU
            msg = Imu()
            msg.header.seq = seq
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id

            qx, qy, qz, qw = normalize_quaternion(
                safe_float(row.get("qx")), safe_float(row.get("qy")),
                safe_float(row.get("qz")), safe_float(row.get("qw"), 1.0)
            )

            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw

            for i in range(9):
                msg.orientation_covariance[i] = safe_float(row.get(f"q_cov{i}"))

            msg.angular_velocity.x = safe_float(row.get("wx"))
            msg.angular_velocity.y = safe_float(row.get("wy"))
            msg.angular_velocity.z = safe_float(row.get("wz"))
            for i in range(9):
                msg.angular_velocity_covariance[i] = safe_float(row.get(f"w_cov{i}"))

            msg.linear_acceleration.x = safe_float(row.get("ax"))
            msg.linear_acceleration.y = safe_float(row.get("ay"))
            msg.linear_acceleration.z = safe_float(row.get("az"))
            for i in range(9):
                msg.linear_acceleration_covariance[i] = safe_float(row.get(f"a_cov{i}"))

            yield (relative_ns, 1, IMU_TOPIC, msg)

            # MAGNETOMETER
            mag_msg = MagneticField()
            mag_msg.header.seq = seq
            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = frame_id
            mag_msg.magnetic_field.x = safe_float(row.get("mag_x")) * 1e-6
            mag_msg.magnetic_field.y = safe_float(row.get("mag_y")) * 1e-6
            mag_msg.magnetic_field.z = safe_float(row.get("mag_z")) * 1e-6
            mag_msg.magnetic_field_covariance[0] = -1.0

            yield (relative_ns, 2, MAG_TOPIC, mag_msg)

# ============================================================
# GPS EVENTS
# ============================================================
def generate_gps_events(csv_file, gps_reference_ns, gps_offset_ns):
    with open(csv_file, "r", newline="") as f:
        reader = csv.DictReader(f)
        for seq, row in enumerate(reader):
            timestamp_ns = safe_int(row.get("timestamp_ns"))
            if timestamp_ns <= 0: continue
            
            relative_ns = timestamp_ns - gps_reference_ns + gps_offset_ns
            if relative_ns < 0: continue
            
            stamp = ns_to_ros_time(relative_ns)
            frame_id = row.get("frame_id") or "gps_link"

            # GPSINPUT
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
            if hasattr(gps_msg, "yaw"): gps_msg.yaw = safe_int(row.get("yaw"))

            # NAVSATFIX
            navsat_msg = NavSatFix()
            navsat_msg.header.seq = seq
            navsat_msg.header.stamp = stamp
            navsat_msg.header.frame_id = frame_id
            fix_type = safe_int(row.get("fix_type"))
            if fix_type >= 2: navsat_msg.status.status = NavSatStatus.STATUS_FIX
            else: navsat_msg.status.status = NavSatStatus.STATUS_NO_FIX
            
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
                0.0, 0.0, up_variance
            ]
            navsat_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

            yield (relative_ns, 0, GPS_TOPIC, gps_msg)
            yield (relative_ns, 0, NAVSAT_TOPIC, navsat_msg)

# ============================================================
# CAMERA EVENTS
# ============================================================
def generate_camera_events(csv_file, frames_dir, reference_ns):
    with open(csv_file, "r", newline="") as f:
        reader = csv.DictReader(f)
        for seq, row in enumerate(reader):
            timestamp_ns = safe_int(row.get("timestamp_ns"))
            if timestamp_ns <= 0: continue
            
            filename = (row.get("filename") or "").strip()
            if not filename: continue
            
            image_path = os.path.join(frames_dir, filename)
            if not os.path.isfile(image_path): continue

            try:
                with open(image_path, "rb") as image_file:
                    jpeg_data = image_file.read()
            except OSError as exc:
                continue

            if (len(jpeg_data) < 4 or 
                not jpeg_data.startswith(b"\xff\xd8") or 
                not jpeg_data.endswith(b"\xff\xd9")):
                continue

            relative_ns = timestamp_ns - reference_ns
            if relative_ns < 0: continue
            stamp = ns_to_ros_time(relative_ns)

            # -------------------------------------------------
            # COMPRESSED IMAGE MESSAGE
            # -------------------------------------------------
            img_msg = CompressedImage()
            img_msg.header.seq = seq
            img_msg.header.stamp = stamp
            img_msg.header.frame_id = "camera"
            img_msg.format = "bgr8; jpeg [rgb8]"
            img_msg.data = jpeg_data

            # -------------------------------------------------
            # CAMERA INFO MESSAGE
            # -------------------------------------------------
            cam_info = CameraInfo()
            cam_info.header.seq = seq
            cam_info.header.stamp = stamp
            cam_info.header.frame_id = "camera"
            
            cam_info.width = 1280
            cam_info.height = 720
            cam_info.distortion_model = "plumb_bob" # standard ROS rad-tan
            
            # [k1, k2, t1 (p1), t2 (p2), k3]
            cam_info.D = [0.08537173793341997, -0.15724369761506035, -0.0027100251818195555, -0.00067514168023245, 0.0]
            
            # Intrinsic camera matrix [fx, 0, cx; 0, fy, cy; 0, 0, 1]
            cam_info.K = [930.2033563961248, 0.0, 637.3689177579088,
                          0.0, 928.6204814264393, 351.2245451749831,
                          0.0, 0.0, 1.0]
            
            # Rectification matrix (identity for monocular)
            cam_info.R = [1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0]
            
            # Projection/camera matrix [fx', 0, cx', Tx; 0, fy', cy', Ty; 0, 0, 1, 0]
            cam_info.P = [930.2033563961248, 0.0, 637.3689177579088, 0.0,
                          0.0, 928.6204814264393, 351.2245451749831, 0.0,
                          0.0, 0.0, 1.0, 0.0]

            cam_info.binning_x = 0
            cam_info.binning_y = 0

            yield (relative_ns, 3, CAMERA_TOPIC, img_msg)
            yield (relative_ns, 4, CAMERA_INFO_TOPIC, cam_info)

# ============================================================
# MERGE ALL EVENTS
# ============================================================
def merge_generators(generators):
    heap = []
    for generator_index, generator in enumerate(generators):
        try:
            event = next(generator)
            heapq.heappush(heap, (event[0], event[1], generator_index, event))
        except StopIteration:
            pass

    while heap:
        relative_ns, priority, generator_index, event = heapq.heappop(heap)
        yield event
        try:
            next_event = next(generators[generator_index])
            heapq.heappush(heap, (next_event[0], next_event[1], generator_index, next_event))
        except StopIteration:
            pass

# ============================================================
# MAIN
# ============================================================
def main():
    parser = argparse.ArgumentParser(description="Convert camera + GPS + IMU CSV to ROS1 Noetic bag.")
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--output", default=None)
    parser.add_argument("--gps-offset", type=float, default=0.0)
    args = parser.parse_args()

    dataset = os.path.abspath(args.dataset)
    gps_csv = os.path.join(dataset, "gps_data.csv")
    imu_csv = os.path.join(dataset, "imu_data.csv")
    camera_csv = os.path.join(dataset, "camera_frames.csv")
    frames_dir = os.path.join(dataset, "frames")

    for filename in [gps_csv, imu_csv, camera_csv]:
        if not os.path.isfile(filename): raise FileNotFoundError(f"File not found: {filename}")
    if not os.path.isdir(frames_dir): raise NotADirectoryError(f"Frames directory not found: {frames_dir}")

    imu_first_ns = get_first_timestamp(imu_csv)
    camera_first_ns = get_first_timestamp(camera_csv)
    gps_first_ns = get_first_timestamp(gps_csv)

    if imu_first_ns is None: raise RuntimeError("No valid IMU timestamps.")
    if camera_first_ns is None: raise RuntimeError("No valid camera timestamps.")

    gps_offset_ns = int(args.gps_offset * 1e9)
    gps_generator = iter(()) if gps_first_ns is None else generate_gps_events(gps_csv, gps_first_ns, gps_offset_ns)
    imu_generator = generate_imu_events(imu_csv, imu_first_ns)
    camera_generator = generate_camera_events(camera_csv, frames_dir, imu_first_ns)
    
    generators = [gps_generator, imu_generator, camera_generator]
    output_path = os.path.abspath(args.output) if args.output else os.path.join(dataset, "dataset.bag")

    print(f"\nCreating bag: {output_path}\n")
    
    count_map = {GPS_TOPIC: 0, NAVSAT_TOPIC: 0, IMU_TOPIC: 0, MAG_TOPIC: 0, CAMERA_TOPIC: 0, CAMERA_INFO_TOPIC: 0}

    with rosbag.Bag(output_path, mode="w") as bag:
        for relative_ns, priority, topic, msg in merge_generators(generators):
            stamp = ns_to_ros_time(relative_ns)
            msg.header.stamp = stamp
            bag.write(topic, msg, t=stamp)
            if topic in count_map: count_map[topic] += 1
            
            total = sum(count_map.values())
            if total % 5000 == 0: print(f"Messages written: {total}")

    print("\n" + "="*72 + "\nCONVERSION COMPLETE\n" + "="*72)
    for t, c in count_map.items(): print(f"{t} : {c}")
    print(f"\nTOTAL : {sum(count_map.values())}\nOutput: {output_path}\n" + "="*72)

if __name__ == "__main__":
    try: main()
    except KeyboardInterrupt: print("\nConversion interrupted.")
    except Exception as exc: print(f"\nERROR:\n{exc}\n"); raise


    """

PYTHONNOUSERSITE=1 python3 \
/home/idle-omen2/Python/csv_to_bag_v6.py \
--dataset /home/idle-omen2/sensor_logs_local/session_20260922_163726

"""

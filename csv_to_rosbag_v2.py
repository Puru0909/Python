#!/usr/bin/env python3

import os
import csv
import heapq
import argparse
import math

import rospy
import rosbag
import cv2

from cv_bridge import CvBridge

from sensor_msgs.msg import (
    Imu,
    MagneticField,
    NavSatFix,
    NavSatStatus
)

from mavros_msgs.msg import GPSINPUT


# ============================================================
# TOPICS
# ============================================================

CAMERA_TOPIC = "/camera/image_raw"
GPS_TOPIC = "/mavros/gps_input"
NAVSAT_TOPIC = "/gps/fix"
IMU_TOPIC = "/imu/data"
MAG_TOPIC = "/imu/mag"


# ============================================================
# CSV HELPERS
# ============================================================

def safe_int(value, default=0):
    if value is None:
        return default

    value = str(value).strip()

    if value == "":
        return default

    try:
        return int(float(value))
    except (ValueError, TypeError):
        return default


def safe_float(value, default=0.0):
    if value is None:
        return default

    value = str(value).strip()

    if value == "":
        return default

    try:
        return float(value)
    except (ValueError, TypeError):
        return default


def get_first_timestamp(csv_file):
    """
    Get the first valid timestamp_ns from a CSV file.
    """

    with open(csv_file, "r", newline="") as f:

        reader = csv.DictReader(f)

        for row in reader:

            value = row.get("timestamp_ns")

            if value is None:
                continue

            value = str(value).strip()

            if value == "":
                continue

            try:
                return int(float(value))
            except ValueError:
                continue

    raise RuntimeError(
        "No valid timestamp_ns found in {}".format(csv_file)
    )


# ============================================================
# ROS TIME
# ============================================================

def ns_to_ros_time(relative_ns):
    """
    Convert relative nanoseconds into rospy.Time.

    We start the bag at 1 second instead of 0 seconds.
    ROS1 rosbag does not accept Time(0,0) as a valid index time.
    """

    if relative_ns < 0:
        relative_ns = 0

    # Start bag at 1.0 sec
    relative_ns += 1_000_000_000

    sec = relative_ns // 1_000_000_000
    nsec = relative_ns % 1_000_000_000

    return rospy.Time(
        int(sec),
        int(nsec)
    )


# ============================================================
# QUATERNION
# ============================================================

def normalize_quaternion(qx, qy, qz, qw):

    norm = math.sqrt(
        qx * qx +
        qy * qy +
        qz * qz +
        qw * qw
    )

    if norm < 1e-12:

        return (
            0.0,
            0.0,
            0.0,
            1.0
        )

    return (
        qx / norm,
        qy / norm,
        qz / norm,
        qw / norm
    )


# ============================================================
# IMU + MAG EVENTS
# ============================================================

def generate_imu_events(
    csv_file,
    reference_ns
):

    with open(csv_file, "r", newline="") as f:

        reader = csv.DictReader(f)

        # ----------------------------------------------------
        # IMPORTANT:
        #
        # seq = ORIGINAL CSV DATA ROW
        #
        # Header is not counted.
        #
        # First data row = seq 0
        # Second data row = seq 1
        # ----------------------------------------------------

        for seq, row in enumerate(reader):

            timestamp_ns = safe_int(
                row.get("timestamp_ns")
            )

            if timestamp_ns <= 0:
                continue

            relative_ns = (
                timestamp_ns -
                reference_ns
            )

            if relative_ns < 0:
                continue

            stamp = ns_to_ros_time(
                relative_ns
            )

            frame_id = (
                row.get("frame_id")
                or "imu_link"
            )

            # =================================================
            # IMU
            # =================================================

            msg = Imu()

            # CSV row -> ROS seq
            msg.header.seq = seq

            msg.header.stamp = stamp
            msg.header.frame_id = frame_id

            # -------------------------------------------------
            # Orientation
            # -------------------------------------------------

            qx = safe_float(
                row.get("qx")
            )

            qy = safe_float(
                row.get("qy")
            )

            qz = safe_float(
                row.get("qz")
            )

            qw = safe_float(
                row.get("qw"),
                1.0
            )

            (
                qx,
                qy,
                qz,
                qw
            ) = normalize_quaternion(
                qx,
                qy,
                qz,
                qw
            )

            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw

            # Orientation covariance
            for i in range(9):

                msg.orientation_covariance[i] = safe_float(
                    row.get(
                        "q_cov{}".format(i)
                    )
                )

            # -------------------------------------------------
            # Angular velocity
            # -------------------------------------------------

            msg.angular_velocity.x = safe_float(
                row.get("wx")
            )

            msg.angular_velocity.y = safe_float(
                row.get("wy")
            )

            msg.angular_velocity.z = safe_float(
                row.get("wz")
            )

            # Angular velocity covariance
            for i in range(9):

                msg.angular_velocity_covariance[i] = safe_float(
                    row.get(
                        "w_cov{}".format(i)
                    )
                )

            # -------------------------------------------------
            # Linear acceleration
            # -------------------------------------------------

            msg.linear_acceleration.x = safe_float(
                row.get("ax")
            )

            msg.linear_acceleration.y = safe_float(
                row.get("ay")
            )

            msg.linear_acceleration.z = safe_float(
                row.get("az")
            )

            # Linear acceleration covariance
            for i in range(9):

                msg.linear_acceleration_covariance[i] = safe_float(
                    row.get(
                        "a_cov{}".format(i)
                    )
                )

            yield (
                relative_ns,
                1,
                IMU_TOPIC,
                msg
            )

            # =================================================
            # MAGNETOMETER
            # =================================================

            mag_msg = MagneticField()

            # SAME CSV ROW
            mag_msg.header.seq = seq

            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = frame_id

            # -------------------------------------------------
            # Your CSV magnetometer values are treated as uT.
            #
            # sensor_msgs/MagneticField uses Tesla.
            #
            # uT -> Tesla = x 1e-6
            # -------------------------------------------------

            mag_msg.magnetic_field.x = (
                safe_float(
                    row.get("mag_x")
                ) * 1e-6
            )

            mag_msg.magnetic_field.y = (
                safe_float(
                    row.get("mag_y")
                ) * 1e-6
            )

            mag_msg.magnetic_field.z = (
                safe_float(
                    row.get("mag_z")
                ) * 1e-6
            )

            # Covariance not present in CSV
            mag_msg.magnetic_field_covariance[0] = -1.0

            yield (
                relative_ns,
                2,
                MAG_TOPIC,
                mag_msg
            )


# ============================================================
# GPS EVENTS
# ============================================================

def generate_gps_events(
    csv_file,
    gps_reference_ns,
    gps_offset_ns
):

    with open(csv_file, "r", newline="") as f:

        reader = csv.DictReader(f)

        # ----------------------------------------------------
        # seq = original GPS CSV data-row number
        # ----------------------------------------------------

        for seq, row in enumerate(reader):

            timestamp_ns = safe_int(
                row.get("timestamp_ns")
            )

            if timestamp_ns <= 0:
                continue

            # -------------------------------------------------
            # GPS has a different epoch clock.
            #
            # First convert GPS to relative time.
            # Then add optional synchronization offset.
            # -------------------------------------------------

            relative_ns = (
                timestamp_ns -
                gps_reference_ns +
                gps_offset_ns
            )

            if relative_ns < 0:
                continue

            stamp = ns_to_ros_time(
                relative_ns
            )

            frame_id = (
                row.get("frame_id")
                or "gps_link"
            )

            # =================================================
            # MAVROS GPSINPUT
            # =================================================

            gps_msg = GPSINPUT()

            gps_msg.header.seq = seq

            gps_msg.header.stamp = stamp
            gps_msg.header.frame_id = frame_id

            # -------------------------------------------------
            # GPS fields
            # -------------------------------------------------

            gps_msg.fix_type = safe_int(
                row.get("fix_type")
            )

            gps_msg.gps_id = safe_int(
                row.get("gps_id")
            )

            gps_msg.ignore_flags = safe_int(
                row.get("ignore_flags")
            )

            gps_msg.time_week_ms = safe_int(
                row.get("time_week_ms")
            )

            gps_msg.time_week = safe_int(
                row.get("time_week")
            )

            # -------------------------------------------------
            # MAVLink degE7 representation.
            #
            # DO NOT divide these.
            # -------------------------------------------------

            gps_msg.lat = safe_int(
                row.get("lat")
            )

            gps_msg.lon = safe_int(
                row.get("lon")
            )

            gps_msg.alt = safe_float(
                row.get("alt")
            )

            gps_msg.hdop = safe_float(
                row.get("hdop")
            )

            gps_msg.vdop = safe_float(
                row.get("vdop")
            )

            gps_msg.vn = safe_float(
                row.get("vn")
            )

            gps_msg.ve = safe_float(
                row.get("ve")
            )

            gps_msg.vd = safe_float(
                row.get("vd")
            )

            gps_msg.speed_accuracy = safe_float(
                row.get("speed_acc")
            )

            gps_msg.horiz_accuracy = safe_float(
                row.get("horiz_acc")
            )

            gps_msg.vert_accuracy = safe_float(
                row.get("vert_acc")
            )

            gps_msg.satellites_visible = safe_int(
                row.get("satellites")
            )

            if hasattr(gps_msg, "yaw"):

                gps_msg.yaw = safe_int(
                    row.get("yaw")
                )

            # =================================================
            # NAVSATFIX
            # =================================================

            navsat_msg = NavSatFix()

            # SAME GPS CSV ROW
            navsat_msg.header.seq = seq

            # SAME TIMESTAMP
            navsat_msg.header.stamp = stamp

            navsat_msg.header.frame_id = frame_id

            # -------------------------------------------------
            # STATUS
            # -------------------------------------------------

            fix_type = safe_int(
                row.get("fix_type")
            )

            if fix_type >= 2:

                navsat_msg.status.status = (
                    NavSatStatus.STATUS_FIX
                )

            else:

                navsat_msg.status.status = (
                    NavSatStatus.STATUS_NO_FIX
                )

            navsat_msg.status.service = (
                NavSatStatus.SERVICE_GPS
            )

            # -------------------------------------------------
            # POSITION
            # -------------------------------------------------

            # Your CSV uses MAVLink degE7.
            #
            # GPSINPUT:
            #     130529250
            #
            # NavSatFix:
            #     13.0529250
            #

            navsat_msg.latitude = (
                safe_float(
                    row.get("lat")
                ) / 1e7
            )

            navsat_msg.longitude = (
                safe_float(
                    row.get("lon")
                ) / 1e7
            )

            navsat_msg.altitude = safe_float(
                row.get("alt")
            )

            # -------------------------------------------------
            # ENU POSITION COVARIANCE
            # -------------------------------------------------
            #
            # We have:
            #
            #   horiz_acc
            #   vert_acc
            #
            # We do not have separate East/North accuracies.
            #
            # Therefore use isotropic horizontal assumption:
            #
            #   sigma_E^2 = horiz_acc^2 / 2
            #   sigma_N^2 = horiz_acc^2 / 2
            #
            # and:
            #
            #   sigma_U^2 = vert_acc^2
            #
            # Matrix:
            #
            # [ EE  EN  EU ]
            # [ NE  NN  NU ]
            # [ UE  UN  UU ]
            #
            # = 
            #
            # [ h^2/2  0      0 ]
            # [ 0      h^2/2  0 ]
            # [ 0      0      v^2 ]
            #
            # -------------------------------------------------

            horiz_acc = safe_float(
                row.get("horiz_acc")
            )

            vert_acc = safe_float(
                row.get("vert_acc")
            )

            east_variance = (
                horiz_acc *
                horiz_acc /
                2.0
            )

            north_variance = (
                horiz_acc *
                horiz_acc /
                2.0
            )

            up_variance = (
                vert_acc *
                vert_acc
            )

            navsat_msg.position_covariance = [

                east_variance,
                0.0,
                0.0,

                0.0,
                north_variance,
                0.0,

                0.0,
                0.0,
                up_variance
            ]

            navsat_msg.position_covariance_type = (
                NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            )

            # -------------------------------------------------
            # SAME GPS CSV ROW PRODUCES TWO MESSAGES
            # -------------------------------------------------

            yield (
                relative_ns,
                0,
                GPS_TOPIC,
                gps_msg
            )

            yield (
                relative_ns,
                0,
                NAVSAT_TOPIC,
                navsat_msg
            )


# ============================================================
# CAMERA EVENTS
# ============================================================

def generate_camera_events(
    csv_file,
    frames_dir,
    reference_ns
):

    bridge = CvBridge()

    with open(csv_file, "r", newline="") as f:

        reader = csv.DictReader(f)

        # ----------------------------------------------------
        # seq = original camera CSV row
        # ----------------------------------------------------

        for seq, row in enumerate(reader):

            timestamp_ns = safe_int(
                row.get("timestamp_ns")
            )

            if timestamp_ns <= 0:
                continue

            filename = (
                row.get("filename")
                or ""
            ).strip()

            if not filename:

                rospy.logwarn(
                    "Empty camera filename at CSV row %d",
                    seq
                )

                continue

            image_path = os.path.join(
                frames_dir,
                filename
            )

            # IMPORTANT:
            #
            # We do NOT renumber seq if an image is missing.
            #
            # seq always stays equal to the original CSV row.
            #

            if not os.path.isfile(image_path):

                rospy.logwarn(
                    "Image not found: %s "
                    "(CSV row %d)",
                    image_path,
                    seq
                )

                continue

            image = cv2.imread(
                image_path,
                cv2.IMREAD_COLOR
            )

            if image is None:

                rospy.logwarn(
                    "Cannot decode image: %s "
                    "(CSV row %d)",
                    image_path,
                    seq
                )

                continue

            # Camera and IMU use same recording clock
            relative_ns = (
                timestamp_ns -
                reference_ns
            )

            if relative_ns < 0:
                continue

            stamp = ns_to_ros_time(
                relative_ns
            )

            # -------------------------------------------------
            # OpenCV BGR -> ROS Image
            # -------------------------------------------------

            msg = bridge.cv2_to_imgmsg(
                image,
                encoding="bgr8"
            )

            # Original CSV row -> seq
            msg.header.seq = seq

            msg.header.stamp = stamp
            msg.header.frame_id = "camera_link"

            yield (
                relative_ns,
                3,
                CAMERA_TOPIC,
                msg
            )


# ============================================================
# MERGE ALL EVENTS
# ============================================================

def merge_generators(generators):

    heap = []

    # --------------------------------------------------------
    # First event from each generator
    # --------------------------------------------------------

    for generator_index, generator in enumerate(
        generators
    ):

        try:

            event = next(generator)

            relative_ns = event[0]
            priority = event[1]

            heapq.heappush(
                heap,
                (
                    relative_ns,
                    priority,
                    generator_index,
                    event
                )
            )

        except StopIteration:

            pass

    # --------------------------------------------------------
    # Chronological merge
    # --------------------------------------------------------

    while heap:

        (
            relative_ns,
            priority,
            generator_index,
            event
        ) = heapq.heappop(heap)

        yield event

        try:

            next_event = next(
                generators[
                    generator_index
                ]
            )

            heapq.heappush(
                heap,
                (
                    next_event[0],
                    next_event[1],
                    generator_index,
                    next_event
                )
            )

        except StopIteration:

            pass


# ============================================================
# MAIN
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description=(
            "Convert camera + GPS + IMU CSV "
            "to one ROS1 Noetic bag."
        )
    )

    parser.add_argument(
        "--dataset",
        required=True,
        help=(
            "Dataset directory containing "
            "gps_data.csv, imu_data.csv, "
            "camera_frames.csv and frames/"
        )
    )

    parser.add_argument(
    "--output",
    default=None,
    help=(
        "Output bag filename. "
        "If not specified, saves as dataset.bag "
        "inside the dataset folder."
    )
)


    parser.add_argument(
        "--gps-offset",
        type=float,
        default=0.0,
        help=(
            "GPS time offset in seconds. "
            "Default = 0.0"
        )
    )

    args = parser.parse_args()

    dataset = os.path.abspath(
        args.dataset
    )

    # ========================================================
    # FILE PATHS
    # ========================================================

    gps_csv = os.path.join(
        dataset,
        "gps_data.csv"
    )

    imu_csv = os.path.join(
        dataset,
        "imu_data.csv"
    )

    camera_csv = os.path.join(
        dataset,
        "camera_frames.csv"
    )

    frames_dir = os.path.join(
        dataset,
        "frames"
    )

    # ========================================================
    # CHECK FILES
    # ========================================================

    for filename in [
        gps_csv,
        imu_csv,
        camera_csv
    ]:

        if not os.path.isfile(filename):

            raise FileNotFoundError(
                "File not found: {}".format(
                    filename
                )
            )

    if not os.path.isdir(frames_dir):

        raise NotADirectoryError(
            "Frames directory not found: {}".format(
                frames_dir
            )
        )

    # ========================================================
    # FIRST TIMESTAMPS
    # ========================================================

    imu_first_ns = get_first_timestamp(
        imu_csv
    )

    camera_first_ns = get_first_timestamp(
        camera_csv
    )

    gps_first_ns = get_first_timestamp(
        gps_csv
    )

    gps_offset_ns = int(
        args.gps_offset * 1e9
    )

    # ========================================================
    # DISPLAY INFORMATION
    # ========================================================

    print()
    print("=" * 72)
    print("ROS1 Noetic CSV -> SINGLE ROS BAG")
    print("=" * 72)

    print()
    print("Dataset:")
    print(dataset)

    print()
    print("Input:")
    print("  gps_data.csv")
    print("  imu_data.csv")
    print("  camera_frames.csv")
    print("  frames/")

    print()
    print("First timestamps:")
    print("  IMU    :", imu_first_ns)
    print("  Camera :", camera_first_ns)
    print("  GPS    :", gps_first_ns)

    print()
    print(
        "Camera - IMU start: {:.6f} sec".format(
            (
                camera_first_ns -
                imu_first_ns
            ) / 1e9
        )
    )

    print(
        "GPS offset: {:.6f} sec".format(
            args.gps_offset
        )
    )

    print()
    print("Output topics:")
    print(
        "  {} -> sensor_msgs/Image".format(
            CAMERA_TOPIC
        )
    )
    print(
        "  {} -> mavros_msgs/GPSINPUT".format(
            GPS_TOPIC
        )
    )
    print(
        "  {} -> sensor_msgs/NavSatFix".format(
            NAVSAT_TOPIC
        )
    )
    print(
        "  {} -> sensor_msgs/Imu".format(
            IMU_TOPIC
        )
    )
    print(
        "  {} -> sensor_msgs/MagneticField".format(
            MAG_TOPIC
        )
    )

    print()
    print("=" * 72)
    print()

    # ========================================================
    # COMMON REFERENCES
    # ========================================================

    # IMU and camera share same recording clock
    imu_reference_ns = imu_first_ns
    camera_reference_ns = imu_first_ns

    # GPS has separate epoch clock
    gps_reference_ns = gps_first_ns

    # ========================================================
    # GENERATORS
    # ========================================================

    gps_generator = generate_gps_events(
        gps_csv,
        gps_reference_ns,
        gps_offset_ns
    )

    imu_generator = generate_imu_events(
        imu_csv,
        imu_reference_ns
    )

    camera_generator = generate_camera_events(
        camera_csv,
        frames_dir,
        camera_reference_ns
    )

    generators = [
        gps_generator,
        imu_generator,
        camera_generator
    ]

    # ========================================================
    # OUTPUT
    # ========================================================

    if args.output is None:
         output_path = os.path.join(
        dataset,
        "dataset.bag"
    )
    else:
      output_path = os.path.abspath(
        args.output
    )


    print("Creating bag:")
    print(output_path)
    print()

    count_gps = 0
    count_navsat = 0
    count_imu = 0
    count_mag = 0
    count_camera = 0

    # ========================================================
    # CREATE BAG
    # ========================================================

    with rosbag.Bag(
        output_path,
        mode="w"
    ) as bag:

        for event in merge_generators(
            generators
        ):

            (
                relative_ns,
                priority,
                topic,
                msg
            ) = event

            stamp = ns_to_ros_time(
                relative_ns
            )

            # Keep message time equal to bag time
            msg.header.stamp = stamp

            bag.write(
                topic,
                msg,
                t=stamp
            )

            # =================================================
            # COUNTERS
            # =================================================

            if topic == GPS_TOPIC:

                count_gps += 1

            elif topic == NAVSAT_TOPIC:

                count_navsat += 1

            elif topic == IMU_TOPIC:

                count_imu += 1

            elif topic == MAG_TOPIC:

                count_mag += 1

            elif topic == CAMERA_TOPIC:

                count_camera += 1

            total = (
                count_gps +
                count_navsat +
                count_imu +
                count_mag +
                count_camera
            )

            if total % 5000 == 0:

                print(
                    "Messages written: {}".format(
                        total
                    )
                )

    # ========================================================
    # FINAL REPORT
    # ========================================================

    total = (
        count_gps +
        count_navsat +
        count_imu +
        count_mag +
        count_camera
    )

    print()
    print("=" * 72)
    print("CONVERSION COMPLETE")
    print("=" * 72)

    print()
    print(
        "/camera/image_raw : {}".format(
            count_camera
        )
    )

    print(
        "/mavros/gps_input : {}".format(
            count_gps
        )
    )

    print(
        "/gps/fix          : {}".format(
            count_navsat
        )
    )

    print(
        "/imu/data          : {}".format(
            count_imu
        )
    )

    print(
        "/imu/mag           : {}".format(
            count_mag
        )
    )

    print()
    print(
        "TOTAL              : {}".format(
            total
        )
    )

    print()
    print("Output:")
    print(output_path)

    print()
    print("=" * 72)


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":

    try:

        main()

    except KeyboardInterrupt:

        print(
            "\nConversion interrupted."
        )

    except Exception as exc:

        print()
        print("ERROR:")
        print(exc)
        print()

        raise

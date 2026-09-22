#!/usr/bin/env python3

import socket
import struct
import threading
import time

import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

from mavros_msgs.msg import GPSINPUT


class BMI3x0ReceiverNode:

    def __init__(self):
        rospy.init_node(
            "bmi3x0_receiver_node",
            anonymous=True
        )

        # ============================================================
        # CONNECTION PARAMETERS
        # ============================================================

        self.host = rospy.get_param(
            "~host",
            "127.0.0.1"
        )

        self.telemetry_port = rospy.get_param(
            "~telemetry_port",
            8080
        )

        self.camera_port = rospy.get_param(
            "~camera_port",
            8081
        )

        # ============================================================
        # FRAME IDs
        # ============================================================

        self.default_frame_id_imu = rospy.get_param(
            "~imu_frame_id",
            "imu_link"
        )

        self.default_frame_id_gps = rospy.get_param(
            "~gps_frame_id",
            "gps_link"
        )

        self.frame_id_cam = rospy.get_param(
            "~camera_frame_id",
            "camera_link"
        )

        # ============================================================
        # ROS PUBLISHERS
        # ============================================================

        self.imu_pub = rospy.Publisher(
            "/imu/data",
            Imu,
            queue_size=50
        )

        self.mag_pub = rospy.Publisher(
            "/imu/mag",
            MagneticField,
            queue_size=50
        )

        self.gps_pub = rospy.Publisher(
            "/mavros/gps_input/gps_input",
            GPSINPUT,
            queue_size=10
        )

        self.raw_img_pub = rospy.Publisher(
            "/camera/image_raw",
            Image,
            queue_size=5
        )

        self.comp_img_pub = rospy.Publisher(
            "/camera/image_raw/compressed",
            CompressedImage,
            queue_size=5
        )

        self.running = True

        # ============================================================
        # THREADS
        # ============================================================

        self.telemetry_thread = threading.Thread(
            target=self.telemetry_loop,
            daemon=True
        )

        self.camera_thread = threading.Thread(
            target=self.camera_loop,
            daemon=True
        )

    # =================================================================
    # START
    # =================================================================

    def start(self):
        rospy.loginfo(
            "[BMI3x0 Receiver] Connecting to %s "
            "(Telemetry: %d, Camera: %d)...",
            self.host,
            self.telemetry_port,
            self.camera_port
        )

        self.telemetry_thread.start()
        self.camera_thread.start()

        rospy.spin()

        self.running = False

    # =================================================================
    # TELEMETRY LOOP
    # =================================================================

    def telemetry_loop(self):
        IMU_PAYLOAD_SIZE = 292
        GPS_PAYLOAD_SIZE = 81

        while not rospy.is_shutdown() and self.running:
            sock = None

            try:
                sock = socket.socket(
                    socket.AF_INET,
                    socket.SOCK_STREAM
                )

                sock.connect(
                    (
                        self.host,
                        self.telemetry_port
                    )
                )

                rospy.loginfo(
                    "[Telemetry] Connected to Android Daemon Port 8080"
                )

                while not rospy.is_shutdown() and self.running:

                    # ------------------------------------------------
                    # SYNC
                    # ------------------------------------------------

                    sync_bytes = sock.recv(
                        2,
                        socket.MSG_WAITALL
                    )

                    if len(sync_bytes) != 2:
                        raise ConnectionError(
                            "Telemetry sync incomplete: "
                            f"{len(sync_bytes)}/2"
                        )

                    if (
                        sync_bytes[0] != 0xAA
                        or sync_bytes[1] != 0x55
                    ):
                        rospy.logwarn_throttle(
                            2.0,
                            "[Telemetry] Invalid sync bytes: %s",
                            sync_bytes.hex()
                        )
                        continue

                    # ------------------------------------------------
                    # PACKET TYPE
                    # ------------------------------------------------

                    type_byte = sock.recv(1)

                    if len(type_byte) != 1:
                        raise ConnectionError(
                            "Telemetry packet type incomplete"
                        )

                    packet_type = type_byte[0]

                    # ------------------------------------------------
                    # IMU
                    # ------------------------------------------------

                    if packet_type == 0x01:

                        payload = sock.recv(
                            IMU_PAYLOAD_SIZE,
                            socket.MSG_WAITALL
                        )

                        if len(payload) != IMU_PAYLOAD_SIZE:
                            raise ConnectionError(
                                "IMU packet incomplete: "
                                f"{len(payload)}/"
                                f"{IMU_PAYLOAD_SIZE}"
                            )

                        self.process_imu_packet(payload)

                    # ------------------------------------------------
                    # GPS
                    # ------------------------------------------------

                    elif packet_type == 0x02:

                        payload = sock.recv(
                            GPS_PAYLOAD_SIZE,
                            socket.MSG_WAITALL
                        )

                        if len(payload) != GPS_PAYLOAD_SIZE:
                            raise ConnectionError(
                                "GPS packet incomplete: "
                                f"{len(payload)}/"
                                f"{GPS_PAYLOAD_SIZE}"
                            )

                        self.process_gps_packet(payload)

                    else:
                        rospy.logwarn_throttle(
                            2.0,
                            "[Telemetry] Unknown packet type: 0x%02X",
                            packet_type
                        )

            except ConnectionResetError as e:
                rospy.logwarn(
                    "[Telemetry] Connection reset by daemon: %s",
                    e
                )

            except BrokenPipeError as e:
                rospy.logwarn(
                    "[Telemetry] Broken pipe: %s",
                    e
                )

            except ConnectionError as e:
                rospy.logwarn(
                    "[Telemetry] Connection error: %s",
                    e
                )

            except OSError as e:
                rospy.logerr(
                    "[Telemetry] OS socket error: errno=%s, message=%s",
                    e.errno,
                    e
                )

                rospy.logerr(
                    "[Telemetry] OS exception traceback:",
                    exc_info=True
                )

            except Exception as e:
                rospy.logerr(
                    "[Telemetry] Unexpected error: %s",
                    e
                )

                rospy.logerr(
                    "[Telemetry] Exception traceback:",
                    exc_info=True
                )

            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass

            if not rospy.is_shutdown() and self.running:
                time.sleep(2)

    # =================================================================
    # IMU PACKET
    # =================================================================

    def process_imu_packet(self, payload):

        fmt = "<q 16s 3f 3f 3f 4f 9d 9d 9d"

        expected_size = struct.calcsize(fmt)

        if len(payload) != expected_size:
            rospy.logerr(
                "[IMU] Invalid payload size: %d != %d",
                len(payload),
                expected_size
            )
            return

        data = struct.unpack(
            fmt,
            payload
        )

        timestamp_ns = data[0]

        frame_id_raw = (
            data[1]
            .decode(
                "utf-8",
                errors="ignore"
            )
            .rstrip("\x00")
        )

        frame_id = (
            frame_id_raw
            if frame_id_raw
            else self.default_frame_id_imu
        )

        accel = data[2:5]
        gyro = data[5:8]
        mag = data[8:11]
        quat = data[11:15]

        orientation_cov = data[15:24]
        angular_velocity_cov = data[24:33]
        linear_acceleration_cov = data[33:42]

        # ============================================================
        # TIMESTAMP
        # ============================================================

        stamp = (
            rospy.Time.from_sec(
                timestamp_ns / 1e9
            )
            if timestamp_ns > 0
            else rospy.Time.now()
        )

        # ============================================================
        # IMU MESSAGE
        # ============================================================

        imu_msg = Imu()

        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = frame_id

        imu_msg.linear_acceleration.x = accel[0]
        imu_msg.linear_acceleration.y = accel[1]
        imu_msg.linear_acceleration.z = accel[2]

        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]

        imu_msg.orientation.x = quat[0]
        imu_msg.orientation.y = quat[1]
        imu_msg.orientation.z = quat[2]
        imu_msg.orientation.w = quat[3]

        imu_msg.orientation_covariance = list(
            orientation_cov
        )

        imu_msg.angular_velocity_covariance = list(
            angular_velocity_cov
        )

        imu_msg.linear_acceleration_covariance = list(
            linear_acceleration_cov
        )

        self.imu_pub.publish(
            imu_msg
        )

        # ============================================================
        # MAGNETIC FIELD
        # ============================================================

        mag_msg = MagneticField()

        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = frame_id

        mag_msg.magnetic_field.x = mag[0] * 1e-6
        mag_msg.magnetic_field.y = mag[1] * 1e-6
        mag_msg.magnetic_field.z = mag[2] * 1e-6

        self.mag_pub.publish(
            mag_msg
        )

    # =================================================================
    # GPS PACKET
    # =================================================================

    def process_gps_packet(self, payload):

        fmt = "<q 16s B B H I H i i 9f B H"

        expected_size = struct.calcsize(fmt)

        if len(payload) != expected_size:
            rospy.logerr(
                "[GPS] Invalid payload size: %d != %d",
                len(payload),
                expected_size
            )
            return

        data = struct.unpack(
            fmt,
            payload
        )

        timestamp_ns = data[0]

        frame_id_raw = (
            data[1]
            .decode(
                "utf-8",
                errors="ignore"
            )
            .rstrip("\x00")
        )

        frame_id = (
            frame_id_raw
            if frame_id_raw
            else self.default_frame_id_gps
        )

        # ============================================================
        # TIMESTAMP
        # ============================================================

        stamp = (
            rospy.Time.from_sec(
                timestamp_ns / 1e9
            )
            if timestamp_ns > 0
            else rospy.Time.now()
        )

        # ============================================================
        # GPSINPUT MESSAGE
        # ============================================================

        gps_msg = GPSINPUT()

        gps_msg.header.stamp = stamp
        gps_msg.header.frame_id = frame_id

        gps_msg.fix_type = data[2]
        gps_msg.gps_id = data[3]
        gps_msg.ignore_flags = data[4]

        gps_msg.time_week_ms = data[5]
        gps_msg.time_week = data[6]

        gps_msg.lat = data[7]
        gps_msg.lon = data[8]

        gps_msg.alt = data[9]

        gps_msg.hdop = data[10]
        gps_msg.vdop = data[11]

        gps_msg.vn = data[12]
        gps_msg.ve = data[13]
        gps_msg.vd = data[14]

        gps_msg.speed_accuracy = data[15]
        gps_msg.horiz_accuracy = data[16]
        gps_msg.vert_accuracy = data[17]

        gps_msg.satellites_visible = data[18]

        gps_msg.yaw = data[19]

        self.gps_pub.publish(
            gps_msg
        )

    # =================================================================
    # CAMERA LOOP
    # =================================================================

    def camera_loop(self):

        # ============================================================
        # CAMERA HEADER
        #
        # C++:
        #
        # struct CameraFrameHeader {
        #     uint32_t magic;
        #     uint32_t payload_size;
        #     uint64_t timestamp_ns;
        #     uint32_t width;
        #     uint32_t height;
        # };
        #
        # Total = 24 bytes
        # ============================================================

        HEADER_SIZE = 24

        HEADER_FORMAT = "<I I q I I"

        HEADER_STRUCT_SIZE = struct.calcsize(
            HEADER_FORMAT
        )

        # Safety limit for corrupted payload_size values.
        MAX_JPEG_SIZE = 4 * 1024 * 1024

        while not rospy.is_shutdown() and self.running:

            sock = None

            try:

                # ----------------------------------------------------
                # CREATE TCP SOCKET
                # ----------------------------------------------------

                sock = socket.socket(
                    socket.AF_INET,
                    socket.SOCK_STREAM
                )

                # Blocking socket.
                #
                # This matches the existing C++ TCP stream behavior.
                # ----------------------------------------------------

                sock.settimeout(None)

                # ----------------------------------------------------
                # CONNECT TO CAMERA SERVER
                # ----------------------------------------------------

                sock.connect(
                    (
                        self.host,
                        self.camera_port
                    )
                )

                rospy.loginfo(
                    "[Camera] Connected to Android Daemon Port 8081"
                )

                # ====================================================
                # RECEIVE CAMERA STREAM
                # ====================================================

                while not rospy.is_shutdown() and self.running:

                    # ------------------------------------------------
                    # RECEIVE COMPLETE HEADER
                    # ------------------------------------------------

                    hdr_data = sock.recv(
                        HEADER_SIZE,
                        socket.MSG_WAITALL
                    )

                    if len(hdr_data) != HEADER_STRUCT_SIZE:
                        raise ConnectionError(
                            "Camera header incomplete: "
                            f"{len(hdr_data)}/"
                            f"{HEADER_STRUCT_SIZE}"
                        )

                    # ------------------------------------------------
                    # UNPACK HEADER
                    # ------------------------------------------------

                    (
                        magic,
                        payload_size,
                        timestamp_ns,
                        width,
                        height
                    ) = struct.unpack(
                        HEADER_FORMAT,
                        hdr_data
                    )

                    # ------------------------------------------------
                    # CHECK MAGIC
                    # ------------------------------------------------

                    if magic != 0x43414D31:
                        raise ValueError(
                            "[Camera] Header magic mismatch: "
                            f"0x{magic:08X}"
                        )

                    # ------------------------------------------------
                    # CHECK PAYLOAD SIZE
                    # ------------------------------------------------

                    if payload_size <= 0:
                        raise ValueError(
                            "[Camera] Invalid JPEG payload size: "
                            f"{payload_size}"
                        )

                    if payload_size > MAX_JPEG_SIZE:
                        raise ValueError(
                            "[Camera] JPEG payload too large: "
                            f"{payload_size} bytes"
                        )

                    # ------------------------------------------------
                    # CHECK DIMENSIONS
                    #
                    # This does not force a particular resolution.
                    # It accepts whatever dimensions the C++ header
                    # reports.
                    # ------------------------------------------------

                    if width == 0 or height == 0:
                        raise ValueError(
                            "[Camera] Invalid image dimensions: "
                            f"{width}x{height}"
                        )

                    rospy.logdebug(
                        "[Camera] Frame %dx%d, %d bytes, timestamp=%d",
                        width,
                        height,
                        payload_size,
                        timestamp_ns
                    )

                    # ------------------------------------------------
                    # RECEIVE COMPLETE JPEG
                    # ------------------------------------------------

                    jpg_bytes = sock.recv(
                        payload_size,
                        socket.MSG_WAITALL
                    )

                    if len(jpg_bytes) != payload_size:
                        raise ConnectionError(
                            "JPEG payload incomplete: "
                            f"{len(jpg_bytes)}/"
                            f"{payload_size}"
                        )

                    # =================================================
                    # ROS TIMESTAMP
                    # =================================================

                    stamp = (
                        rospy.Time.from_sec(
                            timestamp_ns / 1e9
                        )
                        if timestamp_ns > 0
                        else rospy.Time.now()
                    )

                    # =================================================
                    # COMPRESSED IMAGE
                    #
                    # JPEG is published directly without decoding.
                    # =================================================

                    comp_msg = CompressedImage()

                    comp_msg.header.stamp = stamp
                    comp_msg.header.frame_id = (
                        self.frame_id_cam
                    )

                    comp_msg.format = "jpeg"

                    comp_msg.data = jpg_bytes

                    self.comp_img_pub.publish(
                        comp_msg
                    )

                    # =================================================
                    # RAW IMAGE
                    #
                    # Decode only when /camera/image_raw has a
                    # subscriber.
                    # =================================================

                    if self.raw_img_pub.get_num_connections() > 0:

                        try:

                            # -----------------------------------------
                            # JPEG → OpenCV BGR
                            # -----------------------------------------

                            np_arr = np.frombuffer(
                                jpg_bytes,
                                dtype=np.uint8
                            )

                            bgr_img = cv2.imdecode(
                                np_arr,
                                cv2.IMREAD_COLOR
                            )

                            if bgr_img is None:
                                rospy.logwarn(
                                    "[Camera] "
                                    "cv2.imdecode() returned None"
                                )
                                continue

                            # Make sure memory is contiguous before
                            # converting it to ROS byte data.
                            bgr_img = np.ascontiguousarray(
                                bgr_img
                            )

                            # -----------------------------------------
                            # DIRECT sensor_msgs/Image CONSTRUCTION
                            #
                            # This intentionally avoids CvBridge.
                            # -----------------------------------------

                            raw_msg = Image()

                            raw_msg.header.stamp = stamp
                            raw_msg.header.frame_id = (
                                self.frame_id_cam
                            )

                            raw_msg.height = (
                                bgr_img.shape[0]
                            )

                            raw_msg.width = (
                                bgr_img.shape[1]
                            )

                            raw_msg.encoding = "bgr8"

                            raw_msg.is_bigendian = 0

                            raw_msg.step = (
                                bgr_img.shape[1] * 3
                            )

                            raw_msg.data = (
                                bgr_img.tobytes()
                            )

                            self.raw_img_pub.publish(
                                raw_msg
                            )

                        except Exception as image_error:

                            # ------------------------------------------------
                            # IMPORTANT:
                            #
                            # Image conversion failure must NOT close
                            # the TCP connection.
                            # ------------------------------------------------

                            rospy.logerr(
                                "[Camera] Image processing error: %s",
                                image_error
                            )

                            rospy.logerr(
                                "[Camera] Image processing traceback:",
                                exc_info=True
                            )

                            continue

            # ========================================================
            # CAMERA SOCKET ERRORS
            # ========================================================

            except ConnectionResetError as e:

                rospy.logwarn(
                    "[Camera] Connection reset by daemon: %s",
                    e
                )

            except BrokenPipeError as e:

                rospy.logwarn(
                    "[Camera] Broken pipe: %s",
                    e
                )

            except ConnectionError as e:

                rospy.logwarn(
                    "[Camera] Camera TCP connection error: %s",
                    e
                )

            # ========================================================
            # CAMERA PROTOCOL ERRORS
            # ========================================================

            except ValueError as e:

                rospy.logerr(
                    "[Camera] Camera protocol error: %s",
                    e
                )

            # ========================================================
            # OS SOCKET ERRORS
            # ========================================================

            except OSError as e:

                rospy.logerr(
                    "[Camera] OS socket error: "
                    "errno=%s, message=%s",
                    e.errno,
                    e
                )

                rospy.logerr(
                    "[Camera] OS socket traceback:",
                    exc_info=True
                )

            # ========================================================
            # UNEXPECTED ERRORS
            # ========================================================

            except Exception as e:

                rospy.logerr(
                    "[Camera] Unexpected camera error: %s",
                    e
                )

                rospy.logerr(
                    "[Camera] Camera traceback:",
                    exc_info=True
                )

            finally:

                if sock is not None:

                    try:
                        sock.close()
                    except Exception:
                        pass

            # ========================================================
            # RECONNECT
            # ========================================================

            if not rospy.is_shutdown() and self.running:

                rospy.logwarn(
                    "[Camera] Reconnecting in 2 seconds..."
                )

                time.sleep(2)


# =====================================================================
# MAIN
# =====================================================================

if __name__ == "__main__":

    try:

        node = BMI3x0ReceiverNode()

        node.start()

    except rospy.ROSInterruptException:

        pass

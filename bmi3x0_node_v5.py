#!/usr/bin/env python3
import socket
import struct
import threading
import time
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus, MagneticField, Image, CompressedImage

class BMI3x0ReceiverNode:
    def __init__(self):
        rospy.init_node("bmi3x0_receiver_node", anonymous=True)

        self.host = rospy.get_param("~host", "127.0.0.1")
        self.telemetry_port = rospy.get_param("~telemetry_port", 8080)
        self.camera_port = rospy.get_param("~camera_port", 8081)

        self.default_frame_id_imu = rospy.get_param("~imu_frame_id", "imu_link")
        self.default_frame_id_gps = rospy.get_param("~gps_frame_id", "gps_link")
        self.frame_id_cam = rospy.get_param("~camera_frame_id", "camera_link")

        self.imu_pub = rospy.Publisher("/imu/data", Imu, queue_size=50)
        self.mag_pub = rospy.Publisher("/imu/mag", MagneticField, queue_size=50)
        self.gps_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)
        self.raw_img_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=5)
        self.comp_img_pub = rospy.Publisher("/camera/image_raw/compressed", CompressedImage, queue_size=5)

        self.bridge = CvBridge()
        self.running = True

        self.telemetry_thread = threading.Thread(target=self.telemetry_loop, daemon=True)
        self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)

    def start(self):
        rospy.loginfo(f"[BMI3x0 Receiver] Connecting to {self.host} (Telemetry: {self.telemetry_port}, Camera: {self.camera_port})...")
        self.telemetry_thread.start()
        self.camera_thread.start()

        rospy.spin()
        self.running = False

    def telemetry_loop(self):
        IMU_PAYLOAD_SIZE = 292
        GPS_PAYLOAD_SIZE = 123

        while not rospy.is_shutdown() and self.running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self.host, self.telemetry_port))
                rospy.loginfo("[Telemetry] Connected to Android Daemon Port 8080")

                while not rospy.is_shutdown() and self.running:
                    sync_bytes = sock.recv(2, socket.MSG_WAITALL)
                    if len(sync_bytes) < 2: break
                    if sync_bytes[0] != 0xAA or sync_bytes[1] != 0x55: continue

                    type_byte = sock.recv(1)
                    if not type_byte: break
                    packet_type = type_byte[0]

                    if packet_type == 0x01:  # IMU
                        payload = sock.recv(IMU_PAYLOAD_SIZE, socket.MSG_WAITALL)
                        if len(payload) < IMU_PAYLOAD_SIZE: break
                        self.process_imu_packet(payload)

                    elif packet_type == 0x02:  # GPS
                        payload = sock.recv(GPS_PAYLOAD_SIZE, socket.MSG_WAITALL)
                        if len(payload) < GPS_PAYLOAD_SIZE: break
                        self.process_gps_packet(payload)

            except Exception as e:
                rospy.logwarn_throttle(5, f"[Telemetry] Connection lost: {e}. Retrying in 2s...")
                time.sleep(2)

    def process_imu_packet(self, payload):
        # Format: timestamp_ns (q), frame_id (16s), accel (3f), gyro (3f), mag (3f), quat (4f),
        #         orientation_cov (9d), angular_velocity_cov (9d), linear_acceleration_cov (9d)
        fmt = "<q 16s 3f 3f 3f 4f 9d 9d 9d"
        data = struct.unpack(fmt, payload)

        timestamp_ns = data[0]
        frame_id_raw = data[1].decode('utf-8', errors='ignore').rstrip('\x00')
        frame_id = frame_id_raw if frame_id_raw else self.default_frame_id_imu

        accel = data[2:5]
        gyro = data[5:8]
        mag = data[8:11]
        quat = data[11:15]
        orientation_cov = data[15:24]
        angular_velocity_cov = data[24:33]
        linear_acceleration_cov = data[33:42]

        stamp = rospy.Time.from_sec(timestamp_ns / 1e9) if timestamp_ns > 0 else rospy.Time.now()

        # Publish IMU Message
        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = frame_id
        
        imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z = accel
        imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z = gyro
        imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = quat

        imu_msg.orientation_covariance = list(orientation_cov)
        imu_msg.angular_velocity_covariance = list(angular_velocity_cov)
        imu_msg.linear_acceleration_covariance = list(linear_acceleration_cov)
        
        self.imu_pub.publish(imu_msg)

        # Publish Magnetic Field Message
        mag_msg = MagneticField()
        mag_msg.header.stamp = stamp
        mag_msg.header.frame_id = frame_id
        mag_msg.magnetic_field.x, mag_msg.magnetic_field.y, mag_msg.magnetic_field.z = [m * 1e-6 for m in mag]
        # Magnetic covariance can be left default/zero if unknown
        self.mag_pub.publish(mag_msg)

    def process_gps_packet(self, payload):
        # Format: timestamp_ns (q), frame_id (16s), lat, lon, alt (3d), status (b), service (B), cov_type (B), position_covariance (9d)
        fmt = "<q 16s 3d b B B 9d"
        data = struct.unpack(fmt, payload)

        timestamp_ns = data[0]
        frame_id_raw = data[1].decode('utf-8', errors='ignore').rstrip('\x00')
        frame_id = frame_id_raw if frame_id_raw else self.default_frame_id_gps

        lat, lon, alt = data[2:5]
        status = data[5]
        service = data[6]
        cov_type = data[7]
        position_covariance = data[8:17]

        stamp = rospy.Time.from_sec(timestamp_ns / 1e9) if timestamp_ns > 0 else rospy.Time.now()

        gps_msg = NavSatFix()
        gps_msg.header.stamp = stamp
        gps_msg.header.frame_id = frame_id
        
        gps_msg.status.status = status
        gps_msg.status.service = service
        gps_msg.latitude, gps_msg.longitude, gps_msg.altitude = lat, lon, alt

        gps_msg.position_covariance = list(position_covariance)
        gps_msg.position_covariance_type = cov_type
        
        self.gps_pub.publish(gps_msg)

    def camera_loop(self):
        HEADER_SIZE = 24
        HEADER_FORMAT = "<I I q I I"

        while not rospy.is_shutdown() and self.running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self.host, self.camera_port))
                rospy.loginfo("[Camera] Connected to Android Daemon Port 8081")

                while not rospy.is_shutdown() and self.running:
                    hdr_data = sock.recv(HEADER_SIZE, socket.MSG_WAITALL)
                    if len(hdr_data) < HEADER_SIZE: 
                        break

                    magic, payload_size, timestamp_ns, width, height = struct.unpack(HEADER_FORMAT, hdr_data)

                    if magic != 0x43414D31:
                        rospy.logwarn("[Camera] Header magic mismatch! Re-synchronizing...")
                        continue

                    jpg_bytes = sock.recv(payload_size, socket.MSG_WAITALL)
                    if len(jpg_bytes) < payload_size: 
                        break

                    stamp = rospy.Time.from_sec(timestamp_ns / 1e9) if timestamp_ns > 0 else rospy.Time.now()

                    # 1. Zero-copy Compressed Image publishing
                    comp_msg = CompressedImage()
                    comp_msg.header.stamp = stamp
                    comp_msg.header.frame_id = self.frame_id_cam
                    comp_msg.format = "jpeg"
                    comp_msg.data = jpg_bytes
                    self.comp_img_pub.publish(comp_msg)

                    # 2. Decode JPEG to BGR only if raw image topic has active subscribers
                    if self.raw_img_pub.get_num_connections() > 0:
                        np_arr = np.frombuffer(jpg_bytes, dtype=np.uint8)
                        bgr_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                        if bgr_img is not None:
                            raw_msg = self.bridge.cv2_to_imgmsg(bgr_img, encoding="bgr8")
                            raw_msg.header.stamp = stamp
                            raw_msg.header.frame_id = self.frame_id_cam
                            self.raw_img_pub.publish(raw_msg)

            except Exception as e:
                rospy.logwarn_throttle(5, f"[Camera] Connection lost: {e}. Retrying in 2s...")
                time.sleep(2)

if __name__ == "__main__":
    try:
        node = BMI3x0ReceiverNode()
        node.start()
    except rospy.ROSInterruptException:
        pass

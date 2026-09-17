#!/usr/bin/env python3
import socket
import struct
import time
import rospy
from sensor_msgs.msg import Imu, MagneticField, NavSatFix, NavSatStatus

SYNC_BYTES = b"\xAA\x55"

PACKET_TYPE_IMU = 0x01
PACKET_TYPE_GPS = 0x02

# IMU Payload: 60 Bytes -> timestamp_ns (q:8B), accel[3] (3f:12B), gyro[3] (3f:12B), mag[3] (3f:12B), quat[4] (4f:16B)
IMU_PAYLOAD_FMT = "<q13f"
IMU_PAYLOAD_SIZE = struct.calcsize(IMU_PAYLOAD_FMT)  # 60 Bytes

# GPS Payload: 37 Bytes -> timestamp_ns (q:8B), lat (d:8B), lon (d:8B), alt (d:8B), hdop (f:4B), status (b:1B)
GPS_PAYLOAD_FMT = "<q3dfb"
GPS_PAYLOAD_SIZE = struct.calcsize(GPS_PAYLOAD_FMT)  # 37 Bytes

def main():
    rospy.init_node("bmi3x0_node", anonymous=False)
    
    imu_pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=100)
    mag_pub = rospy.Publisher("/imu/mag", MagneticField, queue_size=100)
    gps_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    rospy.loginfo("Connecting to Phone Daemon on 127.0.0.1:8080 via ADB Forward...")
    connected = False
    while not rospy.is_shutdown() and not connected:
        try:
            sock.connect(("127.0.0.1", 8080))
            connected = True
            rospy.loginfo("Connected! Streaming ROS topics...")
        except socket.error as e:
            rospy.logwarn_throttle(2.0, f"Waiting for Phone Daemon: {e}")
            time.sleep(1)

    imu_msg = Imu()
    imu_msg.header.frame_id = "imu_link"

    mag_msg = MagneticField()
    mag_msg.header.frame_id = "imu_link"

    gps_msg = NavSatFix()
    gps_msg.header.frame_id = "gps_link"

    imu_count = 0
    gps_count = 0
    
    stream_buffer = bytearray()

    while not rospy.is_shutdown():
        try:
            chunk = sock.recv(4096)
            if not chunk:
                rospy.logerr("Daemon disconnected or socket closed.")
                break
            stream_buffer.extend(chunk)

            # Sliding window stream buffer parsing
            while len(stream_buffer) >= 3:
                sync_idx = stream_buffer.find(SYNC_BYTES)
                
                if sync_idx == -1:
                    stream_buffer = stream_buffer[-1:]
                    break
                
                if sync_idx > 0:
                    stream_buffer = stream_buffer[sync_idx:]
                
                if len(stream_buffer) < 3:
                    break
                
                pkt_type = stream_buffer[2]

                # 1. IMU Packet (63 Bytes Total)
                if pkt_type == PACKET_TYPE_IMU:
                    req_len = 3 + IMU_PAYLOAD_SIZE
                    if len(stream_buffer) < req_len:
                        break
                    
                    payload = stream_buffer[3:req_len]
                    stream_buffer = stream_buffer[req_len:]

                    (ts_ns, ax, ay, az, gx, gy, gz, 
                     mx, my, mz, qx, qy, qz, qw) = struct.unpack(IMU_PAYLOAD_FMT, payload)

                    now = rospy.Time.now()

                    imu_msg.header.stamp = now
                    imu_msg.linear_acceleration.x = float(ax)
                    imu_msg.linear_acceleration.y = float(ay)
                    imu_msg.linear_acceleration.z = float(az)

                    imu_msg.angular_velocity.x = float(gx)
                    imu_msg.angular_velocity.y = float(gy)
                    imu_msg.angular_velocity.z = float(gz)

                    imu_msg.orientation.x = float(qx)
                    imu_msg.orientation.y = float(qy)
                    imu_msg.orientation.z = float(qz)
                    imu_msg.orientation.w = float(qw)

                    imu_pub.publish(imu_msg)

                    mag_msg.header.stamp = now
                    mag_msg.magnetic_field.x = float(mx) * 1e-6
                    mag_msg.magnetic_field.y = float(my) * 1e-6
                    mag_msg.magnetic_field.z = float(mz) * 1e-6
                    mag_pub.publish(mag_msg)

                    imu_count += 1
                    rospy.loginfo_throttle(1.0, f"[STREAM ACTIVE] Published {imu_count} IMU frames | {gps_count} GPS fixes")

                # 2. GPS Packet (40 Bytes Total)
                elif pkt_type == PACKET_TYPE_GPS:
                    req_len = 3 + GPS_PAYLOAD_SIZE
                    if len(stream_buffer) < req_len:
                        break
                    
                    payload = stream_buffer[3:req_len]
                    stream_buffer = stream_buffer[req_len:]

                    (ts_ns, lat, lon, alt, hdop, status) = struct.unpack(GPS_PAYLOAD_FMT, payload)

                    now = rospy.Time.now()

                    gps_msg.header.stamp = now
                    gps_msg.latitude = float(lat)
                    gps_msg.longitude = float(lon)
                    gps_msg.altitude = float(alt)

                    if status >= 0:
                        gps_msg.status.status = NavSatStatus.STATUS_FIX
                    else:
                        gps_msg.status.status = NavSatStatus.STATUS_NO_FIX

                    gps_msg.status.service = NavSatStatus.SERVICE_GPS

                    var = (float(hdop) * 3.0) ** 2
                    gps_msg.position_covariance = [
                        var, 0.0, 0.0,
                        0.0, var, 0.0,
                        0.0, 0.0, var * 2.0
                    ]
                    gps_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

                    gps_pub.publish(gps_msg)

                    gps_count += 1
                    rospy.loginfo(f"--> [GPS FIX #{gps_count}] Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.1f}m")

                else:
                    stream_buffer = stream_buffer[2:]

        except socket.error as e:
            rospy.logerr(f"Socket error: {e}")
            break

    sock.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

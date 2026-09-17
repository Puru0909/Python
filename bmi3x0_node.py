#!/usr/bin/env python3
import socket
import struct
import time
import rospy
from sensor_msgs.msg import Imu, MagneticField

# Exactly 60 bytes: 1 int64 (8B) + 13 floats (52B)
PACKET_FMT = "<q13f"
PACKET_SIZE = struct.calcsize(PACKET_FMT)

def main():
    rospy.init_node("bmi3x0_node", anonymous=False)
    
    imu_pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=100)
    mag_pub = rospy.Publisher("/imu/mag", MagneticField, queue_size=100)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    rospy.loginfo("Connecting to Phone Daemon on 127.0.0.1:8080 via ADB Forward...")
    connected = False
    while not rospy.is_shutdown() and not connected:
        try:
            sock.connect(("127.0.0.1", 8080))
            connected = True
            rospy.loginfo("Connected successfully to BMI3x0 Daemon on Phone!")
        except socket.error as e:
            rospy.logwarn_throttle(2.0, f"Waiting for Phone Daemon to start: {e}")
            time.sleep(1)

    imu_msg = Imu()
    imu_msg.header.frame_id = "imu_link"

    mag_msg = MagneticField()
    mag_msg.header.frame_id = "imu_link"

    pkt_count = 0

    while not rospy.is_shutdown():
        try:
            data = bytearray()
            while len(data) < PACKET_SIZE:
                packet = sock.recv(PACKET_SIZE - len(data))
                if not packet:
                    rospy.logerr("Socket connection closed by Phone Daemon.")
                    return
                data.extend(packet)

            (ts_ns, ax, ay, az, gx, gy, gz, 
             mx, my, mz, qx, qy, qz, qw) = struct.unpack(PACKET_FMT, data)

            now = rospy.Time.now()

            # 1. IMU Message
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

            # 2. Magnetic Field Message (Convert micro-Tesla uT to Tesla T)
            mag_msg.header.stamp = now
            mag_msg.magnetic_field.x = float(mx) * 1e-6
            mag_msg.magnetic_field.y = float(my) * 1e-6
            mag_msg.magnetic_field.z = float(mz) * 1e-6

            mag_pub.publish(mag_msg)

            pkt_count += 1
            if pkt_count % 400 == 0:
                rospy.loginfo(f"Processed {pkt_count} clean frames")

        except socket.error as e:
            rospy.logerr(f"Socket receive error: {e}")
            break

    sock.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

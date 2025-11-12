import socket
import numpy as np
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class UdpJointTrigger(Node):
    def __init__(self):
        super().__init__('udp_joint_trigger')

        # --- UDP setup ---
        self.IP = "0.0.0.0"
        self.PORT = 5000
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.IP, self.PORT))
        self.sock.setblocking(False)  # biar tidak nge-freeze saat tidak ada pesan

        # --- ROS2 publisher ---
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 1)

        # Timer cek UDP tiap 100ms
        self.timer = self.create_timer(0.1, self.listen_udp)

        self.get_logger().info(f"UDP listening on {self.IP}:{self.PORT}")

    def listen_udp(self):
        """Cek apakah ada pesan UDP masuk"""
        try:
            data, addr = self.sock.recvfrom(1024)
        except BlockingIOError:
            return  # tidak ada data masuk

        char = data.decode('utf-8').strip()
        self.get_logger().info(f"Dapat karakter '{char}' dari {addr}")

        if char.upper() == 'S':
            # --- Kirim perintah JointTrajectory ---
            traj = JointTrajectory()
            traj.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4', 'gripper']  # sesuaikan nama joint kamu

            point = JointTrajectoryPoint()
            # ready
            point.positions = [0.0, 0.0 + 0.1667, 0.0, 0.0, 0.0, 0.0]  # target posisi
            point.time_from_start.sec = 0
            traj.points.append(point)

            # above packet
            point = JointTrajectoryPoint()
            point.positions = [1.164279, -0.793773 + 0.1667, 1.220805, -0.000009, -1.2000466, 0.0]  # target posisi
            point.time_from_start.sec = 2
            traj.points.append(point)

            point = JointTrajectoryPoint()
            point.positions = [1.164279, -0.793773 + 0.1667, 1.220805, -0.000009, -1.2000466, 0.0]  # target posisi
            point.time_from_start.sec = 3
            traj.points.append(point)

            # at packet
            point = JointTrajectoryPoint()
            point.positions = [1.164279, -1.274675 + 0.1667, 1.202548, -0.000011, -0.663905, 0.0]  # target posisi
            point.time_from_start.sec = 4
            traj.points.append(point)

            point = JointTrajectoryPoint()
            point.positions = [1.164279, -1.274675 + 0.1667, 1.202548, -0.000011, -0.663905, 0.0]  # target posisi
            point.time_from_start.sec = 5
            traj.points.append(point)

            # gripper close
            point = JointTrajectoryPoint()
            point.positions = [1.164279, -1.274675 + 0.1667, 1.202548, -0.000011, -0.663905, -1.0]  # target posisi
            point.time_from_start.sec = 6
            traj.points.append(point)

            point = JointTrajectoryPoint()
            point.positions = [1.164279, -1.274675 + 0.1667, 1.202548, -0.000011, -0.663905, -1.0]  # target posisi
            point.time_from_start.sec = 7
            traj.points.append(point)

            # above packet
            point = JointTrajectoryPoint()
            point.positions = [1.164279, -0.793773 + 0.1667, 1.220805, -0.000009, -1.2000466, -1.0]  # target posisi
            point.time_from_start.sec = 8
            traj.points.append(point)

            point = JointTrajectoryPoint()
            point.positions = [1.164279, -0.793773 + 0.1667, 1.220805, -0.000009, -1.2000466, -1.0]  # target posisi
            point.time_from_start.sec = 9
            traj.points.append(point)

            # above mobile robot
            point = JointTrajectoryPoint()
            point.positions = [-0.480349, -0.886963 + 0.1667, 0.628731, 0.000000, -1.496643, -1.0]  # target posisi
            point.time_from_start.sec = 11
            traj.points.append(point)

            point = JointTrajectoryPoint()
            point.positions = [-0.480349, -0.886963 + 0.1667, 0.628731, 0.000000, -1.496643, -1.0]  # target posisi
            point.time_from_start.sec = 12
            traj.points.append(point)

            # on mobile robot
            point = JointTrajectoryPoint()
            point.positions = [-0.480348, -0.886989 + 0.1667, 0.628671, 0.000001, -1.570514, -1.0]  # target posisi
            point.time_from_start.sec = 13
            traj.points.append(point)

            point = JointTrajectoryPoint()
            point.positions = [-0.480348, -0.886989 + 0.1667, 0.628671, 0.000001, -1.570514, -1.0]  # target posisi
            point.time_from_start.sec = 14
            traj.points.append(point)

            # gripper open
            point = JointTrajectoryPoint()
            point.positions = [-0.480348, -0.886989 + 0.1667, 0.628671, 0.000001, -1.570514, 0.0]  # target posisi
            point.time_from_start.sec = 15
            traj.points.append(point)

            point = JointTrajectoryPoint()
            point.positions = [-0.480348, -0.886989 + 0.1667, 0.628671, 0.000001, -1.570514, 0.0]  # target posisi
            point.time_from_start.sec = 16
            traj.points.append(point)

            point = JointTrajectoryPoint()
            point.positions = [0.0, 0.0 + 0.1667, 0.0, 0.0, 0.0, 0.0]  # target posisi
            point.time_from_start.sec = 18
            traj.points.append(point)

            self.pub.publish(traj)
            self.get_logger().info("JointTrajectory dikirim ke /joint_trajectory_controller/joint_trajectory")

            time.sleep(18)

            # --- Kirim balasan UDP ---
            reply = 'R'.encode('utf-8')
            self.sock.sendto(reply, addr)
            self.get_logger().info(f"Mengirim 'R' ke {addr}")


def main(args=None):
    rclpy.init(args=args)
    node = UdpJointTrigger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

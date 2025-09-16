# ~/oculus_ws/src/oculus_hmd_sim/oculus_hmd_sim/hmd_pose_to_gz_servos.py
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

def quat_to_rpy(x, y, z, w):
    # roll (x)
    sinr_cosp = 2*(w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y)
    sinp = 2*(w*y - z*x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    # yaw (z)
    siny_cosp = 2*(w*z + x*y)
    cosy_cosp = 1 - 2*(y*y + z*z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

class HmdPoseToGZServos(Node):
    def __init__(self):
        super().__init__('hmd_pose_to_gz_servos')
        self.sub = self.create_subscription(PoseStamped, '/hmd/pose', self.cb, 50)
        self.pub_yaw   = self.create_publisher(Float64, '/gz/hmd/neck_yaw/cmd', 10)
        self.pub_pitch = self.create_publisher(Float64, '/gz/hmd/neck_pitch/cmd', 10)
        self.pub_roll  = self.create_publisher(Float64, '/gz/hmd/head_roll/cmd', 10)
        # límites opcionales
        self.yaw_lim, self.pitch_lim, self.roll_lim = math.pi, 1.2, math.pi
        self.get_logger().info('HMD Pose → GZ servos listo.')

    def clamp(self, v, lim): return max(min(v, lim), -lim)

    def cb(self, msg: PoseStamped):
        q = msg.pose.orientation
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)  # rad
        yaw   = self.clamp(yaw,   self.yaw_lim)
        pitch = self.clamp(pitch, self.pitch_lim)
        roll  = self.clamp(roll,  self.roll_lim)
        self.pub_yaw.publish(Float64(data=yaw))
        self.pub_pitch.publish(Float64(data=pitch))
        self.pub_roll.publish(Float64(data=roll))

def main():
    rclpy.init()
    node = HmdPoseToGZServos()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

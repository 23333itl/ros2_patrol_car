#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock


class JointStateRelay(Node):
    def __init__(self):
        super().__init__("joint_state_relay")
        self.sim_stamp = rclpy.time.Time()

        # 订阅gazebo bridge输出的原始joint_states
        self.sub_joint = self.create_subscription(
            JointState, "/joint_states", self.cb_joint, 10
        )
        # 订阅仿真时钟 /clock
        self.sub_clock = self.create_subscription(
            Clock, "/clock", self.cb_clock, 10
        )
        # 发布修正时间戳之后的话题
        self.pub_joint = self.create_publisher(
            JointState, "/joint_states_fixed", 10
        )

    def cb_clock(self, msg: Clock):
        """保存Gazebo仿真时间戳"""
        self.sim_stamp = rclpy.time.Time.from_msg(msg.clock)

    def cb_joint(self, msg: JointState):
        """替换joint_states消息的header.stamp为仿真时间"""
        out_msg = JointState()
        out_msg.header.stamp = self.sim_stamp.to_msg()
        out_msg.header.frame_id = msg.header.frame_id
        out_msg.name = msg.name
        out_msg.position = msg.position
        out_msg.velocity = msg.velocity
        out_msg.effort = msg.effort
        self.pub_joint.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateRelay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

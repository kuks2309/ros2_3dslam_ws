#!/usr/bin/env python3
"""
Odom to TF Publisher (+ corrected odom republisher)

Gazebo OdometryPublisher 플러그인은 로봇의 월드 좌표를 `/odom` 으로 그대로
publish 한다. ROS 관례상 `odom` 프레임은 로봇 시작 pose 를 원점으로 해야
하므로 이 스크립트가:
  1) 첫 메시지에서 initial pose 저장
  2) `odom → base_footprint` TF 는 (pose - initial) 로 보정 broadcast
  3) `/odom_corrected` 토픽을 동일한 보정 pose 로 재발행
     → RViz Odometry display / navigation stack 에서 소비 시 일관성 확보
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTF(Node):
    def __init__(self):
        super().__init__(
            'odom_to_tf',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.pub_corrected = self.create_publisher(
            Odometry, '/odom_corrected', 10)
        # 초기 위치 (첫 번째 odom 메시지 기준)
        self.initial_x = None
        self.initial_y = None
        self.initial_z = None

    def odom_callback(self, msg: Odometry):
        if self.initial_x is None:
            self.initial_x = msg.pose.pose.position.x
            self.initial_y = msg.pose.pose.position.y
            self.initial_z = msg.pose.pose.position.z
            self.get_logger().info(
                f'Initial position set: ({self.initial_x:.2f}, {self.initial_y:.2f}, {self.initial_z:.2f})')

        ix = self.initial_x or 0.0
        iy = self.initial_y or 0.0
        iz = self.initial_z or 0.0
        dx = msg.pose.pose.position.x - ix
        dy = msg.pose.pose.position.y - iy
        dz = msg.pose.pose.position.z - iz

        # TF broadcast (odom -> base_footprint)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = dx
        t.transform.translation.y = dy
        t.transform.translation.z = dz
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

        # Republish corrected Odometry (same content minus initial offset)
        corrected = Odometry()
        corrected.header = msg.header
        corrected.child_frame_id = msg.child_frame_id
        corrected.pose.pose.position.x = dx
        corrected.pose.pose.position.y = dy
        corrected.pose.pose.position.z = dz
        corrected.pose.pose.orientation = msg.pose.pose.orientation
        corrected.pose.covariance = msg.pose.covariance
        corrected.twist = msg.twist  # velocity is frame-independent
        self.pub_corrected.publish(corrected)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

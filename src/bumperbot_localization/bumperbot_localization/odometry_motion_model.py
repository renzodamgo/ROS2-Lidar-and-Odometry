#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdometryMotionModel(Node):

    def __init__(self):
        super().__init__("odometry_motion_model")
        self.odom_sub_ = self.create_subscription(
            Odometry, "bumperbot_controller/odom_noisy", self.odomCallback, 10
        )
        self.odom_pub_ = self.create_publisher(
            Odometry, "bumperbot_controller/odom_kalman", 10
        )

    def odomCallback(self, odom):
        self.kalman_odom_ = odom

        if self.is_first_odom_:
            self.last_angular_z_ = odom.twist.twist.angular.z
            self.is_first_odom_ = False
            self.mean_ = odom.twist.twist.angular.z
            return

        self.motion_ = odom.twist.twist.angular.z - self.last_angular_z_

        self.statePrediction()
        self.measurementUpdate()

        # Update for the next iteration
        self.last_angular_z_ = odom.twist.twist.angular.z

        # Update and publish the filtered odom message
        self.kalman_odom_.twist.twist.angular.z = self.mean_
        self.odom_pub_.publish(self.kalman_odom_)


def main():
    rclpy.init()

    odometry_motion_model = OdometryMotionModel()
    rclpy.spin(odometry_motion_model)

    odometry_motion_model.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

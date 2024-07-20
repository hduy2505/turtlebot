#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class ParticleFilter:
    def __init__(self):
        rospy.init_node('particle_filter', anonymous=True)

        self.num_particles = 100
        self.particles = np.zeros((self.num_particles, 3))  # Each particle has x, y, theta
        self.weights = np.ones(self.num_particles) / self.num_particles

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.particles_pub = rospy.Publisher('/particle_filter/particles', PoseArray, queue_size=1)

        self.prev_odom = None
        self.map = None  # You should load your map here

        rospy.spin()

    def odom_callback(self, msg):
        if self.prev_odom is None:
            self.prev_odom = msg
            return
        
        delta_x = msg.pose.pose.position.x - self.prev_odom.pose.pose.position.x
        delta_y = msg.pose.pose.position.y - self.prev_odom.pose.pose.position.y
        delta_theta = self.yaw_from_quaternion(msg.pose.pose.orientation) - self.yaw_from_quaternion(self.prev_odom.pose.pose.orientation)

        self.prev_odom = msg

        self.motion_update(delta_x, delta_y, delta_theta)

    def scan_callback(self, msg):
        self.sensor_update(msg)
        self.resample_particles()
        self.publish_particles()

    def yaw_from_quaternion(self, q):
        return np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def motion_update(self, delta_x, delta_y, delta_theta):
        for i in range(self.num_particles):
            self.particles[i, 0] += delta_x + np.random.normal(0, 0.1)
            self.particles[i, 1] += delta_y + np.random.normal(0, 0.1)
            self.particles[i, 2] += delta_theta + np.random.normal(0, 0.01)

    def sensor_update(self, scan_data):
        for i in range(self.num_particles):
            # Compare scan_data with predicted scan from self.particles[i] and update self.weights[i]
            pass

        self.weights += 1.e-300  # avoid round-off to zero
        self.weights /= np.sum(self.weights)

    def resample_particles(self):
        indices = np.random.choice(range(self.num_particles), size=self.num_particles, p=self.weights)
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles

    def publish_particles(self):
        particles_msg = PoseArray()
        particles_msg.header.stamp = rospy.Time.now()
        particles_msg.header.frame_id = 'map'
        for particle in self.particles:
            pose = Pose()
            pose.position.x = particle[0]
            pose.position.y = particle[1]
            # Set orientation based on particle[2]
            particles_msg.poses.append(pose)
        self.particles_pub.publish(particles_msg)

if __name__ == '__main__':
    try:
        ParticleFilter()
    except rospy.ROSInterruptException:
        pass

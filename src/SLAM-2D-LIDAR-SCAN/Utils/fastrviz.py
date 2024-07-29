import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose

class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__('particle_filter_node')
        self.publisher = self.create_publisher(PoseArray, 'particle_poses', 10)
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.pf = ParticleFilter(...)  # Initialize your Particle Filter here

    def timer_callback(self):
        # Process sensor data
        # Assuming you have a way to get sensor data and update particle filter
        # sensor_data = readJson(...)
        # self.pf.updateParticles(sensor_data, count)

        # Publish particle poses
        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'  # Adjust frame_id as per your setup
        for particle in self.pf.particles:
            pose = Pose()
            pose.position.x = particle.xTrajectory[-1]
            pose.position.y = particle.yTrajectory[-1]
            pose.position.z = 0.0  # Adjust if needed
            pose.orientation.w = 1.0  # Default orientation
            pose_array.poses.append(pose)
        self.publisher.publish(pose_array)

        # Publish laser scan data (if applicable)
        # Construct LaserScan message and publish it
        # self.scan_publisher.publish(laser_scan_message)

def main(args=None):
    rclpy.init(args=args)
    particle_filter_node = ParticleFilterNode()
    rclpy.spin(particle_filter_node)
    particle_filter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


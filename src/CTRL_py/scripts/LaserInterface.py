import rospy
from sensor_msgs.msg import LaserScan


class LaserInterface:

    def __init__(self):
        """initialize the laser interface. Add subscriber.
        """
        self.distance = 0
        self.angle = 0

        self.laserSub = rospy.Subscriber('/project_ctrl/laser/scan', LaserScan, self.__laser_callback)

    def __laser_callback(self, msg):
        """Callback function for process the feedback from laser scanner
        """
        self.distance = min(msg.ranges)
        index = msg.ranges.index(min(msg.ranges))
        self.angle = index / 720.0 * 180
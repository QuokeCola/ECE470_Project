import math

from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_driver.msg import gripper_input
import rospy
import numpy as np
from scipy.linalg import expm
import rospkg


class UR3Interface:

    def __init__(self, spinrate_=20):
        """Init the interface, including register the callback functions and initialize variables.

        :param spinrate_ the rate communicate with ROS
        """
        self._angle_feedback = [0, 0, 0, 0, 0, 0]
        self._gripper_feedback = False
        self._target_reached = True

        self._target_angle = [0, 0, 0, 0, 0, 0]
        self._target_sprayer_state = False
        self._target_speed = 0.0
        self._target_acceleration = 0.0

        self._cmd = rospy.Publisher('ur3/command', command, queue_size=10)
        self._position = rospy.Subscriber('ur3/position', position, self.__pos_callback)
        self._gripper = rospy.Subscriber('ur3/gripper_input', gripper_input, self.__gripper_callback)
        self._SPINRATE = spinrate_

    def __pos_callback(self, msg):
        """Callback function for 6 joints' angle feedback
        """
        for i in range(0, 6):
            self._angle_feedback[i] = msg.position[i]

    def __gripper_callback(self, msg):
        """Callback function for gripper's feedback
        """
        self._gripper_feedback = msg.DIGIN

    def set_gripper(self, sprayer_state_):
        """Set gripper of UR3

        :param sprayer_state_ (True/False) whether the gripper is on or off
        """
        self._target_sprayer_state = sprayer_state_

    def get_gripper_sensor(self):
        return self._gripper_feedback

    def set_angle(self, target_angle_, speed_, acceleration_):
        """Set gripper of UR3

        :param target_angle_ target angle of 6 joints
        :param speed_ moving speed of UR3
        :param acceleration_ acceleration of UR3
        """
        for i in range(0, 6):
            self._target_angle[i] = target_angle_[i]
        if speed_ <= 0.0:
            return

        self._target_speed = speed_
        self._target_acceleration = acceleration_
        self.__publish_variables()
        self._target_reached = False
        spin_count = 0
        while not self._target_reached:
            self._target_reached = (abs(self._target_angle[0] - self._angle_feedback[0]) < 0.0005) and \
                                   (abs(self._target_angle[1] - self._angle_feedback[1]) < 0.0005) and \
                                   (abs(self._target_angle[2] - self._angle_feedback[2]) < 0.0005) and \
                                   (abs(self._target_angle[3] - self._angle_feedback[3]) < 0.0005) and \
                                   (abs(self._target_angle[4] - self._angle_feedback[4]) < 0.0005) and \
                                   (abs(self._target_angle[5] - self._angle_feedback[5]) < 0.0005)

            if spin_count > self._SPINRATE * 5:
                self.__publish_variables()
                spin_count = 0
            spin_count = spin_count + 1

    def __publish_variables(self):
        """Publish the variables stored in the class to ros.
        """
        ur3_msg = command()
        ur3_msg.destination = self._target_angle
        ur3_msg.v = self._target_speed
        ur3_msg.a = self._target_acceleration
        ur3_msg.io_0 = self._target_sprayer_state
        self._cmd.publish(ur3_msg)

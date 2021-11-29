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
        self._gripper_feedback = False
        self._target_reached = True
        self._pose = []

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

        M, S = self.__Get_MS()
        theta = self._angle_feedback
        theta[0] -= math.pi
        theta[3] += (0.5*math.pi)
        T = np.identity(4)
        for i in range(6):
            T = np.matmul(T, expm(self.__skew(S[i]) * theta[i]))
        self._pose = np.matmul(T, M)

    def __gripper_callback(self, msg):
        """Callback function for gripper's feedback
        """
        self._gripper_feedback = msg.DIGIN

    def set_gripper(self, sprayer_state_):
        """Set gripper of UR3
        @param sprayer_state_ (True/False) whether the gripper is on or off
        """
        self._target_sprayer_state = sprayer_state_

    def get_gripper_sensor(self):
        return self._gripper_feedback

    def set_angle(self, target_angle_, speed_, acceleration_):
        """Set gripper of UR3
        @param target_angle_ target angle of 6 joints
        @param speed_ moving speed of UR3
        @param acceleration_ acceleration of UR3
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

    def __Get_MS(self):
        M = np.array([[0,-1,0,(540.0-150.0)/1000],[0,0,-1,(150.0+120.0-93.0+83.0+82.0+59.0)/1000],[1,0,0,(10+152+53.5)/1000],[0,0,0,1]])
        S1 = [0,0,1,0.15,0.15,0]
        S2 = [0,1,0,-0.162,0,-0.15]
        S3 = [0,1,0,-0.162,0,0.094]
        S4 = [0,1,0,-0.162,0,0.3070]
        S5 = [1,0,0,0,0.1620,-0.2600]
        S6 = [0,1,0,-0.1620,0,0.3900]
        S = [S1,S2,S3,S4,S5,S6]
        return M, S

    def __skew(self, vector):
        return np.array([[0,            -vector[2],     vector[1],      vector[3]],
                         [vector[2],    0,              -vector[0],     vector[4]],
                         [-vector[1],   vector[0],      0,              vector[5]],
                         [0,            0,              0,              0]])

    def get_current_pose(self):
        return self._pose
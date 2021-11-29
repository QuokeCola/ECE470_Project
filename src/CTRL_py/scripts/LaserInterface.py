import matplotlib.pyplot
import numpy as np
import rospy
from filterpy.kalman import KalmanFilter
from sensor_msgs.msg import LaserScan
from Ur3Interface import *
from math import cos
from math import sin
from math import pi
import cv2
from scipy.optimize import curve_fit


class LaserInterface:

    def __init__(self, ur3IF):
        """initialize the laser interface. Add subscriber.
        """
        self.distance = 0
        self.angle = 0

        self.laserSub = rospy.Subscriber('/project_ctrl/laser/scan', LaserScan, self.__laser_callback)
        self._ur3IF = ur3IF
        self.distance = []
        self.angle = np.arange(0,720,1)/719.0*180.0
        self.resolution = 1000

    def get_raw_data(self):
        """
        Get the 2d original path of detected shape
        :return: [[x_series], [y_series]] of scanned surface data in descartes coordinate.
        """
        original_descartes_xy_points = []
        angle_range = []
        for i in range(0,720):
            if self.distance[i] < 1:
                angle_range.append(self.angle[i])
                calculated_point = self._IR2S(self.angle[i], self.distance[i])
                original_descartes_xy_points.append([calculated_point[0][0],calculated_point[0][1]])
                # Data validation
        if len(original_descartes_xy_points) > 10:
            # Create one white canvas 1m x 1m, unit is 1mm.
            canvas = np.ones((self.resolution, self.resolution, 1), dtype="uint8")
            # Set canvas to white
            canvas *= 255
            # Get current pose of UR3 robot
            curr_pos = self._ur3IF.get_current_pose()
            # Get the direction of object relative to robot in s-frame
            min_angle = 180 - angle_range[0]
            max_angle = 180 - angle_range[-1]
            lidar_direct_b_edge0 = np.array(
                [[cos(min_angle / 180 * pi)], [sin(min_angle / 180 * pi)], [0], [0]])
            lidar_direct_s_edge0 = np.matmul(curr_pos, lidar_direct_b_edge0)
            lidar_direct_b_edge1 = np.array(
                [[cos(max_angle / 180 * pi)], [sin(max_angle / 180 * pi)], [0], [0]])
            lidar_direct_s_edge1 = np.matmul(curr_pos, lidar_direct_b_edge1)
            # Calculated edge points on sides (for concave)
            first_edge = [original_descartes_xy_points[0][0] + lidar_direct_s_edge0[0],
                          original_descartes_xy_points[0][1] + lidar_direct_s_edge0[1]]
            second_edge = [original_descartes_xy_points[-1][0] + lidar_direct_s_edge1[0],
                           original_descartes_xy_points[-1][1] + lidar_direct_s_edge1[1]]
            # Append edge points to descartes points set
            original_descartes_xy_points = [[first_edge[0], first_edge[1]]] + original_descartes_xy_points + [
                [second_edge[0], second_edge[1]]]
            # Data conversion, mapping data from 0-1 to canvas(mm)
            original_descartes_xy_points = (np.array(original_descartes_xy_points) * self.resolution)

            return [original_descartes_xy_points.T[0], original_descartes_xy_points.T[1]]
        # if null
        return [[], []]

    def get_extended_path(self):
        """
        Get the 2d extended path of detected shape
        :return: [height, [x_series], [y_series], [yaw_x, yaw_y]] of extended path.
        """
        original_descartes_xy_points = []
        angle_range = []
        for i in range(0,720):
            if self.distance[i] < 1:
                angle_range.append(self.angle[i])
                calculated_point = self._IR2S(self.angle[i], self.distance[i])
                original_descartes_xy_points.append([calculated_point[0][0],calculated_point[0][1]])

        # Data validation
        if len(original_descartes_xy_points) > 10:
            # Create one white canvas 1m x 1m, unit is 1mm.
            canvas = np.ones((self.resolution, self.resolution, 1), dtype="uint8")
            # Set canvas to white
            canvas *= 255
            # Get current pose of UR3 robot
            curr_pos = self._ur3IF.get_current_pose()
            # Get the direction of object relative to robot in s-frame
            min_angle = 180-angle_range[0]
            max_angle = 180-angle_range[-1]
            lidar_direct_b_edge0 = np.array([[cos(min_angle/180*pi)],[sin(min_angle/180*pi)],[0],[0]])
            lidar_direct_s_edge0 = np.matmul(curr_pos, lidar_direct_b_edge0)
            lidar_direct_b_edge1 = np.array([[cos(max_angle/180*pi)],[sin(max_angle/180*pi)],[0],[0]])
            lidar_direct_s_edge1 = np.matmul(curr_pos, lidar_direct_b_edge1)
            # Calculated edge points on sides (for concave)
            first_edge = [original_descartes_xy_points[0][0]+lidar_direct_s_edge0[0], original_descartes_xy_points[0][1]+lidar_direct_s_edge0[1]]
            second_edge = [original_descartes_xy_points[-1][0]+lidar_direct_s_edge1[0], original_descartes_xy_points[-1][1]+lidar_direct_s_edge1[1]]
            # Append edge points to descartes points set
            original_descartes_xy_points = [[first_edge[0],first_edge[1]]] + original_descartes_xy_points + [[second_edge[0],second_edge[1]]]
            # Data conversion, mapping data from 0-1 to canvas(mm)
            original_descartes_xy_points = (np.array(original_descartes_xy_points)*self.resolution)
            original_descartes_xy_points = np.array(original_descartes_xy_points, np.int32).reshape((-1,1,2))
            # Draw the close loop detected shape.
            cv2.fillPoly(canvas, pts=[original_descartes_xy_points], color=(0,0,0))
            cv2.imwrite('original_data.png', canvas)
            # Blur the shape
            blurred_path = cv2.GaussianBlur(canvas, (int(self.resolution/20.0)*2+1, int(self.resolution/20.0)*4+1), int(self.resolution/20.0)+1)
            cv2.imwrite('blurred_data.png', blurred_path)
            # Use threshold to obtain extended shape
            threshold_path = cv2.inRange(blurred_path, 0,254)
            cv2.imwrite('filtered_data.png', threshold_path)
            # Get extended path
            _, extended_path, _ = cv2.findContours(threshold_path, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            gradient_x = cv2.Sobel(blurred_path, cv2.CV_64F, 1, 0, ksize=9)
            gradient_y = cv2.Sobel(blurred_path, cv2.CV_64F, 0, 1, ksize=9)
            cv2.imwrite('gradient_x.png', gradient_x/2+200)
            cv2.imwrite('gradient_x.png', gradient_y/2+200)
            yaw_x = []
            yaw_y = []

            for i in extended_path[0]:
                yaw_x.append(gradient_x[i[0][1]][i[0][0]]*10)
                yaw_y.append(gradient_y[i[0][1]][i[0][0]]*10)

            return [curr_pos[2][3],extended_path[0].T[0][0],extended_path[0].T[1][0], [yaw_x,yaw_y]]
        # if null
        return [[],[],[[],[]]]

    def __laser_callback(self, msg):
        """Callback function for process the feedback from laser scanner
        """
        self.distance = msg.ranges

    def _IR2S(self, laser_angle, laser_dist):
        """
        Convert IR sensor data to laser_dist
        :param laser_angle: Angle data [Array or Number] from laser
        :param laser_dist: Distance data [Array or Number] from laser
        :return: The poses of angles
        """
        angle = 180 - laser_angle  # laserIF.angle
        distance = laser_dist  # laserIF.distance
        curr_pos = self._ur3IF.get_current_pose()
        distance -= 0.05
        try:
            vect = np.array([[cos(angle / 180 * pi) * distance - 0.0535], [sin(angle / 180 * pi) * distance], [0], [1]])
            world_pos = np.matmul(curr_pos, vect)
            return world_pos.T
        except ValueError:
            pass
        return np.zeros(4)
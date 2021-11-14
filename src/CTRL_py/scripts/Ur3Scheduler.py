from Ur3Interface import *

class UR3Scheduler():
    def __init__(self, ur3IF_):
        self.ur3IF = ur3IF_

    def move_along_discrete_trace(self, trace, vel, accel):
        converted_thetas = []
        print ("Perform Inverse Kinematics Calculation")
        print ("\n")
        for i in trace:
            converted_thetas.append(self.__inv_kinetics(*i))
        for i in converted_thetas:
            self.ur3IF.set_angle(i, vel, accel)

    def __inv_kinetics(self, xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
        """Calculate the joint angles for given position.

        :param xWgrip: target X in world frame
        :param yWgrip: target Y in world frame
        :param zWgrip: target Z in world frame
        :param yaw_WgripDegree: YAW angle of the plate
        :return:
        """
        L1 = 0.152
        L2 = 0.12
        L3 = 0.244
        L4 = 0.093
        L5 = 0.213
        L6 = 0.083
        L7 = 0.083
        L8 = 0.082
        L9 = 0.0535
        L10 = 0.059
        xgrip = xWgrip + 0.15
        ygrip = yWgrip - 0.15
        zgrip = zWgrip - 0.01
        xcen = xgrip - L9 * np.cos(np.deg2rad(yaw_WgripDegree))
        ycen = ygrip - L9 * np.sin(np.deg2rad(yaw_WgripDegree))
        zcen = zgrip
        alpha = np.arctan2(0.027 + L6, np.sqrt(xcen**2+ycen**2-(L6 + 0.027)**2))
        theta1 = np.arctan2(ycen,xcen) -  alpha
        theta6 = np.pi/2 + theta1-np.deg2rad(yaw_WgripDegree)
        x3end = xcen - L7*np.cos(theta1) + (0.027+L6)*np.sin(theta1)
        y3end = ycen - L7*np.sin(theta1) - (0.027+L6)*np.cos(theta1)
        z3end = zcen + L10 + L8
        M1 = np.sqrt(x3end**2 + (z3end-L1)**2 + y3end**2)
        omega = np.arctan2((z3end - L1), np.sqrt(x3end**2+y3end**2))
        theta3 = np.pi - np.arccos((L3**2 + L5**2 - M1**2)/(2*L3*L5))
        theta2 = -(omega + np.arccos((L3**2 + M1**2 - L5**2)/(2*L3*M1)))
        theta4 = -(theta3 + theta2)
        theta5 = -np.pi/2
        thetas = self.__convert_angles(theta1, theta2, theta3, theta4, theta5, theta6)
        return thetas

    def __convert_angles(self, theta1, theta2, theta3, theta4, theta5, theta6):
        """Convert angles to joint angles for robot
        """
        return_value = [None, None, None, None, None, None]

        return_value[0] = theta1 + math.pi
        return_value[1] = theta2
        return_value[2] = theta3
        return_value[3] = theta4 - (0.5*math.pi)
        return_value[4] = theta5
        return_value[5] = theta6
        return return_value
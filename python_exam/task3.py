#!/usr/bin/env python

from robot_control_class import RobotControl

class ExamControl():
    def get_laser_readings(self):
        self.laser_value_left = self.robot_control.get_laser(719)
        self.laser_value_right = self.robot_control.get_laser(0)
        return [self.laser_value_left, self.laser_value_right]

    def __init__(self):
        self.robot_control = RobotControl(robot_name="turtlebot3")
        self.laser_readings = self.get_laser_readings()
        print (self.laser_readings)
        print (type(self.laser_readings))

if __name__ == "__main__":
    ExamControl()

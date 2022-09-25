#!/usr/bin/env python

from robot_control_class import RobotControl

class RobotMotion():
    def turn_robot(self):
        self.robot_control.turn("clockwise", 0.4, 4)
    def __init__(self):
        self.robot_control = RobotControl(robot_name='turtlebot3')
        self.front_laser_value = self.robot_control.get_front_laser()
        while (self.front_laser_value > 1):
            self.front_laser_value = self.robot_control.get_front_laser()
            self.robot_control.move_straight()
            if (self.front_laser_value <= 1):
                print (self.front_laser_value)
                break
        if (self.front_laser_value <= 1):
            self.robot_control.stop_robot()
            self.turn_robot()

if __name__ == "__main__":
    RobotMotion()


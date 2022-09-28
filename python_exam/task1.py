#!/usr/bin/env python

from robot_control_class import RobotControl
laser_readings_list = []
lowest_value = 0
highest_value = 0


def get_highest_lowest():
    robot_control = RobotControl(robot_name="turtlebot3")
    laser_readings = robot_control.get_laser_full()

    for i in range(len(laser_readings)):
        laser_readings_list.append(laser_readings[i])
        i += 1
        global lowest_value
        global highest_value
        lowest_value = min(laser_readings_list)
        if(max(laser_readings_list) < 199999):
            highest_value = max(laser_readings_list)

    for i in range(len(laser_readings_list)):
        if (laser_readings_list[i] == lowest_value):
            lowest_value_position = i
            break
    for i in range (len(laser_readings_list)):
        if (laser_readings_list[i] == highest_value):
            highest_value_position = i
            break

    return [highest_value_position+1, lowest_value_position+1]



if __name__ == "__main__":
    get_highest_lowest()



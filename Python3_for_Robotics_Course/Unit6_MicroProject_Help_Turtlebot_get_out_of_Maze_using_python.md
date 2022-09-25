------

#         Python for robotics    

------

#         Unit 6                 MicroProject    

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/turtle_proj2.png)

##         6.1                 Scene    

This first part of the project will be based on a new environment, which is a maze like this one:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/maze.png)

##         6.2                 Turtlebot    

For this project, you will use Turtlebot robot.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/kobuki.jpg)

##         6.3                 Task to accomplish    

Create a Python program that, using ALL the methods from the **robot_control_class.py** Python script, helps the Turtlebot robot to get out of the maze.

Just to help you out, let's make a summary of all the methods you've been using throughout the course:

- get_laser (direction)

  : As the name itself says,  this method will allow you to get data from the laser of the robot. You  will need to pass one parameter to this method:

  - **direction**: Here you will specify a number between 0 and 719, which will indicate the direction of the laser reading.

- **get_laser_full()**: As the name itself says, this  method will allow you to get all the data from ALL the laser beams of  the robot. As I've said before, this is a total of 719 different  readings. So, when called, this method will return a **LIST** containing all the 719 different readings from the laser beams.

- **move_straight()**: As the name says, this method will allow you to start moving the robot in a straight line.

- **stop_robot()**: As the name says, this method will allow you to stop the robot from moving.

- move_straight_time(motion, speed, time)

  : As the  name itself says, this method will allow you to move the robot in a  straight line. You will need to pass three parameters to it.

  - **motion**: Specify here if you want your robot to move forward (**"forward"**) or backward (**"backward"**).
  - **speed**: Specify here the speed at which you want your robot to move (in m/s).
  - **time**: Specify here how long you want your robot to keep moving (in seconds).

- turn (clockwise, speed, time)

  : As the name itself says, this method will allow you to turn the robot. You will need to pass three parameters to it.

  - **clockwise**: Specify here whether you want your robot to turn clockwise (**"clockwise"**) or counter-clockwise (**"counter-clockwise"**).
  - **speed**: Specify here the speed at which you want your robot to turn (in rad/s).
  - **time**: Specify here how long you want your robot to keep turning (in seconds).

Finally, and for this project, you are going to be able to use a new method.

- rotate (degrees)

  : This method will allow your robot to rotate for the specified number of degrees.

  - **degrees**: The number of degreees you want your robot to rotate.

**NOTE**: In theory, you can also make the  robot rotate a certain number of degrees by using the **turn()** method  and playing with the *speed* and *time* values. However, this method is  going to be less precise than the **rotate()** method, because it's not  taking into account external sources like the odometry of the robot  (which the **rotate()** method does).

​    \- robot_control_class.py -

Below you can find the updated code for the **robot_control_class.py** class, which contains the new **rotate()** method.





```
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time
from math import radians, copysign, sqrt, pow, pi
import PyKDL


class RobotControl():

    def __init__(self):
        rospy.init_node('robot_control_node', anonymous=True)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.summit_vel_publisher = rospy.Publisher('/summit_xl_control/cmd_vel', Twist, queue_size=1)
        self.laser_subscriber = rospy.Subscriber(
            '/kobuki/laser/scan', LaserScan, self.laser_callback)
        self.summit_laser_subscriber = rospy.Subscriber(
            '/hokuyo_base/scan', LaserScan, self.summit_laser_callback)
        self.odom_sub = rospy.Subscriber ('/odom', Odometry, self.odom_callback)
        self.cmd = Twist()
        self.laser_msg = LaserScan()
        self.summit_laser_msg = LaserScan()
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        self.tf_listener = tf.TransformListener()
        self.odom_frame = '/odom'
        self.base_frame = '/base_link'
        self.angular_tolerance = radians(2)
        rospy.on_shutdown(self.shutdownhook)

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            summit_connections = self.summit_vel_publisher.get_num_connections()
            if connections > 0 or summit_connections > 0:
                self.vel_publisher.publish(self.cmd)
                self.summit_vel_publisher.publish(self.cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def laser_callback(self, msg):
        self.laser_msg = msg

    def summit_laser_callback(self, msg):
        self.summit_laser_msg = msg

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

    def get_laser(self, pos):
        time.sleep(1)
        return self.laser_msg.ranges[pos]

    def get_laser_summit(self, pos):
        time.sleep(1)
        return self.summit_laser_msg.ranges[pos]

    def get_front_laser(self):
        time.sleep(1)
        return self.laser_msg.ranges[360]

    def get_laser_full(self):
        time.sleep(1)
        return self.laser_msg.ranges

    def stop_robot(self):
        #rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def move_straight(self):

        # Initilize velocities
        self.cmd.linear.x = 0.5
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        # Publish the velocity
        self.publish_once_in_cmd_vel()

    def move_straight_time(self, motion, speed, time):

        # Initilize velocities
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        self.cmd.angular.z = 0

        if motion == "forward":
            self.cmd.linear.x = speed
        elif motion == "backward":
            self.cmd.linear.x = - speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            self.summit_vel_publisher.publish(self.cmd)
            i += 0.1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Moved robot " + motion + " for " + str(time) + " seconds"
        return s


    def turn(self, clockwise, speed, time):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        if clockwise == "clockwise":
            self.cmd.angular.z = -speed
        else:
            self.cmd.angular.z = speed

        i = 0
        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        
        while (i <= time):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            self.summit_vel_publisher.publish(self.cmd)
            i += 0.1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Turned robot " + clockwise + " for " + str(time) + " seconds"
        return s

    def get_odom(self):

        # Get the current transform between the odom and base frames
        tf_ok = 0
        while tf_ok == 0 and not rospy.is_shutdown():
            try:
                self.tf_listener.waitForTransform('/base_link', '/odom', rospy.Time(), rospy.Duration(1.0))
                tf_ok = 1
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                pass

        try:
            (trans, rot)  = self.tf_listener.lookupTransform('odom', 'base_link', rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), self.quat_to_angle(Quaternion(*rot)))

    def rotate(self, degrees):

        position = Point()

        # Get the current position
        (position, rotation) = self.get_odom()

        # Set the movement command to a rotation
        if degrees > 0:
            self.cmd.angular.z = 0.3
        else:
            self.cmd.angular.z = -0.3

        # Track the last angle measured
        last_angle = rotation
        
        # Track how far we have turned
        turn_angle = 0

        goal_angle = radians(degrees)

        # Begin the rotation
        while abs(turn_angle + self.angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle         
            self.vel_publisher.publish(self.cmd) 
            self.rate.sleep()
            
            # Get the current rotation
            (position, rotation) = self.get_odom()
            
            # Compute the amount of rotation since the last lopp
            delta_angle = self.normalize_angle(rotation - last_angle)
            
            turn_angle += delta_angle
            last_angle = rotation
        
        self.stop_robot()

    def quat_to_angle(self, quat):
        rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
        return rot.GetRPY()[2]
        
    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res


if __name__ == '__main__':
    #rospy.init_node('robot_control_node', anonymous=True)
    robotcontrol_object = RobotControl()
    try:
        robotcontrol_object.move_straight()

    except rospy.ROSInterruptException:
        pass
```
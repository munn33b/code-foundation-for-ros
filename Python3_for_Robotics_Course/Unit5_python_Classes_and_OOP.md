------

#         Python for robotics    

------

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/python3_logo.png)

#         Unit 5                 Classes and Object Oriented Programming    

​    \- Summary -

Estimated time to completion: **2 hours**

 In this unit, you are going to learn **how to organize your code properly to prevent messy code** when things get complicated (that usually happens when working with  robots). The organization method/principle used in Python is called **Object Oriented Programming** and it is based on the concept of **Python classes**.

​    \- End of Summary -

##         5.1                 Object-Oriented Programming    

In all the programs we've written up until now, we have designed our  programs around methods, which manipulate data stored in variables. This is called the *procedure-oriented* way of programming.

There is another way of programming! This one is based on combining data and methodality, and wrapping them inside *things* known as **objects** (also called classes). This way of programming is known as *object-oriented programming*, or OOP.

Most of the time, you will still be using procedural programming for  simple projects, but when writing larger and more complex programs, this method can become a pain. In those cases, it is much better to use OOP, since your code will be better organized and it will be much **easier to understand, debug, maintain, and upgrade**.

So, as I've said before, OOP is based on the concept of "objects." These objects are usually defined by two things:

- Attributes: This is the data associated with the object, defined in fields.
- Methods: These are the procedures or methods associated with the object.

The most important feature of objects is that the methods associated  with them can access and modify the data fields of the object with which they are associated.

##         5.2                 What are Python classes?    



So, everything sounds super cool and interesting, but I haven't read a word yet about Python classes. What the heck are Python classes? Let's  see!

**A Python class is, basically, a code template for creating an object**.

For example, the class *Jedi* below describes how to create an object of type Jedi, so you can use that object of that type to store  and manipulate data the way the class defines.

For example, if my code has created the class *Jedi*, then I can create an object of that type by doing:





```
my_object = Jedi("Qui-Gong-Jin")
```

That means that the object *my_object* is a variable of type *Jedi*.

Remember, we already used that in previous chapters. For example:





```
rc = RobotControl()
```

In the previous code, we were creating a variable (or object) named *rc* of type *RobotControl*.

The difference between the *Jedi* class and the *RobotConnector* class is that the former requires a parameter to build the object  (requires the name of the Jedi), and the later requires no parameter to  build the obejct.

**NOTE:** When we create a variable of class X, we say that we have **created an instance of the class X**. In the code, above we created an instance of the class Jedi.

##         5.3                 How to define a class in Python    

What follows is a simple example of the definition of a Python class named *Jedi*:

**Python File: jedi_class.py** 





```
class Jedi:
    def __init__(self, name):
        self.jedi_name = name

    def say_hi(self):
        print('Hello, my name is ', self.jedi_name)

j1 = Jedi('ObiWan')
j1.say_hi()

j2 = Jedi('Anakin')
j2.say_hi()
```

**END Python File: jedi_class.py** 

The code above has two parts:

1. Where we define the class
2. Where we use the definition to create and use variables of type *Jedi* (*j1* and *j2*)

###         5.3.1                 Definition of a class    

Let's quickly analyze the above class. The first thing that we see is the tag *class* and the name *Jedi*.





```
class Jedi:
```

This means, let's define a class named *Jedi*. Then, everything that goes beneath that line and that contains an indentation belongs to the code of that class.

Let's see the different parts that can contain a class.

###         5.3.2                 methods of the class    

Then, we continue with the tag *def*. As you know from the  previous chapter, that means that what follows is the definition of a  method. However, in this case, the method is just defined for the class *Jedi* because, as you can see, it has an indentation within the class.

As you can see, in the code, there are two *methods of the class*:





```
def __init__(self,name):
```





```
def say_hi(self):
```

Note that each of the methods contain the **self** keyword as a parameter. That keyword indicates that **the methods belong to the class**. Including the *self* keyword is mandatory for each method of the class.

###         5.3.3                 What does it mean that a method is only of the class?    

It means that you can only call that method through an instance of the class. That is, you cannot call the method *say_hi()* in your programs by doing:





```
say_hi()
```

In order to call that method, you need to **first create an instance of the class \*Jedi\* and then call the method**. Like this:





```
j = Jedi()
j.say_hi()
```

You are going to understand the reason why below.

###         5.3.4                 The class constructor    

From the two methods defined above, there is one that is very special and that is called **the constructor of the class**. That method is the following:





```
def __init__(self, name):
    self.jedi_name = name
```

For any class that you create (the *Jedi* class, the *RobotControl* class, or any other that you may invent), the method named **__init__** is called the constructor of the class.

What is special about the constructor? **The constructor is automatically called (by the Python system) every time that a new instance of the class is created**.

For example, when we created the instance of the *Jedi* class, the **__init__(self, name)** method was called. As you know from previous chapters, the parameter *name* is provided to the constructor. So, the constructor uses that parameter to store it in the variable *self.jedi_name*.

The constructor of a class is a very important method because it is  the one that is used to initialize the class with the proper values,  based on the parameters provided. For example, in the code above:





```
j1 = Jedi('ObiWan')
```

We are creating an instance of the class *Jedi*, providing to it the parameter *ObiWan*. That is used by the **__init__** method of the *Jedi* class (the constructor of the *Jedi* class) to initialize that *self.jedi_name* variable. That is the only thing that we do in the constructor, but we could add anything else required to have the *Jedi* instance properly configured.

###         5.3.5                 Variables of the class    

A class usually has variables inside to store proper values for every instance. Those variables allow differentiation of every instance of  the same class from each other. For example, in the code above, the  class *Jedi* has a variable of the class named *self.jedi_name*. That variable must contain the name of the Jedi for each instance of the class.

That is why, later in the code, we have:





```
j1 = Jedi('ObiWan')
j2 = Jedi('Anakin')
```

We have created two different instances of the same class. But each  instance is associated with different Jedi names. The value of *self.jedi_name* for *j1* is *ObiWan*, but the value of *self.jedi_name* for *j2* is *Anakin*.

You see!? You can create as many instances of a class as you want, but each one has a different value in their class parameters.

**IMPORTANT:** To create a variable of a class, it always has to start by the prefix *self.* and then the variable name. REMEMBER to add that *self.* prefix on any place of the class.

**Variables of the class are like \*global\* variables for that class!** What does that mean? It means that the variables can be used in any  method that belongs to the class, like when we were using the *global* keyword in Unit 4.

**IMPORTANT 2:** This is one of the main points of using classes. You will not have to use global variables that can be accessed in any part of the code, contributing to the building of a mess. By **encapsulating** the variables within the class, only the methods of the class can  access those variables, hence, you have less chance of messing it up.

**IMPORTANT 3:** From now on, you cannot use the *global* keyword in your programs. That is a very bad practice!!!  Instead, you should organize your Python program to use classes that do  not need *global* variables (instead, they use the variables of the  class).

###         5.3.6                 Adding methods to the class    

Apart from the constructor of the class, you can define any other  method that you need to make your class useful. For example, in the  class *Jedi*, there is an extra method named *say_hi()*, which makes the class print a hello message.





```
def say_hi(self):
        print('Hello, my name is', self.jedi_name)
```

This is another method of the class. In this case, it just contains a simple print inside which takes into account the value of the variable *self.jedi_name*.

**IMPORTANT:** As you can see in the method *say_hi()*, the method can access the variable of the class *self.jedi_name*, without having to use the *global* keyword, because the *self.jedi_name* is a variable of the class, hence it can be accessed by any method of the class.

**NOTE:** The methods of the class can modify any  variable of the class! They can access those values for reading or even  for modifying the values.

###         5.3.7                 Calling a class method    

As explained above, in order to call a method that belongs to the  class, you need to first create an instance of the class. This allows  you to have different instances of the class, that, when calling any of  its member methods, will produce different results.

For example, in the code above, we have the following code:





```
j1 = Jedi('ObiWan')
j1.say_hi()

j2 = Jedi('Anakin')
j2.say_hi()
```

This means that we are creating **two different instances of the same class \*Jedi\***. Each instance is about a different Jedi. Hence, calling the *say_hi()* method for each instance will produce different results.

Go to the code above that contains the whole code for the *Jedi* class, select the cell and execute the code by pressing the *play* button. You should get an output like this:





```
Hello, my name is  ObiWan
Hello, my name is  Anakin
```

The first *Hello ...* sentence is the output of calling the *say_hi()* method for the first instance (j1) of the *Jedi* class. The second sentence is the output of calling the method for the second instance (j2) of the *Jedi* class.

That is the beauty of classes: getting different results to the same method based on the internal values of the class variables!

​    \- Exercise 5.1 -

**Create a Python program that makes the robot perform 2 squares. The second one will be double the size of the first one.**

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/summit_square.png)

ALL the parameters that you need to pass to the methods in order to control the robot (**motion, clockwise, speed, time, and maybe others**) will have to be declared as **attributes** of the class.

You will need to:

1. Import the *RobotControl* class.
2. Use the *move_straight_time()* and *turn()* methods of that class.
3. Create the constructor of your class.
4. Create the necessary methods of your class, in order to perform a square.
5. Create 2 different instances of the class, one for each square movement.

​    \- End of Exercise 5.1 -

​    \- Solution for Exercise 5.1 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/TC-logo-blue.png)

Please try to do it by yourself unless you get stuck or need some  inspiration. You will learn much more if you fight for each exercise.

test_class.py





```
from robot_control_class import RobotControl

class MoveRobot:
    def __init__(self, motion, clockwise, speed, time):
        self.robotcontrol = RobotControl(robot_name="summit")
        self.motion = motion
        self.clockwise = clockwise
        self.speed = speed
        self.time = time
        self.time_turn = 7.0 # This is an estimate time in which the robot will rotate 90 degrees

    def do_square(self):

        i = 0

        while (i < 4):
            self.move_straight()
            self.turn()
            i+=1

    def move_straight(self):
        self.robotcontrol.move_straight_time(self.motion, self.speed, self.time)

    def turn(self):
        self.robotcontrol.turn(self.clockwise, self.speed, self.time_turn)


mr1 = MoveRobot('forward', 'clockwise', 0.3, 4)
mr1.do_square()
mr2 = MoveRobot('forward', 'clockwise', 0.3, 8)
mr2.do_square()
```

​    \- End Solution for Exercise 5.1 -

##         5.4                 How the class RobotControl works    

Now that you are equipped with all the knowledge to write your own Python programs, it is time for you to check the *RobotControl* class that you have been using throughout the whole course!

This is the code of the class:

robot_control_class.py





```
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time


class RobotControl():

    def __init__(self, robot_name="turtlebot"):
        rospy.init_node('robot_control_node', anonymous=True)

        if robot_name == "summit":
            rospy.loginfo("Robot Summit...")
            cmd_vel_topic = "/summit_xl_control/cmd_vel"
            # We check sensors working
            self._check_summit_laser_ready()
        else:      
            rospy.loginfo("Robot Turtlebot...")      
            cmd_vel_topic='/cmd_vel'
            self._check_laser_ready()

        # We start the publisher
        self.vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.cmd = Twist()        

        self.laser_subscriber = rospy.Subscriber(
            '/kobuki/laser/scan', LaserScan, self.laser_callback)
        self.summit_laser_subscriber = rospy.Subscriber(
            '/hokuyo_base/scan', LaserScan, self.summit_laser_callback)
        
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)

    
    def _check_summit_laser_ready(self):
        self.summit_laser_msg = None
        rospy.loginfo("Checking Summit Laser...")
        while self.summit_laser_msg is None and not rospy.is_shutdown():
            try:
                self.summit_laser_msg = rospy.wait_for_message("/hokuyo_base/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /hokuyo_base/scan READY=>" + str(self.summit_laser_msg))

            except:
                rospy.logerr("Current /hokuyo_base/scan not ready yet, retrying for getting scan")
        rospy.loginfo("Checking Summit Laser...DONE")
        return self.summit_laser_msg

    def _check_laser_ready(self):
        self.laser_msg = None
        rospy.loginfo("Checking Laser...")
        while self.laser_msg is None and not rospy.is_shutdown():
            try:
                self.laser_msg = rospy.wait_for_message("/kobuki/laser/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /kobuki/laser/scan READY=>" + str(self.laser_msg))

            except:
                rospy.logerr("Current /kobuki/laser/scan not ready yet, retrying for getting scan")
        rospy.loginfo("Checking Laser...DONE")
        return self.laser_msg

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
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
            i += 1
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
            i += 1
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

        s = "Turned robot " + clockwise + " for " + str(time) + " seconds"
        return s


if __name__ == '__main__':
    
    robotcontrol_object = RobotControl()
    try:
        robotcontrol_object.move_straight()

    except rospy.ROSInterruptException:
        pass
```

robot_control_class.py

###         5.4.1                 Code Explanation    

First of all, let me point out that, within this class, some ROS  concepts are already introduced. This might be a little confusing to you right now, at least until you take the [**ROS Basics in 5 Days Course**](http://www.theconstructsim.com/construct-learn-develop-robots-using-ros/robotigniteacademy_learnros/ros-courses-library/ros-python-course/). Anyway, at this point, you should be able to fully understand the different parts of the class and its main structure.

First of all, we find all the *imports* here:





```
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
```

As you already learnt in Unit 2, imports allow you to include in your program the code created in other Python modules. In this case, we are  importing some ROS-related modules that we will use, and also the *time* module, which you have already used in previous lessons.

Next, we can see the definition of the class.





```
class RobotControl():
```

Nothing new here. We are defining a class, named *RobotControl*, which doesn't take any argument. Let's keep going!

Now we can see the **__init__** method, also know as *constructor*, as you already know.





```
def __init__(self, robot_name="turtlebot"):
        rospy.init_node('robot_control_node', anonymous=True)

        if robot_name == "summit":
            rospy.loginfo("Robot Summit...")
            cmd_vel_topic = "/summit_xl_control/cmd_vel"
            # We check sensors working
            self._check_summit_laser_ready()
        else:      
            rospy.loginfo("Robot Turtlebot...")      
            cmd_vel_topic='/cmd_vel'
            self._check_laser_ready()

        # We start the publisher
        self.vel_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.cmd = Twist()        

        self.laser_subscriber = rospy.Subscriber(
            '/kobuki/laser/scan', LaserScan, self.laser_callback)
        self.summit_laser_subscriber = rospy.Subscriber(
            '/hokuyo_base/scan', LaserScan, self.summit_laser_callback)
        
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)
```

And here some sensor checks to avoid that we look into sensors that dont have any data:





```
def _check_summit_laser_ready(self):
        self.summit_laser_msg = None
        rospy.loginfo("Checking Summit Laser...")
        while self.summit_laser_msg is None and not rospy.is_shutdown():
            try:
                self.summit_laser_msg = rospy.wait_for_message("/hokuyo_base/scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /hokuyo_base/scan READY=>" + str(self.summit_laser_msg))

            except:
                rospy.logerr("Current /hokuyo_base/scan not ready yet, retrying for getting scan")
        rospy.loginfo("Checking Summit Laser...DONE")
        return self.summit_laser_msg

def _check_laser_ready(self):
    self.laser_msg = None
    rospy.loginfo("Checking Laser...")
    while self.laser_msg is None and not rospy.is_shutdown():
        try:
            self.laser_msg = rospy.wait_for_message("/kobuki/laser/scan", LaserScan, timeout=1.0)
            rospy.logdebug("Current /kobuki/laser/scan READY=>" + str(self.laser_msg))

        except:
            rospy.logerr("Current /kobuki/laser/scan not ready yet, retrying for getting scan")
    rospy.loginfo("Checking Laser...DONE")
    return self.laser_msg
```

Inside the constructor of the class, we are initializing and defining many different ROS elements, so we are going to ignore this part for  now. Just keep in mind that here we are doing all the setup in order to  be able to communicate with the simulated robot.

Do you know what comes next? Yes! It's the methods of the class. For  this class, we have many of them. Most of them, though, might already  sound familiar to you since you have been using them all along in this  course.

For instance, you can see all the laser methods, which have allowed you to get the laser readings from the robots:





```
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
```

Or also the different methods that you have been using in order to move the robots around:





```
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
        i += 1
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
        i += 1
        self.rate.sleep()

    # set velocity to zero to stop the robot
    self.stop_robot()

    s = "Turned robot " + clockwise + " for " + str(time) + " seconds"
    return s
```

Again, most of the code you can see here is ROS related, so we are  not going to enter into details. But you can get the general idea, which is that the Python class *RobotControl* has many different  methods that allow you to interact with the robot, either to get data  from it (laser readings) or to send data to it (move it).

Finally, you can see the *main* method:





```
if __name__ == '__main__':

    robotcontrol_object = RobotControl()
    try:
        robotcontrol_object.move_straight()

    except rospy.ROSInterruptException:
        pass
```

For our course, this piece of code is not very important because we  are not going to use it. But let's try to explain it very quickly. All  throughout the course, you have been importing this *RobotControl* class and using its different methods. But... what if you try to  execute this file directly? Let's say, with the following command:





```
python robot_control_class.py
```

In this case, the *main* method would be executed. Ignoring the *ROSInterruptException* part, what you have inside this method is pretty simple (and familiar).





```
robotcontrol_object = RobotControl()
robotcontrol_object.move_straight()
```

As you can see, all you are doing here is creating an instance of the *RobotControl()* class in the *robotcontrol_object* variable. And then, you call the method *move_straight()* of this class.

So, applying this to the *Jedi* class, for instance, it would result in something like this:

**Python File: jedi_class.py (UPDATED)** 





```
class Jedi:
    def __init__(self, name):
        self.jedi_name = name

    def say_hi(self):
        print('Hello, my name is ', self.jedi_name)
        
if __name__ == '__main__':

    j1 = Jedi('ObiWan')
    j1.say_hi()

    j2 = Jedi('Anakin')
    j2.say_hi()
```

**Python File: jedi_class.py (UPDATED)** 

And remember, if you are interested in learning more about ROS, and  what all those methods are really doing behind the scenes, you can get  all that knowledge from our course [**ROS Basics in 5 Days**](http://www.theconstructsim.com/construct-learn-develop-robots-using-ros/robotigniteacademy_learnros/ros-courses-library/ros-python-course/), where you will also have a chance to practice everything you've learned from Python.

##         5.5                 Conclusion    

This is the end of the course. Along the course, you have learnt the  basics of Python, but there is still a lot more to learn about that  excellent language.

Our recommendation is that you start learning the ROS Basics in 5  days course now using your knowledge of Python, and whenever you find a  piece of code in the ROS programs we provide you that has not been  explained in this course, go to the internet and find out what it means. You will be able to undersntad it very quickly now that you have a  basic knowledge of the language.

[ROS Basics in 5 Days](https://app.theconstructsim.com/#/Course/55)

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/basics_course.png)

**FINAL RECOMMENDATION:** If you go to the ROS Basics  course now, you will see that you will have to do a lot of exercises  writing Python code for ROS. You have two options there: write the code  using a procedural method, or using Python classes. Our recommendation  is that you always try to do the exercises using classes, since it is  the method of professional ROS programmers.

### Keep pushing your Python learning!
------

#         Python for robotics    

------

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/python3_logo.png)

#         Unit 2                 Python Essentials    

​    \- Summary -

Estimated time to completion: **2 hours**

In this unit, you are going to learn **how to create variables in Python**, which **types of variables** you can create, and how you can **operate with those variables**. Remember that variables are like storage space for interesting data. We are going to use variables to store sensor data and manipulate it to  understand the current situation of the robot. Additionally, you will  learn **how to add comments** to the code.

​    \- End of Summary -

In this course, we want to keep the focus on Python, not on ROS.

However, in order to be able to interact with the simulated robot  while hiding all the ROS stuff, we are providing you with a Python class in charge of managing all the ROS connections under the hood. This  class is called *RobotControl*. So, during this course, you will  be interacting with this Python class by calling its methods to get data from the robots and send commands to them.

**NOTE:** maybe some of these concepts, like Python *classes* or *methods*, sound weird to you right now, but don't  worry, you will learn about them later in the course.

So in the next section you will add this **RobotControl** class. It's not important that you understand the code of this class now, but you will by the end of this course.

Also, because we are using **ROS noetic, Python 3 comes by default**. This means that you won't need any you virtual environment.

And that's it! Now you are ready to start working with Python3 and ROS!

So, with the proper introductions made, let's now go to work!

##         2.1                 Data Types and Variables    

As we like to do at the Ignite Academy, let's start with some  practice. In the following exercise, you are going to create and execute a very simple Python script that will allow us to introduce some  important basic concepts related to Python.

​    \- Exercise 2.1 -

Let's create a directory to store our RobotControl class code. You will name this directory **robot_control**. Inside this **robot_control** directory, we will create a new Python script named **robot_control_class.py**, which will store our class code, and another script named **pyscript1.py**, which we will use for testing this class.

​             Execute in Shell #1





```
cd ~/catkin_ws/src/
```





```
mkdir robot_control
```





```
cd robot_control
```





```
touch pyscript1.py
touch robot_control_class.py
```

Copy the following code into **robot_control_class.py**

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

end robot_control_class.py

**NOTE**: The *touch* command is used to  create a new empty file. If you want, though, you can also create a new  file directly through the IDE.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/newfile2.png)

After the file creation, you should end up with the following files inside your **robot_control** folder.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/robot_control_contents.png)

Now, inside the **pyscript1.py** Python script, copy the following contents:

pyscript1.py





```
from robot_control_class import RobotControl

rc = RobotControl()

a = rc.get_laser(360)

print ("The distance measured is: ", a)
```

pyscript1.py

Have a look at the code and try to understand what it does.

Now, let's execute the script. For that, you can execute the following command in your WebShell:

​             Execute in Shell #1





```
python pyscript1.py
```

You should get an output like this one:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/py3venv_pyscript1.png)

​    \- End of Exercise 2.1 -

So... what just happened? Let's try to explain it.

###         2.1.1                 Code explanation    

The first thing we can see in the script is the following line:





```
from robot_control_class import RobotControl
```

There, we are importing a Python class named **RobotControl**. This class is the one we at Robot ignite Academy have created to help  you interact with the ROS robot without having to actually use ROS.

#### What does it mean *to import*?

**Imports** are very useful and common in Python. **Imports** allow you to include in your program the code created in other Python  modules. This means that the Python code in your program can execute the Python code defined in another program file (in this case, the code of  the *RobotControl* class), without having to rewrite it.

To import the code of another Python module, indicate the following:





```
from <name_of_the_module> import <method_or_class_to_be_imported>
```

In our case, we are importing a Python class named *RobotControl*, which is defined in the Python module named *robot_control_class*.

**How do you know which modules and classes to import?**

That is a very good question! You must know beforehand which modules  may be interesting for your work, and which methods or classes those  modules provide for importing. In order to know that, you must check the documentation of the code provided by the people who created those  modules.

In this case, we (Robot Ignite Academy) have created the module *robot_control_class* and we know that inside that module, there is a class named *Robotcontrol* that provides all that we need to interact with the robot.

If you are curious and want to have a look at the *robot_control_class* and what it contains and provides, you can use the webshell and  navigate to the following directory and check the code we have created  for this course:





```
/home/simulations/public_sim_ws/src/all/ros_basics_examples/python_course_class
```

#### Creating a variable

The next line we can see is the following one:





```
rc = RobotControl()
```

Here we are creating a variable named **rc**, which is of the **RobotControl** type. We will use that variable to call the methods of that class that  allow us to get robot sensor data. You will learn about this in the  Python Classes chapter, later in the course. For now, just remember how  to create a variable of a certain type.

#### Calling a method of the class

The next line of the code is:





```
a = rc.get_laser(360)
```

Three new things here:

1. We are **calling the \*get_laser()\* method**, which is provided by the **RobotControl** class.
2. We are **providing a parameter to the method** (the number *360*).
3. Then, we are **storing** the output of the method **into the new variable \*a\***.

#### What is this *get_laser()* method?

- The method **get_laser (ray_number)**: this method  allows you to get data from the laser of the robot. When you call this  method, it will return the distance measured by the laser ray that you  specify as the parameter.

- The parameter **ray_number**: here is where you specify a number between 0 and 719, which will indicate the ray of the laser  reading you want to get the measured distance.

Right now, this may sound a bit confusing, so let's try to explain  it. As you can see in the Turtlebot robot in the simulation, it has a  laser mounted on its top plate.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/turtle_laser.png)

Now, as you may already know, this laser is projecting many beams  (720 to be precise) in a 2D plane parallel to the ground. Those rays are measuring distance to the closest obstacle that intercepts the ray.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/laser_beams.jpg)

Those beams are projected from the laser in all directions, covering a range of 180 degrees in front of the robot, more or less. Check out the image below:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/turtle_laser3.png)

This image represents the laser beams (although in reality there are  many more, we cannot draw the 720!!) being projected from the laser in a range of 180 degrees. Also, keep in mind the numbers that appear in  this image, since they will be very important in just a moment.

What the numbers in this image mean are the following:

- If we pass the number 0 to the **get_laser()** method, we will get the reading of the first laser beam at the right side of the robot.
- If we pass the number 360 to the **get_laser()** method, we will get the reading of the laser beam right at the front of the robot.
- If we pass the number 719 to the **get_laser()** method, we will get the reading of the last laser beam at the left side of the robot.
- And the same applies to all numbers in-between those.

So, by calling the **get_laser()** method with the  number 360 as argument, the method will return the reading of the laser  right in front of the robot. This means that we are measuring the  distance to the closest obstacle just in front of the robot. And what do we have right in front of the robot? Well, we have the wall. And... do  you remember what output did we get when we executed our script?

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/py3venv_pyscript1.png)

Well then, what this output means is that the laser beam right in  front of the robot (position 360) is detecting a wall at 2.5 meters.  Awesome, right?

#### What does it mean to call a method?

Before going to the final line of our script, let's introduce another concept: **call a method**. We are going to talk deeply about methods in Unit 4, but for now, in  order to be able to interact with the simulation, we need to introduce  the concept of calling a method. In order to call a method, you just  have to specify the name of the method, followed by the open and close  parentheses. Then, inside these parentheses, you will specify the  parameters that this method needs.





```
method_name(parameter1, parameter2, ...)
```

In our case, we only have 1 parameter, **ray_number**, so we call our method like this:





```
rc.get_laser(360)
```

Also note that before the name of the method, we are putting the **rc** object. This is because the **get_laser()** method is defined by this object. You will learn more about those  concepts in Unit 5, so don't worry too much about this right now, and  just keep in mind how to call a method. I can promise you that by the  end of the course, you will dominate all these concepts.

#### Getting the output of the method

When you call the *get_laser()* method, the method does its magic and computes the distance detected by the ray you specified. Then, **it returns that value** to the caller.

This means that calling the method provides a result that must be stored somewhere. That *somewhere* is the variable *a*.

But wait... **where does the variable \*a\* come from?**

Good question!

The variable *a* comes from nowhere. It is created automatically on the same line that we call the method.

But wait... **which type of variable is \*a\* ?**

Another good question!

The type of the variable is automatically assigned by the Python  system, and it matches the type of the value that is returned by the *get_laser()* method. This means, you don't actually need to know the type, as far as you can operate with it. Cool, isn't it?

#### Showing data on the screen

Finally, in the last line of our script, we have a **print()** method.





```
print ("The distance measured is: ", a)
```

We will talk about the *print()* method later in this chapter, so for now, just keep in mind that it's used in order to print something on the screen.

In this case, we are printing the output provided by the *get_laser()* method, that is, the distance measured by the ray.

As you can see, the script you have executed is very simple, but it  will allow us to introduce a couple of very important concepts that you  will need for all your Python scripts: **data types** and **variables**.

###         2.1.2                 Variables    

A variable can be seen as a container that stores some data: it can be a number, text, or more complex data types.

While our program is being executed, variables can be accessed or  even changed, which means that a new value will be assigned to the  variable.

In most programming languages, like C++ or Java, you need to **declare a variable** before being able to use it. But in Python, this is much easier. **There's no need to declare a variable** in Python. If you want to use a variable, you just think of a name and  start assigning a value to it! You can see this in the following snippet of code.





```
a = 5
b = 'This is a string'
c = ['This', 'is', 'a', 'list', 1, 2, 3]
```

As we've already said, you can also change a variable's value during  the program. For instance, we could do the following script:





```
a = 5

a = 6

print (a)
```

As a result of the above script, in the final *print*, we would get the value of the variable **a** as 6.

You can execute this Python script by clicking on it (select the  cell) and then clicking on the play button on the top right-hand corner  of the IPython notebook.



![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/font-awesome_step-forward.png)




 After executing, you should see the output right below the cell, like this:



​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/cell_output2.png)

​    \- End Expected Output -

But that's not all. Python also allows you to change the **type** of the variable during the program. For instance, you could do something like this:





```
a = 5

a = 'This is now a string'

print (a)
```

Here, when we print the variable a, we would get the following text as output: **This is now a string**.

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/cell_output1.png)

​    \- End Expected Output -

​    \- Exercise 2.2 -

a) Inside the **robot_control** folder, create a new Python script named **variables.py**. Inside this script, add the necessary code that does the following:

- First, you will call the method **get_laser()**, with any number, and store its response into a variable named **laser1**.
- Second, you will print this value, in order to see what you get.
- Then, you will call the **get_laser()** method again, with a different a number, and you will store its response in another variable named **laser2**
- Then again, you will print this value, in order to see what you get now.
- Finally, you will call the **get_laser()** method one last time, with a different number. Now, you will store its response in the same variable **laser2**.
- And you will print again the value of this **laser2** variable, to see what you get now.

​    \- End of Exercise 2.2 -

​    \- Solution for Exercise 2.2 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/TC-logo-blue.png)

Please try to do it by yourself unless you get stuck or need some  inspiration. You will learn much more if you fight for each exercise.

variables.py





```
from robot_control_class import RobotControl

robotcontrol = RobotControl()

laser1 = robotcontrol.get_laser(0)
print ("The laser value received is: ", laser1)

laser2 = robotcontrol.get_laser(360)
print ("The laser value received is: ", laser2)

laser2 = robotcontrol.get_laser(719)
print ("The laser value received is: ", laser2)
```

​    \- End Solution for Exercise 2.2 -

So, as you can see, it's very easy to work with variables in Python. During the notebook, we've already referred to *data types* a couple of times, so let's now get a look at them!

###         2.1.3                 Data Types    

Every value in Python has a data type. Basically, the data type of a  variable indicates what this variable contains: Is it a number? Is it  text?

There are various data types in Python. For this chapter, we are going to introduce just some of the most important ones.

- Numbers
- Strings
- Lists
- Tuples
- Dictionaries

#### Numbers

Inside the numbers data type, we can divide into different types of numbers. For now, let's just differentiate into **integers** and **floats**.





```
a = 5   # This is an integer

a = 0.5 # This is a float
```

#### Strings

Strings are basically sequences of characters. We can use single quotes (' ') or double quotes (" ") to represent strings:





```
s1 = 'This is a string'

s2 = "This is also a string"
```

As a sequence of characters, strings can be subscripted or indexed.  The first character of a string has the index 0. So, for instance, if we had the following code snippet:





```
s1 = 'This is a string'

s2 = "This is also a string"

print (s1[0])
print (s2[8])
```

The expression **s1[0]** would refer to the first character of the **s1** string, while the expression **s2[8]** would refer to the 9th character of the **s2** string. If you execute the previous script, then, you will get the following output:

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/strings_output.png)

​    \- End Expected Output -

Finally, you can also concatenate different strings. For instance, check out the following script:





```
s1 = 'This is'

s2 = " a string"

s3 = s1 + s2

print (s3)
```

So, if you execute the above script, you will get in **s3** the string that results from adding **s1** + **s2**.

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/string_concat.png)

​    \- End Expected Output -

#### Lists

A Python List is an ordered sequence of items, where all the items of the list do not need to be of the same type. To declare a Python List,  you just have to put all the items inside brackets [ ], and separate  each item by commas. Check out the following script:





```
l = [1, 2, 3, 'This', 'is', 'a', 'list']

print (l)
```

Similar to strings, we can also access each specific item of the  list, or even a range of items from the list. Check out the example  below:





```
l = [1, 2, 3, 'This', 'is', 'a', 'list']

print (l[2])
print (l[0:3])
print (l[3:])
```

- In the 1st print, we are getting the 3rd item from the list (remember that the 1st item is the 0 position).
- In the 2nd print, we are getting all the items between the 1st and the 3rd ones.
- In the 3rd print, we are getting all the items from the 4th one to the last one.

So, if you execute the script, you'll get something like this:

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/list_output.png)

​    \- End Expected Output -

Finally, another important thing is that lists can be updated. Check out the following example:





```
l = [1, 2, 3, 'This', 'is', 'a', 'list']
print (l)

l[2] = 8
print (l)
```

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/output_list2.png)

​    \- End Expected Output -

#### Tuples

Tuples consist of a series of values separated by commas, which are  enclosed within parentheses (). The particularity of tuples is that they cannot be updated, they are **read-only**. So basically,  they are the same as lists, the only difference being that the values  inside a tuple cannot be updated. Check out the following example:





```
t = (1, 2, 3, 'This', 'is', 'a', 'tuple')

print (t)
```

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/output_tuple.png)

​    \- End Expected Output -

#### Dictionaries

Dictionaries are also similar to lists, in the sense that they  contain a list of values and that they can be updated. But the main  difference is that items in dictionaries are accessed via keys and not  via their position. So basically, dictionaries are a list of items, with each item being a pair made up of a key and a value.

Check out the following example:





```
dict = {"Jon": 25, "Daenerys": 22, "Cersei": 31, "Night King": 35}

print (dict["Jon"])
print (dict["Night King"])
```

As you can see in the code above, dictionary values are accessed  through the key. So, if you execute the above code, you will get  something like this:

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/output_dict.png)

​    \- End Expected Output -

As with lists, dictionaries can also be updated.





```
dict = {"Jon": 25, "Daenerys": 22, "Cersei": 31, "Night King": 35}

dict["Jon"] = 10
print (dict["Jon"])
```

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/output_dict2.png)

​    \- End Expected Output -

Great! So with all these new data types introduced, let's get into a  couple of exercises in order to put them in practice! But, for the  following exercises, we are going to introduce a new method that you  will be able to call in your programs:

- **get_laser_full()**: As the name itself says, this  method will allow you to get all the data from ALL the laser beams of  the robot. As I've said before, this is a total of 719 different  readings. So, when called, this method will return a **LIST** containing all the 719 different readings from the laser beams.

​    \- Exercise 2.3 -

a) Create a new Python script named **lists.py**. Inside this script, add the necessary code that does the following:

- First, you will call the **get_laser_full()** method, and will store its response in a Python list.

- Then, you will print the positions 0, 360, and 719 from the full list of readings.

​    \- End of Exercise 2.3 -

​    \- Expected Output for Exercise 2.3 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/py3venv_lists.png)

​    \- End Expected Output -

​    \- Solution for Exercise 2.3 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/TC-logo-blue.png)

Please try to do it by yourself unless you get stuck or need some  inspiration. You will learn much more if you fight for each exercise.

lists.py





```
from robot_control_class import RobotControl

rc = RobotControl()

l = rc.get_laser_full()

print ("Position 0: ", l[0])
print ("Position 360: ", l[360])
print ("Position 719: ", l[719])
```

​    \- End Solution for Exercise 2.3 -

​    \- Exercise 2.4 -

a) Create a new Python script named **dictionaries.py**. Inside this script, add the necessary code that does the following:

- First, you will call the **get_laser_full()** method, and will store its response in a Python list.

- Then, you will create a dictionary that will contain the position in the list and its corresponding value as key-value pairs. Check the  example below:

  - Position 1: 5

  - Position 52: 32

  - Position 231: 0

  - Position 644: 21

    You will do this for the following positions in the list: 0, 100, 200, 300, 400, 500, 600, 719.

- Finally, you will print the resulting dictionary.

​    \- End of Exercise 2.4 -

​    \- Expected Output for Exercise 2.4 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/py3venv_dictionaries.png)

​    \- End Expected Output -

​    \- Solution for Exercise 2.4 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/TC-logo-blue.png)

Please try to do it by yourself unless you get stuck or need some  inspiration. You will learn much more if you fight for each exercise.

dictionaries.py





```
from robot_control_class import RobotControl

rc = RobotControl()

l = rc.get_laser_full()

dict = {"P0": l[0], "P100": l[100], "P200": l[200], "P300": l[300], "P400": l[400], "P500": l[500], "P600": l[600], "P719": l[719]}

print (dict)
```

​    \- End Solution for Exercise 2.4 -

##         2.2                 I/O methods    

###         2.2.1                 Print    

In this chapter, we have already used the **print**  method many times, so probably, at this point, you already know what  it's used for. Basically, the print method is used to write into the  standard output of the program. This is especially useful for  communicating with the user that is interacting with the program, to let him know what's going on; but it's also very useful when we want to  debug our own programs.

Check out the following example in order to see some uses for the print method:





```
a = 5

print ("Simple print")
print ("Now we print the variable a = " + str(a))
print ("Now we print the variable a = %d" % a)
print ("This is an example of a \n new line")
```

​    

```
Simple print
Now we print the variable a = 5
Now we print the variable a = 5
This is an example of a 
 new line
```

- The 1st print is a simple one, as you can see.
- The 2nd print combines a regular print with a variable. Note that we are converting the variable to a string value.
- The 3rd print also combines a regular print with a variable. Note  that in this case we don't need to convert the variable to a string  value, because we are using the % formatting.
- The 4th print shows an example of how to jump to a new line using the **/n** expression.

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/print_result_new.png)

​    \- End Expected Output -

###         2.2.2                 Input    

In the same way that you may want to write messages to the user who  is executing your program, you might also want to ask this user to  provide some data to your program. In fact, all programs need to  communicate with the environment or "outside world" (usually a user). In programming, this is usually known as I/O (input/output)  functionaliies.

As we have already seen, the output in this case will be the **print()** method, while the input will be performed by the **input()** method. Check out the following example:





```
name = input("What's your name? ")

print("Nice to meet you, " + name)
```

Inside the **robot_control** folder, create a new Python script named **input.py**, and copy into the file the code showed above. Let's now execute this file to see how it works.

​             Execute in Shell #1





```
python input.py
```

When executing the file, you will be asked to introduce your name, like this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/py3venv_input1.png)

Introduce your name and press the *Return* key. At the end, you should get something like this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/py3venv_input2.png)

Whenever the input method is called, the program will stop until the  user has given an input and has ended the input with the return key.

Now let's have a look at this other example:





```
age = input("What's your age? ")

age2 = age + 1

print("So next year you will be %d years old!" % age2)
```

There's not much of a difference with respect to the previous  example, right? So... what do you think? Will it work correctly? Let's  have a look!

Inside the **robot_control** folder, create another Python script named **input2.py**, and copy into the file the code showed above. Let's now execute this file to see if it works.

​             Execute in Shell #1





```
python input2.py
```

So, after introducing your age and pressing the *Return* key, you will get something like this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/input2_err1.png)

So it's actually failing! Interesting... but can you guess why? Well, in fact, there are a couple of problems in the code.

First of all, you need to know that Python 3 doesn't evaluate the data received with the **input** method and returns it as it is, as a string. This means that the **age** variable will contain an *string*. So, in the operation **age + 1**, we are actually trying to add a *string* and an *integer*. This operation is not possible, so we are getting an error. Then... what can we do?

Well, if we want to make sure that the content of the **age** variable is an *integer*, the easiest way is to explicitly convert it. For this we can use the **int()** method, like this:





```
age = int(input("What's your age? "))
```

The **int()** method converts whatever it has inside the parenthesis to an *integer*. Now, we make sure that the **age** variable will be an integer, thus we will be able to add it to another integer.

Update the code inside the **input2.py** file with the following code:





```
age = int(input("What's your age? "))

age2 = age + 1

print("So next year you will be %d years old!" % age2)
```

When it's updated, try to execute it again.

​             Execute in Shell #1





```
python input2.py
```

You will now get something like this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/input2_good.png)

Let's do the following exercise.

​    \- Exercise 2.5 -

a) Create a new Python script named **test_input.py**.

b) Inside the script, create a code that does the following:

- First, it asks the user to enter a number between 0 and 719.
- Second, it calls the **get_laser()** method, using the entered number as the parameter.
- Finally, it prints the response of the **get_laser()** method to the user.

​    \- End of Exercise 2.5 -

​    \- Solution for Exercise 2.5 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/TC-logo-blue.png)

Please try to do it by yourself unless you get stuck or need some  inspiration. You will learn much more if you fight for each exercise.

test_input.py





```
from robot_control_class import RobotControl

num = int(input("Select a number between 0 and 719: "))

rc = RobotControl()
a = rc.get_laser(num)

print ("The laser value received is: ", a)
```

​    \- End Solution for Exercise 2.5 -

##         2.3                 Operators    

In Python, operators are used to perform operations on variables and  values. We can divide the operators into the following basic groups:

- Arithmetic Operators
- Assignment Operators
- Comparison Operators
- Logical Operators

###         2.3.1                 Arithmetic Operators    

| Operator | Name           | Example   |
| -------- | -------------- | --------- |
| +        | Addition       | 1 + 1 = 2 |
| -        | Substraction   | 2 - 1 = 1 |
| *        | Multiplication | 2 * 2 = 4 |
| /        | Division       | 5 / 2 = 2 |
| %        | Modulus        | 5 % 2 = 1 |





```
a = 5
b = 2

print (a+b)
print (a-b)
print (a*b)
print (a/b)
print (a%b)
```

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/operators_output1.png)

​    \- End Expected Output -

###         2.3.2                 Assignment Operators    

| Operator | Example | Same As   |
| -------- | ------- | --------- |
| =        | x = 5   | x = 5     |
| +=       | x += 3  | x = x + 3 |
| -=       | x -= 3  | x = x - 3 |
| *=       | x *= 3  | x = x * 3 |
| /=       | x /= 3  | x = x / 3 |
| %=       | x %= 3  | x = x % 3 |





```
x = 5
x += 1
print (x)

x -= 2
print (x)

x *= 2
print (x)
```

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/operators_output2.png)

​    \- End Expected Output -

###         2.3.3                 Comparison Operators    

| Operator | Means                    | Same As |
| -------- | ------------------------ | ------- |
| ==       | Equal                    | 5 == 5  |
| !=       | Not Equal                | 4 != 5  |
| >        | Greater than             | 5 > 4   |
| <        | Less than                | 4 < 5   |
| >=       | Greater than or equal to | 5 >= 4  |
| <=       | Less than or equal to    | 4 <= 5  |





```
a = 5
b = 6

if (a == b):
    print ("a is equal to b")

if (a > b):
    print ("a is greater than b")
    
if (a < b):
    print ("a is less than b")
    
if (a >= b):
    print ("a is greater than or equal to b")
    
if (a <= b):
    print ("a is less than or equal to b")
```

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/operators_output3.png)

​    \- End Expected Output -

**NOTE:** Worried about the new **if** expression introduced? Don't worry, you are going to learn about it in the next chapter!

##         2.4                 Comments    

In programming, it is very common to add comments to your code.  Comments are, basically, the parts of the code that are never going to  be executed. They are very helpful for making your code much more  readable and understandable to third party users.

In Python, comments are added using the **#** symbol:





```
# This is a comment

print ("This is a demo to show how to use comments in Python")
```

##         2.5                 Conclusion    

During this second lesson, you have learned some of the most basic  (but also important) concepts related to Python, like variables and  different data types. Also, you have learned some basic syntax that you  will need to use in most of your Python programs. Now, with all these  concepts, you are ready to start creating more complex Python programs.  So... what are you waiting for?
------

#         Python for robotics    

------

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/python3_logo.png)

#         Unit 1                 Introduction to the Course    

​    \- Summary -

Estimated time to completion: **15 minutes**



This course is about learning Python. But not just *Python as a computer programming language*, but **Python as a tool for programming the robots of the future!**

This unit is an introduction to the **Python For Robotics** Course. You'll have a quick preview of the contents you are going to  cover during the course, and you will also do your first practice in  Python with the simulated robot.

So, in this course, we are going to show you the basic main concepts  that you need to know in order to start programming robots with Python. Upon completion of this course, you will be able to follow  the rest of the Robot Ignite Academy Courses (the ones based in Python)  without any problem.

​    \- End of Summary -

##         1.1                 Why do you need to learn Python?    

You need to learn Python **if you want to become a robotics developer**. Robotics developers are the programmers that create the software for  robots. They are the builders of the intelligence of the robots. That is the most difficult job in robotics!!

- **Python is the most popular programming language for robots**, together with C++. However, Python is winning every year more space in  the robotics sector. Especially in robotics research, Python is the  number one preferred language.

- **Many of the artificial intelligence libraries used for robotics are written in Python**: the [reinforcement learning algorithms of OpenAI](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/Course_Python0.html?AWSAccessKeyId=AKIAJLU2ZOTUFJRMDOAA&Signature=2n7md45MYcIgUWhnrpz4q1rD6Rs%3D&Expires=1663932847), the learning libraries of [Scikit-learn](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/Course_Python0.html?AWSAccessKeyId=AKIAJLU2ZOTUFJRMDOAA&Signature=2n7md45MYcIgUWhnrpz4q1rD6Rs%3D&Expires=1663932847), the [OpenCV libraries for image processing](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/Course_Python0.html?AWSAccessKeyId=AKIAJLU2ZOTUFJRMDOAA&Signature=2n7md45MYcIgUWhnrpz4q1rD6Rs%3D&Expires=1663932847), the [deep learning of Tensorflow](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/Course_Python0.html?AWSAccessKeyId=AKIAJLU2ZOTUFJRMDOAA&Signature=2n7md45MYcIgUWhnrpz4q1rD6Rs%3D&Expires=1663932847), and many others. 

- Finally, if you want to become a robotics developer, you will need  to learn ROS. Even if ROS can be programmed using Python or C++, **the fastest and easiest way to learn ROS is by using Python**. That is the path we recommend you take.

##         1.2                 Let's start practicing!    

With the proper introductions made, it is time to actually start.  And... as we always do in the Robot Ignite Academy, let's start with  practice!

Let's create a simple Python program to control the robotic arm of the simulation.

###         1.2.1                 Let's create your first Python program    

Python programs are **created inside text files** and **executed using a terminal**.

Let's now create your first real Python program to control the robot and execute using the terminal.

​    \- Demo 1.1 -

#### Create your first Python program

- Go to the IDE and select the *src* folder, inside the *catkin_ws*.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/catkinws_src.png)

- *Right-click* and then select *New File* (as you can see in the below image)

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/new_file.png)

- Write the name of the python program we are going to create: *arm_control.py*

**NOTE:** All Python programs must have an extension of the file as *.py*

- The *arm_control.py* file should have opened automatically in the IDE. You should see the file empty. If the file is not opened  automatically or you closed it, just *double-click* on the file shown in the navigation area of the IDE.

- Copy the code shown right below, and paste it into the *arm_control.py* file in the IDE.

**CONGRATULATIONS !** You have **created** your first Python program!

arm_control.py





```
from smart_grasping_sandbox.smart_grasper import SmartGrasper
from tf.transformations import quaternion_from_euler
from math import pi
import time

sgs = SmartGrasper()

sgs.pick()

sgs.reset_world()
```

#### Execute your first Python program

Now let's execute this program as a Python program is usually  executed. For that, we are going to use the terminals below the IDE (we  also call them *WebShells*).

​             Execute in Shell #1

Type the following commands just to make sure your Python program is there:





```
cd /home/user/catkin_ws/src
```





```
ls
```

You should see your Python program listed there. To execute the Python program, type:





```
python arm_control.py
```

By typing the previous command, you are executing the Python program  you created to control the robot arm. The robot arm must be moving and  grasping the red ball.

**CONGRATULATIONS!** You have **executed** your first Python program!!

**NOTE:** After executing the Python program, you will probably see the following error appear:





```
Failed to import pyassimp, see https://github.com/ros-planning/moveit/issues/86 for more info
```

This is a **KNOWN BUG** on ROS Kinetic and  Ubuntu 16.04. Just ignore it, since it won't affect in anything on this  demo, neither in the following course.

After executing the program, you should see that the robot arm in the simulation moves and picks up the red ball, something like this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/grasp_demo.gif)

As you can see, the code is written in Python and is used to control the robotic arm. **Do not worry if you don't understand the code now**. You will be able to understand it by the end of this course.

​    \- End of Demo 1.1 -

#### Important things from the previous exercise:

1. Every Python program you create must follow the name convention <your−program−name>

1. .py
2. In order to execute a Python program, you must type:





```
python <your_program_name>.py
```

1. Also, to execute the previous command, remember that you must be  located at the same directory that your Python program is located.

##         1.3                 What will you learn with this course?    

In the code above, you have applied many of the Python concepts that you are going to learn in this course:

- How to store data into variables
- How to operate with the data in the variables
- How to change behavior based on conditions
- How to create methods that can be called from other places of the code
- How to encapsulate the code into classes so you can have clean and robust code

##         1.4                 How will you learn all this?    

You will learn through hands-on experience from day one! In the Robot Ignite Academy, we strongly believe that the best way to learn is by  practicing, practicing, and then... practicing some more!

Since this is a Python Course, we are going to focus on the  programming language, leaving robotics aside. And during the course, you are going to be able to interact with the simulation environment as  well. Specifically, you are going to work with a Summit XL robot.

**Summit XL**:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/summit_xl.png)

Also, for the Course Project, you'll be using a Turtlebot robot.

**Turtlebot**:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/kobuki.jpg)

##         1.5                 Requirements for the course:    

## Requirements for the course:

In order to be able to fully understand the contents of this course,  you need to know how to use the Linux terminal (as you have done in *Demo 1.1*. If you do not understand the commands and language of *Demo 1.1,* then you need to do the following course before attemping to do this course:

- [Linux For Robotics](https://app.theconstructsim.com/#/Course/40).

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/linux_course.png)

Really, if you don't know how the Linux terminal works, you are going to be lost in the contents of the next chapters. So, go for it, if that is what you want. We'll be here waiting for you!

##         1.6                 Conclusion    

In this first, lesson you have learnt how to create a Python program  and how to execute it to control a robotic arm. Now, let's move to the  next lessons to really understand the Python syntax and how to build  complex programs for robots.

##         1.7                 Special Thanks    

This course would not have been possible without the work of the following people:

- The [Shadow Robot](http://www.shadowrobot.com) Company (especially Ugo Cupcic, creator of the grasping demo).

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/shadow-logo.jpg)

- [Robotnik](http://www.robotnik.es), creators of the Summit XL robot and simulation.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/logo_robotnik.jpg)

- And, of course, we have to thank the whole [ROS community](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/www.ros.org) for all the work done that allows us to build this course for you. Our course is standing on top of the shoulders of giants!

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/rosorg.jpg)
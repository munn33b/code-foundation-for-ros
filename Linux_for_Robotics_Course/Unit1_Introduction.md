------

#         Linux for robotics    

------

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/Linux-cover.png)

#         Unit 1                 Introduction to the Course    

​    \- Summary -

Estimated time to completion: **10 minutes**

 This unit is an introduction to the **Linux for Robotics**  Course. You'll have a quick preview of the contents you are going to  cover during the course, and you will also view a practical demo.

​    \- End of Summary -

##         1.1                 What's this course about?    

This course is about Linux. Linux is a free, open source operating  system, which comes, by default, with a set of utilities that will make  your life much easier.

##         1.2                 Why You Need To Learn Linux?    

### Robotics Development

Let me tell you one simple detail. Have you ever heard of ROS? The  Robotics Operating System that is the reason for the existence of this  beloved Academy? In fact, 99% of the courses that you will find here  will be about ROS. Well, at present, **ROS only fully supports Linux systems**.

### Linux is everywhere

Did you know that every time you use Google or Facebook or any other  major Internet site, you are communicating with servers running Linux?  Most DVRs, airplane and automobile entertainment systems, and even smart TVs run on Linux. Furthermore, if you are using an Android phone, you  are also using Linux. As a matter of fact, Linux is used in 498 of the  500 world's speediest supercomputers.

### Versatility

Linux is available under the GNU GPL license, which means it can be  freely used on almost any product or service you’re developing, as long  as the license terms are respected. Also, Linux development is  community-based. This means that you can work with other Linux  developers to share knowledge and learning.

### Security

Linux is one of the most secure operating systems around. From  devices/files to programs, access mechanisms, and secure messaging, you  name it.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/convincedyet.png)

Damn! You're a hard one to convince. Let's try with the ace up my sleeve.

Right below you can do a practical demo of one of the topics you are going to learn during the course. So... let's go!

​    \- Demo 1.1 -

a) In the following demo, you are going to execute a **bash script**. Bash scripts are a very useful and common tool used by Linux  developers. Basically, it is the tool used for creating programs for  Linux. So... let's go for it!

In order to complete this demo, you will need to execute some commands on the **Web Shell**. The Web Shell is just a regular **Linux Shell**, which runs on a web browser (since this course is based on a web browser). But now... what is a Linux Shell?

Well, we can say that the Shell is the tool that **allows you to communicate with the Linux system** in order to tell it what to do (by sending commands). On the course screen, you will find the Web Shells (see image below).

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course60.png)

You will find 4 available Web Shells, and you can switch between them by clicking on the different tabs (although, for this course, you  should only need one).

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/prompt.png)

This is known as the **prompt** of the Shell. It contains some basic information like the current user (in this case its **user**) or the current path you are on (you will learn more about this during  the course). You will also see that it has a white block flickering.  This means that the Shell is active and waiting to receive your orders!

​             Execute in Shell #1





```
cd /home/simulations/public_sim_ws/src/all/ros_basics_examples/linux_demo/
```

The above command will send you to the folder where the bash script  is located. If you've done it right, you will see how your prompt gets  updated with the following path:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/prompt_demo.png)

Now, let's execute the bash script! But before anything, let's  quickly explain how it works. In this case, the bash script you are  going to execute is expecting an argument. The script, then, will  evaluate the argument received and do one thing or another. For this  script, you can send 3 different arguments:

- **forward**: This will move the BB8 robot forward.

- **rotate**: This will make the BB8 robot rotate on its own axis.

- **stop**: This will make the BB8 robot stop any movement.

So, basically, you can execute the bash script in the following ways.

​             Execute in Shell #1





```
./demo.sh rotate
```

Once you execute the above command, you will get the following output on the Shell:

​             Shell #1 Output





```
user:/home/simulations/public_sim_ws/src/all/ros_basics_examples/linux_demo$ ./demo.sh forward
publishing and latching message. Press ctrl-C to terminate
```

Also, you will see how the BB-8 robot starts rotating:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/bb8_rotate.gif)

Great!! So, now we have our bash script running, which is making the  BB8 robot rotate. But now... how do you stop it? Well, it's very simple. In fact, you have a hint in the Web Shell output. You just have to  click **Ctrl + C** simultaneously on your keyboard. But... and this is **VERY IMPORTANT!!!** In order to be able to send the **Ctrl + C** signal to stop the bash script, you need to have the focus on the Web Shell where the script is running.

Once you stop the program, you will be able to type again in the Web  Shell. Anyways, you may have noticed that even though you have stopped  the script, the robot keeps rotating. This is because the robot, in this case, will keep doing the last order received (which was to rotate).  So, in order to stop its movement, you will have to execute the script  again, using a different argument this time.

​             Execute in Shell #1





```
./demo.sh stop
```

So, after executing the above command, the robot will stop rotating.  Excellent! Now, let's try to run the script one more time using the **forward** argument. Remember that before being able to execute commands on the  Shell again, you will need to first stop the running script with **Ctrl + C**.

​             Execute in Shell #1





```
./demo.sh forward
```

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/bb8_forward.gif)

So, now you will see how the BB8 robot starts moving forward. And  remember, it will keep doing so until you tell it to stop. Execute the  bash script using the different arguments and see how the BB8 robot  behaves depending on the argument sent.

And remember, if you want to learn more about bash scripts, like how  to create your own, you will find everything inside the course (along  with many other interesting topics).

​    \- End of Demo 1.1 -

##         1.3                 What will you learn with this course?    

Basically, during this course, you will address the following topics:

- How to navigate through a Linux filesystem
- How to interact with a Linux filesystem
- How to edit files using the Shell (vi editor) 
- Manage access to files (Permissions)
- Create simple Linux programs (Bash Scripts)
- Manage execution of Linux programs (Processes)
- How to connect to the remote computer of a robot (ssh)

##         1.4                 How will you learn all this?    

You will learn through hands-on experience from day one! In the Robot Ignite Academy, we strongly believe that the best way to learn is by  practicing, practicing, and then... practicing some more!

Since this is a Linux Course, we are going to focus on the Linux  utilities, leaving ROS aside. But during the course, you are going to  still be able to interact with the simulation environment as well.  Specifically, you are going to work with a BB8 robot. Isn't that cool?

**BB-8**:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/bb8.jpg)

##         1.5                 Minimum requirements for the course    

**None!!** You can start this Course even if you don't have any kind of background or previous knowledge.
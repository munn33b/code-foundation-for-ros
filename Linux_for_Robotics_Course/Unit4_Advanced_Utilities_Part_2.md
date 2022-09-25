Loading [MathJax]/jax/output/CommonHTML/fonts/TeX/fontdata.js

------

#         Linux for robotics    

------

#         Unit 4                 Advanced Utilities Part 2    

​    \- Summary -

Estimated time to completion: **3 hours**

In this unit, you are going to learn some advanced utilities that will allow you to interact deeply with a Linux system.

​    \- End of Summary -

##         4.1                 Linux Processes    

During the entire course, you have been executing different programs, most of them in order to play with the BB8 robot. But... did you know  that each time you did so, you were actually starting a **process** on the Linux machine?

A process refers to a program in execution. Basically, it’s a running instance of a program. It is made up of the program instructions, data  read from files, other programs or input from a system user.

Basically, there are 2 types of processes in Linux:

- **Foreground processes** : These are initialized and  controlled through a terminal session. In other words, there has to be a user connected to the system to start such processes; they haven’t  started automatically as part of the system functions/services.

- **Background processes**: These are processes not connected to a terminal. This means that they don’t expect any user input.

###         4.1.1                 Visualize processes    

There exist several different commands that will allow you to  visualize the running processes on the system. In this course, though,  we are going to focus on 2 of them: **htop** and **ps**.

​             Execute in Shell #1





```
htop
```

You will see something similar to this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course29.png)

The **htop** command is an improved version of **top** (which is the classical command to visualize processes). Usually, it’s not installed by default on most Linux distributions.

We can see several pieces of useful data here. The whole window above can be split up into three sections for the ease of our understanding.  The top-left section comprises the CPU and memory usage information. The top-right section provides info about load average and uptime. The rest of the information contains real-time data of processes with stats like priority, CPU and memory consumption, etc.

However, this tool is a bit complex, especially for newcomers to  Linux. For less than expert users, it's usually best to just use the **ps** command.

​             Execute in Shell #1





```
ps faux
```

You will see something similar to this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course30.png)

At first glance, there are also many processes here. But remember, we have the **grep** tool for that! However, all the processes you are seeing now (like the  ones related to Gazebo) have been automatically launched by the Academy  and you won't be able to interact much with them. So... why don't we  start our own process? Let's go for it!

​    \- Exercise 4.1 -

Inside the **my_scripts** folder, create a new file named **test_process.py**. Then, inside this file, paste the following code:

test_process.py





```
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class MoveBB8():

    def __init__(self):
        self.bb8_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuous publishing systems, this is no big deal, but in systems that publish only
        once, it IS very important.
        """
        while not self.ctrl_c:
            connections = self.bb8_vel_publisher.get_num_connections()
            if connections > 0:
                self.bb8_vel_publisher.publish(self.cmd)
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.stop_bb8()
        self.ctrl_c = True

    def stop_bb8(self):
        rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def move_bb8(self, linear_speed=0.2):

        self.cmd.linear.x = -linear_speed
        self.cmd.angular.z = 0

        while not self.ctrl_c:
            self.publish_once_in_cmd_vel()
            rospy.loginfo("Moving BB-8 forward!!")
            self.rate.sleep()

        self.stop_bb8()

if __name__ == '__main__':
    rospy.init_node('process_test', anonymous=True)
    movebb8_object = MoveBB8()
    try:
        movebb8_object.move_bb8()
    except rospy.ROSInterruptException:
        pass
```

test_process.py

The program is very simple: it will keep moving the BB8 robot forward until the program is terminated, and that is when it will stop the  robot's movement.

Now, execute the program using the following command:

​             Execute in Shell #1





```
rosrun move_bb8_pkg test_process.py
```

You will see the BB8 robot start moving forward.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/bb8_forward.gif)

Now, on a different Shell, type the following command:

​             Execute in Shell #1





```
ps faux | grep test_process
```

You should get something like this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course32.png)

And here we can see our process running!





```
user     15981 10.8  0.6 577536 23488 pts/0    Rl+  11:40   0:01      \_ python /home/user/catkin_ws/src/linux_course_files/move_bb8_pkg/my_scripts/test_process.py
```

The first result, which will look something like this...





```
user     18416  0.0  0.0  11288  1012 pts/1    S+   11:40   0:00  |   \_ grep --color=auto test_process
```

...is associated with the **grep** program itself, and it's not related at all to the **test_process.py** program we have just executed.

​    \- End of Exercise 4.1 -

###         4.1.2                 Kill processes    

At this pont, you should have something like this on your Shell:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/process_shell1.png)

So, the program you launched in the previous exercise is still  running, and so is the process associated with it. In this case, the  process is a **foreground process**, since it is directly connected to the current Shell session.

So... BB8 is probably starting to get too far away and it might even  be a little bit tired... so what about stopping it? But now, how do you  terminate this process?

I bet most of you might have already thought about using **Ctrl + C** to stop the program. And you are right! Basically, you have 2 ways to terminate this process: using **Ctrl + C** or **Ctrl + Z**.

###         4.1.3                 Ctrl + C    

*Ctrl + C* is used to kill a process with the signal **SIGINT**, and can be intercepted by a program (in our case, it's **test_process.py**) so that it can clean itself up before exiting (in our case, stop the  robot), or not exit at all. It depends on how the application is built.

So, on the Shell where you have the *Hello there!* stream, click **Ctrl + C** on your keyboard. You will get something similar to the image below, and you will be able to type again on the Shell.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/process_shell2.png)

Let's see what happened with the process...

​             Execute in Shell #1





```
ps faux | grep test_process
```

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course55.png)

All right!! So as you can see, the process has also been terminated. Great!

**NOTE:** When you filter the processes list using the **grep** tool, you will always get at least 1 result, as in  this case we have just seen. This process is associated with the  **grep** program itself, and it's not related at all to the  **test_process.py** program.

###         4.1.4                 Ctrl + Z    

*Ctrl + Z* is used for suspending a process by sending it the signal **SIGSTOP**, which cannot be intercepted by the program. Basically, it sends **SIGTSTOP** to a **foreground application**, effectively putting it in the background, suspended. But then... does  this means that the process will still be there? We'll check it in a  moment!

So, let's first start our application again.

​             Execute in Shell #1





```
rosrun move_bb8_pkg test_process.py
```

Now, on the Shell where you have the *Hello there!* stream, click **Ctrl + Z** on your keyboard. You will get something similar to the image below, and you will be able to type again on the Shell.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/ctrlz_process.png)

As you can see, you can also control the Shell again, but the output  you get is a bit different from before, isn't it? Let's see what  happened to the process.

​             Execute in Shell #1





```
ps faux | grep test_process
```

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course59.png)

OK!! So as you can see, in this case, the process is still there, running in the background. But how can I stop the process?

###         4.1.5                 Command "kill"    

In the previous section, we sent a foreground process to the background, using the command *Ctrl+Z*. But this opened up a new question: how can I stop a process that is  running in the background? Well, for this case, Linux provides the **kill** command. It's very easy to use, but you need to know the Process ID (**PID**) of the process you want to terminate. Let's see how it works!

So first, let's find out the PID of the process we want to kill.

​             Execute in Shell #1





```
ps faux | grep test_process
```

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course59.png)

The PID of the process is the number that appears in the second  column. In the case of our process, it is 26244, but in your case, it  will probably be different. So now, in order to kill, you just have to  execute the following command:

​             Execute in Shell #1





```
kill PID
```

Now, let's check what happened to the process again.

​             Execute in Shell #1





```
ps faux | grep test_process
```

So... what happened? The process is still there, right? Can you guess why?

Well, the problem here is that when you use **Ctrl + Z**, besides sending the process to the background, you are also suspending  it (sending a SIGSTOP signal). So, any signal we send now to the process (for instance, to kill it) will be ignored by the process. So... what  can we do?

For this, the **bg** command comes in handy. The **bg** command is used in order to resume the execution of a suspended  background process. Let's try it! So, on the same Web Shell where you  stopped the process using *Ctrl + Z*, now execute the following command:

​             Execute in Shell #1





```
bg
```

Now, you should see something like this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/bg_process.png)

So, now that the process execution has been resumed, the **kill** signal we sent before has been properly received, so the BB8 robot will stop moving and the process will be killed.

​             Execute in Shell #1





```
ps faux | grep test_process
```

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course55.png)

###         4.1.5                 Starting a process on the background    

Now, in a previous section, we sent a **foreground** process to the **background** using the *Ctrl + Z* command. But it is also possible to directly start a process in the background. For instance, try the following command.

​             Execute in Shell #1





```
rosrun move_bb8_pkg test_process.py &
```

You will see something like this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/bg_process3.png)

First, you will get the PID assigned to the process:





```
[1] 9779
```

And then, the output of the application will start. But let's try one thing: try to kill the process with either *Ctrl+C* or *Ctrl+Z*. Are you able to do it? I bet you can't!!

This happens because the process has actually been started in the **background**, so you can't use either *Ctrl+C* or *Ctrl+Z*, which are commands for processes that are running in the **foreground**. For a background process, you will have to use the **kill** command.

Since we already know the PID assigned to our process, we can directly execute the command in order to kill it.

**NOTE:** For this case, since the Web Shell where you execute the program will keep printing its output, it will be easier to execute other commands in another one (ie. Web Shell #2).

​             Execute in Shell #2





```
kill PID
```

Once you execute the kill command, you will see the following on the Web Shell where you executed the program.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/bg_process2.png)

Also, you can check that the process has been correctly killed.

​             Execute in Shell #1





```
ps faux | grep test_process
```

##         4.2                 ssh protocol    

**Secure Shell**, most commonly known as **ssh**, is a protocol that allows users to connect to a remote machine in a  secure way. It is based on a Client-Server architecture. So, from your  local machine (Client), you can log into the remote machine (Server) in  order to transfer files between the two machines, execute commands on  the remote machine, etc...

In robotics, it is mostly used to access the remote machine that runs in a real (physical) robot from any computer in order to control it and send commands to it. Unfortunately, in this course, there is no way to  test this with a real robot. But don't worry!! We are going to explain  the basic steps to establish the connection anyway. In the end, it will  almost be the same process.

So, let's imagine the following scenario.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/laptop1.jpg) ![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/bb8_realrobot.png)

We have our personal laptop (at the left) and a real BB8 robot. The  BB8 robot has a Linux computer inside. The computer inside BB8 has an IP (let's say, 127.0.0.1), and it has a SSH Server running on it. So now,  what would we do in order to connect to it so that we can control and  move the BB8 robot?

All right, all right, let me provide some extra information in order to help. The ssh command structure is like the following:





```
ssh <user>@<host>
```

**<host>** makes reference to the remote machine you want to access (where the SSH Server is running), and **<user>** makes reference to the account in which you want to login from the remote machine.

As a hint, I will tell you that there's an account named **student** in the BB8 computer. So... any ideas? Let's try the following:

​             Execute in Shell #1





```
ssh student@127.0.0.1
```

So... what's going on? Any error there? In fact, you should get the following error message.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/ssh_refused.png)

But what the heck does this mean? Well, as the message states, the  host (BB8 computer) has refused our connection for some reason. So,  let's try to get some information. At this point, what we know for sure  is that in the host there must be an SSH Server running, so that we can  connect to it. Right?

Hey!!! Wait a moment! That's something! If there's an SSH Server  running, there must also be a process associated with it, right? Let's try to see if we find something!

​             Execute in Shell #1





```
ps faux | grep ssh
```

You should get something similar to the following:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/ssh_process.png)

There it is!! We can see that there is an **ssh** process, started by the *root* user.





```
root        11  0.0  0.0  51492  3784 ?        S    15:09   0:00  \_ sudo /usr/sbin/sshd -p 8090 -D
```

Now, let's focus on the command associated with the process:





```
sudo /usr/sbin/sshd -p 8090 -D
```

And... what does this mean? Well, let me help you with this one.  Basically, this is the command executed by the host in order to start  the SSH Server. So, we can actually confirm that the SSH Server is  running. But also, it has one particularity. The SSH Server has been  launched with the **-p** argument. This argument is used in order to specify the port in which the SSH Server will run. And we can  see that the port specified is **8090**.

Now, if we review the error message that we got before...

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/ssh_refused.png)

...we can see that our SSH Client was trying to connect to the SSH  Sever using the port 22. Hey!! Could this be the error? Let's try the  following command:

​             Execute in Shell #1





```
ssh student@127.0.0.1 -p 8090
```

As you can see, we can also specify the port of the Server on the **ssh** command!

Right after executing the above command, you will be asked for the password. In this case, the password is **student**. So, introduce the password and click *Enter* on the keyboard. If you have properly written the password, you will see something like this in your Web Shell.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/ssh_login.png)

**NOTE**: You will see that the warning  message *LC_ALL: cannot change locale (en_US.UTF-8)*. Do not pay  attention to it, since it is not important right now.

And there we are!! We have finally connected to the BB8 remote  machine! At this point, we would be able to see all the files and  folders, execute commands, etc...

Unfortunately, as we already said at the beginning of the section, in this example, we don't have the real robot and the host is empty (in  fact, the host is the same machine as the client). But keep in mind the  general idea of the whole process, since it would be almost the same for connecting to a real robot.

Finally, in order to close an ssh session and go back to your local machine, all you have to do is to execute the **exit** command on the Web Shell.

​             Execute in Shell #1





```
exit
```

Afer executing this command, you will go back to your normal user.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/exit_ssh.png)

**NOTE:** You will need to exit your ssh  session before continuing with the following section of the chapter, so  make sure you have done so.

##         4.3                 Commands "apt" and "sudo"    

Linux uses a **dpkg** packaging system. A packaging  system is a way to provide programs and applications for installation.  This way, you don’t have to build a program from the source code.

**APT** (Advanced Package Tool) is the command line tool to interact with this packaging system.You can use it to find and  install new packages, upgrade packages, clean packages, etc.

apt-get basically works on a database of available packages. If you  don’t update this database, the system won’t know if there are newer  packages available or not. In fact, this is the first command you need  to run in any Linux system after a fresh install. So... let's update our database!

​             Execute in Shell #1





```
apt-get update
```

What's wrong? Is it not working? Maybe you are getting an error like this one?

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course57.png)

Don't worry, this is totally normal. Basically, you are getting this  error because you don't have permissions for access the packages  database. So... what do we do now? Let's try with the following command:

​             Execute in Shell #1





```
sudo apt-get update
```

Ok!! Now it has worked! But why? What is this mysterious **sudo** command that has arrived to save the day?

**Sudo** is a utility of Linux systems (and other Unix  based systems) that allows users to execute a program or a command with  the privileges (permissions) of another user (usually the **root** user). This this way, it allows you to become, temporarily, a **superuser**. So... it's basically like becoming a superhero for a little while!

So, now that we are available to use **apt-get**, and we have already updated our packages database, let's try to install a new  package! And, since this will be the last section of the course, what  about having some fun?

​             Execute in Shell #1





```
sudo apt-get install ltris
```

After executing the command, you will get something like this in your Web Shell:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/tetris6.png)

As you can see, since it is going to install many new packages into  the system, it is asking you to confirm the operation. If you want to go ahead, just click ***Enter\*** on your keyboard. If you would like to abort the operation, you would have to enter first the ***n\*** character, and then ***Enter\***.

Confirm the operation and wait a couple of minutes until all the  packages are installed in the system. Or you can go grab a coffee  meanwhile! When the installation process has totally finished, execute  the following command:

​             Execute in Shell #1





```
/usr/games/ltris
```

You should get something like this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/tetris1.png)

**NOTE**: The 1st time you try to run you might run into the following error:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_tetris_err.png)

If this is your case, just hit **Ctrl+C** and run the command again. 

Now, in order to visualize the game, open the Graphical Tools window by hitting this icon at the top-right corner of the IDE.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/font-awesome_desktop.png)

In the Graphical Tools window, you will then be able to visualize the Tetris game!

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/tetris2.png)

Here, click on the **New Game** button.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/tetris7.png)

And modify the Game mode to set it to **Vs CPU**.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/tetris8.png)

Finally, click on the **Start Game** button...

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/tetris3.png)

...and try to beat the CPU!! You will be able to play by using the **arrow keys** on your keyboard. Good luck!

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/tetris_crop.png)
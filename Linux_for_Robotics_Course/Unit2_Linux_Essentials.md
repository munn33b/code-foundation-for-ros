Loading [MathJax]/extensions/Safe.js

------

#         Linux for robotics    

------

#         Unit 2                 Linux Essentials    

​    \- Summary -

Estimated time to completion: **3 hours**

In this unit, you are going to learn some of the basic and most  important commands and tools you need to know in order to be able to  work and interact with Linux-based systems.

​    \- End of Summary -

In a regular Linux Course, we would start now probably by explaining  some basic Linux commands, how to navigate through the filesystem,  etc... and showing some basic examples of how to do it. But... as you  may already know, this is not a regular Linux Course! In this course, we want to teach you all the Linux basic concepts, of course, but with a  focus on robotics developers. That's why we have called this course  Linux for Robotics!

Anyways, we want to keep the focus of the course on Linux. So, in  order to be able to interact with the simulation while keeping all the  ROS stuff under the hood (so that we can focus only on Linux), we have  created a git repository that you will be using (and modifying) during  the whole course.

So, with the proper introductions made, let's now go to work! First  of all, as I've already said before, you will download the repository of the course to your workspace. For that, execute the following commands.

​             Execute in Shell #1





```
cd /home/user/catkin_ws/src/
```





```
git clone https://bitbucket.org/theconstructcore/linux_course_files.git
```

After this, you should see the following folder appear in your workspace.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course44.png)

Now, let's start doing some real work with Linux! And we will start  with the most basic concept in Linux, which is how to navigate through  the files and folders.

##         2.1                 Navigating through the filesystem    

One of the most important "skills" to learn in order to work in a  Linux system is to be able to move around all the folders and files that your system has. Summarizing a lot, we can say that Linux systems are  composed of 2 main elements: **folders** and **files**. The basic difference between the two is that files store data, while  folders store files and other folders. The folders, often referred to as directories, are used to organize files on your computer. The folders  themselves take up virtually no space on the hard drive.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/bored.jpg)

Ok, ok... I'm sure this is very cool and  interesting, but I'm starting to get a bit bored... I just want to play  with the BB8 robot!

All right, all right! Let's start controlling the robot! But let me  make it a little bit more interesting. What if I tell that, in your  current filesystem, you already have a file that will allow you to  control the BB8 robot using your keyboard? The name of the file is **bb8_keyboard.py**, but I won't tell you where that file is. Would you be able to find it by yourself? What commands would you use to find it?

###         2.1.1                 Command "cd"    

So, as we already said before, a filesystem is composed of folders  and files. Have a look at the following picture to get a better idea:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/filesystem2.png)

So, now let's imagine that we want to get to the file that is marked  with a red cross. Let's imagine that this is the file that will allow us to control the BB8 robot! As you can see, you would have to enter  through many different folders in order to arrive there. So... how would you do that? Of course, using the **cd** command!

The **cd** command is one of the most important ones in  Linux. It allows you to go into a specific directory. For instance,  let's execute the following command:

​             Execute in Shell #1





```
cd /home/user/catkin_ws/src/linux_course_files/move_bb8_pkg/src/
```

In the example above, the command you executed has taken you directly to the folder named **src**, passing though many different folders on its way (**user**, **catkin_ws**, **src**, **linux_course_files**, **move_bb8_pkg**...).

But... why did we go to this folder specifically? Does it have  anything special? Of course it does!! This folder contains the script  that will allow you to move the BB8 robot using the keyboard! So, let's  go for it!

​    \- Demo 2.1 -

So now, in order to launch the program, execute the following commands:

​             Execute in Shell #1





```
python bb8_keyboard.py
```

​             Shell #1 Output





```
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >
t : up (+z)
b : down (-z)
anything else : stop
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit

currently:      speed 0.5       turn 1.0
```

Now, you will be able to move the BB-8 robot around using your keyboard.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/keys_move_turtle.png)

​    \- End of Demo 2.1 -

Great! So after having some fun, how about we continue learning some new and interesting stuff?

For instance, did you know that you can check the current path you are on using the command **pwd**? Let's try it.

​             Execute in Shell #1





```
pwd
```

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course45.png)

Also, as you may have already noticed, the current directory where you are at has appeared in blue. In this case, it's **linux_course_files**.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course46.png)

In the previous example, we went directly to a path. This is because  we already knew where we wanted to go, so we could use a shortcut,  moving directly to the folder we wanted. But this is not always the  case. Many times, you will have to move folder by folder, checking what  each folder contains. Sometimes, you will even need to go back to the  previous folder. All of this can be achieved using the **cd** command.

For instance, imagine that we now need to move back to the **move_bb8_pkg** folder. How would you do that? Any idea? Maybe you are thinking about using a command like this one:





```
cd /home/user/catkin_ws/src/linux_course_files/move_bb8_pkg/
```

Well, that is actually correct. At least, it will do the job. But let me tell you that there's a much easier way to achieve the same. Let's  try the following:

​             Execute in Shell #1





```
cd ../
```

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course47.png)

And now, let's move back to the **src** folder.

​             Execute in Shell #1





```
cd src
```

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course48.png)

As you can see, the current directory where you are at, which appears in blue, gets updated each time you move to a different folder.

### HINT

At the beginning of the section, you used the following command:





```
cd /home/user/catkin_ws/src/linux_course_files/move_bb8_pkg/src/
```

Note the **/home/user/** part that appears at the beginning of the path. This is commonly known as the **HOME** folder of the user (in this case, the user **user**). The Home folder can be abbreviated using the character **~**. So, in this case, the above command can be substituted with the following one:





```
cd ~/catkin_ws/src/linux_course_files/move_bb8_pkg/src/
```

You can also see this **~** symbol on the Shell prompt.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course6.png)

Great! So this is very nice, but... what if I don't know exactly  where I want to move? What if I need to check which folders are  available? Or even better, what if I tell you that there's another  script that you can execute in order to move the BB8 robot, which is  located in the same folder as **bb8_keyboard.py**? Would you be able to find out its name?

###         2.1.2                 Command "ls"    

The command **ls** is used for seeing all the contents of a folder or directory. For instance, files or other folders.

Let's move the folder that contains the **bb8_keyboard.py** script, and execute the **ls** command in order to see what we have there:

​             Execute in Shell #1





```
ls
```

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/forward_backward_file.png)

And voilà! We can see all the files! As you can see, there are other files besides **bb8_keyboard.py**. They are named **move_bb8_circle.py** and **move_bb8_forward_backward.py**. Let's try to execute one of them.

​    \- Demo 2.2 -

Execute the file **move_bb8_circle.py** using the following command:

​             Execute in Shell #1





```
python move_bb8_circle.py
```

Now, you will see the robot moving in a square-like movement.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/bb8_linuxmove.gif)

​    \- End of Demo 2.2 -

Anyways, the command *ls* is a bit limited. For instance, it doesn't allow you to visualize **hidden files**. Let's go back to the **home** folder.

​             Execute in Shell #1





```
cd /home/user
```

Now, let's execute the *ls* command here.

​             Execute in Shell #1





```
ls
```

You should get something like this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course38.png)

Now, let's type the following command:

​             Execute in Shell #1





```
ls -la
```

You should now get the following:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course39.png)

As you can see, many new files and folders have appeared. They all  have in common 1 thing, which is that their names start with a dot **"."**. This means that they are hidden (which means that they are not visible through the regular **ls** command, or through a default GUI).

### AMPLIFICATION

A **hidden folder** or **hidden file** is a folder or file that filesystem utilities (like **ls**) do not display by default when showing a directory listing. They are  commonly used for configuration purposes, and are frequently created  automatically by other utilities (which means that they are not directly created by the user).

In Linux systems, as you have already seen, any folder or file that starts with a dot (.), for instance **/home/user/.bashrc** (keep an eye on this file, since we will learn more about it later on in the course), will be treated as hidden.

The **ls** command, though, has many more options.  Fortunately for us, Linux also provides a couple of commands that can  help us learn more about other Linux commands.

###         2.1.3                 Getting info about commands    

Type the following command in your WebShell.

​             Execute in Shell #1





```
ls --help
```

You should now get something similar to this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course5.png)

Here you can see all the options available for the **ls** command (including the **-a** option that you used in the previous section).

Alternatively, you can also use the **man** command.

​             Execute in Shell #1





```
man ls
```

It's very similar to the previous option, but with a bit more of an extension.

- To *exit* the **man** command, you have to hit the key **q** .

- To *scroll* through the pages, hit the **DOWN or UP ARROW Keys**.

- To *scroll* through the pages, hit the **spacebar**.

Obviously, you can use both options with any Linux command that you want.

##         2.2                 Let's rock BB8!    

​    \- Exercise 2.1 -

Using all the new concepts introduced, find the Python file named **move_bb8_forward_backward.py** in your workspace. Once you find it, execute it using the following command:

​             Execute in Shell #1





```
python move_bb8_forward_backward.py
```

You should now see how BB8 starts moving forward, then stops after a few seconds, and starts moving backwards.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/bb8_forward_backward.gif)

​    \- End of Exercise 2.1 -

​    \- Solution for Exercise 2.1 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/TC-logo-blue.png)

Please try to do it by yourself unless you get stuck or need some  inspiration. You will learn much more if you fight for each exercise.

​             Execute in Shell #1





```
cd ~/catkin_ws/src/linux_course_files/move_bb8_pkg/src/
ls move_bb8_forward_backward.py
```

Now execute the script:

​             Execute in Shell #1





```
python move_bb8_forward_backward.py
```

​    \- End Solution for Exercise 2.1 -

##         2.3                 Interacting with the filesystem    

So, up until this point, you've been learning about some very  important tools in order to be able to navigate around any Linux-based  machine. In the following section, though, you are going to start  learning about some tools that will allow you to actually interact with  the system, which basically means that you will be able to modify it. Or even better, create your own files in order to play with BB8!!

###         2.3.1                 Command "mkdir"    

The **mkdir** command is another essential tool in  Linux. It allows you to create a new directory. For instance, try  executing the following commands:

​             Execute in Shell #1





```
cd /home/user
```

As you already know, this will take you to your HOME folder.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course6.png)

​             Execute in Shell #1





```
ls
```

Now, using the **ls** command, you will visualize the contents of this folder.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course38.png)

Finally, let's execute the **mkdir** command.

​             Execute in Shell #1





```
mkdir my_folder
```

Now, if you execute the **ls** command again, you will be able to visualize your new folder.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course40.png)

###         2.3.2                 Command "touch"    

On the other hand, you can also create new files. There are several  ways in which you can create a new file in Linux. I would say that the  most commonly used is with the **touch** command. Go inside the folder you created in the previous section, and create a new file named **my_file.txt**.

​             Execute in Shell #1





```
cd ~/my_folder
```





```
touch my_file.txt
```

Now, you will be able to visualize our new file using the **ls** command.

###         2.3.3                 "vi" visual editor    

Great! So we have created a new file, but... it's empty! How can we  fill it? For this purpose, Linux provides many different tools. But for  this course, we are going to focus on **vi**, which is the default tool for Linux systems.

Open the file you created in the previous section using the **vi** command:

​             Execute in Shell #1





```
vi my_file.txt
```

You will now see something like this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course7.png)

Basically, **vi** has 2 different modes: **Command** and **Insert** modes:

- Command Mode: This mode allows you to use commands in order to  interact with the file. For instance, go to a certain line of the file,  delete certain lines, etc...
- Insert Mode: This mode allows you to insert text into the file.

By default, vi opens with the **command** mode activated. In order to switch to the **insert** mode, you will have to type the character **i**.

So, go to the Shell where you have **vi** opened, and type the character **i** in order to turn on the **insert** mode. You will see the following appear at the bottom-left corner of the vi editor.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course8.png)

This means that you are now on the **insert** mode. So,  any character that you type now will be written into the file. Now, type any phrase or word you want into the file, and save it.

Hey!!! Don't go so fast! You haven't explained yet how to save a  file! All right, all right... I just got too excited. So, at this point, you are still in the **insert** mode. In order to save the file, you need to go back to the **command** mode. And how do you do this? Very simple, just by clicking on the **esc** key of your keyboard. Once you do so, you will see how the **-- INSERT --** word disappears from your vi editor.

So, now you're back again to the **command** mode. In order to save the file, you have to enter the following sequence of characters -> **:wq**. Check out the image below:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course9.png)

Now, just type **enter** on your keyboard, and the file  will be saved. If you are asking yourself why these characters and not  others, well, the character **w** stands for **write** and the character **q** stands for **quit**. So basically, you are telling your file to save and exit.

Now, you can enter your file again and you should see whatever you've written into the file.

There are many more commands you can use though, as you may already  imagine. From what we know up to now, we could deduce that if, for  instance, we just want to exit the editor, we can then type the sequence **:q**. And that's right. Let's try it:

​             Execute in Shell #1





```
vi my_file.txt
```

Remember that whenever you enter vi for the first, you are set by default to the **command** mode. So now, all you need to do is to type **:q** and click enter in order to exit the file.

Let's now repeat the same process one more time, but this time,  before exiting the editor, you will do some modifications to your file  (remember how to enter the **insert** mode). Once you have added some new text to your file, try to exit the editor with the sequence **:q**. What happens? You will get a message like the following:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course10.png)

Basically, this means that since the last time you saved the file,  there have been some modifications to it, so you cannot just exit the  editor. At this point, you have 2 options:

- Use the **:w** sequence first, in order to **write** the latest changes. After this, you will be able to exit the editor using the sequence **:q**.
- Use the sequence **:q!**. This will exit the editor, ignoring the last changes to the file.

In general terms, you won't need much more than this. Anyway, if  required, you can search for all the commands available in any of the  many online tutorials that are out there. Also, you can use vi to learn  more. For that, you can execute the following commands:

​             Execute in Shell #1





```
vi
```

Once you are on the **vi** main screen, enter **:help** in order to access the online help.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course12.png)

​    \- Exercise 2.2 -

Inside the **linux_course_files** folder you downloaded at the beginning of this unit (the full path should be **~/catkin_ws/src/linux_course_files/**), create a new folder named **my_scripts**.

Then, inside this new folder, create a new file named **move_bb8_square.py**. Inside this file, paste the following contents.

move_bb8_square.py





```
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time
import math

class MoveBB8():
    
    def __init__(self):
        self.bb8_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10)
    
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
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()
        
    def shutdownhook(self):
        # works better than the rospy.is_shut_down()
        self.stop_bb8()
        self.ctrl_c = True

    def stop_bb8(self):
        rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def move_x_time(self, moving_time, linear_speed=0.2, angular_speed=0.2):
        
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        
        self.publish_once_in_cmd_vel()
        time.sleep(moving_time)
    
    def move_square(self):
        
        i = 0
        
        while not self.ctrl_c and i < 4:
            # Move Forward
            self.move_x_time(moving_time=4.0, linear_speed=0.2, angular_speed=0.0)
            # Turn, the turning is not affected by the length of the side we want
            self.move_x_time(moving_time=4.0, linear_speed=0.0, angular_speed=0.2)
            i += 1
            
        self.stop_bb8()
        rospy.loginfo("######## Finished Moving in a Square")
            
if __name__ == '__main__':
    rospy.init_node('move_bb8_test', anonymous=True)
    movebb8_object = MoveBB8()
    try:
        movebb8_object.move_square()
    except rospy.ROSInterruptException:
        pass
```

move_bb8_square.py

**NOTE:** Keep in mind that you don't have  to write the code line by line. You can just copy the whole code and  paste it into the text editor. Also, keep in mind that in order to be  able to paste the code, you will need to be on the **INSERT** mode.

So, at the end, you should have a structure like the one below:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course14.png)

Finally, execute this file using the following command:

​             Execute in Shell #1





```
rosrun move_bb8_pkg move_bb8_square.py
```

What's going on? Do you see anything strange? Do not worry, just keep reading!

​    \- End of Exercise 2.2 -

So... what's going on? It doesn't work? Are you getting an error? Could it be, by any chance, an error like the following one?

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course50.png)

Do not panic! Let's explain what's happening. First, you need to know that **rosrun** is a ROS command used in order to run executables that are located inside a package. The structure of the command is like this:





```
rosrun <package_name> <executable_file>
```

So, in the command you executed on Exercise 1.2, the system was trying to find an executable file named **move_bb8_square.py** inside the **move_bb8_pkg** package. Actually, these are ROS concepts, so we are not going to get  into them much. But then... why am I explaining this? Well, with this  explanation, we can see that the error we are getting is because our  script is not inside the **move_bb8_pkg** folder. This is the real important issue here.

So, in order to solve our error, we better **move** our file into the correct folder, right? Now, how can we do this?

###         2.3.4                 Command "mv"    

The command **mv** in Linux stands for **move**, so it's quite self-explanatory. It allows you to move files or folders  from one location to another. For instance, let's try the following  commands. First off, let's make sure we are on the directory of the  file/folder we want to move.

​             Execute in Shell #1





```
cd  ~/catkin_ws/src/linux_course_files/
```

Now, let's move the folder **my_scripts** into the folder **move_bb8_pkg**.

​             Execute in Shell #1





```
mv my_scripts move_bb8_pkg
```

As you can see, the structure of the command is very simple.





```
mv <file/folder we want to move> <destination>
```

Since we have previously gone into the **linux_course_files** folder, which contains both the **my_scripts** and **move_bb8_pkg** folders, we can then type those names directly. Although, if we weren't in that directory, we could still execute the command. In that case, we would just need to specify the full paths. It would be something like  this:





```
mv ~/catkin_ws/src/linux_course_files/my_scripts ~/catkin_ws/src/linux_course_files/move_bb8_pkg
```

So at the end, you should have the folder **my_scripts** inside the **move_bb8_pkg**.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course17.png)

Now, along with the **mv** command, there are a couple of other basic Linux commands that I would like to introduce. Let's get to them!

###         2.3.5                 Command "cp"    

In Linux systems, the command **cp** stands for **copy**. So basically, it allows you to copy a file or a folder (or multiples) from one location to another one.

For instance, let's try the following commands.

First, let's move to the **my_scripts** folder.

​             Execute in Shell #1





```
cd  ~/catkin_ws/src/linux_course_files/move_bb8_pkg/my_scripts
```

Now, let's copy the **move_bb8_square.py** file into a new file named **move_bb8_square_copy.py**.

​             Execute in Shell #1





```
cp move_bb8_square.py move_bb8_square_copy.py
```

So, in the end, you should end up with the following:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course18.png)

As you can see, the structure of the command is very simple.





```
cp <file/folder we want to copy> <name of the new file/folder>
```

You can also check, using **vi**, that the new file created has the exact same contents as the original one.

​             Execute in Shell #1





```
vi move_bb8_square_copy.py
```

Now, let's go one step further. Let's try to copy the whole **my_scripts** folder. For that, let's execute the following command.

​             Execute in Shell #1





```
cp my_scripts/ my_scripts_copy/
```

Oops!! What happened now? Did the command work? I bet it didn't. Did you, by any chance, get a message like the following one?

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course19.png)

Well, don't worry because that's totally normal. You get this message because it is NOT possible to copy a folder using the regular **cp** command. In order to copy a folder, you will need to use the **-r** argument. Let's try again.

​             Execute in Shell #1





```
cp -r my_scripts/ my_scripts_copy/
```

If you now have a look in the folder, you should see something like this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course20.png)

As for most Linux commands, though, there are many other options that you can use with the **cp** command. Remember that you can have a look at them using the following command:

​             Execute in Shell #1





```
cp --help
```

###         2.3.6                 Command "rm"    

So, maybe it's time to clean up all the mess we have created, don't you think? In Linux, the **rm** command stands for **remove**. So, let's try to remove some of the files and folders that we have created lately with the following commands.

First of all, let's make sure we are in the correct folder.

​             Execute in Shell #1





```
cd  ~/catkin_ws/src/linux_course_files/move_bb8_pkg/my_scripts
```

Now, let's remove the copy file we created in the previous section.

​             Execute in Shell #1





```
rm move_bb8_square_copy.py
```

As you can see, the structure of the command is very simple.





```
rm <file to remove>
```

Now, for removing folders, it works exactly the same as with the **copy** command: you need to add a **-r** flag to the command. Let's try to remove the copy folder we created before.

​             Execute in Shell #1





```
rm -r my_scripts_copy
```

Take this chance to clean up any other copied files you may have created during your tests.

All right! So... do you remember how we introduced the **mv** command in order to solve the error we detected in Exercise 1.2? Well, let's go back to that!

​    \- Exercise 2.3 -

So, at this point, if you followed all the steps we've been doing in  the last sections, you should have already solved the error detected in  Exercise 1.2. You should have a file structure like the one below:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course21.png)

Then, let's try to execute our Python file again!

​             Execute in Shell #1





```
rosrun move_bb8_pkg move_bb8_square.py
```

Hell!!! What now? Another error? Got you!! Do not panic and just keep reading :-)

So... what's going on now? It doesn't work yet? Are you getting an  error? Could it be, by any chance, an error like the following one?

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course13.png)

This error is very common when executing Python files. Basically, it means that your file doesn't have **execution permissions**, so the system doesn't detect it as a file that can be executed.

Well, I think that this is enough for this chapter... what do you  think? If you want to learn how to solve this issue, and learn more  about permissions in Linux, move on the next unit!

​    \- End of Exercise 2.3 -
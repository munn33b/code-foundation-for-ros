Loading [MathJax]/jax/output/CommonHTML/fonts/TeX/fontdata.js

------

#         Linux for robotics    

------

#         Unit 3                 Advanced Utilities    

​    \- Summary -

Estimated time to completion: **3 hours**

In this unit, you are going to learn some advanced utilities that will allow you to interact deeply with a Linux system.

​    \- End of Summary -

Hi there! I guess you've come from the last unit, eager to discover  how to solve that last error, huh? We will get there in a moment! But  before that, let me tell you that in Unit 2, you are going to learn  about some other tools and commands that, despite not being needed as  often as the ones introduced in Unit 1, are equally important in order  to work with a Linux-based system.

So, with the proper introductions made, let's go to the real work!

##         3.1                 Permissions    

Although there are already a lot of good security features built into Linux-based systems, one very important potential vulnerability can  come from file permission-based issues resulting from a user not  assigning the correct permissions to files and directories.

Let's start by executing a command we already know.

First of all, let's go to the **my_scripts folder**.

​             Execute in Shell #1





```
cd /home/user/catkin_ws/src/linux_course_files/move_bb8_pkg/my_scripts/
```

Now, let's execute the **ls** command.

​             Execute in Shell #1





```
ls -la
```

You should get something like the following:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course22.png)

As you may have noticed, we have used the known **ls** command using a new flag, **-la**. Basically, the **-la** flag allows us to see some basic data related to the files or folders,  like the permissions of the files/folders, their creation date/time,  etc.





```
-rw-r--r-- 1 user user 2203 Jul 23 23:26 move_bb8_square.py
```

For now, let's just focus on the first part, which is **rw-r--r--**. These are the **PERMISSIONS** of the file.

Each file or directory has 3 permissions types:

- **read**: The Read permission refers to a user's ability to read the contents of the file. It is stated with the character **r**.
- **write**: The Write permission refers to a user's ability to write or modify a file or directory. It is stated with the character **w**.
- **execute**: The Execute permission affects a user's ability to execute a file or view the contents of a directory. It is  stated with the character **x**.

On the other hand, each file or directory has three user-based permission groups:

- **owner**: The Owner permissions apply only to the  owner of the file or directory, and will not impact the actions of other users. They are represented in the first 3 characters.
- **group**: The Group permissions apply only to the  group that has been assigned to the file or directory, and will not  affect the actions of other users. They are represented in the middle 3  characters.
- **all users**: The All Users permissions apply to  all other users on the system, and this is the permission group that you want to watch the most. They are represented in the last 3 characters.

So, applying all this to our file, we can say that the **owner** of the file (in this case, it's us) has **read** (**r**) and **write** (**w**) permissions, and the **group** and the **rest of users** have only **read** (**r**) permissions.





```
rw-/r--/r--
```

So, as you may have already deduced, the only permissions that apply  are the ones that are explicitly specified with their character. If they appear with a **-** symbol, it means that the permissions are not applied.

From this, we can see that this file has no **execution** permissions. And this, if we want to actually execute the file, could  be quite a problem, don't you think? Then... how can we change this?

###         3.1.1                 Command "chmod"    

In Linux, the **chmod** command is used to modify the  permissions of a given file or directory (or many of them). There are a  couple of ways to use this command, though, so let's go by parts. Let's  try the next command.

First of all, let's make sure that we are in the **my_scripts** folder.

​             Execute in Shell #1





```
cd ~/catkin_ws/src/linux_course_files/move_bb8_pkg/my_scripts
```

Now, let's execute the **chmod** command in order to add execution permissions to the file.

​             Execute in Shell #1





```
chmod +x move_bb8_square.py
```

Let's now execute the **ls** command again and see how the permissions have changed.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course41.png)

As you can see, the **execution** permissions, which are represnted by the character **x**, have been added to all the permission groups.

Excellent! But you may be asking yourself... what if I only want to  modify the permissions for 1 of the groups? Or what if I want to remove a permission instead of assigning it? And those are very good questions!  Let's go step by step.

The structure of the **chmod** command goes as follows:





```
chmod  <groups to assign the permissions><permissions to assign/remove> <file/folder names>
```

We already know the 2 last parameters. As for the groups, you can specify them using the following flags:

- **u**: Owner
- **g**: Group
- **o**: Others
- **a**: All users. For all users, you can also leave it blank, as we did in the example command you executed before.

Let's write down some examples in order to better understand this.  So, for instance, if we wanted to give write permissions only to the **group**, how would the command be? Can you guess? If not, you can find the solution right below:





```
chmod g+w move_bb8_square.py
```

The above command would then change these permissions...





```
rwxr-xr-x
```

...to these ones





```
rwxrwxr-x
```

And, what if now we want to delete, for instance, the execution permissions for **group** and **others**? It would be something like this:





```
chmod go-x move_bb8_square.py
```

The command above would change the permissions from this...





```
rwxrwxr-x
```

...to this





```
rwxrw-r--
```

Is it clearer now? You can keep on doing some tests if you want to  get used to it. But as I've already mentioned at the beginning of the  section, there's still another method in which you can modify the  permissions of a file. It is done through **Binary References**. And how does it work?

It's quite simple actually. Basically, the whole string stating the  permissions (rwxrwxrwx) is substituted by 3 numbers. The first number  represents the Owner permission; the second represents the Group  permissions; and the last number represents the permissions for all  other users. And how do these numbers work?

Basically, each permission has a number assigned:

- r = 4
- w = 2
- x = 1

Then, you add the numbers to get the integer/number representing the  permissions you wish to set. You will need to include the binary  permissions for each of the three permission groups.

For instance, let's say that we want to give the owner full  permissions (rwx). Then, we would add 4 + 2 + 1 = 7. Now, for the Group, we just want to give read permissions (r--). Then, that would just be a 4. Finally, for all the other users, we don't want to give them any  permission. Then, that would be a 0. So, at the end, we would have **740**. So, the final command would be like this:

​             Execute in Shell #1





```
chmod 740 move_bb8_square.py
```

This command will result in the following permissions for your file:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course42.png)

​    \- Exercise 3.1 -

Assign to the **move_bb8_square.py** file the following permissions:

- Owner: Read, Write and Execute
- Group: Read and Execute
- All others: Read and Execute

Once the permissions are set, execute the Python script again with the following command.

​             Execute in Shell #1





```
rosrun move_bb8_pkg move_bb8_square.py
```

And... finally it works!! Yes!! You should see how BB8 starts moving, performing a square.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/basic_unit3_exercise3-3_final.gif)

​    \- End of Exercise 3.1 -

##         3.2                 Bash scripts    

In this last exercise, you've executed a Python script using the **rosrun** command, which is a ROS command. But this course is not about ROS, and  we just used that command in order to demonstrate some Linux concepts.  Furthermore, you could work on a Linux system that doesn't have ROS  installed. In which case, how would you execute a Python script?

It's very simple. In fact, you have already done it in the previous unit, in Exercise 1.1. You use the **python** command. For instance, let's execute the **move_bb8_square.py** script again using this command.

​             Execute in Shell #1





```
python move_bb8_square.py
```

Python scripts are very common, but are more of a Python (or even  ROS) related topic than a Linux one. However, Linux has its own scripts. They are commonly know as **bash scripts**.

Basically, a **bash script** is a regular text file that contains a series of commands. These commands are a mixture of commands you would normally type yourself on the command line (such as the ones  we have been reviewing, **cd, ls, or cp**) and also  commands we could type on the command line, but generally wouldn't  (you'll discover these over the next few pages). An important point to  remember though is:

**Anything you can run normally on the command line can be put into a script and it will do exactly the same thing**. Similarly, anything you can put into a script can also be run normally  on the command line and it will do exactly the same thing.

Sounds interesting, right? Let's create our own bash script then!

So, inside the **my_scripts** folder, create a new file named **bash_script.sh**.

​             Execute in Shell #1





```
touch bash_script.sh
```

Now, inside this file, place the following contents:

bash_script.sh





```
#!/bin/bash

echo Hello there, Developers!
```

bash_script.sh

Let me comment on a couple of important things. First, as you can see, the file extension is **.sh**. Usually, this is the extension you will always use when you create a  new bash script. Second, note that the script starts with the line **#!/bin/bash**. All bash scripts will start with this special line. Basically, it let's the Linux system know that this file is a bash script.

Great! So, with the proper explanations provided, let's now execute it!

​             Execute in Shell #1





```
./bash_script.sh
```

So... what happened? Are you getting an error? Maybe something similar to this?

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course15.png)

The error message is quite clear, in fact. It must be something  related to the permissions of the file, don't you think? Check the  permissions of the file and try to solve it by yourself. Come on!

​    \- Exercise 3.2 -

Try to find out, by yourself, what error you are getting that doesn't allow you to execute the bash script. If you can't solve it by  yourself, do not worry. Right below this exercise, you will find out how to do it.

​    \- End of Exercise 3.2 -

So... did you manage to solve it? Well, as we already mentioned  before, it is a permissions issue. And it is a known one. Basically, the problem is that we are trying to execute a bash script that doesn't  have execution permissions. So, in order to solve it, all we have to do  is give it execution permissions.

​             Execute in Shell #1





```
chmod +x bash_script.sh
```

Now, the script permissions should look something like this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course43.png)

Let's now try to execute the script again.

​             Execute in Shell #1





```
./bash_script.sh
```

You should get the following output.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course51.png)

As you can see, this is a very simple script. It just does an **echo** of a string. In a bash script, the command **echo** is used in order to print something on the Shell (similar to a print  function in Python). Anyway, as we said, you can put any kind of command into a bash script.

​    \- Exercise 3.3 -

Create a bash script that does the following:

- First, it goes to the **my_scripts** folder.

- Second, it will list all the files there (showing their permissions)

- Then, it will modify the permissions of the **move_bb8_square.py** file so that all the groups have full permissions.

- Finally, it will list all the files again (showing the updated permissions).

​    \- End of Exercise 3.3 -

​    \- Solution for Exercise 3.3 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/TC-logo-blue.png)

Please try to do it by yourself unless you get stuck or need some  inspiration. You will learn much more if you fight for each exercise.

**ex23_script.sh**





```
#!/bin/bash

echo 'Going to my_scripts folder...'
cd ~/catkin_ws/src/linux_course_files/move_bb8_pkg/my_scripts/
echo 'Listing contents...'
ls -la
echo 'Changing permissions...'
chmod 777 move_bb8_square.py
echo 'Listing contents...'
ls -la
```

When executing it, you will get an output like this one:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course52.png)

​    \- End Solution for Exercise 3.3 -

###         3.2.1                 Passing parameters to a bash script    

As you already saw on the first demo of this course, you can pass  parameters to a bash script. This is used when you want your script to  perform in a different way, depending on the values of the input  parameters (also called arguments).

For instance, let's have a look at the script you used in this first demo, called **demo.sh**. I will just tell you the location of the script. It's in the following path: **/home/simulations/public_sim_ws/src/all/ros_basics_examples/linux_demo/**. The rest you should be able to do by yourself at this point.

So, at the end, you will see that the script contains the following code:

demo.sh





```
#!/bin/bash

ARG1=$1

if [ $ARG1 == 'forward' ]; then
    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: -0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

elif [ $ARG1 == 'rotate' ]; then
    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.2"

elif [ $ARG1 == 'stop' ]; then
    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
fi
```

demo.sh

In fact, it's pretty simple. You can access an argument inside a script using the variables **$1**, **$2**, **$3**, and so on. The variable **$1** refers to the first argument, **$2** to the second argument, and **$3** to the third argument. So, in this case, since we only have 1 argument, we access it using the variable **$1**.

Then, we assign the value of the argument to the variable **$ARG1**. So, the **$ARG1** variable will contain either **forward**, **rotate**, or **stop**. Finally, we check which one of them it contains (using the if  statement), and we execute the corresponding ROS command (rostopic pub)  in order to move the BB8 robot accordingly.

​    \- Exercise 3.4 -

Create a bash script that does the following:

- It will receive one parameter, which will contain either:
  - **circle**
  - **forward_backward**
  - **square**

- If the parameter is **circle**, the script will execute the **move_bb8_circle.py** file.

- If the parameter is **forward_backward**, the script will execute the **move_bb8_forward_backward.py** file.

- If the parameter is **square**, the script will execute the **move_bb8_square.py** file.

​    \- End of Exercise 3.4 -

​    \- Solution for Exercise 3.4 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/TC-logo-blue.png)

Please try to do it by yourself unless you get stuck or need some  inspiration. You will learn much more if you fight for each exercise.

**ex3_4_script.sh**





```
#!/bin/bash

ARG1=$1

if [ "$ARG1" == "circle" ]; then
   echo "circling";
   rosrun move_bb8_pkg move_bb8_circle.py 

elif [ "$ARG1" == 'forward_backward' ]; then
     echo "back and forth";
     rosrun move_bb8_pkg move_bb8_forward_backward.py

elif [ "$ARG1" == "square" ]; then
     echo "square dancing";
     rosrun move_bb8_pkg move_bb8_square.py

else 
echo "Please enter one of the following;
circle
forward_backward
square"

fi
```

​    \- End Solution for Exercise 3.4 -

##         3.3                 The ".bashrc" file    

In the previous section, we just introduced you to bash scripts. Now, let me introduce to a very special bash script. In fact, you already  had a sneek peak at it in the previous unit. I'm talking about the **.bashrc** file. The **.bashrc** file is a special bash script, which Linux executes whenever a new Shell session is initialized.

As you may have noticed, the **.bashrc** file is a  hidden file. It is automatically generated by the Linux system and it is always placed in the HOME folder (in our case, this is **/home/user/**). However, you can still modify it in order to customize your Shell session. Let's have a look at our **.bashrc** file!

​             Execute in Shell #1





```
cd
```





```
vi .bashrc
```

As you can see, it is quite extensive. Basically, it has all the  configurations for our Shell session. We are not going to enter into  details but, for instance, you can find the configuration for the color  of the shell prompt here.





```
if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
    fi
```

In fact, let's play a little with this! In the **.bashrc** file, at the end of it, add the following line and save it.





```
export PS1="\${debian_chroot:+(\$debian_chroot)}\\[\\033[01;36m\\]\\u\\[\\033[00m\\]:\\[\\033[01;34m\\]\\w\\[\\033[00m\\]\\\$ "
```

Now, as I've already said before, the **.bashrc** script is executed each time a new Shell session is initialized. In the case  of the Robot Ignite Academy, though, the Shell sessions are already  started for you, and you can't start a new one. But there is a command  that allows you to force the execution of the **.bashrc** file. This is the **source** command.

Run the following command in order to execute the **.bashrc** script.

​             Execute in Shell #1





```
source .bashrc
```

Now, you will see how the color of your Shell prompt has been modified.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course23.png)

Ok, so this is very cool but... it has nothing to do with robotics at all. So, in order to keep the focus on Linux for robotics, let's  introduce a new concept: **Environment variables**.

##         3.4                 Environment Variables    

An environment variable is a dynamic-named value that can affect the way running processes will behave on a computer.

They are part of the environment in which a process runs. For example, a running process can query the value of the *TEMP* environment variable to discover a suitable location to store temporary files, or the *HOME* variable to find the directory structure owned by the user running the process. Or, for instance, the *PS1* variable, which you modified in the previous section, which defines the color of the Shell of prompt.

Furthermore, environment variables are frequently used in Robotics Development and in ROS.

###         3.4.1                 Command "export"    

In simple terms, environment variables are set when you open a new  Shell session. If you change any of the variable values during the Shell session, the Shell has no way of picking up that change. The **export** command, on the other hand, provides the ability to update the current  Shell session about the change you made to the exported variable. You  don’t have to wait until a new Shell session to use the value of the  variable you changed.  In fact, this is exactly what we did in the previous section: we  modified the *PS1* variable using the export command and added it to the *.bashrc* script. Then, when we executed the *.bashrc* script using the *source* command, the **export** command also got executed, so the *PS1* environment variable got updated.

Let's run the export command by itself in order to see all the environment variables running.

​             Execute in Shell #1





```
export
```

You should get something similar to this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course24.png)

As you can see, you can view all the environment variables with their values. But, there are a lot of variables...

###         3.4.2                 Command "grep"    

In Linux systems, the **grep** command is used in order to filter elements. For, instance, execute the following command.

​             Execute in Shell #1





```
export | grep ROS
```

You should now get something like this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course26.png)

So basically, what we did with the previous command was to first execute the **export** in order to get the environment variables of the system, and then  filter them so that we only get the ones that contains the character **ROS**. Pretty useful, right?

But there's more! You cannot only use the **grep** command alongside **export**. Not at all! In fact, you can use it with any command you want. For instance, try the following.

First, let's go to the **my_scripts** folder.

​             Execute in Shell #1





```
cd ~/catkin_ws/src/linux_course_files/move_bb8_pkg/my_scripts
```

Now execute a regular *ls* command.

​             Execute in Shell #1





```
ls
```

And now, execute it using the **grep** filter.

​             Execute in Shell #1





```
ls | grep bb8
```

As you can see, now you only get the files that containt the **bb8** characters.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course25.png)

###         3.4.3                 Setting a variable with export    

As you already saw in the **.bashrc** file section, you can set environment variables using the command **export**





```
export PS1="\${debian_chroot:+(\$debian_chroot)}\\[\\033[01;36m\\]\\u\\[\\033[00m\\]:\\[\\033[01;34m\\]\\w\\[\\033[00m\\]\\\$ "
```

In this case, though, we are going to work with an environment  variable that is much more related to robotics. First of all, execute  the following command:

​             Execute in Shell #1





```
export | grep ROS
```

You should get the following results.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course26.png)

Let's focus for a moment on the **ROS_PACKAGE_PATH**  environment variable. This variable is used by ROS in order to get the  possible locations for any ROS packages. As you can see, we have the  path "**/home/user/catkin_ws/src**", where our package **move_bb8_pkg** is located. So, what would happen if I remove this path from the variable? Would ROS still be able to find it? Let's find out!

​             Execute in Shell #1





```
export ROS_PACKAGE_PATH="/home/simulations/public_sim_ws/src:/opt/ros/kinetic/share"
```

Now, have a look again at the variables and check if they got updated.

​             Execute in Shell #1





```
export | grep ROS
```

And yes! It was properly updated.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course27.png)

Finally, let's try to execute the **move_bb8_square.py** file using the following command:

​             Execute in Shell #1





```
rosrun move_bb8_pkg move_bb8_square.py
```

What's going on? Does it work? Are you getting an error similar to this one?

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/linux_for_robotics/img/linux_course28.png)

That's great! We have just broken our ROS system!! But, at least, we've learned to use the **export** command :-) And, in fact, it is quite easy to solve. You just need to set back the correct environment variable.

​             Execute in Shell #1





```
export ROS_PACKAGE_PATH="/home/user/catkin_ws/src:/home/simulations/public_sim_ws/src:/opt/ros/kinetic/share"
```

Now, if you try to run the **move_bb8_square.py** again, it will be able to find the package without any problem.
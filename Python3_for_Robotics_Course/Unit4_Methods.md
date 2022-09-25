------

#         Python for robotics    

------

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/python3_logo.png)

#         Unit 4                 methods    

​    \- Summary -

Estimated time to completion: **2 hours**

 In this unit, you are going to learn some more advanced tools that are  going to allow you to create more complex and interesting programs.  Specifically, you are going to learn about methods.

​    \- End of Summary -

During the course, you have already been working with methods quite a lot. Specifically, you have been CALLING methods in order to control  the robot in the simulation. But it's time to learn a bit more about  methods.

The concept of a method is one of the most important ones in  mathematics. A common usage of methods in computer languages is to  implement mathematical methods. Such a method computes one or more  results, which are entirely determined by the parameters passed to it.

In the most general sense, a method is a structuring element in  programming languages, used to group a set of statements so they can be  utilized more than once in a program. The only way to accomplish this  without methods would be to reuse code by copying it and adapting it to a different context. Using methods usually enhances the comprehensibility and quality of the program. It also lowers the cost for development and maintenance of the software.

##         4.1                 Definition    

A method in Python is defined by a **def** statement. The general syntax looks like this:





```
def mymethod():
    print ("Python")
```

As you can see, it's pretty simple. You just need to keep in mind the keyword **def**. Once a method is created, it can be called anywhere inside the program. How? Let's see!

##         4.2                 Calling a method    

In order to call a method, you just have to specify the name of the  method, followed by the open and close parentheses. Check the example  below:





```
def mymethod():
    print ("The method mymethod() has been called")
    
mymethod()
```

​    

```
The method mymethod() has been called
```

If you execute the above code, you will get the following output:

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/method_output1.png)

​    \- End Expected Output -

##         4.3                 method parameters    

Parameters are specified after the method name, inside the  parentheses. You can add as many parameters as you want, just separate  them with a comma. Check out the example below:





```
def add(a,b):
    res = a + b
    print (res)
    
add(2,2)
```

​    

```
4
```

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/method_output2.png)

​    \- End Expected Output -

The parameter list consists of none or more parameters. Parameters  are called arguments, if the method is called. The method body consists  of indented statements. The method body gets executed every time the  method is called.  Parameters can be **mandatory or optional**. The optional parameters (usually known as **default parameters**) must follow the mandatory parameters.

In the example above, our method had two mandatory parameters: **a** and **b**. And why are they mandatory? Well, because if we do not provide those  parameters to the method, there is no way the method will be able to do  what it's supposed to do (in this case, add them).

What are these optional parameters? Let's have a look at the same  example we used before, but with optional parameters this time:





```
def add(a=2,b=2):
    res = a + b
    print (res)
    
add()
```

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/def_parameters.png)

​    \- End Expected Output -

Note how this time we have called the method without calling any  parameter. This is because with optional parameters (also know as  default parameters), if the method is called without any parameters, it  will call the default ones, which are defined within the parentheses.

​    \- Exercise 4.1 -

a) Create a method that, given an integer number, makes the robot  move straight for that amount of time. For instance, if the given number is a 5, the robot will move straight for 5 seconds.

**NOTE:** For this exercise, you can use the Python **sleep()** method. In order to use it, you just have to import the **time** module, and then call the **sleep()** method, like this:





```
import time

time.sleep(5) # This will make your program sleep for 5 seconds
```

​    \- End of Exercise 4.1 -

​    \- Solution for Exercise 4.1 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/TC-logo-blue.png)

Please try to do it by yourself unless you get stuck or need some  inspiration. You will learn much more if you fight for each exercise.

test_methods1.py





```
from robot_control_class import RobotControl
import time

robotcontrol = RobotControl(robot_name="summit")

def move_x_seconds(secs):
    robotcontrol.move_straight()
    time.sleep(secs)
    robotcontrol.stop_robot()


move_x_seconds(5)
```

​    \- End Solution for Exercise 4.1 -

##         4.4                 Return statement    

In the previous chapters, whenever we performed calls to methods,  they were always returning some kind of value, right? However, in the  examples we've seen up until now, we are not returning anything from our methods, are we?

So, this clearly tells us that the return statement is not mandatory. In any case, most of the methods are meant to return some kind of  value. Let's review again the add() method example, using a return  statement this time:





```
def add(a=2,b=2):
    res = a + b
    return res
    
r = add(3,4)
print (r)
```

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/output_return.png)

​    \- End Expected Output -

A method can return exactly one value, or we should say, one object.  An object can be a numerical value, like an integer or a float, but it  can also be a list or a dictionary (Remember the **get_laser_full()** method). So, if we have to return, for example, three integer values,  we can return a list or a tuple with these three integer values.





```
def return_list():
    
    return [1, 2, 3]
            
l = return_list()
print (l[0])
print (l[1])
print (l[2])
```

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/return_list.png)

​    \- End Expected Output -

​    \- Exercise 4.2 -

a) Create a new method that gets 3 integer numbers (between 0 and 719) as parameters.

b) The method will then call the **get_laser_summit()** method using the integer numbers as parameters, in order to get the corresponding laser reading for each one.

c) Finally, it will return a list containing the 3 different laser readings. Outside the method, you will print these 3 values.

**IMPORTANT NOTE**: As you can see, we have introduced a new method called **get_laser_summit()**. This method works exactly the same as the **get_laser()** method, but it is prepared to work with the Summit XL simulation (because the laser topics have different names).

​    \- End of Exercise 4.2 -

​    \- End Solution for Exercise 4.2 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/TC-logo-blue.png)

Please try to do it by yourself unless you get stuck or need some  inspiration. You will learn much more if you fight for each exercise.

test_methods2.py





```
from robot_control_class import RobotControl

robotcontrol = RobotControl(robot_name="summit")

def get_laser_values(a,b,c):
    r1 = robotcontrol.get_laser_summit(a)
    r2 = robotcontrol.get_laser_summit(b)
    r3 = robotcontrol.get_laser_summit(c)

    return [r1, r2, r3]

l = get_laser_values(0, 500, 1000)

print ("Reading 1: ", l[0])
print ("Reading 2: ", l[1])
print ("Reading 3: ", l[2])
```

​    \- End Solution for Exercise 4.2 -

##         4.5                 Local and Global Variables in methods    

Variables are local to the methods in which they are defined. This  means that they cannot be used outside the method. For instance, check  out the following example:





```
def f(): 
    lv = "Local Variable"

f()
print(lv)
```

What do you think will happen if you execute the above code? In theory, the string **"Local Variable"** should get printed. Let's execute the code and see what happens:

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/func_error.png)

​    \- End Expected Output -

As you can see, we are getting an error. Specifically, it says: **name 'lv' is not defined**. As we have previously said, the variable **lv** has been defined inside the method, so it is local to the method. This means that it CANNOT be used outside the method.

For a case like this, Python provides **global variables**. Let's check out an example:





```
def f():
    global gv
    gv = "Global Variable"

gv = "Empty"
print (gv)
f()
print(gv)
```

As you can see, now we have defined the variable **gv** before the method is called. Then, inside the method, we reference this variable using the keyword **global**. This keyword will basically generate a copy of the **gv** global variable inside the method.

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/global_variable.png)

​    \- End Expected Output -

##         4.6                 Let's play some more with the robot!    

For the following exercises, we are going to introduce two new methods:

- move_straight_time (motion, speed, time)

  : As the  name itself says, this method will allow you to move the robot in a  straight line. You will need to pass three parameters to it.

  - **motion**: Specify here if you want your robot to move forward (**"forward"**) or backward (**"backward"**).
  - **speed**: Specify here the speed at which you want your robot to move (in m/s).
  - **time**: Specify here how long you want your robot to keep moving (in seconds).

- turn (clockwise, speed, time)

  : As the name itself says, this method will allow you to turn the robot. You will need to pass three parameters to it.

  - **clockwise**: Specify here whether you want your robot to turn clockwise (**"clockwise"**) or counter-clockwise (**"counter-clockwise"**).
  - **speed**: Specify here the speed at which you want your robot to turn (in m/s).
  - **time**: Specify here how long you want your robot to keep turning (in seconds).

Both methods will **return a string**, indicating the motion they have performed.

​    \- Exercise 4.3 -

a) Inside the **robot_control** folder, add a new Python script named **test_methods3.py**.

b) Inside this new file, do the necessary calls to methods in order to:

- Move the robot forward for 5 seconds
- Turn the robot clockwise for 7 seconds

Also, after each of the movements, print the string returned by the method.

**IMPORTANT NOTE**: Remember, in order to be able to use the methods to control the robot, you will need to make two things.

- You will have to import the Python class contained in the **robot_control_class.py** file. You can do this with the following line (place it at the top of your program).





```
from robot_control_class import RobotControl
```

- You will also have to create an object of the class in order to be  able to call the methods. You can do this with the following line:





```
robotcontrol = RobotControl()
```

- Now, you will be able to call the methods from this object, like this:





```
robotcontrol.move_straight_time(necessary_parameters)
```

​    \- End of Exercise 4.3 -

​    \- End Solution for Exercise 4.3 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/TC-logo-blue.png)

Please try to do it by yourself unless you get stuck or need some  inspiration. You will learn much more if you fight for each exercise.

test_methods3.py





```
from robot_control_class import RobotControl

robotcontrol = RobotControl(robot_name="summit")

robotcontrol.move_straight_time("forward", 0.3, 5)
robotcontrol.turn("clockwise", 0.3, 7)
```

​    \- End Solution for Exercise 4.3 -

Note that in this last exercise, we have introduced a new concept, **Python classes**. And this is exactly the topic we are going to see in the following chapter!

​    \- Exercise 4.4 -

a) Create a new Python script that, using the same methods introduced in the previous exercise, helps the robot enter the room with the  Turtle logo.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/turtle_room.png)

​    \- End of Exercise 4.4 -

​    \- Solution for Exercise 4.4 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/TC-logo-blue.png)

Please try to do it by yourself unless you get stuck or need some  inspiration. You will learn much more if you fight for each exercise.

**IMPORTANT NOTE**: Keep in mind that the  way Summit XL turns is not very accurate. This means that the solution  to this exercise will not be exact. Just keep trying different  combinations until you achieve it.

test_methods4.py





```
from robot_control_class import RobotControl

robotcontrol = RobotControl(robot_name="summit")

robotcontrol.turn("counter-clockwise", 0.3, 4)
robotcontrol.move_straight_time("forward", 0.3, 6)
robotcontrol.turn("counter-clockwise", 0.3, 4)
robotcontrol.move_straight_time("forward", 0.3, 7)
```

​    \- End Solution for Exercise 4.4 -
------

#         Python for robotics    

------

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/python3_logo.png)

#         Unit 3                 Conditional Statements and Loops    

​    \- Summary -

Estimated time to completion: **2 hours**

 In this unit, you are going to learn some more advanced tools that are  going to allow you to create more complex and interesting programs.  Specifically, you are going to learn about conditional statements and  loops.

​    \- End of Summary -

##         3.1                 Conditional Statements    

Life is all about making decisions, isn't it? Well, let me tell you  that programming is no different! In programming, decisions are made  using **conditional statements**, mostly in the form of **if statements**. Almost any Python program you will find or create will contain some if  statements... at least, if the program is meant to be useful and solve a problem.

In most cases, the decision will depend on the value of variables or  arithmetic expressions. These expressions are evaluated to the boolean  values **True or False**, in the following way:

- If the conditional statement is True, one action will be taken.
- If it's False, another action will be taken.

Let's imagine the following real-life situation:

**If there is a wall in front of the robot closer than 1 meter, I will stop the robot so that it doesn't crash. Otherwise, I will keep  moving my robot forward! In any case, I will print the reading of the  laser.**

Let's convert this thought into a flowchart, to try to understand it better:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/flowchart_new.png)

The above flowchart may result in a Python script like this:





```
if wall_is_close:
    stop_robot() 
    
else:
    keep_moving()
    
print_reading()
```

As you can see, the if statement is used in order to decide which  block of code is executed. If the first condition is True, then the 1st  block (**stop_robot( )**) will be executed. If it's False, then the 2nd block will be executed (**keep_moving( )**). In both cases, the **print_reading( )** method will be executed.

The simplest form of an if statement in Python looks like this:





```
if condition:
    statement
    statement
    # ... some more statements if necessary
```

##         3.2                 Blocks and Indentations    

As you may have already noticed, the statements that are inside the condition are **indented**. These indented blocks of code are executed only if the "condition" is evaluated as True.





```
if condition:
    # Indented statements
    statement
    statement

else:
    # Indented statements
    statement
    statement
```

For instance, the following example code asks the user about his  favorite movie. The indented print statement will only be executed if  his favorite movie is "Avengers Endgame." If he says any other movie,  nothing will be printed:





```
movie = input("What's your favorite movie? ")

if movie == "Avengers Endgame":
    print("Good choice!")
```

Inside the **robot_control** folder, create a new Python script named **fav_movie.py**, and copy into the file the code showed above. Let's now execute this file and see what you get!

​             Execute in Shell #1





```
python fav_movie.py
```

If you enter *Avengers Endgame* as your favorite movie, you will get the following:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/fav_movie1.png)

Anyway, this example is very poor, isn't it? What happens if our user is more into romantic super productions, and he prefers "Titanic?"  Well, we have some good news for our friend! For this case, we can use  the **or** expression. Check out the following example:





```
movie = input("What's your favorite movie? ")

if movie == "Avengers Endgame" or movie == "Titanic":
    print("Good choice!")
```

Update the file **fav_movie.py** with the code showed above, and execute it again! This time, if you enter the movie *Titanic*, you will get the following:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/fav_movie2.png)

But wait! What if our program is executed by a young padawan, who loves "Star Wars?" Let's update our code again:





```
movie = input("What's your favorite movie? ")

if movie == "Avengers Endgame" or movie == "Titanic":
    print("Good choice!")
if movie == "Star Wars":
    print("Also a good choice!")
```

Update the file **fav_movie.py** with the code showed above, and execute it again! This time, if you enter the movie *Star Wars*, you will get the following:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/fav_movie3.png)

But wait! This new Python script has a disadvantage. Let's assume  that you execute the code and select "Titanic" as your favorite movie.  In this case, as you already know, the first print will be executed.  After this, though, the program will go on and check the second  condition, which we already know is not possible (because the user has  already selected "Titanic"). This means our program will be doing  unnecesary work, which is something we always want to avoid. So... how  do we solve this?

Well, this can be easily solved using the **elif** condition. The **elif** condition will only be checked if the previous condition was evaluated **False**. Check out the following example:





```
movie = input("What's your favorite movie? ")

if movie == "Avengers Endgame" or movie == "Titanic":
    print("Good choice!")
elif movie == "Star Wars":
    print("Also a good choice!")
else:
    print("You really are an interesting specimen")
```

Update the file **fav_movie.py** with the code showed above, and execute it again! This time, if you enter any other movie, you will get the following:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/fav_movie4.png)

Like in the above example, if statements usually have **elif** and **else** branches as well. To be more precise: **There can be more than one "elif" branch, but only one "else" branch**. The else branch has to ALWAYS be at the end of the if statement.

So, summarizing, the general form of the if statement in Python looks like this:





```
if condition_1:
    statement_block_1
elif condition_2:
    statement_block_2

...

elif another_condition:    
    another_statement_block
else:
    else_block
```

- If the condition **condition_1** is True, the statements of the block **statement_block_1** will be executed. 
- If not, **condition_2** will be evaluated. If **condition_2 evaluates** to True, **statement_block_2** will be executed.
- If **condition_2** is False, the other conditions of the following **elif** conditions will be checked
- Finally, if none of them has been evaluated to True, the indented block below the **else** keyword will be executed. 

Ok, so with the proper explanations made, let's now go back to our  beloved Turtlebot robot, and our beloved exercises! But before that, we  will need to introduce two new methods that you will need to use in  order to complete the following exercise:

- **move_straight()**: As its name says, this method will allow you to start moving the robot in a straight line.

- **stop_robot()**: As its name says, this method will allow you to stop the robot from moving.

Let's now go to the exercise!

​    \- Exercise 3.1 -

a) Inside your robot_control folder, create a new Python script named **test_if.py**.

b) Inside the script, create a program that reproduces the [flowchart](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/Course_Python2.html?AWSAccessKeyId=AKIAJLU2ZOTUFJRMDOAA&Signature=aXGnO7%2F8jPZHHO2MWG8qOCxxe2M%3D&Expires=1663933245#flowchart) showed above.

​    \- End of Exercise 3.1 -

​    \- Expected Result for Exercise 3.1 -

In the Web Shell Ouput, you should get something similar to this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/py3venv_testif.png)

In the simulation, you should see how your robot starts moving  because the distance to the wall is higher than 1 meter, until it  crashes against the wall.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/testif_gif.gif)

​    \- End Expected Result -

​    \- Solution for Exercise 3.1 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/TC-logo-blue.png)

Please try to do it by yourself unless you get stuck or need some  inspiration. You will learn much more if you fight for each exercise.

test_if.py





```
from robot_control_class import RobotControl

robotcontrol = RobotControl()

a = robotcontrol.get_laser(360)

if a < 1:
    robotcontrol.stop_robot()

else:
    robotcontrol.move_straight()

print ("The laser value received was: ", a)
```

​    \- End Solution for Exercise 3.1 -

I'm pretty sure we will agree that, as it is right now, the program  you created in the previous exercise is not very useful. At least, if  what we want to achieve is to make the robot avoid colliding with walls.

This happens because in our program, the condition (is the wall  closer than 1 meter?) is only evaluated one time, when we execute the  program. At that point, the wall is further than 1 meter, so we tell our robot to start moving forward. But... what happens after? As the robot  starts moving forward, the wall keeps getting closer and closer.  Unfortunately, we are not evaluating the condition anymore, so we can't  detect that the wall has gotten closer.

So... how could we keep evaluating this condition in our program in  order to be able to detect the updated distance to the wall? Well, let  me introduce you to the **Loops**!

##         3.3                 Loops    

In many cases, when creating more complex programs, it will be  necessary to evaluate a sequence of statements repeatedly. For instance, check out the following flowchart:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/flowchart_loop.png)

The above flowchart may result in a Python script like this:





```
while wall_is_not_closer_than_1meter:
    keep_moving()
    
stop_robot()
```

In the flowchart above, the condition is checked repeatedly. In this  case, the condition we are checking is if there is a wall less than 1  meter in front of the robot. If not, we will have to keep moving the  robot until the condition is matched. Only when the laser detects the  wall at less than 1 meter (the condition is True) will we be able to go  outside the loop.

Python provides two different types of loops: the **while loop** and the **for loop**.

###         3.3.1                 While loop    

This concept might be a little bit confusing, so I think it's better  to look at an example. In the following script, you will see a very  simple example of a while loop:





```
counter = 0

while counter < 10:
    counter += 1
    print (counter)
    
print ("Outside the loop!")
```

Execute the above script and see what happens.

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/output_while.png)

​    \- End Expected Output -

Basically, this is what's going on in the above example:

- **Counter** is initially 0. The condition in the while statement is **counter < 10**, which is **true**, so the loop body executes. 

- Inside the loop body, **counter is incremented by 1**, and then printed.

- When the body of the loop has finished, the program returns to the  top of the loop and the condition is evaluated again. Now, the value of  the counter is 1, so the condition is still true, so the body executes  again.

- This continues until the counter becomes 10. At that point, when the condition is evaluated, it is **false**, so the loop terminates. The execution of the program will resume at the first statement following the loop body, which is the last print (**"Outside the loop!"**).

​    \- Exercise 3.2 -

a) Create a new Python script named **test_while.py**.

b) Inside the script, create a program that reproduces the [flowchart](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/Course_Python2.html?AWSAccessKeyId=AKIAJLU2ZOTUFJRMDOAA&Signature=aXGnO7%2F8jPZHHO2MWG8qOCxxe2M%3D&Expires=1663933245#flowchart_loop) showed above.

​    \- End of Exercise 3.2 -

​    \- Expected Result for Exercise 3.2 -

In the Web Shell Ouput, you should get something similar to this:

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/py3venv_testwhile.png)

In the simulation, you should see how your robot starts moving  because the distance to the wall is higher than 1 meter, initially.  Then, it keeps moving until the wall is closer than 1 meter, which is  when it will stop.

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/testwhile_gif.gif)

​    \- End Expected Result -

​    \- End Solution for Exercise 3.2 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/TC-logo-blue.png)

Please try to do it by yourself unless you get stuck or need some  inspiration. You will learn much more if you fight for each exercise.

test_while.py





```
from robot_control_class import RobotControl

robotcontrol = RobotControl()

a = robotcontrol.get_laser(360)

while a > 1:
    robotcontrol.move_straight()
    a = robotcontrol.get_laser(360)
    print ("Current distance to wall: %f" % a)

robotcontrol.stop_robot()

print ("Wall is at %f meters! Stop the robot!" % a)
```

​    \- End Solution for Exercise 3.2 -

###         3.3.2                 For loop    

For loops are used for iterating over a sequence. It steps through  the items of lists, strings, the keys of dictionaries, and other  iterables. The Python for loop starts with the keyword **for,** followed by an arbitrary variable name, which will hold the values of  the following sequence object, which is stepped through. The general  syntax looks like this:





```
for variable in sequence:
    statement
```

The items of the sequence object are assigned one after another to  the loop variable. Then, for each item of the sequence, the loop body is executed.

Check out the following example of a simple for loop in Python:





```
names = ["Yoda", "ObiWan", "Anakin", "Palpatine"]

for x in names:
    print(x)
```

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/for_output1.png)

​    \- End Expected Output -

​    \- Exercise 3.3 -

a) Create a new Python script named **test_for.py**

b) First, you will call the **get_laser_full()** method in order to get the full list of laser readings.

c) Then, you will create a for loop that iterates over all the values of the list, and calculates the higher value of them all.

d) Finally, you will print this value to the user.

​    \- End of Exercise 3.3 -

​    \- End Solution for Exercise 3.3 -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/TC-logo-blue.png)

Please try to do it by yourself unless you get stuck or need some  inspiration. You will learn much more if you fight for each exercise.

test_for.py





```
from robot_control_class import RobotControl

robotcontrol = RobotControl()

l = robotcontrol.get_laser_full()

maxim = 0

for value in l:
    if value > maxim:
        maxim = value

print ("The higher value in the list is: ", maxim)
```

​    \- End Solution for Exercise 3.3 -

#### Range method

It is very common to use the **for** expression alongside the **range()** method. Check out the following example:





```
for i in range(5):
    print (i)
```

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/output_for.png)

​    \- End Expected Output -

###         3.3.3                 Interruption of Loop Iteration    

In each example that you have seen so far, the entire body of the  loop is executed on each iteration. Python provides two keywords that  terminate a loop iteration prematurely:

- **break**: It immediately terminates a loop entirely. Program execution proceeds to the first statement following the loop body.

- **continue**: It immediately terminates the current  loop iteration. Execution jumps to the top of the loop, and the  condition is re-evaluated to determine whether the loop will execute  again or terminate.

Check out the following examples:





```
counter = 0

while counter < 10:
    counter += 1
    if counter == 3:
        break
    print (counter)
    
print ("Outside the loop!")
```

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/break_example.png)

​    \- End Expected Output -





```
counter = 0

while counter < 10:
    counter += 1
    if counter == 3:
        continue
    print (counter)
    
print ("Outside the loop!")
```

​    \- Expected Output -

![img](https://s3.eu-west-1.amazonaws.com/notebooks.ws/basic_Python/img/continue_example.png)

​    \- End Expected Output -
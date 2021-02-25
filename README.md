### Experimental robotics assignment 3

This is the assgnment of the lab 3 of Experimental robotics lab simulating a pet robot interacting with a human in ros_noetic

## Software architecture



*'person_command'*
This node publishes a topic command. The user gives a command either **sleep*** or **play**.If its play a random coordinate is generated and assigned so that the dog will go there.

*'process_image'*
This node 	converts images and detects features

*'state_machine'*
The state_machine shifts the three states namely : 
**sleep** : In sleep state the robot dog goes to its home an sleeps after certain time it goes to the normal state
**normal** : In normal state the robot just roams arround untill it receives a command to play or it goes to sleep again
**play** : In play state the robot receive a command from a preson and goes there

## State diagram
The original was supposed to be like this
![Screenshot from 2021-02-25 23-11-04](https://user-images.githubusercontent.com/62798224/109227183-34e85680-77c0-11eb-91d2-b523de01c7b8.png)

I managed to do this

![Screenshot from 2021-02-25 23-11-16](https://user-images.githubusercontent.com/62798224/109227251-4f223480-77c0-11eb-82d7-504fcbcffdaf.png)




**sleep** : In sleep state the robot dog goes to its home an sleeps after certain time it goes to the normal state
**normal** : In normal state the robot just roams arround untill it receives a command to play or it goes to sleep again
**play** : In play state the robot receive a command from a preson and goes there
This state was supposed to be implimented but I could not do it in time.

## Package and files list
* Launch : The simulation.launch file is the main launcher file.
* Nodes : Not all nodes are executable.
* person_command : Is supposed to take commands from user but for now it just automets it.
* process_image : This is to recognise the balls and store coordinates.

Along with those files a zip file named doc is present.which contains the documentation from Doxygen.General folder could not be uploaded since it had too many files.

## Installation and running the code
*'Clone the repository'*
```
git clone https://github.com/raghuveer-sid/exp_assignment3
```
*'run roscore'*
```
roscore
```
*'open new terminal'*
*'goto specific directory'*
```
cd exp_assignment3
```
*'open new terminal'*
*' goto workspace'*
*'source'*
```
source devel/setup.bash
```
*'make'*
```
catkin_make
```
*'launch file'*
```
roslaunch exp_assignment3 simulation.launch
```
## Working

It is as simple as it sounds.Initially the dog wakes up and goes to the normal state where it moves randomly untill it receives a command from a person if it receives the command then it goes to the command point but if it does not receive a command after some time then the dog will go to sleep

But for this assignment 
* Normal to Sleep
* Sleep to Normal
are the only working states


## Limitations and Possible improvements

*The whole assignment is not completed So the full capabilities of this assignment is currently not known.


## Author and contact

**Raghuveer Siddaraboina**
**raghuveersiddaraboina@gmail.com**






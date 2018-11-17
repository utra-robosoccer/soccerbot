ROS will send twist message called /cmd_vel, this node will subscribe
from /cmd_vel and convert the twist message (angular, linear velocities) into 
motor velocities, then it will send the motor velocities to the microcontroller
probably 2 bytes, telling you the speed of each motor

Any questions?
so the microcontroller needs to deal with PID of motors
We don't have shaft encoders... rip
RIP

gonna have to estimate then
ok
but what are the more highlevel messages that turn into motor contol cmds?
I don't think we need them, the micrcontroller should start
alreay reading the 2 floats streamed into the comm


how often these commands are received?
100 Hz is good
rip
can I just get the twist message?
If you want to encode the entire twist struct yeah sure, but some of it is useless
(etc z velocity and x,y angular velocity)
oh so it has an angular velocity + linear velocity?

Yes, twist = 3 angular velocity + 3 linear velocity
Yes!!!!!!
I'll use IMU to get feedback then rather than using shaftencoder

That is also ok
(easier lol)
are you processing IMU on microcontroller?
yes
kk good luck
???
should I pass it to ros?
Yes, it will help localization, but not abosultely necessary especially if the imu feedbackk loop on the 
microcontroller is good, but yeah 6 floats sent back if you can
ok I'll simplify the twist message and send the essentials to the microcontroller

cool

to run it, you'll run

roslaunch omibot_hardware omnibot_hardware.launch

will run the python script

but of course you need to change the 
CMakelist.txt and package.xml files to make sure things work (names)
ok...
btw
this is my PC, I need to setup stuff on my laptop as well
also
how do I push to github?

Yes lemme explain, do you want command line method or clion method
both lol
ok command line first
questions?
what happened to CI???
It gives an X, which means it failed.
Your code must build properly

To build
catkin build

To clean build
catkin clean
catkin build

This is the error, needs to be fixed, gotta change the names in the CMakeList..txt

More questions?

Reminder on how to command line method

git status # check status
git branch -a # look at all branches
git checkout -b sr_branchname #crete new breanch
git add . # stage your files to be commited
git commit -m # Commit your files
git push origin --set-upstream origin sr_branchname # push your changes to github

Then you need to manually create a pull request


Also
git pull (pulls latest commits)
git fetch (pulls all remote branches to local branches)
git clean -fd && git reset --hard # Clean everything from your branch
git reset HEAD^1 # Go back one commit
git checkout branchname # switch to another branch

Thats all you need
where is the dir that the local branch exists
soccer_ws

The the local branch and remote branches, its all hidden in the git tree








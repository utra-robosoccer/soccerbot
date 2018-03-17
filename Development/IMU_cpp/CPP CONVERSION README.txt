-See this thread for how to convert an eclipse project http://www.openstm32.org/forumthread2509?topics_offset=8

VERY IMPORTANT:
-Cubemx automatically generates .c files for main and freertos. So, EACH TIME you want to modify something in cube, RENAME THE USER-MODIFIED FILES (ie. freertos.cpp and main.cpp) BACK TO BE .c FILES. 
Then, make the necessary modifications in cube, generate the code, then rename the files you changed to be .cpp files. Now you can run and modify them further in eclipse.
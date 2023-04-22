## URDF and xacro are now both named the same:
- walls                      <-- have all the walls, and all the shelves in the middle of the map
- mobile_robot      	       <-- Single mobile robot
- world_mobile_x             <-- world for sims we've been using, x is the number of agents i.e. world_mobile_4
- world_no_shelves_mobile_x  <-- Self explanatory


## Still in testing/upcoming
- arms_2 			                  <-- 2 panda arms on a block
- world_arms_y 			          <-- world with just arms, y such arms
- world_mobile_x_arms_y  	          <-- Our current testing worlds with arms added in
- world_no_shelves_mobile_x_arms_y  <-- you get the idea

## The agents.txt file will now be called a "test file" so you can have different tests saved and rerun easily
- Format has changed so top line has the whole world description
- Avoids us having to rebuild main.cpp each time we change the world.
- i.e. for a 9 mobile agent file, the top line in the file will be, without the quotations, "world_mobile_9"

## When running the planner, select the trial that you're running as follows:
`roslaunch mamp_planner mamp_planner.launch trial:="world_mobile_9_arms_2_test_1.txt"`
    
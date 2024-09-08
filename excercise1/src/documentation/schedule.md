# Schedule:
1. Research/Learning 
**1h. Very important, so time is flexible. E.g. until I feel confident with ROS**.
    - What, why, how ROS?


2. Installation/Requirements
**10min. I know Docker and already use Ubuntu, Git and IDEs**.
- Get Docker image etc.
- Write handy scripts to configure/launch/connect containers

3. Follow the beginner tutorial step by step
**30 minutes
Official ROS tutorial was a good starting point.

4. Design a solution
**5 minutes
- EX1 is very similar to the ROS Publisher-Subscriber tutorial.
- which is heavily based on this tutorial.

5. Develop solution
**1.5h**
- change message type, topic name, publishing rate and callback parameter
- visualise and log traffic

# Problems on the way:
- Could not connect docker container to x-display:  Add docker as xhost (`xhost +local:docker `)
- In what order to run `source run_ros.sh`, `catkin build` and `source ./devel/setuo.bash`.
- In which folder to run the above three.
- Difficulties in finding the correct PlotJuggler installation with a ros-topic listener. (`sudo apt install ros-${ROS_DISTRO}-plotjuggler-ros`)
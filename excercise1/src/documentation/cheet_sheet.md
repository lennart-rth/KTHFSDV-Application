# Roscore
1. `sudo ./startup.sh`
2. `cd ..`
3. [`source ./ros_entrypoint.sh`]   (included in startup.sh)
4. `catkin build`
5. `source ./devel/setup.bash`
6. `roscore`

# Publisher Node
1. `sudo ./connect.sh`
2. `./init.sh`
3. [`source ./ros_entrypoint.sh`]   (included in init.sh)
4. `source ./devel/setup.bash`
5. `rosrun publisher talker.py`

# Subscriber Node
1. `sudo ./connect.sh`
2. `./init.sh`
3. [`source ./ros_entrypoint.sh`]   (included in init.sh)
4. `source ./devel/setup.bash`
5. `rosrun subscriber listener.py`

# Plotjuggler
rosrun plotjuggler plotjuggler
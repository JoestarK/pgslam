# pgslam
## introduction
pgslam (pose graph slam) is a slam package based on ROS. pgslam reads laser scans and odometry(tf) and publish occupancy map and pose(tf). pgslam dance icp algorithm to get pose as constrain and give these constrains to isam to construct a graph.
## installation
1. install ros indigo
 1. sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
 2. sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
 3. sudo apt-get update
 4. sudo apt-get install ros-indigo-desktop-full
2. install isam
 1. wget http://people.csail.mit.edu/kaess/isam/isam_v1_7.tgz
 2. tar xzvf isam_V1_7.tgz
 3. cd isam_V1_7
 4. make
 5. sudo make install
3. create catkin workspace
 1. mkdir -p ~/catkin_ws/src
 2. cd ~/cattkin_ws/src/
 3. catkin_init_workspace
5. git clone https://github.com/yukunlinykl/pgslam ~/catkin_ws/src/pgslam
6. cd ~/catkin_ws && catkin_make
7. cd ~/catkin_ws && catkin_make install

## run
1. source /opt/ros/indigo/setup.bash
2. source catkin_ws/install/setup.bash
3. roslaunch pgslam playbag.launch

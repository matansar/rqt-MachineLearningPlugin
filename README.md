# Instructions rqt Machine-Learning-Plugin (MLP) #
This repository offers a new rqt plugin that allows you to produce a dataset for machine learning. <br/>
#### In order to install the rqt plugin, MLP, please follow the following steps: ####
 * Install Ubuntu 14.04 64-bit
 * Install Indigo ROS distribution: http://wiki.ros.org/indigo/Installation/Ubuntu
 * Install and configure your ROS environment: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
 * Install RoboTiCan project: http://wiki.ros.org/robotican/Tutorials/Installation
 * open the linux's terminal/bash and run:
   ```{r, engine='sh', count_lines}
   cd ~/catkin_ws/src/
   ```
 * Download or clone this project using terminal/bash:
   ```{r, engine='sh', count_lines}
   git clone https://github.com/matansar/rqt-MachineLearningPlugin.git
   ```
 * run ./setup
 * move to catkin_ws
 * do: catkin_make
 * In the first time you open rqt, you need to run: rqt --force-discover
   
The code is based on python. So, you need to install the following python packages:
 * install pandas package:
   ```{r, engine='sh', count_lines}
   sudo apt-get install python-pip
   ```
   -- if you have dependence problems, run the following command to fix it:
   ```{r, engine='sh', count_lines}
   sudo apt-get -f install
   ```
   then run:
   ```{r, engine='sh', count_lines}
   sudo pip install numpy
   sudo pip install pandas
   ```
 * install statistics package:
      ```{r, engine='sh', count_lines}
   sudo pip install statistics
   ```

need to add chmod +x 

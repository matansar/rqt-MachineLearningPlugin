# Instructions rqt Machine-Learning-Plugin (MLP) #
This repository offers a new rqt plugin that allows you to produce a dataset for machine learning. <br/>
#### In order to install the rqt plugin, MLP, please follow the following steps: ####
 * Install Ubuntu 14.04 64-bit
 * Install Indigo ROS distribution: http://wiki.ros.org/indigo/Installation/Ubuntu
 * Install and configure your ROS environment: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
 * Install RoboTiCan project: http://wiki.ros.org/robotican/Tutorials/Installation
 * Install python
 * The code is based on python. So, you need to install the following python packages:
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

 * Download or clone this project using terminal/bash:
   ```{r, engine='sh', count_lines}
   cd ~/catkin_ws/src/
   git clone https://github.com/matansar/rqt-MachineLearningPlugin.git
   ```
 * Run the setup.sh file and compile:
   ```{r, engine='sh', count_lines}
   cd ~/catkin_ws/
   rqt-MachineLearningPlugin/rqt_mlp/install.setup.sh
   catkin_make
   ```
 * At the first time you open rqt, you need to run:
   ```{r, engine='sh', count_lines}
   rqt --force-discover
   ```

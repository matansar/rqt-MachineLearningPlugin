import subprocess
import time
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

def Run_Scenario(scen_obj):
    time.sleep(10)
    ros_launch = "roslaunch robotican_armadillo armadillo4.launch kinect2:=true softkinetic:=true  " \
		 "gmapping:=true hector_slam:=true move_base:=true lidar:=true " \
                 "world_name:=\"`rospack find robotican_common`/worlds/objects_on_table.world\" "
    launch_cmd = ros_launch
    subprocess.Popen(launch_cmd, shell=True)
    apply_statistics()
    run_rviz()
    apply_diagnostic()
    scen_obj.generate_bag()


def apply_statistics():
    enable_statistics = "rosparam set enable_statistics true"
    subprocess.Popen(enable_statistics, shell=True)


def run_rviz():
    rviz = "rosrun rviz rviz"
    subprocess.Popen(rviz, shell=True)


def apply_diagnostic():
    ros_profiler = "rosrun rosdiagnostic rosdiagnostic"
    subprocess.Popen(ros_profiler, shell=True)


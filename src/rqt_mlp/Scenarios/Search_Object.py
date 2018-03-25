import subprocess
import time
import rospy
from std_msgs.msg import String

SLEEPING_TIME = 15

def Run_Scenario(scen_obj):
    time.sleep(10)
    ros_launch = "roslaunch robotican_armadillo armadillo4.launch kinect2:=true softkinetic:=true gmapping:=true hector_slam:=true move_base:=true lidar:=true gazebo:=true world_name:=\"`rospack find robotican_common`/worlds/objects_on_table.world\""
    launch_cmd = ros_launch
    subprocess.Popen(launch_cmd, shell=True)
    apply_statistics()
    time.sleep(SLEEPING_TIME)
    run_rviz()
    apply_diagnostic()
    apply_simulation(scen_obj)
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


def when_found(msg, scen_obj):
  if msg.data == "find object":
    scen_obj.close_bag()
    print "shut down"


def publising_goals(scen_obj):
  import time
  rospy.Subscriber('/robot_state', String, when_found, scen_obj)
  msg = "t"
  time.sleep(2)
  publish(msg)


def publish(msg):
  goal = "rostopic pub /robot_state std_msgs/String " + msg
  subprocess.Popen(goal, shell=True)
  print "Published" #: " + str(msg)

def apply_simulation(scen_obj):
    import time
    print "simulation started..."
    time.sleep(10)
    ros_run = "roslaunch robotican_demos_upgrade demo.launch"
    subprocess.Popen(ros_run, shell=True)
    time.sleep(10)
    publising_goals(scen_obj)


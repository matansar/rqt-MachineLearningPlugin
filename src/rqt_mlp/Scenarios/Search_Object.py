import subprocess
import time
import rospy
from std_msgs.msg import String

SLEEPING_TIME = 10


def Run_Scenario(scen_obj):
    ros_launch = "roslaunch robotican_armadillo armadillo4.launch kinect2:=true gmapping:=true hector_slam:=true move_base:=true lidar:=true gazebo:=true world_name:=\"`rospack find robotican_common`/worlds/objects_on_table1.world\""
    launch_cmd = ros_launch
    subprocess.Popen(launch_cmd, shell=True)
    apply_statistics()
    time.sleep(SLEEPING_TIME)
    #run_rviz()
    apply_diagnostic()
    # topics = scen_obj.get_topics_list()
    # apply_simulation(scen_obj, topics)
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
  if msg.data == "find object" or msg.data == "not find object":
    scen_obj.close_bag()
    print "shut down"


def publising_goals(scen_obj):
  rospy.Subscriber('/plp/trigger', String, when_found, scen_obj)
  msg = "t"
  publish(msg)


def publish(msg):
  goal = "rostopic pub /plp/trigger std_msgs/String " + msg
  subprocess.Popen(goal, shell=True)
  print "Published"  #: " + str(msg)


def apply_simulation(scen_obj):
    import time
    print "simulation started..."
    time.sleep(10)
    #ros_run = "roslaunch robotican_demos_upgrade demo_new.launch"
    ros_run = "roslaunch robotican_demos_upgrade demo1.launch"
    subprocess.Popen(ros_run, shell=True)
    # time.sleep(1.6)
    # goal1 = "rosbag record -O sub"
    # for item in topics:
    #     goal1 += " " + item
    # # goal1 += " --split --duration=15"
    # print goal1
    # subprocess.Popen(goal1, shell=True)
    publising_goals(scen_obj)

def run_attacker():
    rviz = "roslaunch robotican_demos_upgrade attacker.launch"
    subprocess.Popen(rviz, shell=True)

